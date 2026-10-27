"""Captura real de redes: nmcli para inventario, iw para RSSI directo en dBm.

No asigna APs a esquinas por fuerza de señal ni inventa nodos ESP32.
No se presenta SIGNAL (%) de nmcli como si fuese RSSI medido (dBm).
"""
from __future__ import annotations

import json
import math
import re
import subprocess
import time
from collections import defaultdict
from pathlib import Path

import numpy as np

MAC = re.compile(r'^[0-9A-F]{2}(?::[0-9A-F]{2}){5}$')
BSS = re.compile(r'^BSS\s+([0-9a-f:]{17})\b', re.IGNORECASE)
SIGNAL = re.compile(r'^signal:\s*(-?\d+(?:\.\d+)?)\s*dBm', re.I)


def _ejecutar(comando, timeout=20):
    try:
        r = subprocess.run(comando, capture_output=True, text=True, check=True, timeout=timeout)
        return r.stdout
    except FileNotFoundError as e:
        raise RuntimeError(f'Falta el comando del sistema: {comando[0]}') from e
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f'Fallo {" ".join(comando)}: {(e.stderr or e.stdout).strip()}') from e
    except subprocess.TimeoutExpired as e:
        raise RuntimeError(f'El comando {comando[0]} excedio {timeout} segundos.') from e


def listar_vecinos_lan():
    """Lee vecinos ya conocidos; no realiza barridos masivos de IPs."""
    try:
        salida = _ejecutar(['ip', 'neigh', 'show'])
    except RuntimeError as exc:
        return {'error': str(exc), 'dispositivos': []}
    resultado = []
    for linea in salida.splitlines():
        partes = linea.split()
        if 'lladdr' in partes:
            indice = partes.index('lladdr')
            if indice + 1 < len(partes):
                resultado.append({'ip': partes[0], 'mac': partes[indice + 1].upper(),
                                  'estado': partes[-1]})
    return {'dispositivos': resultado}


def _campos_nmcli(linea):
    """Lee salida terse escapada de nmcli (BSSID y SSID pueden contener ':')."""
    campos, actual, escapado = [], [], False
    for caracter in linea:
        if escapado:
            actual.append(caracter)
            escapado = False
        elif caracter == '\\':
            escapado = True
        elif caracter == ':':
            campos.append(''.join(actual))
            actual = []
        else:
            actual.append(caracter)
    if escapado:
        actual.append('\\')
    campos.append(''.join(actual))
    return campos


def inventario_nmcli():
    """Devuelve SSID, BSSID y porcentaje reportado por NetworkManager."""
    salida = _ejecutar(['nmcli', '-t', '-e', 'yes', '-f', 'BSSID,SSID,FREQ,SIGNAL',
                        'device', 'wifi', 'list', '--rescan', 'yes'])
    redes = []
    for linea in salida.splitlines():
        partes = _campos_nmcli(linea)
        if len(partes) != 4:
            continue
        mac, ssid, freq, pct = partes
        mac = mac.upper()
        if not MAC.fullmatch(mac):
            continue
        try:
            redes.append({'bssid': mac, 'ssid': ssid, 'frecuencia_mhz': int(freq),
                          'signal_pct_nmcli': int(pct)})
        except ValueError:
            continue
    return redes


def analizar_iw_scan(salida):
    """Extrae RSSI observado por iw (dBm) del propio receptor, sin conversion %->dBm."""
    redes = []
    actual = None
    for linea in salida.splitlines():
        s = linea.strip()
        cabecera = BSS.match(s)
        if cabecera:
            if actual is not None and actual.get('rssi_dbm') is not None:
                redes.append(actual)
            actual = {'bssid': cabecera.group(1).upper(), 'ssid': '', 'rssi_dbm': None}
            continue
        if actual is None:
            continue
        m = SIGNAL.match(s)
        if m:
            actual['rssi_dbm'] = float(m.group(1))
        elif s.startswith('SSID:'):
            actual['ssid'] = s.split(':', 1)[1].strip()
    if actual is not None and actual.get('rssi_dbm') is not None:
        redes.append(actual)
    return redes


def capturar_iw(interfaz, barridos, pausa_seg):
    if not interfaz or not re.fullmatch(r'[a-zA-Z0-9_.-]+', interfaz):
        raise ValueError('Especifique una interfaz Wi-Fi valida con --interfaz.')
    if barridos < 1 or pausa_seg < 0:
        raise ValueError('El numero de barridos debe ser positivo y la pausa no negativa.')
    registros = defaultdict(list)
    ssids = {}
    for indice in range(barridos):
        salida = _ejecutar(['iw', 'dev', interfaz, 'scan'], timeout=40)
        for red in analizar_iw_scan(salida):
            registros[red['bssid']].append(red['rssi_dbm'])
            ssids[red['bssid']] = red['ssid']
        if indice + 1 < barridos:
            time.sleep(pausa_seg)
    return [{'bssid': mac, 'ssid': ssids.get(mac, ''),
             'rssi_dbm': float(np.mean(muestras)),
             'sigma_rssi_db': float(np.std(muestras, ddof=1)) if len(muestras) >= 2 else None,
             'numero_muestras': len(muestras), 'origen': 'iw_scan'}
            for mac, muestras in sorted(registros.items())]


def cargar_anclas(ruta):
    datos = json.loads(Path(ruta).read_text(encoding='utf-8'))
    if not isinstance(datos, list) or len(datos) < 3:
        raise ValueError('Las anclas deben ser una lista JSON de al menos tres dispositivos medidos.')
    ids, macs, coords, salida = set(), set(), [], []
    for p in datos:
        nombre = str(p['id'])
        mac = str(p['bssid']).upper()
        if not MAC.fullmatch(mac) or mac in macs or nombre in ids:
            raise ValueError(f'BSSID / ID invalido o repetido: {nombre}, {mac}')
        x, y = float(p['x_m']), float(p['y_m'])
        if not all(math.isfinite(c) for c in (x, y)):
            raise ValueError('Coordenadas no finitas.')
        ids.add(nombre)
        macs.add(mac)
        coords.append((x, y))
        salida.append({'id_ancla': nombre, 'bssid': mac, 'coordenadas': (x, y)})
    geom = np.asarray(coords, dtype=float)
    if np.linalg.matrix_rank(geom[1:] - geom[0]) < 2:
        raise ValueError('Las posiciones de las anclas son colineales.')
    return salida


def cargar_lecturas_json(ruta):
    """Importa mediciones aportadas por la persona usuaria, sin crear lecturas ficticias."""
    datos = json.loads(Path(ruta).read_text(encoding='utf-8'))
    if not isinstance(datos, list):
        raise ValueError('Las lecturas deben ser una lista JSON.')
    lecturas = []
    for elemento in datos:
        mac = str(elemento['bssid']).upper()
        rssi = float(elemento['rssi_dbm'])
        if not MAC.fullmatch(mac) or not math.isfinite(rssi):
            raise ValueError(f'Lectura invalida: {mac}, {rssi}')
        sigma = elemento.get('sigma_rssi_db')
        if sigma is not None:
            sigma = float(sigma)
            if sigma < 0 or not math.isfinite(sigma):
                raise ValueError(f'Desviacion RSSI invalida para {mac}')
        lecturas.append({'bssid': mac, 'rssi_dbm': rssi, 'sigma_rssi_db': sigma,
                         'numero_muestras': elemento.get('numero_muestras'),
                         'origen': 'json_proporcionado_por_usuario'})
    return lecturas


def asociar_lecturas_anclas(anclas, mediciones):
    por_mac = {r['bssid']: r for r in mediciones}
    faltantes = [a['id_ancla'] + ' (' + a['bssid'] + ')' for a in anclas if a['bssid'] not in por_mac]
    if faltantes:
        raise RuntimeError('Faltan mediciones de anclas registradas: ' + ', '.join(faltantes))
    resultado = []
    for a in anclas:
        r = por_mac[a['bssid']]
        resultado.append({**a, 'rssi_efectivo_dbm': r['rssi_dbm'],
                          'sigma_rssi_db': r['sigma_rssi_db'],
                          'numero_muestras': r.get('numero_muestras'), 'origen': r['origen']})
    return resultado


def construir_ecuacion_calibracion_n(rssi_ref_dbm, d_ref_m, pt_dbm, pl_d0, alpha, d0=1.0):
    if min(d_ref_m, d0) <= 0 or math.isclose(d_ref_m, d0):
        raise ValueError('Las distancias de calibracion y referencia deben ser positivas y distintas.')
    def f(n):
        return 10.0 * n * math.log10(d_ref_m / d0) + alpha * d_ref_m + rssi_ref_dbm - pt_dbm + pl_d0
    return f


def construir_ecuacion_distancia(rssi_medido, pt_dbm, pl_d0, n, alpha, d0=1.0):
    if n <= 0 or d0 <= 0 or alpha < 0:
        raise ValueError('n y d0 deben ser positivos; alpha no puede ser negativo.')
    constante = rssi_medido - pt_dbm + pl_d0
    def f(d):
        if d <= 0:
            raise ValueError('La distancia debe ser positiva.')
        return 10.0 * n * math.log10(d / d0) + alpha * d + constante
    def df(d):
        if d <= 0:
            raise ValueError('La distancia debe ser positiva.')
        return 10.0 * n / (d * math.log(10.0)) + alpha
    return f, df


def ensamblar_sistema_trilateracion(nodos_ancla, distancias_estimadas):
    nodos = np.asarray(nodos_ancla, dtype=float)
    distancias = np.asarray(distancias_estimadas, dtype=float)
    if len(nodos) < 3 or len(nodos) != len(distancias) or nodos.shape[1] != 2:
        raise ValueError('Nodos/distancias inconsistentes o menos de tres anclas.')
    if np.any(distancias <= 0):
        raise ValueError('Distancias no positivas.')
    piv = nodos[-1]
    dp = distancias[-1]
    A = 2.0 * (nodos[:-1] - piv)
    B = np.sum(nodos[:-1]**2, axis=1) - np.sum(piv**2) - (distancias[:-1]**2 - dp**2)
    if np.linalg.matrix_rank(A) != 2:
        raise ValueError('La geometria no permite trilateracion bidimensional.')
    return A, B, A.T @ A, A.T @ B
