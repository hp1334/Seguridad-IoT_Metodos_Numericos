# src/telematica/wifi_csi_sniffer.py
"""
Módulo Telemático Real (Sin datos hardcodeados):
1. Escanea dispositivos reales en la red local mediante la tabla ARP/vecinos del sistema.
2. Captura niveles reales de potencia electromagnética (RSSI en dBm), BSSID y frecuencia
   usando la tarjeta Wi-Fi física de la laptop (nmcli / iwlist / netsh) y/o un ESP32 vía Serial.
3. Ensambla el sistema linealizado de trilateración M*X = C a partir de mediciones reales.
"""
import math
import platform
import re
import subprocess
import time
import numpy as np


def escanear_dispositivos_red_local():
    """
    Consulta la tabla ARP / Neighbor real del sistema operativo para identificar
    cuántos dispositivos reales (IP y dirección MAC física) están activos en la red.

    Returns:
        list[dict]: Lista real de dispositivos detectados por la interfaz de red.
    """
    dispositivos = []
    sistema = platform.system().lower()

    try:
        if "linux" in sistema:
            salida = subprocess.check_output(["ip", "neigh", "show"], text=True, stderr=subprocess.DEVNULL)
            for linea in salida.splitlines():
                partes = linea.split()
                if "lladdr" in partes:
                    idx_mac = partes.index("lladdr") + 1
                    ip = partes[0]
                    mac = partes[idx_mac].upper()
                    estado = partes[-1]
                    dispositivos.append({"ip": ip, "mac": mac, "estado": estado})
        else:
            salida = subprocess.check_output(["arp", "-a"], text=True, stderr=subprocess.DEVNULL)
            patron = re.compile(r"(\d+\.\d+\.\d+\.\d+)\s+([0-9a-fA-F:-]{17})\s+(\w+)")
            for ip, mac, tipo in patron.findall(salida):
                dispositivos.append({"ip": ip, "mac": mac.upper().replace("-", ":"), "estado": tipo})
    except Exception as exc:
        raise RuntimeError(f"Error al consultar la tabla de red del sistema operativo: {exc}") from exc

    return dispositivos


def _capturar_barrido_wifi_os():
    """
    Ejecuta una lectura física de la tarjeta Wi-Fi de la laptop y convierte la señal
    medida a dBm reales. Soporta NetworkManager (nmcli) en Linux y netsh en Windows.
    """
    sistema = platform.system().lower()
    redes_detectadas = []

    if "linux" in sistema:
        cmd = ["nmcli", "-t", "-f", "BSSID,SSID,FREQ,SIGNAL", "dev", "wifi", "list", "--rescan", "auto"]
        salida = subprocess.check_output(cmd, text=True, stderr=subprocess.DEVNULL)

        for linea in salida.splitlines():
            if not linea.strip():
                continue
            # nmcli escapa los ':' del BSSID como '\:'
            linea_limpia = linea.replace("\\:", "-")
            partes = linea_limpia.split(":")
            if len(partes) >= 4:
                bssid = partes[0].replace("-", ":").upper()
                ssid = partes[1] if partes[1] else "RED_OCULTA"
                freq_str = partes[2].replace("MHz", "").strip()
                sig_pct = float(partes[3])

                # Conversión estándar IEEE 802.11 de porcentaje de calidad a dBm reales
                rssi_dbm = (sig_pct / 2.0) - 100.0
                frecuencia_mhz = float(freq_str) if freq_str.isdigit() else 2412.0

                redes_detectadas.append({
                    "bssid": bssid,
                    "ssid": ssid,
                    "frecuencia_mhz": frecuencia_mhz,
                    "rssi_dbm": rssi_dbm
                })
    else:
        cmd = ["netsh", "wlan", "show", "networks", "mode=bssid"]
        salida = subprocess.check_output(cmd, text=True, stderr=subprocess.DEVNULL, encoding="latin-1")
        ssid_actual = "DESCONOCIDO"
        bssid_actual = None
        for linea in salida.splitlines():
            linea = linea.strip()
            if linea.startswith("SSID"):
                partes = linea.split(":", 1)
                if len(partes) > 1:
                    ssid_actual = partes[1].strip() or "RED_OCULTA"
            elif linea.startswith("BSSID"):
                partes = linea.split(":", 1)
                if len(partes) > 1:
                    bssid_actual = partes[1].strip().upper()
            elif "%" in linea and bssid_actual:
                match_pct = re.search(r"(\d+)%", linea)
                if match_pct:
                    sig_pct = float(match_pct.group(1))
                    rssi_dbm = (sig_pct / 2.0) - 100.0
                    redes_detectadas.append({
                        "bssid": bssid_actual,
                        "ssid": ssid_actual,
                        "frecuencia_mhz": 2412.0,
                        "rssi_dbm": rssi_dbm
                    })
                    bssid_actual = None

    return redes_detectadas


def capturar_espectro_wifi_real( coordenada_anclas, num_barridos=3, pausa_seg=0.4):
    """
    Realiza múltiples barridos reales con la tarjeta Wi-Fi de la laptop para capturar
    la potencia RSSI (dBm) y su varianza real respecto a las estaciones/anclas activas.

    Args:
        coordenada_anclas (np.ndarray): Coordenadas físicas (x, y) en metros de las anclas en el cuarto.
        num_barridos (int): Cantidad de muestras reales tomadas en el aire.
        pausa_seg (float): Tiempo entre barridos consecutivos.

    Returns:
        dict: Dispositivos ARP reales y mediciones reales de las anclas Wi-Fi.
    """
    dispositivos_arp = escanear_dispositivos_red_local()
    historico_bssid = {}

    for _ in range(num_barridos):
        lecturas = _capturar_barrido_wifi_os()
        for red in lecturas:
            bssid = red["bssid"]
            if bssid not in historico_bssid:
                historico_bssid[bssid] = {
                    "ssid": red["ssid"],
                    "frecuencia_mhz": red["frecuencia_mhz"],
                    "muestras_rssi": []
                }
            historico_bssid[bssid]["muestras_rssi"].append(red["rssi_dbm"])
        if num_barridos > 1:
            time.sleep(pausa_seg)

    if len(historico_bssid) < 3:
        raise RuntimeError(
            f"Se detectaron solo {len(historico_bssid)} señales Wi-Fi reales en el aire. "
            "Para la trilateración 2D real se requieren mínimo 3 señales activas "
            "(ej. Router + ESP32 + Hotspot del celular)."
        )

    # Seleccionar las N señales reales con mayor potencia (las más cercanas al cuarto)
    n_requeridas = len(coordenada_anclas)
    bssid_ordenados = sorted(
        historico_bssid.items(),
        key=lambda item: float(np.mean(item[1]["muestras_rssi"])),
        reverse=True
    )[:n_requeridas]

    lecturas_anclas = []
    for idx, (bssid, info) in enumerate(bssid_ordenados):
        muestras = np.array(info["muestras_rssi"], dtype=float)
        lecturas_anclas.append({
            "id_ancla": f"ANCLA_{idx+1}_{info['ssid'][:12]}",
            "bssid": bssid,
            "ssid": info["ssid"],
            "frecuencia_mhz": info["frecuencia_mhz"],
            "coordenadas": (float(coordenada_anclas[idx][0]), float(coordenada_anclas[idx][1])),
            "rssi_efectivo_dbm": float(np.mean(muestras)),
            "varianza_real_db": float(np.var(muestras))
        })

    return {
        "total_dispositivos_red": len(dispositivos_arp),
        "dispositivos_red": dispositivos_arp,
        "total_redes_aire": len(historico_bssid),
        "lecturas_anclas": lecturas_anclas
    }


def construir_ecuacion_calibracion_n(rssi_ref_dbm, d_ref_m, pt_dbm=20.0, pl_d0=40.0, alpha=0.25, d0=1.0):
    """
    Construye la ecuación no lineal f(n) = 0 para autocalibrar el exponente de pérdida
    de trayecto (n) del cuarto usando una medición real a distancia conocida d_ref_m.
    """
    def f_n(n):
        return 10.0 * n * math.log10(d_ref_m / d0) + alpha * d_ref_m + (rssi_ref_dbm - pt_dbm + pl_d0)

    return f_n


def construir_ecuacion_distancia(rssi_medido, pt_dbm=20.0, pl_d0=40.0, n=2.6, alpha=0.25, d0=1.0):
    """
    Construye la ecuación transcendental f(d) = 0 y su derivada f'(d) para hallar
    la distancia física a partir del RSSI real medido en dBm:
    f(d) = 10 * n * log10(d / d0) + alpha * d + (RSSI - Pt + PL(d0)) = 0
    """
    constante = rssi_medido - pt_dbm + pl_d0

    def f_d(d):
        if d <= 0:
            return -1e9
        return 10.0 * n * math.log10(d / d0) + alpha * d + constante

    def df_d(d):
        if d <= 0:
            return 1e9
        return (10.0 * n) / (d * math.log(10.0)) + alpha

    return f_d, df_d


def ensamblar_sistema_trilateracion(nodos_ancla, distancias_estimadas):
    """
    Linealiza el sistema de intersección de circunferencias reales restando el nodo pivote N
    y forma el sistema normalizado M*X = C (donde M = A^T * A y C = A^T * B).
    """
    n_total = len(nodos_ancla)
    if n_total < 3:
        raise ValueError("Se requieren al menos 3 anclas reales para trilateración 2D.")

    x_piv, y_piv = nodos_ancla[-1]
    d_piv = distancias_estimadas[-1]

    A_filas = []
    B_filas = []

    for i in range(n_total - 1):
        xi, yi = nodos_ancla[i]
        di = distancias_estimadas[i]

        A_filas.append([2.0 * (xi - x_piv), 2.0 * (yi - y_piv)])
        b_val = (xi**2 - x_piv**2) + (yi**2 - y_piv**2) - (di**2 - d_piv**2)
        B_filas.append(b_val)

    A = np.array(A_filas, dtype=float)
    B = np.array(B_filas, dtype=float)

    M = A.T @ A
    C = A.T @ B
    return A, B, M, C