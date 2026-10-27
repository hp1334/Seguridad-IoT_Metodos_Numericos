"""Red y radio: inventario, ESP32 SNIFF, RSSI de AP y modelo de trilateración.

El ESP32 original informa SNIFF,MAC,RSSI en su propio canal; esos datos NO
constituyen múltiples anclas. La localización opcional requiere RSSI de varios
AP con coordenadas conocidas, medidos desde el mismo receptor.
"""
from __future__ import annotations
import csv
import math
import platform
import re
import statistics
import subprocess
import time
from collections import defaultdict
from pathlib import Path
import numpy as np

MAC_RE = re.compile(r'^[0-9A-F]{2}(?::[0-9A-F]{2}){5}$')


def normalizar_mac(mac):
    valor = str(mac).strip().replace('-', ':').upper()
    if not MAC_RE.fullmatch(valor):
        raise ValueError(f'MAC/BSSID no válida: {mac!r}')
    return valor


def comando(args, timeout=15):
    salida = subprocess.run(args, text=True, capture_output=True, timeout=timeout, check=False)
    if salida.returncode:
        raise RuntimeError(f'{args[0]} (código {salida.returncode}): {(salida.stderr or salida.stdout).strip()[:500]}')
    return salida.stdout


def descubrir_interfaz():
    """Elige interfaz Wi-Fi por OS; se puede sobrescribir con --interfaz."""
    if platform.system() != 'Linux':
        return None
    try:
        salida = comando(['iw','dev'])
        interfaces = re.findall(r'^\s*Interface\s+(\S+)',salida,re.MULTILINE)
        return interfaces[0] if interfaces else None
    except (OSError, RuntimeError, subprocess.TimeoutExpired):
        return None


def inventario_wifi():
    """Lista redes observadas: SIGNAL de nmcli/netsh es porcentaje, NO dBm."""
    redes=[]
    if platform.system()=='Linux':
        salida=comando(['nmcli','-t','-f','BSSID,SSID,FREQ,SIGNAL','dev','wifi','list','--rescan','yes'],timeout=25)
        for linea in salida.splitlines():
            partes=re.split(r'(?<!\\):',linea,maxsplit=3)
            if len(partes)!=4: continue
            try:
                mac=normalizar_mac(partes[0].replace('\\:',':'))
                señal=float(partes[3])
                frecuencia=re.search(r'\d+',partes[2])
            except ValueError:
                continue
            redes.append({'bssid':mac,'ssid':partes[1].replace('\\:',':'),
                          'frecuencia_mhz':int(frecuencia.group()) if frecuencia else None,
                          'senal_porcentaje':señal,'origen':'nmcli'})
    elif platform.system()=='Windows':
        salida=comando(['netsh','wlan','show','networks','mode=bssid'],timeout=25)
        ssid=''
        mac=None
        for linea in salida.splitlines():
            dato=linea.strip()
            if re.match(r'^SSID\s+\d+\s*:',dato,re.I):
                ssid=dato.split(':',1)[1].strip()
            elif re.match(r'^BSSID\s+\d+\s*:',dato,re.I):
                mac=normalizar_mac(dato.split(':',1)[1].strip())
            elif mac and re.match(r'^(Signal|Señal)\s*:',dato,re.I):
                m=re.search(r'(\d+)\s*%',dato)
                if m:
                    redes.append({'bssid':mac,'ssid':ssid,'frecuencia_mhz':None,
                                  'senal_porcentaje':float(m.group(1)),'origen':'netsh'})
                mac=None
    return redes


def vecinos_lan():
    """Entradas de caché ARP/neigh; no presupone que estén activas ni que sean IoT."""
    if platform.system()=='Linux':
        salida=comando(['ip','neigh','show'])
        dispositivos=[]
        for l in salida.splitlines():
            p=l.split()
            if 'lladdr' in p:
                i=p.index('lladdr')
                dispositivos.append({'ip':p[0],'mac':p[i+1].upper(), 'estado':p[-1]})
        return dispositivos
    salida=comando(['arp','-a'])
    encontrados=[]
    for ip,mac in re.findall(r'(\d+\.\d+\.\d+\.\d+)\s+([0-9a-f:-]{17})',salida,re.I):
        encontrados.append({'ip':ip,'mac':mac.upper(),'estado':'caché ARP'})
    return encontrados


def puerto_esp32_automatico():
    """Solo intenta autoprobado de puertos con pistas de fabricante ESP32."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return None
    candidatos=[]
    for p in list_ports.comports():
        huella=' '.join(str(x or '') for x in (p.description,p.manufacturer,p.hwid)).lower()
        if any(x in huella for x in ('cp210','ch340','ch341','espressif','usb serial','silicon labs')):
            candidatos.append(p.device)
    return candidatos[0] if len(candidatos)==1 else None


def analizar_linea_esp32(linea):
    """Convierte una trama del firmware a (MAC, RSSI) o None si es inválida."""
    partes = str(linea).strip().split(',')
    if len(partes) != 3 or partes[0] != 'SNIFF':
        return None
    try:
        mac, rssi = normalizar_mac(partes[1]), int(partes[2])
    except ValueError:
        return None
    return (mac, rssi) if -128 <= rssi <= 0 else None


def capturar_esp32(puerto, segundos=5.0, baudrate=115200, mac_objetivo=None):
    """Lee EXACTAMENTE el protocolo del firmware aportado: SNIFF,MAC,RSSI.

    Una MAC puede corresponder a un transmisor aleatorio; no se atribuye al
    teléfono salvo que el experimentador lo haya comprobado.
    """
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError('Instale pyserial: python -m pip install pyserial') from exc
    objetivo=normalizar_mac(mac_objetivo) if mac_objetivo else None
    muestras=defaultdict(list)
    lineas=0
    with serial.Serial(puerto,baudrate=baudrate,timeout=0.3) as s:
        # Muchas placas ESP32 reinician al abrir el puerto serial.
        tiempo_inicio=time.monotonic()
        while time.monotonic()-tiempo_inicio < segundos+2.0:
            linea=s.readline().decode('utf-8',errors='replace').strip()
            trama = analizar_linea_esp32(linea)
            if trama is None: continue
            mac, rssi = trama
            if objetivo and mac!=objetivo: continue
            muestras[mac].append(rssi)
            lineas+=1
    return {'puerto':puerto,'tramas_validas':lineas,'origen':'ESP32_PROMISCUO_CANAL_PROPIO',
            'objetivo_filtrado':objetivo,'emisores':[
                {'mac':mac,'cantidad':len(v),'rssi_medio_dbm':round(statistics.mean(v),3),
                 'sigma_db':round(statistics.pstdev(v),3),'muestras_rssi_dbm':v}
                for mac,v in sorted(muestras.items(),key=lambda item:len(item[1]),reverse=True)]}


def extraer_iw(texto):
    """Parsea BSS y signal de 'iw dev INTERFAZ scan' sin inventar conversiones."""
    datos={}
    actual=None
    for linea in texto.splitlines():
        m=re.search(r'^BSS\s+([0-9a-f:]{17})\b',linea,re.I)
        if m:
            actual=normalizar_mac(m.group(1))
            datos.setdefault(actual,[])
        if actual is not None:
            m=re.search(r'\bsignal:\s*(-?\d+(?:\.\d+)?)\s*dBm',linea,re.I)
            if m: datos[actual].append(float(m.group(1)))
    return {mac:vals for mac,vals in datos.items() if vals}


def capturar_rssi_aps(interfaz,barridos=3,pausa=0.4):
    """Mide los RSSI de AP que escucha la laptop (localización DE LA LAPTOP)."""
    if platform.system()!='Linux':
        raise RuntimeError('Lecturas live iw solo disponibles en Linux; en Windows use --lecturas CSV con RSSI reales.')
    if not interfaz: raise ValueError('Debe proporcionar --interfaz.')
    muestras=defaultdict(list)
    for i in range(barridos):
        texto=comando(['iw','dev',interfaz,'scan'],timeout=30)
        for mac, valores in extraer_iw(texto).items():
            muestras[mac].extend(valores)
        if i<barridos-1: time.sleep(pausa)
    return {mac:{'rssi_dbm':statistics.mean(v),
                 'sigma_db':statistics.pstdev(v) if len(v)>1 else None,
                 'muestras':len(v)} for mac,v in muestras.items()}


def cargar_lecturas_csv(ruta):
    """CSV proveniente de mediciones previas: columnas bssid,rssi_dbm[,sigma_db]."""
    resultado={}
    with Path(ruta).open(encoding='utf-8-sig',newline='') as f:
        for fila in csv.DictReader(f):
            bssid=normalizar_mac(fila['bssid'])
            rssi=float(fila['rssi_dbm'])
            sigma=float(fila['sigma_db']) if (fila.get('sigma_db') or '').strip() else None
            if not math.isfinite(rssi) or rssi > 0 or (sigma is not None and (not math.isfinite(sigma) or sigma < 0)):
                raise ValueError(f'RSSI o sigma fuera del dominio para {bssid}')
            if bssid in resultado:
                raise ValueError(f'BSSID repetida en CSV: {bssid}')
            resultado[bssid]={'rssi_dbm':rssi,'sigma_db':sigma,'muestras':1}
    if not resultado: raise ValueError('CSV sin lecturas.')
    return resultado


def ecuacion_calibracion(rssi,d_ref,pt,pl,alpha,d0=1.0):
    if min(d_ref,d0)<=0 or math.isclose(d_ref,d0):
        raise ValueError('Las distancias de calibración d_ref y d0 deben ser distintas y positivas.')
    return lambda n: 10*n*math.log10(d_ref/d0)+alpha*d_ref+rssi-pt+pl


def ecuacion_distancia(rssi,pt,pl,n,alpha,d0=1.0):
    if n<=0 or d0<=0 or alpha<0: raise ValueError('n,d0 positivos y alpha no negativo.')
    def f(d):
        if d<=0: raise ValueError('Distancia no positiva.')
        return 10*n*math.log10(d/d0)+alpha*d+rssi-pt+pl
    def df(d):
        if d<=0: raise ValueError('Distancia no positiva.')
        return 10*n/(d*math.log(10))+alpha
    return f,df


def trilateracion(nodos,radios):
    """Sistema 2D A X=B; normales M=AᵀA, C=AᵀB para N>=3 anclas."""
    p=np.asarray(nodos,float);d=np.asarray(radios,float)
    if p.ndim!=2 or p.shape[1]!=2 or len(p)<3 or len(d)!=len(p) or np.any(d<=0):
        raise ValueError('Anclas y radios inconsistentes.')
    piv=p[-1]
    A=2*(p[:-1]-piv)
    B=np.sum(p[:-1]**2,axis=1)-np.sum(piv**2)-(d[:-1]**2-d[-1]**2)
    if np.linalg.matrix_rank(A)<2: raise ValueError('Geometría degenerada (anclas colineales).')
    return A,B,A.T@A,A.T@B
