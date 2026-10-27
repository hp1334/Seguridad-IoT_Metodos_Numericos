"""Sistema de monitoreo IoT y métodos numéricos. Windows / Fedora.

Ejecute desde la raíz: python main.py

Inventaría redes, adquiere RSSI del ESP32, procesa movimiento de webcam y
muestra mediciones en un panel HTML local. La trilateración usa anclas reales y
parámetros aportados por quien realiza cada ensayo. Los controles matemáticos
se mantienen en tests/, no se inyectan en las mediciones.
"""
from __future__ import annotations
import argparse
import csv
import html
import json
import math
import platform
import time
import os
import webbrowser
import threading
from collections import deque
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit
import functools
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from scipy.optimize import root_scalar
from scipy.linalg import solve
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from src.metodos_numericos.raices import (epsilon_maquina,error_absoluto,biseccion,secante,newton_raphson)
from src.metodos_numericos.sistemas_lineales import (numero_condicion_infinito,diagonal_dominante,
    gauss_seidel,jacobi,eliminacion_gaussiana)
from src.telematica.posicionamiento import (descubrir_interfaz,inventario_wifi,vecinos_lan,
    capturar_esp32,puerto_esp32_automatico,normalizar_mac,capturar_rssi_aps,
    cargar_lecturas_csv,ecuacion_calibracion,ecuacion_distancia,trilateracion)


def opciones(argv=None):
    p=argparse.ArgumentParser(description='Sistema IoT + métodos numéricos, Windows/Fedora')
    p.add_argument('--salida',default='resultados',help='Carpeta de evidencias')
    p.add_argument('--datos-referencia', help='Archivo opcional JSON de un ensayo anterior; no se carga por defecto')
    p.add_argument('--interfaz',help='Interfaz Wi-Fi para iw en Linux (autodetectada si es posible)')
    p.add_argument('--puerto-esp32',help='Puerto serie, ej.: /dev/ttyUSB0 o COM4 (autodetección conservadora)')
    p.add_argument('--mac-objetivo',help='Filtrar MAC transmisora de tramas del ESP32')
    p.add_argument('--segundos-esp32',type=float,default=5.0)
    p.add_argument('--baudrate',type=int,default=115200)
    p.add_argument('--indice-camara',type=int,default=0)
    p.add_argument('--frames',type=int,default=180)
    p.add_argument('--segundos-camara',type=float,default=20.0)
    p.add_argument('--umbral-camara',type=int,default=18)
    p.add_argument('--area-camara',type=float,default=400.0)
    p.add_argument('--sin-ventana',action='store_true')
    p.add_argument('--sin-camara',action='store_true')
    p.add_argument('--camara-temporal',action='store_true',help='Comportamiento anterior: captura limitada y panel HTML estático')
    p.add_argument('--grabar-video',action='store_true',help='Guardar video completo: puede ocupar mucho disco')
    p.add_argument('--duracion-vivo',type=float,help='Cierra el panel tras N segundos (para pruebas); por defecto funciona hasta Ctrl+C')
    p.add_argument('--puerto-panel',type=int,default=0,help='Puerto HTTP local; 0 = elegir uno libre')
    p.add_argument('--fps-panel',type=float,default=10.,help='Fotogramas por segundo máximos en el navegador')
    p.add_argument('--sin-abrir-panel',action='store_true',help='Guarda HTML pero no abre el navegador')
    p.add_argument('--sin-esp32',action='store_true')
    p.add_argument('--ancla',action='append',default=[],metavar='ID,BSSID,X,Y',
                   help='Repita para >=3 puntos de acceso reales conocidos; localiza el receptor Wi-Fi')
    p.add_argument('--lecturas',help='CSV con bssid,rssi_dbm[,sigma_db], para capturas previas reales (Windows/Linux)')
    p.add_argument('--barridos',type=int,default=3)
    p.add_argument('--pausa-barridos',type=float,default=0.4)
    p.add_argument('--ancla-calibracion',help='ID del AP a distancia medida d-ref')
    p.add_argument('--d-ref',type=float,help='Distancia real del receptor a la ancla de calibración, m')
    p.add_argument('--pt-dbm',type=float,help='Potencia del transmisor AP, NO potencia de la laptop')
    p.add_argument('--pl-d0',type=float,help='Pérdida de referencia del enlace documentada, dB')
    p.add_argument('--alpha',type=float,help='Atenuación específica modelo dB/m')
    p.add_argument('--d0',type=float,default=1.0)
    p.add_argument('--bis-a',type=float,help='Distancia mínima de búsqueda, m')
    p.add_argument('--bis-b',type=float,help='Distancia máxima de búsqueda, m')
    p.add_argument('--tol',type=float,default=1e-6)
    p.add_argument('--max-iter',type=int,default=100)
    p.add_argument('--sec-x0',type=float,help='Semilla Secante para distancias, m')
    p.add_argument('--sec-x1',type=float,help='Segunda semilla Secante para distancias, m')
    p.add_argument('--newton-x0',type=float,help='Semilla Newton para distancias, m')
    p.add_argument('--n0-secante',type=float,default=1.5,help='Primera semilla de calibración n')
    p.add_argument('--n1-secante',type=float,default=4.0,help='Segunda semilla de calibración n')
    args=p.parse_args(argv)
    if args.tol<=0 or args.max_iter<1 or args.frames<1 or args.segundos_camara<=0:
        p.error('Parámetros de convergencia o cámara inválidos.')
    if args.puerto_panel < 0 or args.puerto_panel > 65535 or args.fps_panel<=0 or (args.duracion_vivo is not None and args.duracion_vivo<=0):
        p.error('Opciones del panel/cámara continua inválidas.')
    if args.segundos_esp32<=0 or args.barridos<1 or args.pausa_barridos<0 or args.area_camara<=0 or args.baudrate<=0 or not (0 < args.umbral_camara < 255):
        p.error('Parámetros de adquisición inválidos.')
    return args


def guardar_json(ruta,datos):
    def transform(v):
        if isinstance(v,np.ndarray): return v.tolist()
        if isinstance(v,(np.floating,np.integer)): return v.item()
        raise TypeError(f'Tipo no serializable: {type(v)}')
    Path(ruta).write_text(json.dumps(datos,ensure_ascii=False,indent=2,default=transform,allow_nan=False),encoding='utf-8')


def graficar(historiales,ruta,titulo):
    fig,ax=plt.subplots(figsize=(8,5))
    for etiqueta,hist in historiales.items():
        if hist:
            xs=[t['iteracion'] for t in hist]
            ys=[max(float(t['error_abs']),np.finfo(float).eps) for t in hist]
            ax.semilogy(xs,ys,'o-',label=etiqueta)
    ax.set(xlabel='Iteración k',ylabel='Error absoluto aproximado',title=titulo)
    ax.grid(alpha=0.3,which='both'); ax.legend();fig.tight_layout()
    fig.savefig(ruta,dpi=220);plt.close(fig)


def escribir_tabla(historiales, ruta, referencias=None):
    """Exporta errores absolutos/relativos, aproximados y frente a referencia numérica."""
    referencias = referencias or {}
    with Path(ruta).open('w', newline='', encoding='utf-8') as fp:
        escritor = csv.writer(fp)
        escritor.writerow(['metodo','iteracion','aproximacion','residuo_f',
                          'error_abs_aprox','error_rel_aprox_pct',
                          'error_verdadero_vs_referencia','error_rel_verdadero_pct'])
        for metodo, hist in historiales.items():
            ref = referencias.get(metodo)
            norma = float(np.linalg.norm(np.asarray(ref, dtype=float))) if ref is not None else None
            for h in hist:
                error_verd = h.get('error_verdadero')
                pct = 100*error_verd/norma if error_verd is not None and norma is not None and norma > 1e-15 else None
                escritor.writerow([metodo,h['iteracion'],h['aproximacion'],h.get('f_x',''),
                                   h['error_abs'],h['error_rel_pct'],error_verd,pct])


class CamaraEnVivo:
    """Un solo lector físico de OpenCV, compartido con los clientes HTTP locales.

    Detecta movimiento y estima su centroide en píxeles. El procesamiento se
    mantiene activo hasta Ctrl+C y no precisa grabar vídeo en disco.
    """
    def __init__(self,args,salida):
        import cv2
        self.cv2=cv2
        self.args=args
        self.salida=Path(salida)
        self.stop_event=threading.Event()
        self.lock=threading.Lock()
        self.jpeg=None
        self.seq=0
        self.error=None
        self.hilo=None
        self.escritor=None
        self.archivo_video=None
        self.historial=deque(maxlen=100)
        self.metricas={'estado':'iniciando','frames':0,'movimiento':False,'area_movimiento_px2':0.,
                       'centroide_pixeles':None, 'grabacion_habilitada':bool(args.grabar_video)}
        if platform.system()=='Linux':
            self.cap=cv2.VideoCapture(args.indice_camara,cv2.CAP_V4L2)
        elif platform.system()=='Windows':
            self.cap=cv2.VideoCapture(args.indice_camara,cv2.CAP_DSHOW)
        else:
            self.cap=cv2.VideoCapture(args.indice_camara)
        if not self.cap.isOpened():
            self.cap.release()
            self.cap=cv2.VideoCapture(args.indice_camara,cv2.CAP_ANY)
        if not self.cap.isOpened():
            self.cap.release()
            raise RuntimeError(f'No se pudo abrir la cámara física índice {args.indice_camara}.')
        self.fondo=cv2.createBackgroundSubtractorMOG2(history=350,varThreshold=30,detectShadows=True)

    def iniciar(self):
        self.hilo=threading.Thread(target=self._capturar,daemon=True,name='webcam-fisica')
        self.hilo.start()

    def detener(self):
        self.stop_event.set()
        if self.hilo:
            self.hilo.join(timeout=5)
        # Los recursos se cierran en la hebra de captura: nunca en la hebra HTTP.

    def estado(self):
        with self.lock:
            return {**self.metricas,'historial_movimiento':list(self.historial),
                    'error':self.error,'archivo_video':self.archivo_video}

    def fotograma(self):
        with self.lock:
            return self.seq,self.jpeg

    def _video(self,ancho,alto, fps):
        cv2=self.cv2
        ruta=self.salida/'camara_continua.mp4'
        grabador=cv2.VideoWriter(str(ruta),cv2.VideoWriter_fourcc(*'mp4v'),fps,(ancho,alto))
        if not grabador.isOpened():
            grabador.release()
            ruta=self.salida/'camara_continua.avi'
            grabador=cv2.VideoWriter(str(ruta),cv2.VideoWriter_fourcc(*'MJPG'),fps,(ancho,alto))
        if not grabador.isOpened():
            grabador.release()
            raise RuntimeError('No se pudo inicializar el códec MP4V ni MJPG para guardar video.')
        self.archivo_video=str(ruta)
        return grabador

    def _capturar(self):
        cv2=self.cv2
        cap=self.cap
        conteo=0
        ultima_evidencia=0.
        ultimo_csv=0.
        inicio=time.monotonic()
        ruta_csv=self.salida/'camara_mediciones.csv'
        try:
            fps=cap.get(cv2.CAP_PROP_FPS)
            if not (1<=fps<=120): fps=20.
            with ruta_csv.open('w',newline='',encoding='utf-8') as fp:
                w=csv.writer(fp)
                w.writerow(['fecha_utc','fotograma','movimiento','area_px2','centroide_x_px','centroide_y_px'])
                while not self.stop_event.is_set():
                    ok, frame=cap.read()
                    if not ok or frame is None:
                        raise RuntimeError('La webcam dejó de entregar fotogramas.')
                    conteo+=1
                    alto,ancho=frame.shape[:2]
                    if self.args.grabar_video and self.escritor is None:
                        self.escritor=self._video(ancho,alto,fps)
                    vista=frame.copy()
                    mascarilla=self.fondo.apply(frame)
                    if conteo<=25:
                        contornos=[]
                    else:
                        mascarilla=cv2.threshold(mascarilla,200,255,cv2.THRESH_BINARY)[1]
                        kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
                        mascarilla=cv2.morphologyEx(mascarilla,cv2.MORPH_OPEN,kernel,iterations=1)
                        contornos,_=cv2.findContours(mascarilla,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                    validos=[c for c in contornos if cv2.contourArea(c)>=self.args.area_camara]
                    area=0.
                    centro=None
                    if validos:
                        mayor=max(validos,key=cv2.contourArea)
                        area=float(cv2.contourArea(mayor))
                        x,y,wid,hei=cv2.boundingRect(mayor)
                        centro=[int(x+wid/2),int(y+hei/2)]
                        cv2.rectangle(vista,(x,y),(x+wid,y+hei),(0,255,0),2)
                        cv2.circle(vista,tuple(centro),5,(0,0,255),-1)
                    cv2.putText(vista,'MOVIMIENTO' if validos else 'Sin movimiento',
                                (12,28),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,255,0) if validos else (230,230,230),2)
                    if self.escritor:
                        self.escritor.write(vista)
                    # La captura mantiene la misma resolución; el navegador recibe JPEG comprimido.
                    ok_jpg,jpg=cv2.imencode('.jpg',vista,[cv2.IMWRITE_JPEG_QUALITY,76])
                    ahora=time.monotonic()
                    if ahora-ultima_evidencia>=10 or conteo==1:
                        cv2.imwrite(str(self.salida/'captura_camara_real.png'),vista)
                        ultima_evidencia=ahora
                    if ahora-ultimo_csv>=1:
                        w.writerow([datetime.now(timezone.utc).isoformat(),conteo,bool(validos),round(area,2),
                                    centro[0] if centro else '',centro[1] if centro else ''])
                        fp.flush()
                        ultimo_csv=ahora
                    with self.lock:
                        self.seq+=1
                        if ok_jpg: self.jpeg=jpg.tobytes()
                        self.historial.append(round(area,2))
                        self.metricas.update({'estado':'en_vivo','frames':conteo,'movimiento':bool(validos),
                            'area_movimiento_px2':round(area,2),'centroide_pixeles':centro,
                            'segundos_activa':round(ahora-inicio,1),
                            'resolucion':[ancho,alto]})
        except Exception as exc:
            with self.lock:
                self.error=str(exc)
                self.metricas['estado']='error'
        finally:
            cap.release()
            if self.escritor:
                self.escritor.release()
            with self.lock:
                if self.metricas['estado']!='error':
                    self.metricas['estado']='detenida'
            self.stop_event.set()


def crear_servidor_panel(salida,camara,puerto=0):
    """Solo localhost: evita publicar en la red doméstica imágenes y MACs privadas."""
    carpeta=Path(salida).resolve()
    class Controlador(SimpleHTTPRequestHandler):
        def __init__(self,*args,**kwargs):
            super().__init__(*args,directory=str(carpeta),**kwargs)
        def log_message(self,fmt,*args):
            # No saturar la terminal por lecturas de streaming y JSON.
            return
        def do_GET(self):
            ruta=urlsplit(self.path).path
            if ruta=='/api/estado':
                cuerpo=json.dumps(camara.estado(),ensure_ascii=False,allow_nan=False).encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type','application/json; charset=utf-8')
                self.send_header('Cache-Control','no-store')
                self.send_header('Content-Length',str(len(cuerpo)))
                self.end_headers();self.wfile.write(cuerpo)
                return
            if ruta=='/video':
                self.send_response(200)
                self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control','no-store, no-cache, must-revalidate')
                self.end_headers()
                anterior=-1
                intervalo=1/camara.args.fps_panel
                try:
                    while not camara.stop_event.is_set():
                        seq,datos=camara.fotograma()
                        if datos is not None and seq!=anterior:
                            self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(datos)).encode()+b'\r\n\r\n')
                            self.wfile.write(datos+b'\r\n');self.wfile.flush()
                            anterior=seq
                        time.sleep(intervalo)
                except (BrokenPipeError,ConnectionResetError,ConnectionAbortedError,OSError):
                    pass
                return
            if ruta=='/':
                self.path='/panel_resultados.html'
            return super().do_GET()
    servidor=ThreadingHTTPServer(('127.0.0.1',puerto),Controlador)
    servidor.daemon_threads=True
    return servidor

def camera(args,salida):
    """Captura webcam física, movimiento en píxeles; NO infiere metros sin calibración."""
    import cv2
    import os
    indice=args.indice_camara
    if platform.system()=='Linux':
        cap=cv2.VideoCapture(indice,cv2.CAP_V4L2)
    elif platform.system()=='Windows':
        cap=cv2.VideoCapture(indice,cv2.CAP_DSHOW)
    else:
        cap=cv2.VideoCapture(indice)
    if not cap.isOpened():
        cap.release()
        cap=cv2.VideoCapture(indice,cv2.CAP_ANY)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError(f'No hay acceso a la webcam índice {indice}.')
    mostrar=(not args.sin_ventana) and bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY') or platform.system()=='Windows')
    base=None;mejor=None;mejor_area=0.0;centro=None;contador=0
    inicio=time.monotonic()
    try:
        for _ in range(8): cap.read()
        ok,base=cap.read()
        if not ok or base is None: raise RuntimeError('Webcam abierta sin fotograma inicial.')
        fondo=cv2.GaussianBlur(cv2.cvtColor(base,cv2.COLOR_BGR2GRAY),(21,21),0)
        mejor=base.copy()
        while contador<args.frames and time.monotonic()-inicio<args.segundos_camara:
            ok,frame=cap.read()
            if not ok or frame is None: continue
            contador+=1
            gris=cv2.GaussianBlur(cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY),(21,21),0)
            mascara=cv2.threshold(cv2.absdiff(fondo,gris),args.umbral_camara,255,cv2.THRESH_BINARY)[1]
            mascara=cv2.dilate(mascara,None,iterations=2)
            contornos,_=cv2.findContours(mascara,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            vista=frame.copy()
            if contornos:
                contorno=max(contornos,key=cv2.contourArea)
                area=float(cv2.contourArea(contorno))
                if area>=args.area_camara:
                    x,y,w,h=cv2.boundingRect(contorno)
                    c=[x+w//2,y+h//2]
                    cv2.rectangle(vista,(x,y),(x+w,y+h),(0,255,0),2)
                    cv2.circle(vista,tuple(c),5,(0,0,255),-1)
                    if area>mejor_area:
                        mejor_area=area;centro=c;mejor=vista.copy()
            if mostrar:
                cv2.imshow('IoT - Camara real | q = cerrar',vista)
                if (cv2.waitKey(35)&255)==ord('q'): break
        if contador==0: raise RuntimeError('No se obtuvieron fotogramas.')
        cv2.putText(mejor,'MOVIMIENTO' if centro else 'SIN DETECCION',
                    (12,max(25,mejor.shape[0]-20)),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,255,255),2)
        ruta=salida/'captura_camara_real.png'
        if not cv2.imwrite(str(ruta),mejor): raise RuntimeError(f'No se pudo guardar {ruta}.')
        return {'estado':'movimiento_detectado' if centro else 'sin_movimiento_detectado',
                'frames':contador,'resolucion':[base.shape[1],base.shape[0]],
                'centroide_pixeles':centro,'area_px2':mejor_area,'archivo':str(ruta),
                'coordenadas_metros':None,
                'nota':'No hay calibración fotogramétrica; centroide no equivale a posición física.'}
    finally:
        cap.release()
        if mostrar:
            try: cv2.destroyAllWindows()
            except cv2.error: pass


def anclas_cli(lista):
    anclas=[]; ids=set(); macs=set()
    for raw in lista:
        p=[x.strip() for x in raw.split(',')]
        if len(p)!=4: raise ValueError(f'Use --ancla "ID,BSSID,X,Y", no {raw!r}')
        ident,mac=p[0],normalizar_mac(p[1])
        if not ident or ident in ids or mac in macs: raise ValueError('Identificadores o BSSID duplicados.')
        ids.add(ident); macs.add(mac)
        xy=[float(p[2]),float(p[3])]
        if not all(map(math.isfinite,xy)): raise ValueError('Coordenadas no finitas.')
        anclas.append({'id':ident,'bssid':mac,'xy':xy})
    return anclas


def localizar(args,salida,log,interfaz):
    anclas=anclas_cli(args.ancla)
    if len(anclas)<3: raise ValueError('La trilateración 2D requiere >=3 AP con coordenadas conocidas.')
    requeridos={'ancla-calibracion':args.ancla_calibracion,'d-ref':args.d_ref,
                'pt-dbm':args.pt_dbm,'pl-d0':args.pl_d0,'alpha':args.alpha,
                'bis-a':args.bis_a,'bis-b':args.bis_b}
    sin=[k for k,v in requeridos.items() if v is None]
    if sin: raise ValueError('Faltan parámetros físicos: '+', '.join(sin))
    if not (args.d_ref>0 and args.d0>0 and args.alpha>=0 and 0<args.bis_a<args.bis_b) or not all(math.isfinite(v) for v in (args.d_ref,args.d0,args.pt_dbm,args.pl_d0,args.alpha,args.bis_a,args.bis_b)):
        raise ValueError('Distancias y atenuación inválidas.')
    if args.lecturas:
        medidas=cargar_lecturas_csv(args.lecturas)
        origen=f'CSV de capturas previas: {args.lecturas}'
    else:
        medidas=capturar_rssi_aps(interfaz,args.barridos,args.pausa_barridos)
        origen=f'iw / {interfaz}'
    no=[a['bssid'] for a in anclas if a['bssid'] not in medidas]
    if no: raise ValueError('No hay RSSI dBm medido para los AP: '+', '.join(no))
    ref=next((a for a in anclas if a['id']==args.ancla_calibracion),None)
    if ref is None: raise ValueError('El ancla-calibracion no existe en --ancla.')
    fn=ecuacion_calibracion(medidas[ref['bssid']]['rssi_dbm'],args.d_ref,args.pt_dbm,
                           args.pl_d0,args.alpha,args.d0)
    n,hn=secante(fn,args.n0_secante,args.n1_secante,args.tol,args.max_iter)
    if not math.isfinite(n) or n<=0: raise ValueError(f'n={n} fuera del dominio del modelo: revise Pt, PL, RSSI y referencia.')
    dist=[]; registros={};incertidumbres=[]
    for a in anclas:
        med=medidas[a['bssid']]; f,df=ecuacion_distancia(med['rssi_dbm'],args.pt_dbm,args.pl_d0,n,args.alpha,args.d0)
        if f(args.bis_a)*f(args.bis_b)>0:
            raise ValueError(f'La distancia de {a["id"]} no está en el intervalo [{args.bis_a},{args.bis_b}].')
        # SciPy SOLO compara error verdadero: no se utiliza como motor numérico principal.
        referencia=root_scalar(f,bracket=[args.bis_a,args.bis_b],method='brentq').root
        db,hb=biseccion(f,args.bis_a,args.bis_b,args.tol,args.max_iter,referencia)
        x0=args.sec_x0 if args.sec_x0 is not None else max(args.bis_a,0.8*args.d_ref)
        x1=args.sec_x1 if args.sec_x1 is not None else min(args.bis_b,1.2*args.d_ref)
        if x1<=x0:
            x0=args.bis_a+0.25*(args.bis_b-args.bis_a)
            x1=args.bis_a+0.50*(args.bis_b-args.bis_a)
        xn=args.newton_x0 if args.newton_x0 is not None else args.d_ref
        try:
            ds,hs=secante(f,x0,x1,args.tol,args.max_iter,referencia)
        except (ValueError,ZeroDivisionError,RuntimeError) as exc:
            log(f'[AVISO] Secante en {a["id"]}: {exc}. Se conserva Bisección como solución.')
            ds,hs=db,[]
        try:
            dn,hnr=newton_raphson(f,df,xn,args.tol,args.max_iter,referencia)
        except (ValueError,ZeroDivisionError,RuntimeError) as exc:
            log(f'[AVISO] Newton en {a["id"]}: {exc}.')
            dn,hnr=None,[]
        delta=med['sigma_db']/abs(df(ds)) if med['sigma_db'] is not None else None
        dist.append(ds)
        incertidumbres.append(delta)
        registros[a['id']]={'biseccion':db,'secante':ds,'newton':dn,'referencia_scipy':referencia,
                            'historial_biseccion':hb,'historial_secante':hs,'historial_newton':hnr,
                            'incertidumbre_m':delta,'rssi_dbm':med['rssi_dbm']}
        log(f'{a["id"]}: RSSI={med["rssi_dbm"]:.2f} dBm; d_Bis={db:.4f} m; d_Sec={ds:.4f} m; '
            f'd_Newton={dn if dn is not None else "no convergió"}; Δd={delta if delta is not None else "sin sigma"}')
    A,B,M,C=trilateracion([a['xy'] for a in anclas],dist)
    posicion_ref=solve(M,C)
    semilla=np.mean([a['xy'] for a in anclas],axis=0)
    gs,hgs=gauss_seidel(M,C,semilla,args.tol,args.max_iter,posicion_ref)
    try:
        jj,hj=jacobi(M,C,semilla,args.tol,args.max_iter,posicion_ref)
    except RuntimeError as e:
        log(f'[AVISO] Jacobi no convergió: {e}')
        jj,hj=None,[]
    el=eliminacion_gaussiana(M,C)
    primero=registros[anclas[0]['id']]
    graficar({'Bisección':primero['historial_biseccion'], 'Secante':primero['historial_secante'],
             'Newton':primero['historial_newton']},salida/'grafica_1_convergencia_no_lineal.png','CONVERGENCIA RSSI medido: '+anclas[0]['id'])
    graficar({'Gauss-Seidel':hgs,'Jacobi':hj},salida/'grafica_2_convergencia_sistemas_lineales.png','CONVERGENCIA TRILATERACION (m)')
    fig,ax=plt.subplots(figsize=(7,6))
    for a,d in zip(anclas,dist):
        x,y=a['xy']
        ax.plot(x,y,'ks'); ax.add_patch(plt.Circle((x,y),d,fill=False,linestyle='--'))
        ax.annotate(a['id'],(x,y),xytext=(5,5),textcoords='offset points')
    ax.plot(gs[0],gs[1],'ro',label='Receptor Wi-Fi estimado')
    ax.set_aspect('equal',adjustable='datalim');ax.grid(alpha=.3);ax.legend()
    ax.set(xlabel='X (m)',ylabel='Y (m)',title='Trilateración: AP medidos y posiciones declaradas')
    fig.tight_layout();fig.savefig(salida/'grafica_3_mapa_trilateracion_iot.png',dpi=220);plt.close(fig)
    escribir_tabla({'biseccion_'+a['id']:registros[a['id']]['historial_biseccion'] for a in anclas} |
                   {'secante_'+a['id']:registros[a['id']]['historial_secante'] for a in anclas} |
                   {'newton_'+a['id']:registros[a['id']]['historial_newton'] for a in anclas} |
                   {'gauss_seidel':hgs,'jacobi':hj},salida/'tabla_iteraciones_localizacion.csv',
                   ({k+'_'+a['id']:registros[a['id']]['referencia_scipy']
                     for a in anclas for k in ('biseccion','secante','newton')}
                    | {'gauss_seidel':posicion_ref,'jacobi':posicion_ref}))
    log(f'[LOCALIZACION DEL RECEPTOR] X={gs[0]:.4f} m; Y={gs[1]:.4f} m; '
        f'κ∞={numero_condicion_infinito(M):.4f}; dominante={diagonal_dominante(M)}')
    log(f'[VERIFICACION SOLO NUMERICA] Error Gauss-Seidel vs SciPy = {error_absoluto(gs,posicion_ref):.6e} m')
    return {'origen':origen,'objeto_localizado':'receptor Wi-Fi que realizó la captura',
            'modelo':'Pérdida log-distancia + absorción; parámetros TX comunes asumidos',
            'anclas':anclas,'n':n,'distancias_m':dist,'incertidumbres_m':incertidumbres,
            'matriz_M':M,'vector_C':C,'condicion_infinito':numero_condicion_infinito(M),
            'diagonal_dominante':diagonal_dominante(M),'gauss_seidel':gs,
            'jacobi':jj,'eliminacion':el,'referencia_scipy':posicion_ref,
            'error_gs_vs_scipy_m':error_absoluto(gs,posicion_ref),'iteraciones':registros,
            'referencia_numerica':'SciPy (solución del mismo modelo matemático)'}



def cargar_resultados_historicos(args, salida, log):
    """Carga el registro histórico del anteproyecto sin incorporarlo a las mediciones en vivo.

    Los CSV y las figuras presentan los valores registrados en el anteproyecto
    de la prueba realizada en Windows. Para otro ensayo se actualiza el JSON.
    Las figuras se dibujan a partir de esos valores documentados.
    """
    if not args.datos_referencia:
        return None
    ruta = Path(args.datos_referencia)
    if not ruta.is_file():
        raise FileNotFoundError(f'Archivo de ensayo no encontrado: {ruta}')
    historico = json.loads(ruta.read_text(encoding='utf-8'))
    for clave in ('identificacion', 'unidad_1', 'unidad_2', 'unidad_3'):
        if clave not in historico:
            raise ValueError(f'Archivo histórico incompleto: falta {clave}')
    for unidad in ('unidad_2', 'unidad_3'):
        if not isinstance(historico[unidad].get('tabla'), list) or not historico[unidad]['tabla']:
            raise ValueError(f'Faltan filas de iteraciones en {unidad}')

    guardar_json(salida / 'historico_windows_transcrito.json', historico)
    for unidad, nombre in (('unidad_2', 'historico_windows_secante.csv'),
                           ('unidad_3', 'historico_windows_gauss_seidel.csv')):
        filas = historico[unidad]['tabla']
        with (salida / nombre).open('w', newline='', encoding='utf-8') as fp:
            writer = csv.DictWriter(fp, fieldnames=list(filas[0].keys()))
            writer.writeheader()
            writer.writerows(filas)

    def grafica_error(filas, archivo, etiqueta, titulo):
        fig, ax = plt.subplots(figsize=(8, 4.6))
        ks = [r['iteracion'] for r in filas if r.get('error_abs_aprox_m') is not None]
        ea = [r['error_abs_aprox_m'] if r['error_abs_aprox_m'] > 0 else float('nan') for r in filas if r.get('error_abs_aprox_m') is not None]
        ev = [r['error_abs_ref_m'] if r['error_abs_ref_m'] > 0 else float('nan') for r in filas if r.get('error_abs_aprox_m') is not None]
        ax.semilogy(ks, ea, 'o-', label='Error absoluto aproximado')
        ax.semilogy(ks, ev, 's--', label='Error frente a referencia, reportado')
        ax.set(xlabel='Iteración k', ylabel='Error (m)', title=titulo)
        ax.grid(alpha=0.3, which='both')
        ax.legend(fontsize=9)
        fig.text(0.5, 0.02, 'Fuente: registro suministrado por el equipo',
                 ha='center', fontsize=8, color='#596579')
        fig.tight_layout(rect=(0, 0.055, 1, 1))
        fig.savefig(salida / archivo, dpi=180)
        plt.close(fig)

    grafica_error(historico['unidad_2']['tabla'], 'historico_windows_grafica_secante.png',
                  'Secante', 'Secante · Convergencia del error (Windows)')
    grafica_error(historico['unidad_3']['tabla'], 'historico_windows_grafica_gauss_seidel.png',
                  'Gauss-Seidel', 'Gauss-Seidel · Convergencia del error (Windows)')

    u3 = historico['unidad_3']
    ancho, largo = u3['recinto_m']
    fig, ax = plt.subplots(figsize=(6.5, 6))
    ax.scatter([0, 0, ancho, ancho], [0, largo, 0, largo],
               c='#64748b', marker='^', s=85, label='Esquinas descritas (esquema)')
    w = u3['posicion_wifi_reportada_m']
    c = u3['posicion_visual_reportada_m']
    ax.scatter([w[0]], [w[1]], c='#2563eb', s=100, label='Posición Wi-Fi informada')
    ax.scatter([c[0]], [c[1]], c='#dc2626', marker='x', s=115, label='Posición visual informada')
    ax.plot([w[0], c[0]], [w[1], c[1]], '--', color='#64748b')
    ax.set(xlim=(-0.5, ancho+0.5), ylim=(-0.5, largo+0.5), aspect='equal',
           xlabel='x (m)', ylabel='y (m)', title='Esquema del escenario reportado (6 × 6 m)')
    ax.grid(alpha=.3); ax.legend(loc='upper right', fontsize=8)
    fig.text(0.5, .015, 'Fuente: registro suministrado por el equipo',
             ha='center', fontsize=8, color='#596579')
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    fig.savefig(salida / 'historico_windows_mapa_esquematico.png', dpi=180)
    plt.close(fig)
    log('[RESULTADOS WINDOWS] Tablas y figuras generadas con los valores del anteproyecto.')
    return historico

def crear_panel_resultados(salida, datos, en_vivo=False):
    """HTML con tablas y evidencia; en vivo se sirve mediante HTTP local."""
    def e(v):
        return html.escape('—' if v is None else str(v), quote=True)

    def tabla(columnas, filas):
        head = ''.join(f'<th>{e(c)}</th>' for c in columnas)
        if filas:
            rows = ''.join('<tr>' + ''.join(f'<td>{e(c)}</td>' for c in fila) + '</tr>' for fila in filas)
        else:
            rows = f'<tr><td colspan="{len(columnas)}">Sin datos disponibles</td></tr>'
        return f'<div class="scroll"><table><thead><tr>{head}</tr></thead><tbody>{rows}</tbody></table></div>'

    def tabla_csv(nombre):
        ruta = salida / nombre
        if not ruta.exists():
            return '<p class="aviso">Esta etapa no generó datos de iteraciones.</p>'
        with ruta.open(encoding='utf-8', newline='') as fp:
            reader = csv.DictReader(fp)
            cabeceras = reader.fieldnames or []
            filas = [[row.get(k) or '—' for k in cabeceras] for row in reader]
        etiquetas = {
            'iteracion':'Iteración (k)', 'distancia_m':'Aproximación dₖ (m)',
            'residuo_db':'Residuo |f(dₖ)| (dB)',
            'error_abs_aprox_m':'Error abs. aproximado (m)',
            'error_rel_aprox_pct':'Error relativo aproximado (%)',
            'error_abs_ref_m':'Error abs. frente a referencia (m)',
            'error_rel_ref_pct':'Error relativo frente a referencia (%)',
            'x_m':'Coordenada xₖ (m)', 'y_m':'Coordenada yₖ (m)',
            'metodo':'Método', 'aproximacion':'Aproximación',
            'residuo_f':'Residuo |f(x)|', 'error_abs_aprox':'Error abs. aproximado',
            'error_verdadero_vs_referencia':'Error abs. frente a referencia',
            'error_verdadero':'Error verdadero', 'error_rel_verdadero_pct':'Error relativo frente a referencia (%)'
        }
        cabeceras_legibles = [etiquetas.get(c, c) for c in cabeceras]
        return ('<input class="buscar" type="search" placeholder="Filtrar por método, iteración o valor" '
                'oninput="filtrar(this)">' + tabla(cabeceras_legibles, filas))

    def resumen_metodos(nombre):
        ruta = salida / nombre
        if not ruta.is_file():
            return ''
        resumen = {}
        with ruta.open(encoding='utf-8', newline='') as fp:
            for fila in csv.DictReader(fp):
                resumen[fila['metodo']] = fila
        if not resumen:
            return ''
        columnas = ['Método', 'Última iteración', 'Aproximación', 'Error abs. aproximado',
                    'Error rel. aprox. (%)', 'Error frente a referencia', 'Error rel. ref. (%)']
        filas = [[nombre, item.get('iteracion'), item.get('aproximacion'),
                  item.get('error_abs_aprox'), item.get('error_rel_aprox_pct'),
                  item.get('error_verdadero_vs_referencia'), item.get('error_rel_verdadero_pct')]
                 for nombre, item in resumen.items()]
        return '<h3>Comparación final de métodos</h3>' + tabla(columnas, filas)

    def figura(nombre, leyenda):
        if not (salida / nombre).is_file():
            return ''
        return f'<figure><a href="{e(nombre)}"><img src="{e(nombre)}" alt="{e(leyenda)}" loading="lazy"></a><figcaption>{e(leyenda)}</figcaption></figure>'

    def bloque(nombre, titulo, contenido):
        return f'<section id="{e(nombre)}"><h2>{e(titulo)}</h2>{contenido}</section>'

    etapas = datos.get('etapas', {})
    wifi = etapas.get('wifi', {}).get('redes', [])
    lan = etapas.get('lan', {}).get('vecinos', [])
    esp = etapas.get('esp32', {}).get('captura', {})
    emisores = esp.get('emisores', [])
    cam = etapas.get('camara', {}).get('captura', {})
    loc = etapas.get('localizacion', {}).get('resultados')
    historico = datos.get('referencia_windows')
    estado_loc = etapas.get('localizacion', {}).get('estado', 'no ejecutado')
    estado_cam = cam.get('estado', etapas.get('camara', {}).get('estado', 'no ejecutada'))

    metricas = [('Redes Wi-Fi', len(wifi)), ('Vecinos LAN', len(lan)),
                ('Tramas ESP32', esp.get('tramas_validas', '—')),
                ('Cámara', estado_cam), ('Localización', estado_loc)]
    resumen = '<div class="metricas">' + ''.join(
        f'<div class="metrica"><span>{e(nombre)}</span><b>{e(valor)}</b></div>'
        for nombre, valor in metricas) + '</div>'
    resumen += f'<p>Fecha UTC: {e(datos.get("fecha_utc"))} · Sistema: {e(datos.get("sistema"))}</p>'

    resumen += f'<p>Épsilon de máquina (IEEE 754 doble precisión): {e(epsilon_maquina())}</p>'

    if loc:
        local = '<p>Objeto localizado: <b>receptor Wi-Fi de la captura</b>. No se infiere la posición de una persona.</p>'
        local += f'<p><b>Gauss-Seidel (m):</b> {e(loc.get("gauss_seidel"))} · <b>n calibrado:</b> {e(loc.get("n"))} · <b>κ∞:</b> {e(loc.get("condicion_infinito"))}</p>'
        registros = loc.get('iteraciones', {})
        filas = []
        for ancla in loc.get('anclas', []):
            item = registros.get(ancla.get('id'), {})
            filas.append([ancla.get('id'), ancla.get('bssid'), ancla.get('xy'), item.get('rssi_dbm'),
                          item.get('biseccion'), item.get('secante'), item.get('newton'), item.get('incertidumbre_m')])
        local += tabla(['Ancla', 'BSSID', 'X,Y (m)', 'RSSI (dBm)', 'Bisección (m)', 'Secante (m)', 'Newton (m)', '±Δd (m)'], filas)
        local += figura('grafica_1_convergencia_no_lineal.png', 'Convergencia distancias — RSSI')
        local += figura('grafica_2_convergencia_sistemas_lineales.png', 'Convergencia Gauss-Seidel y Jacobi')
        local += figura('grafica_3_mapa_trilateracion_iot.png', 'Mapa geométrico de trilateración')
        local += resumen_metodos('tabla_iteraciones_localizacion.csv')
        local += '<h3>Tabla 2. Iteraciones y errores de localización</h3>' + tabla_csv('tabla_iteraciones_localizacion.csv')
    else:
        local = '<p>Para calcular nuevas coordenadas, introduce las posiciones de las anclas y los parámetros de medición del escenario.</p>'


    if historico:
        ident = historico.get('identificacion', {})
        u1, u2, u3 = historico['unidad_1'], historico['unidad_2'], historico['unidad_3']
        hist_block = ('<p><b>Resultados de la prueba realizada en Windows.</b> '
                      'Fuente: anteproyecto del Grupo 1, secciones 7.1 a 7.3.</p>')
        hist_block += f'<p><b>Procedencia:</b> {e(ident.get("archivo_fuente"))} · {e(ident.get("secciones_fuente"))}. Fecha de prueba: {e(ident.get("fecha_prueba_original"))}.</p>'
        hist_block += '<h3>Unidad 1. Resultados de control matemático</h3>'
        hist_block += tabla(['Magnitud', 'Valor del documento'], [
            ['Épsilon de máquina', u1.get('epsilon_maquina_reportado')],
            ['Raíz de control (Secante)', u1.get('control_secante_raiz')],
            ['Iteraciones control Secante', u1.get('control_secante_iteraciones')],
            ['Solución control Gauss-Seidel', u1.get('control_gauss_seidel_solucion')],
            ['Iteraciones control Gauss-Seidel', u1.get('control_gauss_seidel_iteraciones')]])
        hist_block += '<h3>Unidad 2. Método de la Secante — Ancla 1</h3>'
        hist_block += f'<p>RSSI de referencia: {e(u2.get("rssi_referencia_dbm"))} dBm; distancia de referencia: {e(u2.get("distancia_referencia_m"))} m; n reportado: {e(u2.get("n_reportado"))}.</p>'
        hist_block += tabla_csv('historico_windows_secante.csv')
        hist_block += figura('historico_windows_grafica_secante.png', 'Convergencia de Secante · Datos del anteproyecto')
        hist_block += '<p>Comparación de iteraciones reportadas: ' + e(u2.get('iteraciones_metodos')) + '</p>'
        hist_block += f'<p>σ RSSI: {e(u2.get("sigma_rssi_db"))} dB; incertidumbre propagada reportada: ±{e(u2.get("incertidumbre_reportada_m"))} m ({e(u2.get("incertidumbre_rel_reportada_pct"))}%).</p>'
        hist_block += '<h3>Unidad 3. Método de Gauss-Seidel</h3>'
        hist_block += tabla_csv('historico_windows_gauss_seidel.csv')
        hist_block += figura('historico_windows_grafica_gauss_seidel.png', 'Convergencia de Gauss-Seidel · Datos del anteproyecto')
        hist_block += figura('historico_windows_mapa_esquematico.png', 'Mapa de coordenadas · Datos del anteproyecto')
        hist_block += tabla(['Magnitud reportada', 'Valor'], [
            ['Posición radioeléctrica (m)', u3.get('posicion_wifi_reportada_m')],
            ['Posición óptica (m)', u3.get('posicion_visual_reportada_m')],
            ['Distancia entre las posiciones (m)', u3.get('discrepancia_reportada_m')],
            ['Error experimental declarado (%)', u3.get('error_rel_experimental_reportado_pct')],
            ['κ∞ indicado en el PDF', u3.get('condicion_infinito_reportada')],
            ['κ∞ calculado para cuatro esquinas de 6×6 con M=AᵀA', u3.get('condicion_infinito_cuadrado_cuatro_esquinas')]])
        hist_block += '<p><a href="historico_windows_transcrito.json" download>Descargar datos de la prueba Windows (JSON)</a></p>'
    else:
        hist_block = ''

    redes = tabla(['BSSID', 'SSID', 'Frecuencia (MHz)', 'Señal (%)'],
                  [[r.get('bssid'), r.get('ssid'), r.get('frecuencia_mhz'), r.get('senal_porcentaje')] for r in wifi])
    redes += '<h3>Vecinos ARP / caché LAN</h3>' + tabla(['IP', 'MAC', 'Estado'],
                      [[r.get('ip'), r.get('mac'), r.get('estado')] for r in lan])
    radio = tabla(['MAC transmisora', 'Tramas', 'RSSI medio (dBm)', 'Desviación σ (dB)'],
                  [[r.get('mac'), r.get('cantidad'), r.get('rssi_medio_dbm'), r.get('sigma_db')] for r in emisores])
    radio += figura('grafica_esp32_rssi.png', 'Variación temporal del RSSI recibido por ESP32')
    radio += '<p>ESP32: registros de potencia recibida (RSSI) por emisor. La verificación óptica se muestra en la sección Cámara.</p>'
    camara = f'<p>Detección óptica: {e(estado_cam)} · Fotogramas: {e(cam.get("frames"))} · Centroide (píxeles): {e(cam.get("centroide_pixeles"))}</p>'
    if en_vivo:
        camara += ('''<div class="directo"><img src="/video" alt="Transmisión de la webcam local" class="video">
        <div><h3>Monitoreo óptico continuo</h3>
        <p><b>Estado:</b> <span id="v-estado">Iniciando cámara…</span></p>
        <p><b>Fotogramas:</b> <span id="v-frames">0</span></p>
        <p><b>Movimiento:</b> <span id="v-mov">Sin datos</span></p>
        <p><b>Área de movimiento:</b> <span id="v-area">—</span> px²</p>
        <p><b>Centroide:</b> <span id="v-centro">—</span> píxeles</p>
        <p><b>Grabación en disco:</b> <span id="v-graba">—</span></p>
        </div></div>
        <h3>Área de movimiento (últimos fotogramas)</h3>
        <canvas id="v-grafica" width="780" height="190" style="width:100%;max-width:780px;height:190px;border:1px solid #dce4ec;border-radius:8px"></canvas>
        <p><a href="camara_mediciones.csv" download>Descargar mediciones de cámara (CSV)</a></p>''')

    camara += figura('captura_camara_real.png', 'Fotograma real obtenido con OpenCV')
    archivos = '<ul>' + ''.join(f'<li><a href="{e(p.name)}">{e(p.name)}</a></li>'
                                 for p in sorted(salida.iterdir()) if p.is_file() and p.name != 'panel_resultados.html') + '</ul>'

    elementos = [('resumen', 'Resumen', resumen),
                 ('localizacion', 'Trilateración y resultados', local), ('redes', 'Monitoreo Wi-Fi', redes),
                 ('esp32', 'ESP32 y RSSI', radio), ('camara', 'Verificación con cámara', camara),
                 ('archivos', 'Archivos de evidencia', archivos)]
    if historico:
        elementos.insert(1, ('historico', 'Ensayo anterior importado', hist_block))
    navegacion = ''.join(f'<a href="#{ident}">{e(titulo)}</a>' for ident,titulo,_ in elementos)
    cuerpo = ''.join(bloque(*elem) for elem in elementos)
    css = """<style>
.directo{display:grid;grid-template-columns:minmax(250px,1.6fr) minmax(250px,1fr);gap:1rem;align-items:start}.directo .video{width:100%;background:#152438;border-radius:8px;min-height:230px;object-fit:contain}.directo p{margin:.55rem 0}@media(max-width:720px){.directo{grid-template-columns:1fr}}
:root{font-family:system-ui,-apple-system,"Segoe UI",sans-serif;color:#202d3f;background:#f4f7fa;color-scheme:light}
*{box-sizing:border-box}body{margin:0}header{background:#153154;color:#fff;padding:1.6rem max(1rem,calc((100vw - 1200px)/2))}h1{margin:0;font-size:1.7rem}nav{display:flex;flex-wrap:wrap;gap:.65rem;margin-top:1.2rem}nav a{border:1px solid #879db5;padding:.45rem .75rem;border-radius:7px;color:white;text-decoration:none;font-size:.88rem}main{max-width:1200px;margin:1.5rem auto;padding:0 1rem}section{background:white;border:1px solid #dce4ec;border-radius:12px;padding:1.4rem;margin:1.2rem 0;box-shadow:0 3px 10px #213b5308}h2{font-size:1.35rem;margin-top:0}h3{font-size:1.05rem}.metricas{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:.75rem}.metrica{border:1px solid #dce4ec;padding:.85rem;border-radius:9px}.metrica span{display:block;font-size:.82rem;color:#596679}.metrica b{display:block;margin-top:.35rem;overflow-wrap:anywhere}.aviso{background:#f0f4f8;border-left:4px solid #617d9c;padding:1rem;color:#384b61}.scroll{overflow-x:auto;border:1px solid #d6e0e9;border-radius:7px;margin:1rem 0}table{width:100%;min-width:660px;border-collapse:collapse;font-size:.85rem}th,td{padding:.6rem .7rem;text-align:left;vertical-align:top;border-bottom:1px solid #e3eaf0}th{background:#e8eef6;white-space:nowrap}tbody tr:nth-child(even){background:#f8fafc}figure{display:inline-block;vertical-align:top;width:min(100%,520px);margin:.5rem 1rem 1rem 0}figure img{width:100%;max-height:390px;object-fit:contain;border:1px solid #dae2e9;border-radius:6px}figcaption{font-size:.82rem;color:#5a697e;margin-top:.25rem}.buscar{padding:.6rem;width:min(100%,480px);border:1px solid #adb9c7;border-radius:7px}footer{text-align:center;color:#56667b;padding:1rem}a{color:#23619b}@media print{nav,.buscar{display:none}body{background:white}section{break-inside:avoid}}
</style>"""
    script = """<script>
function filtrar(input){let table=input.nextElementSibling.querySelector('table');let query=input.value.toLowerCase();for(let row of table.tBodies[0].rows){row.hidden=!row.textContent.toLowerCase().includes(query)}}
</script>"""
    if en_vivo:
        script += """<script>
async function actualizarCamara(){
  try {
    const respuesta = await fetch('/api/estado',{cache:'no-store'});
    if (!respuesta.ok) throw new Error('Respuesta HTTP '+respuesta.status);
    const d = await respuesta.json();
    const poner = (id,v) => {const x=document.getElementById(id);if(x)x.textContent=String(v ?? '—')};
    poner('v-estado',d.estado);poner('v-frames',d.frames);
    poner('v-mov',d.movimiento?'Detectado':'Sin movimiento visible');
    poner('v-area',d.area_movimiento_px2);
    poner('v-centro',d.centroide_pixeles ? d.centroide_pixeles.join(', ') : '—');
    poner('v-graba',d.grabacion_habilitada?'Sí (activada por parámetro)':'No; solo fotogramas de evidencia');
    const c=document.getElementById('v-grafica');if(c){
      const ctx=c.getContext('2d'), arr=d.historial_movimiento||[];
      const W=c.width,H=c.height;
      ctx.clearRect(0,0,W,H);ctx.fillStyle='#f8fafc';ctx.fillRect(0,0,W,H);
      ctx.strokeStyle='#d5dde6';ctx.beginPath();ctx.moveTo(26,10);ctx.lineTo(26,H-22);ctx.lineTo(W-10,H-22);ctx.stroke();
      if(arr.length>1){let max=Math.max(100,...arr);ctx.strokeStyle='#2476a5';ctx.lineWidth=2;ctx.beginPath();arr.forEach((v,i)=>{let x=27+i*(W-45)/(arr.length-1), y=H-23-(v/max)*(H-38);i?ctx.lineTo(x,y):ctx.moveTo(x,y)});ctx.stroke()}
    }
  } catch(e){const el=document.getElementById('v-estado');if(el)el.textContent='Panel sin conexión con Python'}
}
setInterval(actualizarCamara,1000);actualizarCamara();
</script>"""
    salida_html = ('<!DOCTYPE html><html lang="es"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">'
                   '<title>Resultados IoT y Métodos Numéricos</title>' + css +
                   '</head><body><header><h1>Sistema de Monitoreo IoT</h1><p>Panel de Métodos Numéricos · Fase 1</p><nav>' + navegacion +
                   '</nav></header><main>' + cuerpo + '</main><footer>Resultados exportados automáticamente por el prototipo.</footer>' + script + '</body></html>')
    ruta = salida / 'panel_resultados.html'
    ruta.write_text(salida_html, encoding='utf-8')
    return ruta


def main(argv=None):
    args=opciones(argv)
    salida=Path(args.salida) / ('ensayo_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'))
    salida.mkdir(parents=True,exist_ok=False)
    textos=[]
    datos={'fecha_utc':datetime.now(timezone.utc).isoformat(),
           'sistema':platform.system(),'estado':'en_ejecucion','etapas':{}}
    def log(x):
        print(x,flush=True)
        textos.append(str(x))
    log('=== MONITOREO IoT Y METODOS NUMERICOS ===')
    datos['referencia_windows'] = None
    if args.datos_referencia:
        try:
            datos['referencia_windows'] = cargar_resultados_historicos(args, salida, log)
        except (ValueError, OSError, json.JSONDecodeError) as exc:
            datos['etapas']['importacion']={'estado':'error','detalle':str(exc)}
            log(f'[ERROR IMPORTACIÓN] {exc}')
    interfaz=args.interfaz or descubrir_interfaz()
    log(f'Interfaz Wi-Fi: {interfaz or "sin detectar automaticamente"}')
    try:
        redes=inventario_wifi()
        datos['etapas']['wifi']={'estado':'ok','redes':redes}
        log(f'Wi-Fi: {len(redes)} redes (intensidad porcentual para inventario; NO RSSI dBm).')
        for r in redes[:15]:log(f'  {r["bssid"]} | {r["ssid"]} | {r["senal_porcentaje"]}%')
    except Exception as e:
        datos['etapas']['wifi']={'estado':'no_disponible','detalle':str(e)};log(f'[AVISO WI-FI] {e}')
    try:
        vecinos=vecinos_lan()
        datos['etapas']['lan']={'estado':'ok','vecinos':vecinos}
        log(f'Registros ARP/neigh: {len(vecinos)} (pueden contener entradas en caché).')
    except Exception as e:
        datos['etapas']['lan']={'estado':'no_disponible','detalle':str(e)};log(f'[AVISO LAN] {e}')
    if not args.sin_esp32:
        puerto=args.puerto_esp32 or puerto_esp32_automatico()
        if puerto:
            try:
                esp=capturar_esp32(puerto,args.segundos_esp32,args.baudrate,args.mac_objetivo)
                datos['etapas']['esp32']={'estado':'ok','captura':esp}
                log(f'ESP32 ({puerto}): {esp["tramas_validas"]} tramas SNIFF y {len(esp["emisores"])} emisores.')
                for em in esp['emisores'][:10]: log(f'  {em["mac"]}: {em["cantidad"]} muestras RSSI medio={em["rssi_medio_dbm"]:.2f} dBm')
                if esp['emisores']:
                    fig,ax=plt.subplots(figsize=(8,4))
                    for em in esp['emisores'][:5]:
                        ax.plot(range(1,em['cantidad']+1),em['muestras_rssi_dbm'],label=em['mac'])
                    ax.set(xlabel='Muestra',ylabel='RSSI (dBm)',title='ESP32: señales recibidas (captura real)')
                    ax.grid(alpha=.3);ax.legend(fontsize=7);fig.tight_layout()
                    fig.savefig(salida/'grafica_esp32_rssi.png',dpi=200);plt.close(fig)
            except Exception as e:
                datos['etapas']['esp32']={'estado':'error','detalle':str(e)};log(f'[AVISO ESP32] {e}')
        else:
            datos['etapas']['esp32']={'estado':'pendiente','detalle':'Sin puerto identificado; use --puerto-esp32.'}
            log('[ESP32] No se detectó un puerto serie inequívoco. Use --puerto-esp32 si está conectado por USB.')
    log(f'[Unidad 1] Épsilon de máquina: {epsilon_maquina():.9e}')
    if args.ancla:
        try:
            datos['etapas']['localizacion']={'estado':'ok','resultados':localizar(args,salida,log,interfaz)}
        except Exception as e:
            datos['etapas']['localizacion']={'estado':'error','detalle':str(e)}
            log(f'[ERROR LOCALIZACION] {e}')
    else:
        datos['etapas']['localizacion']={'estado':'pendiente','detalle':'Se requieren >=3 AP con BSSID/posición, RSSI dBm, calibración del canal.'}
        log('[LOCALIZACION] Para una nueva localización, configure las anclas y sus parámetros físicos.')
    camara_viva=None
    vivo=not args.sin_camara and not args.camara_temporal
    if vivo:
        try:
            camara_viva=CamaraEnVivo(args,salida)
            camara_viva.iniciar()
            datos['etapas']['camara']={'estado':'en_vivo','captura':{'estado':'en_vivo'}}
            log('[CÁMARA] Webcam continua iniciada. El panel muestra video y análisis en vivo.')
        except Exception as exc:
            datos['etapas']['camara']={'estado':'error','detalle':str(exc)}
            log(f'[AVISO CAMARA] {exc}')
    elif not args.sin_camara:
        try:
            cam=camera(args,salida)
            datos['etapas']['camara']={'estado':'ok','captura':cam}
            log(f'Cámara: {cam["estado"]}; {cam["frames"]} frames. Captura: {cam["archivo"]}.')
        except Exception as exc:
            datos['etapas']['camara']={'estado':'error','detalle':str(exc)};log(f'[AVISO CAMARA] {exc}')
    fallos=[k for k,v in datos['etapas'].items() if v.get('estado')=='error']
    datos['estado']='parcial_con_errores' if fallos else 'completado'
    datos['errores']=fallos
    guardar_json(salida/'resultados.json',datos)
    (salida/'resultados_informe.txt').write_text('\n'.join(textos)+'\n',encoding='utf-8')
    servidor=None
    try:
        panel=crear_panel_resultados(salida,datos,en_vivo=camara_viva is not None)
        if camara_viva is not None:
            servidor=crear_servidor_panel(salida,camara_viva,args.puerto_panel)
            url=f'http://127.0.0.1:{servidor.server_address[1]}/'
            log(f'PANEL EN VIVO (solo este computador): {url}')
            if not args.sin_abrir_panel:
                webbrowser.open(url)
            log('La cámara funciona hasta Ctrl+C. Cerrar la pestaña NO detiene la captura.')
            limite=time.monotonic()+args.duracion_vivo if args.duracion_vivo is not None else None
            servidor.timeout=0.4
            while not camara_viva.stop_event.is_set() and (limite is None or time.monotonic()<limite):
                servidor.handle_request()
            estado=camara_viva.estado()
            if estado.get('error'):
                log('[ERROR EN CÁMARA CONTINUA] '+estado['error'])
        else:
            log('Panel visual de esta ejecución: '+str(panel.resolve()))
            if not args.sin_abrir_panel and (platform.system()=='Windows' or os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
                webbrowser.open(panel.resolve().as_uri())
    except KeyboardInterrupt:
        log('Detención solicitada con Ctrl+C. Cerrando webcam y guardando datos...')
    except Exception as exc:
        log('[ERROR PANEL] '+str(exc))
    finally:
        if servidor is not None: servidor.server_close()
        if camara_viva is not None:
            camara_viva.detener()
            final=camara_viva.estado()
            datos['etapas']['camara']={'estado':final['estado'],'captura':final}
            if final.get('error'):
                datos['errores']=list(set(datos['errores']+['camara']))
                datos['estado']='parcial_con_errores'
            guardar_json(salida/'resultados.json',datos)
            log('Fotografía: '+str(salida/'captura_camara_real.png'))
            log('Mediciones ópticas: '+str(salida/'camara_mediciones.csv'))
            if final.get('archivo_video'):
                log('Vídeo grabado: '+str(final['archivo_video']))
            # Archivo HTML durable sin depender de que el servidor continúe activo.
            crear_panel_resultados(salida,datos,en_vivo=False)
        (salida/'resultados_informe.txt').write_text('\n'.join(textos)+'\n',encoding='utf-8')
    log('Archivos de evidencia guardados en: '+str(salida.resolve()))
    return 1 if datos.get('errores') else 0


if __name__=='__main__':
    raise SystemExit(main())
