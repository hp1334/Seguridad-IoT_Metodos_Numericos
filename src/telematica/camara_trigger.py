"""Captura de video real y deteccion de movimiento, sin posiciones inventadas.

Una camara sin calibracion NO proporciona coordenadas metricas confiables.
La homografia opcional solo es valida si las correspondencias pertenecen al
mismo plano fisico y el montaje de la camara permanece fijo.
"""
from __future__ import annotations

import os
import time
from pathlib import Path

import cv2
import numpy as np


def _preparar_homografia(correspondencias):
    if correspondencias is None:
        return None
    if not isinstance(correspondencias, list) or len(correspondencias) < 4:
        raise ValueError('Se requieren al menos cuatro correspondencias pixel/metros para calibrar la camara.')
    src, dst = [], []
    for punto in correspondencias:
        src.append(punto['pixel'])
        dst.append(punto['metros'])
    src = np.asarray(src, dtype=np.float32)
    dst = np.asarray(dst, dtype=np.float32)
    if src.shape[1:] != (2,) or dst.shape[1:] != (2,):
        raise ValueError('Cada correspondencia debe tener dos coordenadas de pixel y de metros.')
    matriz, mascara = cv2.findHomography(src, dst, method=0)
    if matriz is None or np.linalg.matrix_rank(matriz) < 3:
        raise ValueError('Calibracion invalida: correspondencias degeneradas.')
    return matriz


def ejecutar_verificacion_visual(
    coord_wifi=None,
    *,
    indice_camara=0,
    backend='v4l2',
    num_frames_captura=180,
    tiempo_max_seg=20.0,
    mostrar_ventana=True,
    ruta_captura='resultados/captura_camara_real.png',
    umbral_diferencia=18,
    area_minima_px2=400.0,
    calentamiento_frames=8,
    pausa_ms=35,
    correspondencias=None,
):
    """Devuelve mediciones observadas; None cuando no se detecta movimiento.

    coord_wifi se muestra en la evidencia unicamente como texto informativo.
    """
    if num_frames_captura < 1 or tiempo_max_seg <= 0 or calentamiento_frames < 0:
        raise ValueError('Frames, tiempo maximo y calentamiento deben tener valores validos.')
    if not (0 < umbral_diferencia < 255) or area_minima_px2 <= 0:
        raise ValueError('Umbral y area minima deben ser positivos y validos.')
    if backend not in ('auto', 'v4l2'):
        raise ValueError('Backend permitido: auto o v4l2.')

    if mostrar_ventana:
        if not (os.getenv('DISPLAY') or os.getenv('WAYLAND_DISPLAY')):
            raise RuntimeError('No hay sesion grafica; use --sin-ventana.')
        gui = next((line for line in cv2.getBuildInformation().splitlines()
                    if line.strip().startswith('GUI:')), '')
        if 'NONE' in gui.upper():
            raise RuntimeError('OpenCV sin interfaz grafica. Revise la instalacion de opencv-python.')

    homografia = _preparar_homografia(correspondencias)
    modo = cv2.CAP_V4L2 if backend == 'v4l2' else cv2.CAP_ANY
    cap = cv2.VideoCapture(indice_camara, modo)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError(f'No se pudo abrir /dev/video{indice_camara} con backend {backend}. '
                           'Pruebe v4l2-ctl --list-devices y el diagnostico independiente.')

    frames_validos = 0
    detecciones = 0
    mejor_area = 0.0
    mejor_centro = None
    mejor_frame = None
    resolucion = None
    frame_base = None
    fondo = None
    ruta = Path(ruta_captura)
    ruta.parent.mkdir(parents=True, exist_ok=True)
    inicio = time.monotonic()
    try:
        for _ in range(calentamiento_frames):
            cap.read()
        ok, frame_base = cap.read()
        if not ok or frame_base is None:
            raise RuntimeError('La camara abrio, pero no entrego un fotograma de referencia.')
        resolucion = f'{frame_base.shape[1]}x{frame_base.shape[0]}'
        fondo = cv2.GaussianBlur(cv2.cvtColor(frame_base, cv2.COLOR_BGR2GRAY), (21, 21), 0)
        mejor_frame = frame_base.copy()
        while frames_validos < num_frames_captura and time.monotonic() - inicio < tiempo_max_seg:
            ok, frame = cap.read()
            if not ok or frame is None:
                continue
            frames_validos += 1
            vista = frame.copy()
            gris = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (21, 21), 0)
            diferencia = cv2.absdiff(fondo, gris)
            mascara = cv2.threshold(diferencia, umbral_diferencia, 255, cv2.THRESH_BINARY)[1]
            mascara = cv2.dilate(mascara, None, iterations=2)
            contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contornos:
                contorno = max(contornos, key=cv2.contourArea)
                area = float(cv2.contourArea(contorno))
                if area >= area_minima_px2:
                    x, y, w, h = cv2.boundingRect(contorno)
                    centro = (x + w // 2, y + h // 2)
                    detecciones += 1
                    cv2.rectangle(vista, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(vista, centro, 6, (0, 0, 255), -1)
                    if area >= mejor_area:
                        mejor_area = area
                        mejor_centro = centro
                        mejor_frame = vista.copy()
            if coord_wifi is not None:
                cv2.putText(vista, f'Wi-Fi: {float(coord_wifi[0]):.2f}, {float(coord_wifi[1]):.2f} m',
                            (10, 28), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 255), 2)
            if mostrar_ventana:
                try:
                    cv2.imshow('IoT - Webcam real (q para cerrar)', vista)
                    if cv2.waitKey(max(1, pausa_ms)) & 0xFF == ord('q'):
                        break
                except cv2.error as error:
                    raise RuntimeError(f'Error de interfaz grafica de OpenCV: {error}') from error
            else:
                time.sleep(max(0, pausa_ms) / 1000.0)
    finally:
        cap.release()
        if mostrar_ventana:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass

    if frames_validos == 0 or mejor_frame is None:
        raise RuntimeError('Camara abierta sin fotogramas utilizables.')
    etiqueta = ('MOVIMIENTO DETECTADO' if mejor_centro is not None
                else 'SIN MOVIMIENTO DETECTADO')
    cv2.putText(mejor_frame, etiqueta, (12, max(30, mejor_frame.shape[0] - 18)),
                cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 255, 255), 2)
    if not cv2.imwrite(str(ruta), mejor_frame):
        raise OSError(f'OpenCV no pudo guardar {ruta}')

    coord_metricas = None
    discrepancia = None
    if mejor_centro is not None and homografia is not None:
        punto = np.array([[mejor_centro]], dtype=np.float32)
        proyeccion = cv2.perspectiveTransform(punto, homografia)[0, 0]
        coord_metricas = [float(proyeccion[0]), float(proyeccion[1])]
        if coord_wifi is not None:
            discrepancia = float(np.linalg.norm(np.asarray(coord_wifi) - np.asarray(coord_metricas)))

    return {
        'estado': 'movimiento_detectado' if mejor_centro is not None else 'sin_movimiento_detectado',
        'resolucion_camara': resolucion,
        'frames_analizados': frames_validos,
        'detecciones': detecciones,
        'pixel_detectado': list(mejor_centro) if mejor_centro is not None else None,
        'area_movimiento_px2': mejor_area,
        'coordenada_visual_m': coord_metricas,
        'discrepancia_wifi_camara_m': discrepancia,
        'calibracion_metrica_aplicada': homografia is not None,
        'archivo_evidencia': str(ruta),
        'advertencia': ('La deteccion de movimiento no identifica a la persona ni garantiza que '
                        'corresponda al dispositivo Wi-Fi localizado.'),
    }
