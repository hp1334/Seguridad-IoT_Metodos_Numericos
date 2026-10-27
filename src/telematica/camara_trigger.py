# src/telematica/camara_trigger.py
"""
Módulo de Visión Computacional Real (Sin fotogramas sintéticos):
Abre la cámara web física de la laptop con OpenCV (cv2.VideoCapture),
detecta el movimiento real de la persona mediante diferenciación de fotogramas
y contornos morfológicos, proyecta su posición física y exporta la captura real.
"""
import time
import numpy as np
import cv2


def ejecutar_verificacion_visual(
    coord_wifi,
    dimensiones_cuarto=(6.0, 6.0),
    indice_camara=0,
    num_frames_captura=25,
    mostrar_ventana=True,
    ruta_captura="captura_camara_real.png"
):
    """
    Activa la cámara física de la laptop y detecta los movimientos reales de una persona.

    Args:
        coord_wifi (np.ndarray): Coordenada [x, y] en metros calculada por Gauss-Seidel.
        dimensiones_cuarto (tuple): Dimensiones (ancho_m, largo_m) del área monitoreada.
        indice_camara (int): Índice del dispositivo de video (0 para webcam integrada).
        num_frames_captura (int): Cantidad de fotogramas reales a analizar en la ráfaga.
        mostrar_ventana (bool): Si es True, abre ventana en vivo con el rastreo de movimiento.
        ruta_captura (str): Archivo donde se guarda la evidencia fotográfica real.

    Returns:
        dict: Coordenadas del centroide detectado, área de movimiento y proyección en metros.
    """
    cap = cv2.VideoCapture(indice_camara)
    if not cap.isOpened():
        raise RuntimeError(
            f"No se pudo acceder a la cámara física de la laptop (índice {indice_camara}). "
            "Verifique que la webcam esté habilitada y con permisos en el sistema."
        )

    # Calentamiento del sensor CMOS de la cámara web
    for _ in range(8):
        cap.read()
        time.sleep(0.03)

    ret, frame_base = cap.read()
    if not ret or frame_base is None:
        cap.release()
        raise RuntimeError("Fallo al leer el fotograma base de la cámara web.")

    alto_px, ancho_px = frame_base.shape[:2]
    gris_base = cv2.cvtColor(frame_base, cv2.COLOR_BGR2GRAY)
    gris_base = cv2.GaussianBlur(gris_base, (21, 21), 0)

    # Clasificador frontal/corporal complementario por si la persona está quieta frente a la cámara
    cascade_path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
    detector_rostro = cv2.CascadeClassifier(cascade_path)

    mejor_frame = frame_base.copy()
    max_area_movimiento = 0.0
    centroide_detectado = (ancho_px // 2, alto_px // 2)
    bbox_detectado = (0, 0, 0, 0)
    trayectoria_pixeles = []

    print("[CÁMARA] Capturando video real desde la webcam... Muévase frente a la laptop.")

    for idx_f in range(num_frames_captura):
        ret, frame = cap.read()
        if not ret or frame is None:
            continue

        gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gris_blur = cv2.GaussianBlur(gris, (21, 21), 0)

        # 1. Detección de movimiento real por diferencia absoluta de fotogramas
        delta = cv2.absdiff(gris_base, gris_blur)
        umbral = cv2.threshold(delta, 18, 255, cv2.THRESH_BINARY)[1]
        umbral = cv2.dilate(umbral, None, iterations=3)

        contornos, _ = cv2.findContours(umbral, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        deteccion_en_frame = False

        if contornos:
            contorno_mayor = max(contornos, key=cv2.contourArea)
            area = float(cv2.contourArea(contorno_mayor))
            if area > 400.0:
                x, y, w, h = cv2.boundingRect(contorno_mayor)
                cx, cy = x + w // 2, y + h // 2
                trayectoria_pixeles.append((cx, cy))
                deteccion_en_frame = True

                if area >= max_area_movimiento:
                    max_area_movimiento = area
                    centroide_detectado = (cx, cy)
                    bbox_detectado = (x, y, w, h)
                    mejor_frame = frame.copy()

        # 2. Respaldo con detección biométrica real si el sujeto se mantiene estático
        if not deteccion_en_frame:
            rostros = detector_rostro.detectMultiScale(gris, scaleFactor=1.1, minNeighbors=5, minSize=(40, 40))
            if len(rostros) > 0:
                x, y, w, h = rostros[0]
                cx, cy = x + w // 2, y + h // 2
                area = float(w * h)
                trayectoria_pixeles.append((cx, cy))
                if area >= max_area_movimiento:
                    max_area_movimiento = area
                    centroide_detectado = (cx, cy)
                    bbox_detectado = (x, y, w, h)
                    mejor_frame = frame.copy()

        # Dibujar trazabilidad sobre el frame en vivo
        vista = frame.copy()
        if max_area_movimiento > 0:
            bx, by, bw, bh = bbox_detectado
            cv2.rectangle(vista, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)
            cv2.circle(vista, centroide_detectado, 6, (0, 0, 255), -1)

        if len(trayectoria_pixeles) > 1:
            for p_i in range(1, len(trayectoria_pixeles)):
                cv2.line(vista, trayectoria_pixeles[p_i - 1], trayectoria_pixeles[p_i], (255, 200, 0), 2)

        cv2.putText(
            vista,
            f"Wi-Fi Gauss-Seidel: ({coord_wifi[0]:.2f}m, {coord_wifi[1]:.2f}m)",
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2
        )

        if mostrar_ventana:
            try:
                cv2.imshow("Sistema Hibrido IoT - Verificacion Visual en Vivo", vista)
                if cv2.waitKey(45) & 0xFF == ord("q"):
                    break
            except cv2.error:
                mostrar_ventana = False
        else:
            time.sleep(0.04)

    cap.release()
    if mostrar_ventana:
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass

    # Anotar el mejor fotograma real y guardarlo en disco para el informe
    bx, by, bw, bh = bbox_detectado
    if bw > 0 and bh > 0:
        cv2.rectangle(mejor_frame, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)
    cv2.circle(mejor_frame, centroide_detectado, 7, (0, 0, 255), -1)
    if len(trayectoria_pixeles) > 1:
        for p_i in range(1, len(trayectoria_pixeles)):
            cv2.line(mejor_frame, trayectoria_pixeles[p_i - 1], trayectoria_pixeles[p_i], (255, 200, 0), 2)

    cv2.putText(
        mejor_frame,
        f"Wi-Fi: ({coord_wifi[0]:.2f}m, {coord_wifi[1]:.2f}m) | Cam Px: {centroide_detectado}",
        (15, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 255),
        2
    )
    cv2.imwrite(ruta_captura, mejor_frame)

    # Mapeo fotogramétrico del centroide real (u, v) al plano físico del cuarto (m)
    ancho_m, largo_m = dimensiones_cuarto
    u_px, v_px = centroide_detectado
    x_cam = (u_px / float(ancho_px)) * ancho_m
    y_cam = (v_px / float(alto_px)) * largo_m
    discrepancia_m = float(np.linalg.norm(coord_wifi - np.array([x_cam, y_cam])))

    return {
        "resolucion_camara": f"{ancho_px}x{alto_px}",
        "frames_analizados": num_frames_captura,
        "puntos_trayectoria_detectados": len(trayectoria_pixeles),
        "area_movimiento_px2": round(max_area_movimiento, 2),
        "pixel_detectado": (int(u_px), int(v_px)),
        "coordenada_visual_m": (round(x_cam, 4), round(y_cam, 4)),
        "discrepancia_wifi_camara_m": round(discrepancia_m, 4),
        "archivo_evidencia": ruta_captura,
        "estado_alerta": (
            "MOVIMIENTO HUMANO REAL DETECTADO Y CAPTURADO"
            if max_area_movimiento > 0
            else "CÁMARA ACTIVA (SIN MOVIMIENTO BRUSCO EN ESCENA)"
        )
    }