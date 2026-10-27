"""Diagnóstico local de webcam V4L2/OpenCV en Fedora.

No utiliza datos de prueba ni imágenes sintéticas.
Ejemplos:
  python diagnostico_camara_fedora.py --listar
  python diagnostico_camara_fedora.py --indice 0
  python diagnostico_camara_fedora.py --indice 0 --mostrar --segundos 10
"""

import argparse
import os
import sys
import time
from pathlib import Path

import cv2


def main():
    parser = argparse.ArgumentParser(description="Diagnosticar webcam real en Fedora y guardar evidencia")
    parser.add_argument("--listar", action="store_true", help="Listar dispositivos V4L2 visibles para Python")
    parser.add_argument("--indice", type=int, default=0, help="Índice a probar con OpenCV (predeterminado: 0)")
    parser.add_argument("--mostrar", action="store_true", help="Mostrar la ventana de OpenCV si hay interfaz gráfica")
    parser.add_argument("--segundos", type=float, default=5.0, help="Tiempo máximo de captura")
    parser.add_argument("--salida", default="evidencia_camara_fedora.png", help="Ruta de fotografía real guardada")
    args = parser.parse_args()

    print(f"[INFO] Python: {sys.version.split()[0]}")
    print(f"[INFO] OpenCV: {cv2.__version__}")
    print(f"[INFO] Sistema de ventanas: XDG_SESSION_TYPE={os.environ.get('XDG_SESSION_TYPE', 'no definido')}")
    print(f"[INFO] DISPLAY={os.environ.get('DISPLAY', 'no definido')}; WAYLAND_DISPLAY={os.environ.get('WAYLAND_DISPLAY', 'no definido')}")
    dispositivos = sorted(str(p) for p in Path('/dev').glob('video*'))
    print('[INFO] Dispositivos /dev/video*:', ', '.join(dispositivos) if dispositivos else 'ninguno visible')
    if args.listar:
        return 0 if dispositivos else 2

    if args.segundos <= 0:
        parser.error('--segundos debe ser positivo')

    # No ejecutar imshow() sin GUI: determinadas compilaciones de Qt abortan Python.
    compilacion = cv2.getBuildInformation()
    linea_gui = next((line.strip() for line in compilacion.splitlines() if line.strip().startswith('GUI:')), 'GUI: desconocido')
    print('[INFO] OpenCV', linea_gui)
    if args.mostrar:
        if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            print('[ERROR] No hay sesión gráfica visible. Ejecuta sin --mostrar para probar la captura.')
            return 3
        if 'NONE' in linea_gui.upper():
            print('[ERROR] OpenCV está compilado sin interfaz de ventanas (headless).')
            return 3

    print(f'[CAMARA] Abriendo dispositivo índice {args.indice} con V4L2...')
    cap = cv2.VideoCapture(args.indice, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        print('[ERROR] No se pudo abrir la cámara. Revisa índice, permisos, controladores y otras aplicaciones.')
        return 4

    fotogramas = 0
    evidencia_guardada = False
    inicio = time.monotonic()
    ultimo_error = None
    try:
        while time.monotonic() - inicio < args.segundos:
            ok, frame = cap.read()
            if not ok or frame is None:
                ultimo_error = 'cap.read() no devolvió una imagen'
                time.sleep(0.05)
                continue
            fotogramas += 1
            if not evidencia_guardada:
                evidencia_guardada = bool(cv2.imwrite(args.salida, frame))
                if evidencia_guardada:
                    alto, ancho = frame.shape[:2]
                    print(f'[OK] Primer fotograma real: {ancho} x {alto} px; archivo: {args.salida}')
                else:
                    print(f'[ERROR] No se pudo escribir {args.salida}')
                    return 5
            if args.mostrar:
                cv2.imshow('Diagnostico webcam Fedora (q para cerrar)', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    except cv2.error as exc:
        print('[ERROR OpenCV]', exc)
        return 6
    finally:
        cap.release()
        if args.mostrar:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass

    if not evidencia_guardada:
        print('[ERROR] La cámara abrió pero no entregó fotogramas.', ultimo_error)
        return 7
    print(f'[OK] Webcam operativa: {fotogramas} fotogramas reales recibidos.')
    print('[NOTA] Esta prueba confirma la captura; aún no valida la localización espacial del proyecto.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
