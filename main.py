"""Orquestador Fedora. No fabrica lecturas, anclas ni evidencias experimentales.

Modos:
  diagnostico: inventario LAN + Wi-Fi nmcli + webcam fisica.
  localizacion: anclas verificadas + RSSI dBm (iw o JSON medido) + metodos.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import traceback
from datetime import datetime, timezone
from pathlib import Path

import matplotlib
matplotlib.use('Agg')  # Archivos PNG: compatible incluso si falla GUI de OpenCV.
import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import solve
from scipy.optimize import root_scalar

from src.metodos_numericos.errores import (calcular_epsilon_maquina,
    calcular_numero_condicion, error_absoluto)
from src.metodos_numericos.ecuaciones_no_lineales import (
    metodo_biseccion, metodo_secante, metodo_newton_raphson)
from src.metodos_numericos.sistemas_lineales import (
    gauss_seidel, metodo_jacobi, eliminacion_gaussiana_pivoteo)
from src.telematica.camara_trigger import ejecutar_verificacion_visual
from src.telematica.wifi_csi_sniffer import (inventario_nmcli, listar_vecinos_lan,
    capturar_iw, cargar_anclas, cargar_lecturas_json, asociar_lecturas_anclas,
    construir_ecuacion_calibracion_n, construir_ecuacion_distancia,
    ensamblar_sistema_trilateracion)


def argumentos():
    p = argparse.ArgumentParser(description='Monitoreo IoT y metodos numericos en Fedora')
    p.add_argument('--modo', choices=['diagnostico', 'localizacion'], default='diagnostico')
    p.add_argument('--salida', default='resultados', help='Carpeta para evidencias reales y reportes')
    p.add_argument('--indice-camara', type=int, default=0)
    p.add_argument('--backend-camara', choices=['auto', 'v4l2'], default='v4l2')
    p.add_argument('--camara-frames', type=int, default=180)
    p.add_argument('--camara-segundos', type=float, default=20.0)
    p.add_argument('--camara-umbral', type=int, default=18)
    p.add_argument('--camara-area-min', type=float, default=400.0)
    p.add_argument('--camara-calentamiento', type=int, default=8)
    p.add_argument('--camara-pausa-ms', type=int, default=35)
    p.add_argument('--sin-ventana', action='store_true', help='Capturar sin cv2.imshow')
    p.add_argument('--omitir-camara', action='store_true', help='Omitir prueba fisica de camara')
    p.add_argument('--calibracion-camara', help='JSON con correspondencias pixel / metros')
    p.add_argument('--anclas', help='JSON de BSSID y coordenadas fisicas verificadas')
    p.add_argument('--fuente', choices=['iw', 'json'], default='iw')
    p.add_argument('--interfaz', help='Interfaz Wi-Fi, por ejemplo wlp0s20f3; requerido para iw')
    p.add_argument('--lecturas', help='JSON de lecturas RSSI reales; requerido con fuente json')
    p.add_argument('--barridos', type=int, default=3)
    p.add_argument('--pausa-barridos', type=float, default=0.5)
    p.add_argument('--ancla-calibracion', help='ID de ancla a distancia fisica conocida')
    p.add_argument('--d-ref', type=float, help='Distancia REAL en metros desde el receptor al ancla de calibracion')
    p.add_argument('--pt-dbm', type=float, help='Potencia TX medida/documentada del emisor, dBm')
    p.add_argument('--pl-d0', type=float, help='Perdida a la distancia base d0 en dB, medida/documentada')
    p.add_argument('--alpha', type=float, help='Absorcion del medio medida/estimada en dB/m')
    p.add_argument('--d0', type=float, default=1.0, help='Distancia fisica base d0 del modelo (m)')
    p.add_argument('--bis-a', type=float, help='Limite inferior del intervalo de distancias (m)')
    p.add_argument('--bis-b', type=float, help='Limite superior del intervalo de distancias (m)')
    p.add_argument('--d0-secante', type=float, help='Primera semilla de distancias (m)')
    p.add_argument('--d1-secante', type=float, help='Segunda semilla de distancias (m)')
    p.add_argument('--d0-newton', type=float, help='Semilla Newton para distancias (m)')
    p.add_argument('--n0-secante', type=float, default=1.5)
    p.add_argument('--n1-secante', type=float, default=4.0)
    p.add_argument('--semilla-x', type=float)
    p.add_argument('--semilla-y', type=float)
    p.add_argument('--tol', type=float, default=1e-6)
    p.add_argument('--max-iter', type=int, default=100)
    return p.parse_args()


def guardar_json(ruta, contenido):
    ruta = Path(ruta)
    ruta.parent.mkdir(parents=True, exist_ok=True)
    ruta.write_text(json.dumps(contenido, ensure_ascii=False, indent=2,
                              allow_nan=False, default=lambda x: float(x) if isinstance(x, np.floating) else str(x)),
                    encoding='utf-8')


def config_camara(args, coord_wifi=None):
    calibracion = None
    if args.calibracion_camara:
        archivo = json.loads(Path(args.calibracion_camara).read_text(encoding='utf-8'))
        calibracion = archivo['correspondencias']
    return ejecutar_verificacion_visual(
        coord_wifi=coord_wifi,
        indice_camara=args.indice_camara,
        backend=args.backend_camara,
        num_frames_captura=args.camara_frames,
        tiempo_max_seg=args.camara_segundos,
        mostrar_ventana=not args.sin_ventana,
        ruta_captura=str(Path(args.salida) / 'captura_camara_real.png'),
        umbral_diferencia=args.camara_umbral,
        area_minima_px2=args.camara_area_min,
        calentamiento_frames=args.camara_calentamiento,
        pausa_ms=args.camara_pausa_ms,
        correspondencias=calibracion,
    )


def diagnostico(args, log, resultados):
    log('FASE DIAGNOSTICO (sin fingir trilateracion ni RSSI dBm)')
    try:
        vecinos = listar_vecinos_lan()
        resultados['lan'] = vecinos
        log(f'Vecinos LAN conocidos: {len(vecinos.get("dispositivos", []))}')
    except Exception as exc:
        resultados['error_lan'] = str(exc)
        log(f'[ERROR LAN] {exc}')
    try:
        redes = inventario_nmcli()
        resultados['redes_nmcli'] = redes
        log(f'Redes detectadas via nmcli: {len(redes)} (SIGNAL en %, NO RSSI en dBm)')
        for r in redes:
            log(f'  {r["bssid"]} | SSID={r["ssid"]} | senal={r["signal_pct_nmcli"]}%')
    except Exception as exc:
        resultados['error_wifi'] = str(exc)
        log(f'[ERROR Wi-Fi] {exc}')


def validar_localizacion(args):
    necesarios = {'--anclas': args.anclas, '--ancla-calibracion': args.ancla_calibracion,
                  '--d-ref': args.d_ref, '--pt-dbm': args.pt_dbm,
                  '--pl-d0': args.pl_d0, '--alpha': args.alpha,
                  '--bis-a': args.bis_a, '--bis-b': args.bis_b}
    faltantes = [k for k, v in necesarios.items() if v is None]
    if args.fuente == 'iw' and not args.interfaz:
        faltantes.append('--interfaz')
    if args.fuente == 'json' and not args.lecturas:
        faltantes.append('--lecturas')
    if faltantes:
        raise ValueError('En modo localizacion hacen falta: ' + ', '.join(faltantes))
    if args.tol <= 0 or args.max_iter < 1:
        raise ValueError('La tolerancia y el numero de iteraciones deben ser positivos.')
    if args.d0 <= 0 or args.d_ref <= 0 or args.alpha < 0 or args.bis_a <= 0 or args.bis_b <= args.bis_a:
        raise ValueError('Revise d0, d-ref, alpha e intervalo positivo [bis-a, bis-b].')
    if args.d0 == args.d_ref:
        raise ValueError('d-ref debe ser diferente de d0 para calibrar n.')


def graficar_convergencia(historiales, titulo, archivo):
    fig, ax = plt.subplots(figsize=(8, 5))
    for nombre, historia in historiales.items():
        if not historia:
            continue
        iteraciones = [h['iteracion'] for h in historia]
        errores = [max(float(h['error_abs']), np.finfo(float).eps) for h in historia]
        ax.semilogy(iteraciones, errores, marker='o', label=nombre)
    ax.set_title(titulo)
    ax.set_xlabel('Iteracion')
    ax.set_ylabel('Error absoluto aproximado')
    ax.legend()
    ax.grid(True, which='both', alpha=.3)
    fig.tight_layout()
    fig.savefig(archivo, dpi=200)
    plt.close(fig)


def graficar_mapa(anclas, distancias, posicion, salida):
    fig, ax = plt.subplots(figsize=(7, 6))
    for a, d in zip(anclas, distancias):
        x, y = a['coordenadas']
        ax.plot([x], [y], marker='s', color='black')
        ax.add_patch(plt.Circle((x, y), d, fill=False, linestyle='--', alpha=.7))
        ax.annotate(a['id_ancla'], (x, y), xytext=(5, 5), textcoords='offset points')
    ax.plot([posicion[0]], [posicion[1]], 'ro', label='Posicion estimada por Wi-Fi')
    ax.set_aspect('equal', adjustable='datalim')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Trilateracion RSSI: anclas declaradas y mediciones asociadas')
    ax.legend()
    ax.grid(True, alpha=.3)
    fig.tight_layout()
    fig.savefig(salida, dpi=200)
    plt.close(fig)


def localizacion(args, log, resultados):
    validar_localizacion(args)
    anclas = cargar_anclas(args.anclas)
    log(f'Anclas fisicas declaradas: {len(anclas)}')
    if args.fuente == 'iw':
        medidas = capturar_iw(args.interfaz, args.barridos, args.pausa_barridos)
    else:
        medidas = cargar_lecturas_json(args.lecturas)
    guardar_json(Path(args.salida) / 'lecturas_adquiridas.json', medidas)
    lecturas = asociar_lecturas_anclas(anclas, medidas)
    resultados['lecturas_anclas'] = lecturas
    log('Lecturas asociadas EXCLUSIVAMENTE por BSSID registrado:')
    for a in lecturas:
        log(f'  {a["id_ancla"]}: RSSI={a["rssi_efectivo_dbm"]:.2f} dBm, '
            f'sigma={a["sigma_rssi_db"]} dB, BSSID={a["bssid"]}')

    calibracion = next((a for a in lecturas if a['id_ancla'] == args.ancla_calibracion), None)
    if calibracion is None:
        raise ValueError(f'La ancla de calibracion {args.ancla_calibracion!r} no figura en --anclas.')
    fcal = construir_ecuacion_calibracion_n(calibracion['rssi_efectivo_dbm'], args.d_ref,
                                            args.pt_dbm, args.pl_d0, args.alpha, args.d0)
    ref_n = root_scalar(fcal, x0=args.n0_secante, x1=args.n1_secante, method='secant')
    if not ref_n.converged:
        raise RuntimeError('SciPy no pudo calibrar el exponente n.')
    n, hist_n = metodo_secante(fcal, args.n0_secante, args.n1_secante,
                              args.tol, args.max_iter, ref_n.root)
    if n <= 0:
        raise ValueError(f'n calibrado = {n:.4f} (no positivo). Revise parametros fisicos y RSSI.')
    log(f'Exponente de propagacion n={n:.8f}, referencia SciPy={ref_n.root:.8f}')

    distancias = []
    historias = {}
    propagacion = []
    x0 = args.d0_secante if args.d0_secante is not None else args.bis_a + .25*(args.bis_b-args.bis_a)
    x1 = args.d1_secante if args.d1_secante is not None else args.bis_a + .50*(args.bis_b-args.bis_a)
    newton0 = args.d0_newton if args.d0_newton is not None else args.d_ref
    for a in lecturas:
        fun, derivada = construir_ecuacion_distancia(a['rssi_efectivo_dbm'],
                        args.pt_dbm, args.pl_d0, n, args.alpha, args.d0)
        fa, fb = fun(args.bis_a), fun(args.bis_b)
        if fa * fb >= 0:
            raise ValueError(f'La raiz del ancla {a["id_ancla"]} queda fuera de '
                             f'[{args.bis_a}, {args.bis_b}]. Revise el intervalo/mediciones.')
        ref = root_scalar(fun, bracket=[args.bis_a, args.bis_b], method='brentq').root
        bis, hb = metodo_biseccion(fun, args.bis_a, args.bis_b, args.tol, args.max_iter, ref)
        sec, hs = metodo_secante(fun, x0, x1, args.tol, args.max_iter, ref)
        nr, hn = metodo_newton_raphson(fun, derivada, newton0, args.tol, args.max_iter, ref)
        distancias.append(sec)
        historias[a['id_ancla']] = {'biseccion': hb, 'secante': hs, 'newton_raphson': hn}
        sigma = a['sigma_rssi_db']
        delta = sigma / abs(derivada(sec)) if sigma is not None else None
        propagacion.append({'id_ancla': a['id_ancla'], 'd_m': sec, 'sigma_rssi_db': sigma,
                            'propagacion_delta_m': delta})
        log(f'{a["id_ancla"]}: Biseccion={bis:.6f} m ({len(hb)} iter), '
            f'Secante={sec:.6f} m ({len(hs)} iter), Newton={nr:.6f} m ({len(hn)} iter), '
            f'±delta={delta if delta is not None else "no calculable: falta sigma"}')

    A, B, M, C = ensamblar_sistema_trilateracion([a['coordenadas'] for a in lecturas], distancias)
    cond = calcular_numero_condicion(M)
    referencia = solve(M, C)
    if args.semilla_x is not None and args.semilla_y is not None:
        semilla = np.array([args.semilla_x, args.semilla_y], dtype=float)
    elif args.semilla_x is None and args.semilla_y is None:
        semilla = np.mean([a['coordenadas'] for a in lecturas], axis=0)
    else:
        raise ValueError('Para la semilla debe pasar --semilla-x Y --semilla-y, o ninguna.')
    gs, hgs, dom = gauss_seidel(M, C, semilla, args.tol, args.max_iter, referencia)
    jac, hjac = metodo_jacobi(M, C, semilla, args.tol, args.max_iter, referencia)
    gauss = eliminacion_gaussiana_pivoteo(M, C)
    log(f'M = {M.tolist()}, C = {C.tolist()}')
    log(f'Condicion inf={cond:.6f} | Diagonal dominante={dom[0]} ({dom[1]})')
    log(f'GS: {gs.tolist()} | Jacobi: {jac.tolist()} | Pivoteo: {gauss.tolist()}')
    log(f'SciPy: {referencia.tolist()} | error GS vs SciPy: {error_absoluto(gs, referencia):.3e} m')
    resultados['numerico'] = {
        'n_calibrado': n, 'n_ref_scipy': ref_n.root, 'historial_calibracion': hist_n,
        'distancias': propagacion, 'historiales_distancias': historias,
        'A': A.tolist(), 'B': B.tolist(), 'M': M.tolist(), 'C': C.tolist(),
        'condicion_infinito': cond, 'diagonal_dominante': dom[0],
        'semilla_xy': semilla.tolist(), 'gauss_seidel': gs.tolist(),
        'jacobi': jac.tolist(), 'gauss_pivoteo': gauss.tolist(),
        'referencia_scipy': referencia.tolist(),
        'error_gs_scipy_m': error_absoluto(gs, referencia),
        'historial_gauss_seidel': hgs, 'historial_jacobi': hjac,
        'nota': 'Error frente a SciPy es numerico, NO error de posicion frente a verdad terreno.'}
    primer = lecturas[0]['id_ancla']
    graficar_convergencia(historias[primer], f'Convergencia en {primer}',
                         Path(args.salida) / 'grafica_1_convergencia_no_lineal.png')
    graficar_convergencia({'Gauss-Seidel': hgs, 'Jacobi': hjac}, 'Convergencia 2D',
                         Path(args.salida) / 'grafica_2_convergencia_sistemas_lineales.png')
    graficar_mapa(lecturas, distancias, gs,
                 Path(args.salida) / 'grafica_3_mapa_trilateracion_iot.png')
    log('Graficas numericas generadas y guardadas en carpeta de salida.')
    return gs


def main():
    args = argumentos()
    salida = Path(args.salida)
    salida.mkdir(parents=True, exist_ok=True)
    mensajes = []
    resultado = {'fecha_utc': datetime.now(timezone.utc).isoformat(),
                 'modo': args.modo, 'estado': 'en_ejecucion',
                 'configuracion_cli': vars(args),
                 'epsilon_maquina': calcular_epsilon_maquina(),
                 'advertencias': []}
    def log(mensaje):
        texto = str(mensaje)
        print(texto, flush=True)
        mensajes.append(texto)
    log(f'Proyecto IoT / {args.modo} - {resultado["fecha_utc"]}')
    log(f'Epsilon de maquina: {resultado["epsilon_maquina"]:.9e}')
    codigo = 0
    posicion = None
    try:
        if args.modo == 'diagnostico':
            diagnostico(args, log, resultado)
        else:
            posicion = localizacion(args, log, resultado)
        resultado['estado_numerico_red'] = 'completado'
        if args.modo == 'diagnostico' and (resultado.get('error_wifi') or resultado.get('error_lan')):
            codigo = 1
            resultado['estado_numerico_red'] = 'diagnostico_parcial'
    except Exception as exc:
        codigo = 1
        resultado['estado_numerico_red'] = 'fallo'
        resultado['error_numerico_red'] = str(exc)
        log(f'[ERROR ETAPA DE RED / CALCULO] {exc}')
        log(traceback.format_exc())
    if not args.omitir_camara:
        log('Iniciando comprobacion de webcam fisica...')
        try:
            rep_cam = config_camara(args, posicion)
            resultado['camara'] = rep_cam
            log(f'Camara: {rep_cam["estado"]} | frames={rep_cam["frames_analizados"]} '
                f'| captura={rep_cam["archivo_evidencia"]}')
            if rep_cam['coordenada_visual_m'] is None:
                log('No se calcularon metros desde pixeles: falta deteccion y/o calibracion geometrica.')
            else:
                log(f'Posicion visual CALIBRADA={rep_cam["coordenada_visual_m"]} m '
                    f'| diferencia={rep_cam["discrepancia_wifi_camara_m"]} m')
        except Exception as exc:
            codigo = 1
            resultado['camara'] = {'estado': 'error', 'mensaje': str(exc)}
            log(f'[ERROR CAMARA] {exc}')
            log(traceback.format_exc())
    else:
        resultado['camara'] = {'estado': 'omitida_por_usuario'}
    resultado['estado'] = 'completado' if codigo == 0 else 'parcial_con_errores'
    guardar_json(salida / 'resultados.json', resultado)
    (salida / 'reporte_ejecucion.txt').write_text('\n'.join(mensajes) + '\n', encoding='utf-8')
    log(f'Reporte guardado: {salida / "reporte_ejecucion.txt"}')
    log(f'JSON guardado: {salida / "resultados.json"}')
    return codigo


if __name__ == '__main__':
    raise SystemExit(main())
