# main.py
"""
Script Principal en Vivo - Sistema Híbrido de Seguridad IoT y Métodos Numéricos.
Ejecuta mediciones reales con la tarjeta Wi-Fi de la laptop y la cámara web física:
1. Escaneo real de dispositivos en la red (ARP/Vecinos) y barrido de potencia Wi-Fi (dBm).
2. Autocalibración real del exponente de atenuación (n) con el Método de la Secante (Unidad 2).
3. Despeje de distancias físicas reales con Bisección, Secante y Newton-Raphson (Unidad 2).
4. Trilateración espacial 2D con Gauss-Seidel, Jacobi y Eliminación Gaussiana (Unidad 3).
5. Activación de la cámara web física de la laptop (OpenCV) para capturar el movimiento real.
"""
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import root_scalar
from scipy.linalg import solve

from src.metodos_numericos.errores import (
    calcular_epsilon_maquina,
    calcular_numero_condicion,
)
from src.metodos_numericos.ecuaciones_no_lineales import (
    metodo_biseccion,
    metodo_secante,
    metodo_newton_raphson,
)
from src.metodos_numericos.sistemas_lineales import (
    gauss_seidel,
    metodo_jacobi,
    eliminacion_gaussiana_pivoteo,
)
from src.telematica.wifi_csi_sniffer import (
    capturar_espectro_wifi_real,
    construir_ecuacion_calibracion_n,
    construir_ecuacion_distancia,
    ensamblar_sistema_trilateracion,
)
from src.telematica.camara_trigger import ejecutar_verificacion_visual


def ejecutar_prueba_control_analitico():
    """
    Recomendación Sección 7 del Instructivo:
    Verifica la exactitud matemática del código propio con un caso de solución conocida.
    """
    print("=" * 80)
    print("0. VERIFICACIÓN MATEMÁTICA DE CONTROL (CASO CON SOLUCIÓN ANALÍTICA CONOCIDA)")
    print("=" * 80)

    eps = calcular_epsilon_maquina()
    print(f"[Unidad 1] Épsilon de máquina IEEE 754 calculado : {eps:.4e}")

    raiz_sec, _ = metodo_secante(lambda x: x**2 - 4.0, 1.0, 3.0, tol=1e-8, valor_exacto=2.0)
    print(f"[Unidad 2] Ecuación control x^2 - 4 = 0          : Exacta = 2.0 | Secante = {raiz_sec:.6f}")

    M_ctrl = np.array([[4.0, 1.0], [1.0, 4.0]])
    C_ctrl = np.array([9.0, 6.0])
    sol_gs, _, _ = gauss_seidel(M_ctrl, C_ctrl, x0=[0.0, 0.0], tol=1e-8, sol_exacta=np.array([2.0, 1.0]))
    print(f"[Unidad 3] Sistema control 2x2                   : Exacta = (2.0, 1.0) | Gauss-Seidel = ({sol_gs[0]:.6f}, {sol_gs[1]:.6f})\n")


def main():
    ejecutar_prueba_control_analitico()

    # Coordenadas físicas (en metros) donde están ubicados los transmisores/anclas en tu habitación
    # (Puedes ajustar estos metros según el tamaño real de tu cuarto/laboratorio)
    dimensiones_cuarto = (6.0, 6.0)
    coordenadas_anclas_cuarto = np.array([
        [0.0, 0.0],  # Ancla 1 (Señal más fuerte, ej. ESP32 / Router cercano)
        [6.0, 0.0],  # Ancla 2 (Esquina derecha)
        [0.0, 6.0],  # Ancla 3 (Esquina superior izquierda)
        [6.0, 6.0],  # Ancla 4 (Esquina opuesta - Pivote de linealización)
    ])
    distancia_referencia_ancla1_m = 1.80  # Distancia física conocida al transmisor principal para autocalibrar n

    print("=" * 80)
    print("1. BARRIDO REAL DE DISPOSITIVOS EN RED Y ESPECTRO WI-FI DESDE LA LAPTOP")
    print("=" * 80)
    datos_reales = capturar_espectro_wifi_real(coordenadas_anclas_cuarto, num_barridos=3)

    print(f"Total de redes Wi-Fi reales captadas en el aire : {datos_reales['total_redes_aire']}")
    print(f"Dispositivos activos en la tabla ARP de la red  : {datos_reales['total_dispositivos_red']}")
    for disp in datos_reales["dispositivos_red"][:8]:
        print(f"  -> IP: {disp['ip']:<15} | MAC Real: {disp['mac']:<18} | Estado: {disp['estado']}")

    print("\nAnclas Wi-Fi Reales Seleccionadas para la Trilateración:")
    for ancla in datos_reales["lecturas_anclas"]:
        print(
            f"  -> {ancla['id_ancla']:<20} | BSSID: {ancla['bssid']} | "
            f"Freq: {ancla['frecuencia_mhz']:.0f} MHz | "
            f"RSSI Real: {ancla['rssi_efectivo_dbm']:6.2f} dBm | "
            f"Varianza: {ancla['varianza_real_db']:.4f}"
        )

    # 2. AUTOCALIBRACIÓN REAL DEL EXPONENTE DE TRAYECTORIA (n) CON MÉTODO DE LA SECANTE
    print("\n" + "=" * 80)
    print("2. UNIDAD 2 (PARTE A): AUTOCALIBRACIÓN DEL CANAL INALÁMBRICO REAL f(n) = 0")
    print("=" * 80)
    rssi_ancla_1 = datos_reales["lecturas_anclas"][0]["rssi_efectivo_dbm"]
    f_calib_n = construir_ecuacion_calibracion_n(rssi_ancla_1, d_ref_m=distancia_referencia_ancla1_m)
    n_scipy_ref = root_scalar(f_calib_n, bracket=[0.5, 8.0], method="brentq").root
    n_calibrado, hist_calib_n = metodo_secante(f_calib_n, 1.5, 4.0, tol=1e-6, valor_exacto=n_scipy_ref)

    print(f"RSSI real medido del Ancla 1 a {distancia_referencia_ancla1_m} m : {rssi_ancla_1:.2f} dBm")
    print(f"Exponente de atenuación (n) autocalibrado : {n_calibrado:.6f} (Convergió en {len(hist_calib_n)} iteraciones)")

    # 3. DESPEJE DE DISTANCIAS FÍSICAS REALES CON BISECCIÓN, SECANTE Y NEWTON-RAPHSON
    print("\n" + "=" * 80)
    print("3. UNIDAD 2 (PARTE B): CÁLCULO DE DISTANCIAS FÍSICAS REALES f(d) = 0")
    print("=" * 80)

    distancias_reales_estimadas = []
    hist_biseccion_a1, hist_secante_a1, hist_newton_a1 = [], [], []

    for idx, lectura in enumerate(datos_reales["lecturas_anclas"]):
        rssi = lectura["rssi_efectivo_dbm"]
        f_d, df_d = construir_ecuacion_distancia(rssi, n=n_calibrado)

        sol_scipy_d = root_scalar(f_d, bracket=[0.05, 50.0], method="brentq").root
        d_bis, h_bis = metodo_biseccion(f_d, 0.05, 50.0, tol=1e-6, valor_exacto=sol_scipy_d)
        d_sec, h_sec = metodo_secante(f_d, 0.5, 8.0, tol=1e-6, valor_exacto=sol_scipy_d)
        d_nr, h_nr = metodo_newton_raphson(f_d, df_d, x0=3.0, tol=1e-6, valor_exacto=sol_scipy_d)

        distancias_reales_estimadas.append(d_sec)

        if idx == 0:
            hist_biseccion_a1, hist_secante_a1, hist_newton_a1 = h_bis, h_sec, h_nr

        print(
            f"{lectura['id_ancla']:<20} | RSSI: {rssi:6.2f} dBm | "
            f"d_Secante: {d_sec:6.3f}m ({len(h_sec):2d} it) | "
            f"d_Newton: {d_nr:6.3f}m ({len(h_nr):2d} it) | "
            f"d_Bisección: {d_bis:6.3f}m ({len(h_bis):2d} it)"
        )

    print("\n--- Tabla Trazable de Iteraciones Unidad 2 (Método de la Secante - Ancla 1) ---")
    print(f"{'Iter':<5} | {'d_k (m)':<12} | {'|f(d_k)|':<12} | {'Error Abs (m)':<14} | {'Error Rel (%)':<14} | {'Error Verd. vs SciPy':<18}")
    print("-" * 86)
    for it in hist_secante_a1:
        print(
            f"{it['iteracion']:<5} | {it['aproximacion']:<12.6f} | {abs(it['f_x']):<12.4e} | "
            f"{it['error_abs']:<14.4e} | {it['error_rel_pct']:<14.6f} | {it['error_verdadero']:<18.4e}"
        )

    # 4. TRILATERACIÓN LINEALIZADA CON GAUSS-SEIDEL, JACOBI Y ELIMINACIÓN GAUSSIANA
    print("\n" + "=" * 80)
    print("4. UNIDAD 3: LOCALIZACIÓN ESPACIAL 2D (SISTEMA LINEALIZADO M*X = C)")
    print("=" * 80)

    nodos_usados = np.array([a["coordenadas"] for a in datos_reales["lecturas_anclas"]])
    A, B, M, C = ensamblar_sistema_trilateracion(nodos_usados, distancias_reales_estimadas)
    cond_M = calcular_numero_condicion(M)
    sol_scipy_xy = solve(M, C)

    semilla_xy = np.mean(nodos_usados, axis=0)
    coord_gs, hist_gs, dom_info = gauss_seidel(M, C, semilla_xy, tol=1e-6, sol_exacta=sol_scipy_xy)
    coord_jac, hist_jac = metodo_jacobi(M, C, semilla_xy, tol=1e-6, sol_exacta=sol_scipy_xy)
    coord_gauss_piv = eliminacion_gaussiana_pivoteo(M, C)

    print(f"Diagnóstico de Dominancia Diagonal : {dom_info[1]}")
    print(f"Número de Condición K_inf(M)       : {cond_M:.4f}")
    print(f"Solución por Gauss-Seidel ({len(hist_gs):2d} iter): X = {coord_gs[0]:.4f} m, Y = {coord_gs[1]:.4f} m")
    print(f"Solución por Jacobi       ({len(hist_jac):2d} iter): X = {coord_jac[0]:.4f} m, Y = {coord_jac[1]:.4f} m")
    print(f"Solución Eliminación Gaussiana     : X = {coord_gauss_piv[0]:.4f} m, Y = {coord_gauss_piv[1]:.4f} m")
    print(f"Verificación Externa (SciPy solve) : X = {sol_scipy_xy[0]:.4f} m, Y = {sol_scipy_xy[1]:.4f} m")

    print("\n--- Tabla Trazable de Iteraciones Unidad 3 (Método de Gauss-Seidel) ---")
    print(f"{'Iter':<5} | {'X_k (m)':<10} | {'Y_k (m)':<10} | {'Error Abs (m)':<14} | {'Error Rel (%)':<14} | {'Error Verd. vs SciPy':<18}")
    print("-" * 82)
    for it in hist_gs:
        print(
            f"{it['iteracion']:<5} | {it['x']:<10.5f} | {it['y']:<10.5f} | "
            f"{it['error_abs']:<14.4e} | {it['error_rel_pct']:<14.6f} | {it['error_verdadero']:<18.4e}"
        )

    # 5. TRIGGER DE CÁMARA WEB EN VIVO (OPENCV)
    print("\n" + "=" * 80)
    print("5. TRIGGER HÍBRIDO: DETECCIÓN DE MOVIMIENTO EN VIVO CON CÁMARA DE LA LAPTOP")
    print("=" * 80)
    reporte_camara = ejecutar_verificacion_visual(coord_gs, dimensiones_cuarto=dimensiones_cuarto)
    print(f"Resolución de la Cámara Web        : {reporte_camara['resolucion_camara']} ({reporte_camara['frames_analizados']} frames)")
    print(f"Área de Movimiento Detectada       : {reporte_camara['area_movimiento_px2']} px^2")
    print(f"Centroide del Movimiento en Cámara : Píxel {reporte_camara['pixel_detectado']}")
    print(f"Coordenada Física Cámara (X, Y)    : X = {reporte_camara['coordenada_visual_m'][0]:.4f} m, Y = {reporte_camara['coordenada_visual_m'][1]:.4f} m")
    print(f"Diferencia Wi-Fi vs Cámara         : {reporte_camara['discrepancia_wifi_camara_m']:.4f} m")
    print(f"Evidencia Fotográfica Guardada en  : {reporte_camara['archivo_evidencia']}")
    print(f"Estado del Sistema de Seguridad    : [{reporte_camara['estado_alerta']}]")

    generar_graficas_proyecto(
        hist_biseccion_a1,
        hist_secante_a1,
        hist_newton_a1,
        hist_gs,
        hist_jac,
        datos_reales["lecturas_anclas"],
        distancias_reales_estimadas,
        coord_gs,
        reporte_camara["coordenada_visual_m"]
    )


def generar_graficas_proyecto(h_bis, h_sec, h_nr, h_gs, h_jac, lecturas_anclas, distancias, pos_gs, pos_cam):
    """Exporta las gráficas del prototipo con las mediciones reales obtenidas."""
    plt.figure(figsize=(8, 5))
    plt.plot([i["iteracion"] for i in h_bis], [i["error_abs"] for i in h_bis], "o--", label="Bisección")
    plt.plot([i["iteracion"] for i in h_sec], [i["error_abs"] for i in h_sec], "s-", label="Secante")
    plt.plot([i["iteracion"] for i in h_nr], [i["error_abs"] for i in h_nr], "^-", label="Newton-Raphson")
    plt.yscale("log")
    plt.title("Convergencia en Estimación de Distancia Wi-Fi Real (Unidad 2)")
    plt.xlabel("Número de Iteración (k)")
    plt.ylabel("Error Absoluto Aproximado (m) [Escala Log]")
    plt.grid(True, which="both", linestyle="--", alpha=0.6)
    plt.legend()
    plt.tight_layout()
    plt.savefig("grafica_1_convergencia_no_lineal.png", dpi=300)
    plt.close()

    plt.figure(figsize=(8, 5))
    plt.plot([i["iteracion"] for i in h_gs], [i["error_rel_pct"] for i in h_gs], "b-o", label="Gauss-Seidel")
    plt.plot([i["iteracion"] for i in h_jac], [i["error_rel_pct"] for i in h_jac], "r--s", label="Jacobi")
    plt.yscale("log")
    plt.title("Convergencia en Trilateración Real: Gauss-Seidel vs. Jacobi (Unidad 3)")
    plt.xlabel("Número de Iteración (k)")
    plt.ylabel("Error Relativo Aproximado (%) [Escala Log]")
    plt.grid(True, which="both", linestyle="--", alpha=0.6)
    plt.legend()
    plt.tight_layout()
    plt.savefig("grafica_2_convergencia_sistemas_lineales.png", dpi=300)
    plt.close()

    fig, ax = plt.subplots(figsize=(7, 7))
    colores = ["#1f77b4", "#ff7f0e", "#2ca02c", "#9467bd"]
    for idx, lectura in enumerate(lecturas_anclas):
        ax_x, ax_y = lectura["coordenadas"]
        color = colores[idx % len(colores)]
        circulo = plt.Circle(
            (ax_x, ax_y),
            distancias[idx],
            color=color,
            fill=False,
            linestyle="--",
            alpha=0.7,
            label=f"{lectura['id_ancla']} ({lectura['rssi_efectivo_dbm']:.1f} dBm -> {distancias[idx]:.2f}m)"
        )
        ax.add_patch(circulo)
        ax.plot(ax_x, ax_y, "ks", markersize=9)
        ax.text(ax_x + 0.15, ax_y + 0.15, lectura["id_ancla"], fontsize=8, fontweight="bold")

    ax.plot(pos_gs[0], pos_gs[1], "ro", markersize=10, label=f"Posición Wi-Fi Gauss-Seidel ({pos_gs[0]:.2f}, {pos_gs[1]:.2f}) m")
    ax.plot(pos_cam[0], pos_cam[1], "b^", markersize=10, label=f"Detección Cámara Real ({pos_cam[0]:.2f}, {pos_cam[1]:.2f}) m")

    ax.set_xlim(-3, 9)
    ax.set_ylim(-3, 9)
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("Mapa Espacial de Trilateración con Señales Wi-Fi Reales y Cámara")
    ax.set_xlabel("Coordenada Espacial X (m)")
    ax.set_ylabel("Coordenada Espacial Y (m)")
    ax.grid(True, linestyle=":", alpha=0.7)
    ax.legend(loc="upper right", fontsize=8)
    plt.tight_layout()
    plt.savefig("grafica_3_mapa_trilateracion_iot.png", dpi=300)
    plt.close()

    print("\n[OK] Se generaron exitosamente los archivos reales del proyecto:")
    print("     1. grafica_1_convergencia_no_lineal.png")
    print("     2. grafica_2_convergencia_sistemas_lineales.png")
    print("     3. grafica_3_mapa_trilateracion_iot.png")
    print("     4. captura_camara_real.png (Fotografía real tomada con tu webcam)")


if __name__ == "__main__":
    main()