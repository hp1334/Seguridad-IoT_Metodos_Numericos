# src/metodos_numericos/errores.py
"""
Módulo de la Unidad 1: Teoría de Errores y estabilidad matricial.
Implementación propia conforme a los requisitos del prototipo computacional.
"""
import numpy as np


def calcular_epsilon_maquina():
    """
    Determina experimentalmente el épsilon de máquina en precisión doble (IEEE 754)
    mediante divisiones sucesivas sin usar constantes predefinidas.

    Returns:
        float: Épsilon de máquina aproximado.
    """
    eps = 1.0
    while (1.0 + eps / 2.0) > 1.0:
        eps /= 2.0
    return eps


def error_absoluto(valor_actual, valor_anterior):
    """Calcula el error absoluto entre dos aproximaciones o frente al valor exacto."""
    return float(np.linalg.norm(np.asarray(valor_actual) - np.asarray(valor_anterior)))


def error_relativo_porcentual(valor_actual, valor_anterior):
    """Calcula el error relativo aproximado porcentual controlando división por cero."""
    norma_actual = float(np.linalg.norm(np.asarray(valor_actual)))
    diff = error_absoluto(valor_actual, valor_anterior)
    if norma_actual < 1e-15:
        return diff * 100.0
    return (diff / norma_actual) * 100.0


def norma_infinito_matriz(M):
    """Calcula manualmente la norma infinito de una matriz (máxima suma absoluta por fila)."""
    max_suma = 0.0
    for fila in M:
        suma_fila = sum(abs(val) for val in fila)
        if suma_fila > max_suma:
            max_suma = suma_fila
    return float(max_suma)


def calcular_numero_condicion(M):
    """
    Calcula el número de condición K(M) = ||M||_inf * ||M^-1||_inf.
    Evalúa la sensibilidad del sistema lineal ante perturbaciones del RSSI/CSI.

    Args:
        M (np.ndarray): Matriz cuadrada del sistema.

    Returns:
        float: Número de condición en norma infinito.
    """
    try:
        M_inv = np.linalg.inv(M)
        return norma_infinito_matriz(M) * norma_infinito_matriz(M_inv)
    except np.linalg.LinAlgError:
        return float("inf")


def verificar_diagonal_dominante(M):
    """
    Comprueba si una matriz cumple la condición suficiente de convergencia
    de ser estrictamente diagonal dominante: |m_ii| > sum(|m_ij|) para j != i.

    Args:
        M (np.ndarray): Matriz cuadrada de coeficientes.

    Returns:
        tuple: (bool, str) Indicador de cumplimiento y mensaje diagnóstico.
    """
    n = len(M)
    for i in range(n):
        diag = abs(M[i][i])
        suma_resto = sum(abs(M[i][j]) for j in range(n) if j != i)
        if diag <= suma_resto:
            return False, (
                f"Fila {i} no es estrictamente dominante: "
                f"|{diag:.2f}| <= {suma_resto:.2f}"
            )
    return True, "La matriz es estrictamente diagonal dominante (convergencia garantizada)."