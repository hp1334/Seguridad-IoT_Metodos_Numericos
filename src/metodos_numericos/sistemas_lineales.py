# src/metodos_numericos/sistemas_lineales.py
"""
Módulo de la Unidad 3: Resolución de Sistemas de Ecuaciones Lineales.
Incluye Gauss-Seidel, Jacobi y Eliminación Gaussiana con Pivoteo Parcial.
"""
import numpy as np
from src.metodos_numericos.errores import (
    error_absoluto,
    error_relativo_porcentual,
    verificar_diagonal_dominante,
)


def gauss_seidel(M, C, x0, tol=1e-6, max_iter=100, sol_exacta=None):
    """
    Resuelve el sistema lineal MX = C mediante el método iterativo de Gauss-Seidel.

    Args:
        M (np.ndarray): Matriz cuadrada de coeficientes.
        C (np.ndarray): Vector de términos independientes.
        x0 (np.ndarray): Vector inicial semilla.
        tol (float): Tolerancia de parada para el error relativo.
        max_iter (int): Máximo de iteraciones permitidas.
        sol_exacta (np.ndarray, optional): Solución de referencia para error verdadero.

    Returns:
        tuple: (vector_solucion, historial_iteraciones, diagnostico_dominancia)
    """
    n = len(C)
    if np.any(np.abs(np.diag(M)) < 1e-15):
        raise ZeroDivisionError("Cero detectado en la diagonal principal de la matriz M.")

    es_dom, msg_dom = verificar_diagonal_dominante(M)
    xk = np.array(x0, dtype=float)
    historial = []

    for k in range(1, max_iter + 1):
        xk_nuevo = np.copy(xk)
        for i in range(n):
            suma_previa = sum(M[i][j] * xk_nuevo[j] for j in range(i))
            suma_posterior = sum(M[i][j] * xk[j] for j in range(i + 1, n))
            xk_nuevo[i] = (C[i] - suma_previa - suma_posterior) / M[i][i]

        err_abs = error_absoluto(xk_nuevo, xk)
        err_rel = error_relativo_porcentual(xk_nuevo, xk)
        err_verd = error_absoluto(xk_nuevo, sol_exacta) if sol_exacta is not None else None

        historial.append({
            "iteracion": k,
            "x": float(xk_nuevo[0]),
            "y": float(xk_nuevo[1]),
            "error_abs": err_abs,
            "error_rel_pct": err_rel,
            "error_verdadero": err_verd
        })

        if err_abs < tol:
            return xk_nuevo, historial, (es_dom, msg_dom)

        xk = xk_nuevo

    raise RuntimeError(f"Gauss-Seidel no convergió tras {max_iter} iteraciones.")


def metodo_jacobi(M, C, x0, tol=1e-6, max_iter=100, sol_exacta=None):
    """
    Resuelve el sistema lineal MX = C mediante el método iterativo de Jacobi
    para comparación de velocidad de convergencia frente a Gauss-Seidel.
    """
    n = len(C)
    if np.any(np.abs(np.diag(M)) < 1e-15):
        raise ZeroDivisionError("Cero detectado en la diagonal principal de la matriz M.")

    xk = np.array(x0, dtype=float)
    historial = []

    for k in range(1, max_iter + 1):
        xk_nuevo = np.zeros_like(xk)
        for i in range(n):
            suma = sum(M[i][j] * xk[j] for j in range(n) if j != i)
            xk_nuevo[i] = (C[i] - suma) / M[i][i]

        err_abs = error_absoluto(xk_nuevo, xk)
        err_rel = error_relativo_porcentual(xk_nuevo, xk)
        err_verd = error_absoluto(xk_nuevo, sol_exacta) if sol_exacta is not None else None

        historial.append({
            "iteracion": k,
            "x": float(xk_nuevo[0]),
            "y": float(xk_nuevo[1]),
            "error_abs": err_abs,
            "error_rel_pct": err_rel,
            "error_verdadero": err_verd
        })

        if err_abs < tol:
            return xk_nuevo, historial

        xk = xk_nuevo

    raise RuntimeError(f"Jacobi no convergió tras {max_iter} iteraciones.")


def eliminacion_gaussiana_pivoteo(M, C):
    """
    Resuelve el sistema lineal MX = C mediante Eliminación Gaussiana con Pivoteo Parcial.
    Implementación directa para contrastar con métodos iterativos.
    """
    n = len(C)
    Ab = np.hstack([M.astype(float), C.reshape(-1, 1).astype(float)])

    # Eliminación hacia adelante con pivoteo parcial
    for i in range(n):
        max_row = i + np.argmax(np.abs(Ab[i:, i]))
        if abs(Ab[max_row, i]) < 1e-15:
            raise ValueError("El sistema es singular o tiene infinitas soluciones.")
        if max_row != i:
            Ab[[i, max_row]] = Ab[[max_row, i]]

        for j in range(i + 1, n):
            factor = Ab[j, i] / Ab[i, i]
            Ab[j, i:] -= factor * Ab[i, i:]

    # Sustitución hacia atrás
    x = np.zeros(n, dtype=float)
    for i in range(n - 1, -1, -1):
        x[i] = (Ab[i, -1] - np.dot(Ab[i, i + 1:n], x[i + 1:n])) / Ab[i, i]

    return x