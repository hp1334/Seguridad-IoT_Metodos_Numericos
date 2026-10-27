"""Unidad 1 y Unidad 2: errores y métodos de raíces programados sin SciPy."""
from __future__ import annotations
import math
import numpy as np


def epsilon_maquina():
    eps = 1.0
    while 1.0 + eps/2.0 > 1.0:
        eps /= 2.0
    return eps


def error_absoluto(actual, anterior):
    return float(np.linalg.norm(np.asarray(actual, float)-np.asarray(anterior, float)))


def error_relativo_porcentual(actual, anterior):
    norma = float(np.linalg.norm(np.asarray(actual, float)))
    error = error_absoluto(actual, anterior)
    return 100.0*error/norma if norma > 1e-15 else (0.0 if error < 1e-15 else math.inf)


def _fila(k, x, fx, anterior, referencia=None):
    ea = error_absoluto(x, anterior)
    return {"iteracion": k, "aproximacion": float(x), "f_x": float(fx),
            "error_abs": ea, "error_rel_pct": error_relativo_porcentual(x, anterior),
            "error_verdadero": (error_absoluto(x, referencia) if referencia is not None else None)}


def biseccion(f, a, b, tol=1e-6, max_iter=100, referencia=None):
    """Raíz por intervalo con cambio de signo y trazabilidad de error."""
    if not 0 < tol or max_iter < 1 or a >= b:
        raise ValueError('Tolerancia, límite de iteraciones o intervalo no válido.')
    fa, fb = f(a), f(b)
    if fa == 0: return float(a), [_fila(0, a, fa, a, referencia)]
    if fb == 0: return float(b), [_fila(0, b, fb, b, referencia)]
    if fa*fb > 0:
        raise ValueError('Bisección: f(a) y f(b) tienen el mismo signo.')
    historia = []
    anterior = a
    for k in range(1, max_iter+1):
        medio = (a+b)/2
        fm = f(medio)
        fila = _fila(k, medio, fm, anterior, referencia)
        if k == 1:
            fila['error_abs'] = (b-a)/2
            fila['error_rel_pct'] = None
        historia.append(fila)
        if fm == 0 or (b-a)/2 <= tol:
            return float(medio), historia
        if fa*fm < 0:
            b, fb = medio, fm
        else:
            a, fa = medio, fm
        anterior = medio
    raise RuntimeError(f'Bisección: no convergió en {max_iter} iteraciones.')


def secante(f, x0, x1, tol=1e-6, max_iter=100, referencia=None):
    """Método de la Secante (no utiliza librerías externas de resolución)."""
    if tol <= 0 or max_iter < 1 or x0 == x1:
        raise ValueError('Secante: semillas, tolerancia o límite no válidos.')
    historia = []
    for k in range(1, max_iter+1):
        f0, f1 = f(x0), f(x1)
        delta = f1 - f0
        if abs(delta) <= 1e-15:
            raise ZeroDivisionError('Secante: pendiente nula.')
        x2 = x1 - f1*(x1-x0)/delta
        fx2 = f(x2)
        if not (math.isfinite(x2) and math.isfinite(fx2)):
            raise ValueError('Secante: aproximación no finita.')
        fila = _fila(k, x2, fx2, x1, referencia)
        historia.append(fila)
        if fila['error_abs'] <= tol or fx2 == 0:
            return float(x2), historia
        x0, x1 = x1, x2
    raise RuntimeError(f'Secante: no convergió en {max_iter} iteraciones.')


def newton_raphson(f, df, x0, tol=1e-6, max_iter=100, referencia=None):
    """Newton-Raphson usando la derivada proporcionada por el modelo."""
    if tol <= 0 or max_iter < 1:
        raise ValueError('Newton: tolerancia o límite inválido.')
    historia = []
    for k in range(1, max_iter+1):
        pendiente = df(x0)
        if abs(pendiente) <= 1e-15:
            raise ZeroDivisionError('Newton: derivada nula.')
        x1 = x0 - f(x0)/pendiente
        fx1 = f(x1)
        if not (math.isfinite(x1) and math.isfinite(fx1)):
            raise ValueError('Newton: aproximación no finita.')
        fila = _fila(k, x1, fx1, x0, referencia)
        historia.append(fila)
        if fila['error_abs'] <= tol or fx1 == 0:
            return float(x1), historia
        x0 = x1
    raise RuntimeError(f'Newton: no convergió en {max_iter} iteraciones.')
