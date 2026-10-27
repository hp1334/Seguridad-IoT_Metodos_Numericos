# src/metodos_numericos/ecuaciones_no_lineales.py
"""
Módulo de la Unidad 2: Resolución de Ecuaciones No Lineales.
Incluye Bisección, Secante y Newton-Raphson con trazabilidad de iteraciones y errores.
"""
from src.metodos_numericos.errores import error_absoluto, error_relativo_porcentual


def metodo_biseccion(func, a, b, tol=1e-6, max_iter=100, valor_exacto=None):
    """
    Resuelve f(x) = 0 en el intervalo [a, b] mediante el método de Bisección.

    Args:
        func (callable): Función no lineal f(x).
        a (float): Extremo inferior del intervalo.
        b (float): Extremo superior del intervalo.
        tol (float): Tolerancia de parada.
        max_iter (int): Número máximo de iteraciones.
        valor_exacto (float, optional): Raíz de referencia para calcular error verdadero.

    Returns:
        tuple: (raiz, historial_iteraciones)
    """
    fa, fb = func(a), func(b)
    if fa * fb >= 0:
        raise ValueError(
            f"Falla en Bisección: No existe cambio de signo en [{a}, {b}] "
            f"(f(a)={fa:.4f}, f(b)={fb:.4f})."
        )

    historial = []
    xr_ant = a

    for k in range(1, max_iter + 1):
        xr = (a + b) / 2.0
        fxr = func(xr)

        err_abs = error_absoluto(xr, xr_ant) if k > 1 else abs(b - a) / 2.0
        err_rel = error_relativo_porcentual(xr, xr_ant) if k > 1 else 100.0
        err_verd = error_absoluto(xr, valor_exacto) if valor_exacto is not None else None

        historial.append({
            "iteracion": k,
            "aproximacion": xr,
            "f_x": fxr,
            "error_abs": err_abs,
            "error_rel_pct": err_rel,
            "error_verdadero": err_verd
        })

        if abs(fxr) < 1e-12 or err_abs < tol:
            return xr, historial

        if fa * fxr < 0:
            b = xr
            fb = fxr
        else:
            a = xr
            fa = fxr

        xr_ant = xr

    raise RuntimeError(f"Bisección no convergió tras {max_iter} iteraciones.")


def metodo_secante(func, x0, x1, tol=1e-6, max_iter=100, valor_exacto=None):
    """
    Resuelve f(x) = 0 mediante el método de la Secante.

    Args:
        func (callable): Función no lineal f(x).
        x0 (float): Primera aproximación inicial.
        x1 (float): Segunda aproximación inicial.
        tol (float): Tolerancia de parada.
        max_iter (int): Límite de iteraciones.
        valor_exacto (float, optional): Raíz de referencia para error verdadero.

    Returns:
        tuple: (raiz, historial_iteraciones)
    """
    historial = []

    for k in range(1, max_iter + 1):
        f0, f1 = func(x0), func(x1)
        denominador = f1 - f0

        if abs(denominador) < 1e-15:
            raise ZeroDivisionError(
                f"Falla numérica en Secante (iteración {k}): División por cero (f(x1)-f(x0) ≈ 0)."
            )

        x2 = x1 - f1 * ((x1 - x0) / denominador)
        err_abs = error_absoluto(x2, x1)
        err_rel = error_relativo_porcentual(x2, x1)
        err_verd = error_absoluto(x2, valor_exacto) if valor_exacto is not None else None

        historial.append({
            "iteracion": k,
            "aproximacion": x2,
            "f_x": func(x2),
            "error_abs": err_abs,
            "error_rel_pct": err_rel,
            "error_verdadero": err_verd
        })

        if err_abs < tol:
            return x2, historial

        x0, x1 = x1, x2

    raise RuntimeError(f"El método de la Secante no convergió en {max_iter} iteraciones.")


def metodo_newton_raphson(func, dfunc, x0, tol=1e-6, max_iter=100, valor_exacto=None):
    """
    Resuelve f(x) = 0 mediante el método de Newton-Raphson de segundo orden.

    Args:
        func (callable): Función objetivo f(x).
        dfunc (callable): Derivada analítica f'(x).
        x0 (float): Valor semilla inicial.
        tol (float): Tolerancia de parada.
        max_iter (int): Límite de iteraciones.
        valor_exacto (float, optional): Raíz de referencia.

    Returns:
        tuple: (raiz, historial_iteraciones)
    """
    historial = []
    xk = float(x0)

    for k in range(1, max_iter + 1):
        fx = func(xk)
        dfx = dfunc(xk)

        if abs(dfx) < 1e-15:
            raise ZeroDivisionError(
                f"Falla en Newton-Raphson (iteración {k}): Derivada nula en x={xk:.4f}."
            )

        xk_nuevo = xk - (fx / dfx)
        err_abs = error_absoluto(xk_nuevo, xk)
        err_rel = error_relativo_porcentual(xk_nuevo, xk)
        err_verd = error_absoluto(xk_nuevo, valor_exacto) if valor_exacto is not None else None

        historial.append({
            "iteracion": k,
            "aproximacion": xk_nuevo,
            "f_x": func(xk_nuevo),
            "error_abs": err_abs,
            "error_rel_pct": err_rel,
            "error_verdadero": err_verd
        })

        if err_abs < tol:
            return xk_nuevo, historial

        xk = xk_nuevo

    raise RuntimeError(f"Newton-Raphson no convergió en {max_iter} iteraciones.")