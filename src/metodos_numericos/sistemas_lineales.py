"""Unidad 3: Jacobi, Gauss-Seidel, pivoteo y condicionamiento."""
from __future__ import annotations
import numpy as np
from .raices import error_absoluto, error_relativo_porcentual


def numero_condicion_infinito(M):
    """Número de condición de la matriz, no de las mediciones físicas."""
    A = np.asarray(M, dtype=float)
    try:
        identidad=np.eye(len(A))
        inversa=np.column_stack([eliminacion_gaussiana(A,identidad[:,i]) for i in range(len(A))])
        return float(np.linalg.norm(A, ord=np.inf)*np.linalg.norm(inversa, ord=np.inf))
    except (np.linalg.LinAlgError, ValueError):
        return float('inf')


def diagonal_dominante(M):
    A = np.asarray(M, dtype=float)
    return bool(np.all(2*np.abs(np.diag(A)) > np.sum(np.abs(A), axis=1)))


def _lineal(M, C, x0, tol, max_iter, referencia, modo):
    A, b = np.asarray(M, float), np.asarray(C, float)
    x = np.asarray(x0, float).copy()
    n = len(b)
    if A.shape != (n,n) or x.shape != (n,) or np.any(np.abs(np.diag(A)) < 1e-14):
        raise ValueError('Dimensiones incorrectas o diagonal con ceros.')
    if tol <= 0 or max_iter < 1:
        raise ValueError('Tolerancia o iteraciones no válidas.')
    hist = []
    for k in range(1, max_iter+1):
        nuevo = x.copy()
        for i in range(n):
            previo = nuevo if modo == 'gs' else x
            suma1 = np.dot(A[i,:i], previo[:i])
            suma2 = np.dot(A[i,i+1:], x[i+1:])
            nuevo[i] = (b[i]-suma1-suma2)/A[i,i]
        ea = error_absoluto(nuevo, x)
        hist.append({'iteracion':k,'aproximacion':nuevo.tolist(),
                     'error_abs':ea,'error_rel_pct':error_relativo_porcentual(nuevo,x),
                     'error_verdadero':error_absoluto(nuevo,referencia) if referencia is not None else None})
        if ea <= tol and np.linalg.norm(A@nuevo-b) <= max(1.0,float(np.linalg.norm(b)))*tol:
            return nuevo, hist
        x = nuevo
    raise RuntimeError(f'{modo}: no convergió en {max_iter} iteraciones.')


def gauss_seidel(M,C,x0,tol=1e-6,max_iter=100,referencia=None):
    return _lineal(M,C,x0,tol,max_iter,referencia,'gs')


def jacobi(M,C,x0,tol=1e-6,max_iter=100,referencia=None):
    return _lineal(M,C,x0,tol,max_iter,referencia,'jacobi')


def eliminacion_gaussiana(M,C):
    """Pivoteo parcial propio, sin recurrir a numpy.linalg.solve."""
    A = np.asarray(M,float).copy()
    b = np.asarray(C,float).copy()
    n = len(b)
    if A.shape != (n,n): raise ValueError('Sistema no cuadrado.')
    for i in range(n):
        pivote = i + int(np.argmax(np.abs(A[i:,i])))
        if abs(A[pivote,i]) < 1e-14:
            raise ValueError('Matriz singular.')
        A[[i,pivote]] = A[[pivote,i]]
        b[[i,pivote]] = b[[pivote,i]]
        for j in range(i+1,n):
            factor=A[j,i]/A[i,i]
            A[j,i:]-=factor*A[i,i:]
            b[j]-=factor*b[i]
    x=np.zeros(n)
    for i in range(n-1,-1,-1):
        x[i]=(b[i]-np.dot(A[i,i+1:],x[i+1:]))/A[i,i]
    return x
