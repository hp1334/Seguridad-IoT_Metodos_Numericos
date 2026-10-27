"""Pruebas UNITARIAS matematicas con datos artificiales identificados como tales.
No producen evidencia experimental del sistema IoT.
"""
import unittest
import numpy as np
from src.metodos_numericos.ecuaciones_no_lineales import metodo_biseccion, metodo_secante, metodo_newton_raphson
from src.metodos_numericos.sistemas_lineales import gauss_seidel, metodo_jacobi, eliminacion_gaussiana_pivoteo
from src.metodos_numericos.errores import calcular_epsilon_maquina
from src.telematica.wifi_csi_sniffer import analizar_iw_scan, _campos_nmcli, ensamblar_sistema_trilateracion


class Pruebas(unittest.TestCase):
    def test_epsilon(self):
        self.assertAlmostEqual(calcular_epsilon_maquina(), np.finfo(float).eps)

    def test_raices(self):
        f = lambda x: x*x - 4
        fp = lambda x: 2*x
        for r in [metodo_biseccion(f, 1, 3)[0], metodo_secante(f, 1, 3)[0],
                  metodo_newton_raphson(f, fp, 3)[0]]:
            self.assertAlmostEqual(r, 2, places=5)

    def test_lineales(self):
        M = np.array([[4., 1.], [1., 4.]])
        C = np.array([9., 6.])
        expected = [2., 1.]
        for r in [gauss_seidel(M, C, [0., 0.])[0], metodo_jacobi(M, C, [0., 0.])[0],
                  eliminacion_gaussiana_pivoteo(M, C)]:
            np.testing.assert_allclose(r, expected, atol=1e-5)

    def test_iw_parser(self):
        ejemplo = 'BSS aa:bb:cc:dd:ee:ff(on wlan0)\n\tsignal: -51.50 dBm\n\tSSID: AP_LAB'
        self.assertEqual(analizar_iw_scan(ejemplo)[0]['rssi_dbm'], -51.5)

    def test_nmcli_parser(self):
        linea = 'AA\\:BB\\:CC\\:DD\\:EE\\:FF:Red\\:Laboratorio:2437:80'
        self.assertEqual(_campos_nmcli(linea),
                         ['AA:BB:CC:DD:EE:FF', 'Red:Laboratorio', '2437', '80'])

    def test_trilateracion(self):
        puntos = np.array([[0.,0.],[6.,0.],[0.,6.],[6.,6.]])
        objetivo = np.array([2.2, 3.8])
        distancias = np.linalg.norm(puntos - objetivo, axis=1)
        A, B, M, C = ensamblar_sistema_trilateracion(puntos, distancias)
        np.testing.assert_allclose(np.linalg.solve(M, C), objetivo, atol=1e-10)


if __name__ == '__main__':
    unittest.main()
