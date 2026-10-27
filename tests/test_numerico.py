"""Pruebas reproducibles de cálculo y adquisición textual (sin hardware)."""
import math
import unittest
import numpy as np
from src.metodos_numericos.raices import epsilon_maquina, biseccion, secante, newton_raphson
from src.metodos_numericos.sistemas_lineales import gauss_seidel, jacobi, eliminacion_gaussiana
from src.telematica.posicionamiento import normalizar_mac, extraer_iw, analizar_linea_esp32

class PruebasProyecto(unittest.TestCase):
    def test_epsilon(self):
        self.assertEqual(epsilon_maquina(), 2**-52)

    def test_raices(self):
        f=lambda x:x*x-4
        d=lambda x:2*x
        for raiz,h in (biseccion(f,0,3),secante(f,1,3),newton_raphson(f,d,3)):
            self.assertAlmostEqual(raiz,2,places=5)
            self.assertGreater(len(h),0)

    def test_lineales(self):
        A=np.array([[4.,1.],[1.,4.]])
        b=np.array([9.,6.])
        for x in (gauss_seidel(A,b,[0.,0.])[0],jacobi(A,b,[0.,0.])[0],eliminacion_gaussiana(A,b)):
            np.testing.assert_allclose(x,[2.,1.],atol=1e-5)

    def test_mac_normalizada(self):
        self.assertEqual(normalizar_mac('aa-bb-cc-dd-ee-ff'),'AA:BB:CC:DD:EE:FF')
        with self.assertRaises(ValueError): normalizar_mac('NO-MAC')

    def test_protocolo_esp32(self):
        self.assertEqual(analizar_linea_esp32('SNIFF,aa:bb:cc:dd:ee:ff,-64'),
                         ('AA:BB:CC:DD:EE:FF', -64))
        self.assertIsNone(analizar_linea_esp32('ESP32_LISTO,CANAL,6'))
        self.assertIsNone(analizar_linea_esp32('SNIFF,AA:BB:CC:DD:EE:FF,18'))

    def test_iw(self):
        txt='BSS aa:bb:cc:dd:ee:ff(on wlan0)\n\tsignal: -61.00 dBm\n'
        self.assertEqual(extraer_iw(txt)['AA:BB:CC:DD:EE:FF'],[-61.0])

if __name__ == '__main__':
    unittest.main()
