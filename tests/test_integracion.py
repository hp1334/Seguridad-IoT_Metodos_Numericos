"""Pruebas numéricas y de interfaz sin acceder al hardware.

El caso matemático se construye en memoria y no forma parte de los datos del
programa ni de los resultados experimentales del grupo.
"""
import csv
import math
import tempfile
import unittest
from pathlib import Path
from src.telematica.posicionamiento import cargar_lecturas_csv, ecuacion_calibracion, trilateracion
from src.metodos_numericos.raices import secante
from main import opciones, localizar, crear_panel_resultados, guardar_json


class IntegracionNumerica(unittest.TestCase):
    def test_modelo_calibracion_de_varios_anclajes(self):
        geometria = [(0., 0.), (6., 0.), (0., 6.), (6., 6.)]
        ubicacion = (2.2, 3.8)
        d = [math.dist(p, ubicacion) for p in geometria]
        A, B, M, C = trilateracion(geometria, d)
        self.assertEqual(M.shape, (2, 2))
        n = 2.5
        pt, pl, alpha = 5., 40., .13
        medidas = [pt - pl - 10*n*math.log10(x) - alpha*x for x in d]
        f = ecuacion_calibracion(medidas[0], d[0], pt, pl, alpha)
        estimado, _ = secante(f, 1.5, 4.)
        self.assertAlmostEqual(estimado, n, places=8)

        with tempfile.TemporaryDirectory() as tmp:
            csv_file = Path(tmp) / 'entrada.csv'
            macs = [f'02:00:00:00:00:{i:02X}' for i in range(1, 5)]
            with csv_file.open('w', newline='', encoding='utf8') as handle:
                writer = csv.DictWriter(handle, fieldnames=['bssid', 'rssi_dbm'])
                writer.writeheader()
                for mac, rssi in zip(macs, medidas):
                    writer.writerow({'bssid': mac, 'rssi_dbm': rssi})
            argumentos = ['--sin-camara', '--sin-esp32', '--lecturas', str(csv_file),
                          '--ancla-calibracion', 'A1', '--d-ref', str(d[0]),
                          '--pt-dbm', str(pt), '--pl-d0', str(pl), '--alpha', str(alpha),
                          '--bis-a', '0.1', '--bis-b', '15.0']
            for i, (mac, (x, y)) in enumerate(zip(macs, geometria), start=1):
                argumentos += ['--ancla', f'A{i},{mac},{x},{y}']
            args = opciones(argumentos)
            salida = Path(tmp) / 'salida'
            salida.mkdir()
            res = localizar(args, salida, lambda mensaje: None, None)
            self.assertAlmostEqual(res['gauss_seidel'][0], ubicacion[0], delta=1e-4)
            self.assertAlmostEqual(res['gauss_seidel'][1], ubicacion[1], delta=1e-4)
            self.assertTrue((salida/'tabla_iteraciones_localizacion.csv').is_file())
            self.assertTrue((salida/'grafica_3_mapa_trilateracion_iot.png').is_file())
            datos = {'fecha_utc': 'prueba', 'sistema': 'unittest',
                     'etapas': {'localizacion': {'estado': 'ok', 'resultados': res}}}
            guardar_json(salida/'resultados.json', datos)
            panel = crear_panel_resultados(salida, datos)
            self.assertIn('Trilateración', panel.read_text(encoding='utf8'))
            self.assertNotIn('CONTROL ANALÍTICO', panel.read_text(encoding='utf8'))

    def test_lecturas_rechazan_errores(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / 'entrada.csv'
            p.write_text('bssid,rssi_dbm,sigma_db\n02:00:00:00:00:01,-51,-2\n', encoding='utf8')
            with self.assertRaises(ValueError):
                cargar_lecturas_csv(p)

if __name__ == '__main__':
    unittest.main()

class PanelLocal(unittest.TestCase):
    def test_servidor_local_y_estado(self):
        import urllib.request
        from main import crear_servidor_panel

        class CamaraSimulada:
            class Args:
                fps_panel = 10
            args = Args()
            def estado(self): return {'estado': 'iniciando', 'frames': 0}

        with tempfile.TemporaryDirectory() as directorio:
            carpeta = Path(directorio)
            (carpeta / 'panel_resultados.html').write_text('<h1>Panel local</h1>', encoding='utf8')
            servidor = crear_servidor_panel(carpeta, CamaraSimulada(), 0)
            try:
                self.assertEqual(servidor.server_address[0], '127.0.0.1')
                import threading
                hilo = threading.Thread(target=servidor.serve_forever, daemon=True)
                hilo.start()
                url = f'http://127.0.0.1:{servidor.server_address[1]}'
                with urllib.request.urlopen(url+'/api/estado', timeout=4) as respuesta:
                    self.assertIn('iniciando', respuesta.read().decode('utf8'))
                with urllib.request.urlopen(url+'/', timeout=4) as respuesta:
                    self.assertIn('Panel local', respuesta.read().decode('utf8'))
            finally:
                servidor.shutdown()
                servidor.server_close()
                hilo.join(timeout=3)
