# Seguridad IoT — Métodos Numéricos (versión Fedora revisada)

Proyecto en Python con dos modos separados. **No se fabrican lecturas Wi-Fi, fotografías ni ubicaciones.** El modo diagnóstico permite probar Fedora antes de disponer de anclas verificadas. La localización solo se habilita con medidas en dBm y coordenadas de anclas aportadas explícitamente.

## Preparación (Fedora 43)

Desde la carpeta principal del proyecto:

```bash
sudo dnf install v4l-utils iw NetworkManager
python3 -m venv .venv                   # solo si aún no existe
source .venv/bin/activate
python -m pip install -r requirements.txt
v4l2-ctl --list-devices
ls -l /dev/video*
iw dev                                  # identifica la interfaz Wi-Fi
```

Si ya tienes `.venv`, **no hace falta recrearlo**. Si `python3` no es el de tu entorno, utiliza `python` después de activar `.venv`. No ejecutes los scripts con `sudo` como primera opción. Fedora debe poder acceder al hardware desde tu sesión de usuario. Revisa los permisos de cámara y la aplicación que pudiera estar ocupándola.

## A. Prueba aislada de cámara: primer objetivo

El error `can't open file ... diagnostico_camara_fedora.py` indica que ese archivo no está en el directorio desde el que lo ejecutaste. Ahora se incluye en este paquete.

```bash
python diagnostico_camara_fedora.py --listar
python diagnostico_camara_fedora.py --indice 0 --mostrar --segundos 10
```

La prueba guarda un **fotograma real** en `evidencia_camara_fedora.png` y muestra contadores de fotogramas. Si `/dev/video0` no entrega video, prueba `--indice 1`. `/dev/video1` puede ser un dispositivo de metadatos y no necesariamente un segundo sensor. Si el vídeo se lee pero falla la ventana, vuelve a ejecutarlo sin `--mostrar` para aislar el problema de la interfaz gráfica.

## B. Diagnóstico completo con Wi-Fi y cámara, sin mediciones inventadas

```bash
python main.py --modo diagnostico --indice-camara 0 --camara-frames 180 \
    --camara-segundos 20 --salida resultados
```

Si no existe una sesión gráfica disponible:

```bash
python main.py --modo diagnostico --sin-ventana --salida resultados
```

Produce `resultados/resultados.json`, `resultados/reporte_ejecucion.txt` y, si captura video, `resultados/captura_camara_real.png`.

**Nota:** `nmcli` entrega `SIGNAL (%)`, no necesariamente RSSI en dBm. El programa no falsifica dBm con una regla lineal. En este modo no hace trilateración ni genera gráficas de posición.

## C. Localización real: preparación necesaria

Para utilizar el método numérico del anteproyecto necesitas **un único receptor** (por ejemplo, la laptop) que mida las señales RSSI de **los mismos puntos de acceso fijos** en ubicaciones conocidas. No basta con detectar tres o cuatro redes cualesquiera y asociarlas arbitrariamente a las esquinas.

1. Crea `anclas.json`, sustituyendo los valores de ejemplo siguientes por las **BSSID reales** y las posiciones **medidas físicamente** (en metros) de *tus* puntos de acceso:

```json
[
  {"id": "AP_A", "bssid": "AA:AA:AA:AA:AA:01", "x_m": 0.0, "y_m": 0.0},
  {"id": "AP_B", "bssid": "AA:AA:AA:AA:AA:02", "x_m": 6.0, "y_m": 0.0},
  {"id": "AP_C", "bssid": "AA:AA:AA:AA:AA:03", "x_m": 0.0, "y_m": 6.0},
  {"id": "AP_D", "bssid": "AA:AA:AA:AA:AA:04", "x_m": 6.0, "y_m": 6.0}
]
```

**Estos BSSID y coordenadas son solamente una plantilla de formato, NO son observaciones de tu equipo y NO deben emplearse como evidencia.** Las posiciones no se leen de Wi-Fi; las debe medir la persona que instala las anclas.

2. Identifica la interfaz Wi-Fi con `iw dev` (por ejemplo, `wlp0s20f3`). La utilidad `iw` puede ofrecer los niveles `signal: -xx dBm` por cada BSSID. Algunas tarjetas, configuraciones o permisos no permiten escanear; si `iw dev INTERFAZ scan` falla, el error quedará visible.

3. Usa el `id` de una ancla calibrada a una distancia **medida** `--d-ref`, y proporciona los parámetros reales o experimentales `--pt-dbm`, `--pl-d0` y `--alpha`. También define un intervalo físico de búsqueda `--bis-a` y `--bis-b` donde esperas que estén las distancias.

```bash
python main.py --modo localizacion --anclas anclas.json --fuente iw \
    --interfaz NOMBRE_INTERFAZ --barridos 4 \
    --ancla-calibracion AP_A --d-ref DISTANCIA_MEDIDA \
    --pt-dbm POTENCIA_TX --pl-d0 PERDIDA_D0 --alpha ABSORCION \
    --bis-a LIMITE_INFERIOR --bis-b LIMITE_SUPERIOR \
    --indice-camara 0 --salida resultados
```

Los textos en mayúsculas se deben sustituir por números o nombres reales; no es un comando listo para ejecutar sin calibrar el experimento.

Si ya tienes un archivo con mediciones reales de RSSI dBm de las anclas, úsalo sin `iw`:

```bash
python main.py --modo localizacion --anclas anclas.json --fuente json \
    --lecturas lecturas.json --ancla-calibracion AP_A \
    --d-ref DISTANCIA_MEDIDA --pt-dbm POTENCIA_TX \
    --pl-d0 PERDIDA_D0 --alpha ABSORCION \
    --bis-a LIMITE_INFERIOR --bis-b LIMITE_SUPERIOR --salida resultados
```

La estructura de `lecturas.json` es:

```json
[
  {"bssid": "AA:AA:AA:AA:AA:01", "rssi_dbm": -55.0, "sigma_rssi_db": 1.4},
  {"bssid": "AA:AA:AA:AA:AA:02", "rssi_dbm": -62.0, "sigma_rssi_db": 1.1},
  {"bssid": "AA:AA:AA:AA:AA:03", "rssi_dbm": -58.0, "sigma_rssi_db": 1.7},
  {"bssid": "AA:AA:AA:AA:AA:04", "rssi_dbm": -60.0, "sigma_rssi_db": 1.5}
]
```

**Ejemplo de formato únicamente: valores ficticios. No deben utilizarse para evidencias ni para afirmaciones de una práctica real.** Introduce solo tus mediciones. Si no hay muestras repetidas para estimar `sigma_rssi_db`, omite ese campo; el cálculo de incertidumbre quedará como no disponible.

## Calibración de cámara y limitaciones

La posición `pixel_detectado` solo es un punto en la imagen. El programa no convierte automáticamente la esquina de una imagen en metros del cuarto. Para obtener posiciones métricas, prepara un archivo `camara_calibracion.json` con al menos cuatro correspondencias `pixel` a `metros` sobre **un mismo plano físico**:

```json
{
  "correspondencias": [
    {"pixel": [U1, V1], "metros": [X1, Y1]},
    {"pixel": [U2, V2], "metros": [X2, Y2]},
    {"pixel": [U3, V3], "metros": [X3, Y3]},
    {"pixel": [U4, V4], "metros": [X4, Y4]}
  ]
}
```

Sustituye las letras por mediciones propias. Activa con `--calibracion-camara camara_calibracion.json`. Si no existe una detección o una calibración, el programa devuelve `null` para las coordenadas métricas. **Una detección de movimiento no identifica a una persona específica y no confirma que esa persona sea el dispositivo medido por Wi-Fi.** Una cámara inclinada de laptop no ofrece por sí sola una homografía válida para personas a diferentes alturas y profundidades.

## Archivos de evidencia

- `resultados/reporte_ejecucion.txt`: registro de estados y errores.
- `resultados/resultados.json`: salida completa, medidas de entrada, trazabilidad y flags de error.
- `resultados/lecturas_adquiridas.json`: RSSI adquiridos para las anclas, si se llegó a medir.
- `resultados/captura_camara_real.png`: imagen real de webcam, si fue posible leer frames.
- `resultados/grafica_1_convergencia_no_lineal.png`: métodos de raíces (solo localización).
- `resultados/grafica_2_convergencia_sistemas_lineales.png`: Gauss-Seidel / Jacobi (solo localización).
- `resultados/grafica_3_mapa_trilateracion_iot.png`: mapa de anclas registradas (solo localización).

Si la cámara falla, el cálculo o el inventario Wi-Fi puede continuar y se conserva el registro. Las salidas fallidas no se etiquetan como resultados exitosos.

## Pruebas matemáticas independientes

```bash
python -m unittest discover -s tests -v
```

Estas pruebas de control utilizan datos matemáticos *de prueba*, no evidencias de radiofrecuencia. No se requiere webcam ni red para realizarlas.

## Lo que sigue pendiente científicamente

- El archivo `wifi_csi_sniffer.py` captura inventario de redes y RSSI mediante `iw`. **No realiza CSI**, aunque el nombre del módulo se mantuvo por compatibilidad con el repositorio anterior.
- El error respecto a SciPy valida algoritmos numéricos pero no es error de posicionamiento absoluto contra un sistema de verdad terreno.
- La trilateración estima la posición del **receptor Wi-Fi** frente a anclas fijas, no la posición de una persona observada por la cámara a menos que se diseñe expresamente un experimento con un objetivo común y se valide la geometría.
- El prototipo necesita verificación de mediciones de energía/potencia, fuentes RSSI y parámetros del canal para convertirse en una prueba física reproducible.
