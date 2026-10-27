# Sistema de Monitoreo de Dispositivos IoT Aplicando Métodos Numéricos

**ESPOCH · Ingeniería Telemática · Investigación Formativa, Fase 1 · Grupo 1**

Código fuente del prototipo de adquisición Wi-Fi/ESP32, métodos numéricos y verificación de movimiento con webcam. Funciona desde la terminal en Windows y Fedora; en el navegador local se muestran las mediciones y las gráficas generadas durante cada ejecución.

## 1. Estructura del repositorio

```text
Seguridad-IoT_Metodos_Numericos/
├── main.py                        # Orquestación, cámara continua y panel HTML
├── src/
│   ├── __init__.py
│   ├── metodos_numericos/
│   │   ├── __init__.py
│   │   ├── raices.py              # Unidad 1: errores; Unidad 2: Bisección, Secante, Newton
│   │   └── sistemas_lineales.py   # Unidad 3: Gauss-Seidel, Jacobi, pivoteo, condicionamiento
│   └── telematica/
│       ├── __init__.py
│       └── posicionamiento.py     # Wi-Fi, ESP32, modelo RSSI y trilateración
├── esp32/ESP32_Ancla_IoT/
│   ├── ESP32_Ancla_IoT.ino
│   └── configuracion_local.example.h
├── tests/                         # Verificación de algoritmos y lectura SNIFF
├── requirements.txt
├── .gitignore
└── README.md
```

La carpeta `resultados/` se crea al ejecutar; no se publica en GitHub. Los datos experimentales históricos, credenciales, listas de redes y las configuraciones físicas de cada laboratorio se proporcionan aparte de este repositorio.

## 2. Requisitos

- Python 3 compatible con las versiones de `requirements.txt`, cámara USB/integrada y, para el enlace serie, placa ESP32 conectada por USB.
- Fedora/Linux: herramientas `NetworkManager` (`nmcli`), `iw` e `ip`, disponibles según permisos y modelo de tarjeta.
- Windows: comandos integrados `netsh` y `arp`. La localización con RSSI a partir de lecturas previas se realiza mediante CSV.
- Arduino IDE con soporte **ESP32 by Espressif Systems** para cargar el sketch; USB y controlador adecuados a la placa.

### Fedora

```bash
cd ~/Seguridad-IoT_Metodos_Numericos
python3 -m venv .venv        # solo la primera vez
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m unittest discover -s tests -v
```

### Windows (PowerShell)

```powershell
cd Seguridad-IoT_Metodos_Numericos
py -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python -m unittest discover -s tests -v
```

Si la sesión de Fedora ya contiene `.venv`, actívalo: no necesitas crear otro. Para ver los puertos: `python -m serial.tools.list_ports -v`. Para conocer la interfaz inalámbrica: `iw dev`.

## 3. ESP32: cargar el firmware del proyecto

1. Abre `esp32/ESP32_Ancla_IoT/ESP32_Ancla_IoT.ino` en Arduino IDE.
2. Copia `esp32/ESP32_Ancla_IoT/configuracion_local.example.h` a `esp32/ESP32_Ancla_IoT/configuracion_local.h` y configura **tu propia** red (SSID), contraseña local de al menos 8 caracteres y canal de trabajo. Este archivo queda excluido mediante `.gitignore`.
3. Selecciona el modelo de placa y el puerto serie; compila y sube el sketch.
4. Comprueba en Monitor Serie, **115200 baudios**, la línea `ESP32_LISTO,CANAL,N`. Cuando haya tráfico compatible en ese canal, se emitirán tramas `SNIFF,MAC,RSSI`, una por línea.
5. **Cierra el Monitor Serie** antes de abrir el puerto con Python. Para Fedora usa el puerto identificado por el sistema, no asumas que siempre será `/dev/ttyUSB0`.

El ESP32 escucha el canal configurado. Las MAC observadas son campos de tramas inalámbricas que pueden estar aleatorizados; el usuario debe identificar qué transmisor corresponde al objetivo de su ensayo. El firmware registra RSSI por trama, sin alterar su formato original.

## 4. Ejecución normal: monitoreo continuo

Después de activar `.venv`:

```bash
python main.py --interfaz wlp0s20f3 --puerto-esp32 PUERTO_REAL --indice-camara 0
```

En Windows omite `--interfaz` o indícala según el equipo y sustituye `PUERTO_REAL` por el `COM` correcto. El puerto y la interfaz no están fijados dentro del código.

El programa realiza inventario Wi-Fi, consulta vecinos LAN, adquiere señales del ESP32 si está disponible y muestra la **webcam en un panel HTML servido en `127.0.0.1`**. La detección óptica calcula actividad y centroide en **píxeles**. Continúa hasta `Ctrl+C`; de manera predeterminada guarda solo fotogramas periódicos y CSV, no un video ilimitado.

Para guardar video de una sesión consentida: `--grabar-video`. Para limitar duración: `--duracion-vivo SEGUNDOS`. Para usar webcam de forma temporal: `--camara-temporal`. Para omitir hardware: `--sin-camara --sin-esp32 --sin-abrir-panel`.

**Cada ejecución crea una subcarpeta `resultados/ensayo_FECHA_UTC/`.** El programa imprime la ruta del HTML y guarda automáticamente las evidencias sin sobrescribir ensayos anteriores.

## 5. Parámetros necesarios para obtener coordenadas

La trilateración es configurable desde la línea de comandos. El operador debe proporcionar, para **tres o más anclas no colineales**, los identificadores BSSID y sus posiciones métricas medidas en el recinto, RSSI en dBm del mismo receptor y los parámetros de propagación correspondientes.

| Parámetro | Debe ingresar el operador |
|---|---|
| `--ancla ID,BSSID,X,Y` | Repetir por cada AP de posición conocida; X e Y en metros |
| `--ancla-calibracion` | ID del AP utilizado para calibrar el exponente de pérdida |
| `--d-ref` | Distancia física medida desde el receptor al AP de calibración (m) |
| `--pt-dbm` | Potencia de transmisión de las anclas documentada (dBm) |
| `--pl-d0`, `--d0` | Pérdida de referencia (dB) y distancia de referencia base (m) |
| `--alpha` | Atenuación específica del modelo para el recinto (dB/m) |
| `--bis-a`, `--bis-b` | Límites positivos de búsqueda de distancias (m) |
| `--lecturas` | CSV con RSSI medido previamente; en Fedora puede omitirse para usar `iw` |
| `--tol`, `--max-iter` | Tolerancia numérica y máximo de iteraciones |

```bash
python main.py \
  --sin-camara --sin-esp32 \
  --ancla 'A1,BSSID_1,X_1,Y_1' \
  --ancla 'A2,BSSID_2,X_2,Y_2' \
  --ancla 'A3,BSSID_3,X_3,Y_3' \
  --lecturas RUTA/lecturas.csv \
  --ancla-calibracion A1 --d-ref DISTANCIA_METROS \
  --pt-dbm POTENCIA_AP --pl-d0 PERDIDA_D0 --alpha ATENUACION \
  --bis-a DISTANCIA_MIN --bis-b DISTANCIA_MAX
```

Este bloque describe la **sintaxis de los campos**; reemplaza todos los marcadores antes de ejecutarlo. El CSV contiene cabeceras `bssid,rssi_dbm,sigma_db` (`sigma_db` es opcional). El motor usa Secante para calibración y distancias, Bisección y Newton-Raphson para comparación, y Gauss-Seidel para la solución 2D; Jacobi y eliminación con pivoteo sirven de referencia cruzada. SciPy se utiliza únicamente para contraste de resultados, no para sustituir los métodos implementados.

Si no se proporcionan anclas, el inventario y la cámara siguen disponibles y la localización se marca como **no configurada**. La posición estimada corresponde al receptor Wi-Fi que realizó las mediciones: la comparación con coordenadas de cámara requiere calibración geométrica independiente del mismo escenario.

## 6. Datos anteriores y nuevos ensayos

La aplicación no trae mediciones históricas incorporadas ni genera coordenadas prefijadas al iniciarse. Para consultar un registro anterior del grupo en el panel, se puede proporcionar explícitamente un JSON del formato documentado en el anteproyecto:

```bash
python main.py --datos-referencia RUTA/registro_ensayo.json --sin-camara --sin-esp32
```

Ese archivo **no es parte del código fuente**. Los archivos del ensayo se conservan en el informe del equipo o en almacenamiento privado y nunca se mezclan con la adquisición actual. Para una nueva sesión, actualiza el archivo de mediciones correspondiente y ejecuta los mismos algoritmos.

## 7. Resultados y pruebas

Los productos generados dependen del hardware y de las entradas presentes: `resultados.json`, `resultados_informe.txt`, `panel_resultados.html`, capturas de webcam, `camara_mediciones.csv`, gráfico de RSSI del ESP32 y, cuando exista una configuración de anclas completa, tabla de iteraciones y tres gráficas de convergencia/trilateración.

Ejecuta la batería matemática y de parsers antes de publicar cambios:

```bash
python -m unittest discover -s tests -v
python main.py --sin-camara --sin-esp32 --sin-abrir-panel
```

Las pruebas numéricas en `tests/` emplean problemas de solución conocida para detectar regresiones del software; no se incorporan como datos de una ejecución experimental. El archivo `.gitignore` excluye entornos virtuales, mediciones locales, imágenes/grabaciones y credenciales.

## 8. Documentación

El anteproyecto desarrolla el modelo de pérdida log-distancia con absorción, las ecuaciones de Secante, Gauss-Seidel, la propagación del error, la evaluación de convergencia y el cronograma de la Fase 2. El firmware ESP32 y este repositorio constituyen el prototipo de la Fase 1. Todo procesamiento sobre equipos y señales ajenos debe limitarse a entornos autorizados.
