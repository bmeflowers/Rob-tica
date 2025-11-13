# ARUco Distance & Rotation Detector
# ESTO ES UN CAMBIO 

Sistema de detección en tiempo real de marcadores ARUco que mide distancia y rotación usando la cámara del portátil.

## Características

- Detección de marcadores ARUco en tiempo real
- Medición de distancia en centímetros
- Medición de rotación (Roll, Pitch, Yaw) en grados
- Visualización de ejes 3D sobre los marcadores
- Calibración de cámara para mayor precisión

## Requisitos

- Python 3.7 o superior
- Cámara web (integrada o USB)

## Instalación

### Instalación Automática (Recomendado)

El script de instalación detecta automáticamente tu plataforma y ajusta las dependencias:

```bash
./install_dependencies.sh
```

### Instalación Manual

#### En PC/Mac/Linux (x86/x64):

```bash
pip install -r requirements.txt
```

#### En Raspberry Pi:

**Opción 1: Desde repositorios del sistema (más rápido)**
```bash
sudo apt update
sudo apt install -y python3-opencv python3-numpy
```

**Opción 2: Via pip (más lento)**
```bash
pip install opencv-python-headless numpy
```

## Uso Rápido

### 1. Genera un marcador ARUco

Ve a https://chev.me/arucogen/ e imprime un marcador con:
- Dictionary: 6x6 (250)
- Marker ID: cualquier número (ej. 0, 1, 2...)
- Marker size: 50mm (5 cm)

### 2. Ejecuta el detector

```bash
python aruco_distance_rotation.py
```

### 3. Muestra el marcador a la cámara

El sistema mostrará en tiempo real:
- **ID del marcador**
- **Distancia** en centímetros
- **Roll**: Rotación alrededor del eje X (rojo)
- **Pitch**: Rotación alrededor del eje Y (verde)
- **Yaw**: Rotación alrededor del eje Z (azul)

### Controles

- `q`: Salir del programa
- `c`: Mostrar información de calibración

## Calibración de Cámara (Opcional pero Recomendado)

Para mejor precisión en las mediciones:

### 1. Genera un tablero de ajedrez

Ve a https://markhedleyjones.com/projects/calibration-checkerboard-collection
y descarga/imprime un tablero de **6x9 esquinas internas**.

### 2. Ejecuta el script de calibración

```bash
python camera_calibration.py
```

### 3. Sigue las instrucciones en pantalla

- Opción 1: Capturar imágenes y calibrar
- Captura 15-20 imágenes mostrando el tablero desde diferentes ángulos
- Presiona ESPACIO para capturar cada imagen
- Presiona `q` cuando hayas terminado

### 4. Usa la calibración

El script generará `camera_calibration.npz`. Para usarlo, modifica `aruco_distance_rotation.py`:

```python
# Cargar calibración
calib_data = np.load('camera_calibration.npz')
CAMERA_MATRIX = calib_data['camera_matrix']
DIST_COEFFS = calib_data['dist_coeffs']
```

## Configuración del Marcador

Si usas un tamaño de marcador diferente a 5 cm, ajusta en `aruco_distance_rotation.py`:

```python
MARKER_SIZE = 0.05  # Cambiar según tu tamaño (en metros)
```

Por ejemplo:
- 3 cm → `MARKER_SIZE = 0.03`
- 10 cm → `MARKER_SIZE = 0.10`

## Interpretación de los Ángulos

- **Roll (ángulo X)**: Rotación lateral (inclinación izquierda/derecha)
- **Pitch (ángulo Y)**: Rotación hacia adelante/atrás (cabeceo)
- **Yaw (ángulo Z)**: Rotación sobre el eje perpendicular (giro horizontal)

## Visualización de Ejes

Los ejes 3D dibujados sobre el marcador:
- **Rojo**: Eje X
- **Verde**: Eje Y
- **Azul**: Eje Z

## Solución de Problemas

### Problemas con la cámara en Raspberry Pi

Si recibes el error `ModuleNotFoundError: No module named 'cv2'`:
```bash
# Instalar OpenCV
sudo apt install python3-opencv python3-numpy
```

Si recibes errores de GStreamer o no se puede leer frames:

**1. Ejecuta el script de diagnóstico:**
```bash
python3 diagnose_camera.py
```

Este script verificará:
- Si la cámara está conectada
- Permisos del usuario
- Qué backends de OpenCV funcionan
- Configuración del sistema

**2. Soluciones comunes:**

a) **Añadir usuario al grupo video:**
```bash
sudo usermod -a -G video $USER
# Luego cierra sesión y vuelve a entrar
```

b) **Para cámara oficial Raspberry Pi:**
```bash
# Habilitar cámara en raspi-config
sudo raspi-config
# Selecciona: Interface Options > Camera > Enable
# Reinicia la Raspberry Pi
```

c) **Verificar dispositivos de video:**
```bash
ls -l /dev/video*
v4l2-ctl --list-devices  # Instala con: sudo apt install v4l-utils
```

d) **Si la cámara funciona pero con lag:**
Edita `aruco_distance_rotation.py` y reduce la resolución:
```python
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)   # Reducir de 640
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)  # Reducir de 480
```

### La distancia no es precisa

1. Verifica que `MARKER_SIZE` coincida con el tamaño real impreso
2. Realiza la calibración de cámara
3. Asegúrate de que el marcador esté bien iluminado
4. Mantén el marcador plano (sin arrugas)

### No detecta el marcador

1. Verifica que el diccionario sea correcto (6x6_250)
2. Mejora la iluminación
3. Acerca el marcador a la cámara
4. Asegúrate de que el marcador sea visible completamente

### Mediciones ruidosas

1. Mejora la iluminación
2. Realiza la calibración de cámara
3. Mantén el marcador lo más estable posible
4. Usa un marcador más grande

## Estructura del Proyecto

```
.
├── aruco_distance_rotation.py    # Script principal de detección
├── camera_calibration.py          # Herramienta de calibración
├── diagnose_camera.py             # Diagnóstico de cámara (RPi)
├── install_dependencies.sh        # Instalador automático
├── requirements.txt               # Dependencias
└── README.md                      # Este archivo
```

## Notas para Raspberry Pi

- **Raspberry Pi 3/4**: El script de instalación automática está optimizado para estas plataformas
- **Rendimiento**: Considera reducir la resolución de la cámara si hay lag:
  ```python
  cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
  cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
  ```
- **Cámara Pi**: Compatible con cámaras oficiales y USB
- **OpenCV versión**: La versión desde repositorios (apt) puede ser más antigua pero es estable

## Recursos Adiciales

- **Generador de marcadores ARUco**: https://chev.me/arucogen/
- **Patrón de calibración**: https://markhedleyjones.com/projects/calibration-checkerboard-collection
- **Documentación OpenCV ARUco**: https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html

## Notas Técnicas

- El sistema usa el diccionario ARUco 6x6_250 (250 marcadores únicos)
- Las mediciones son relativas a la cámara
- La precisión depende de la calibración y condiciones de iluminación
- Los ángulos de Euler pueden sufrir gimbal lock en ciertas orientaciones
