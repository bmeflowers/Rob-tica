# ARUco Distance & Rotation Detector

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

1. Instala las dependencias:

```bash
pip install -r requirements.txt
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
├── aruco_distance_rotation.py    # Script principal
├── camera_calibration.py          # Herramienta de calibración
├── requirements.txt               # Dependencias
└── README.md                      # Este archivo
```

## Recursos Adiciales

- **Generador de marcadores ARUco**: https://chev.me/arucogen/
- **Patrón de calibración**: https://markhedleyjones.com/projects/calibration-checkerboard-collection
- **Documentación OpenCV ARUco**: https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html

## Notas Técnicas

- El sistema usa el diccionario ARUco 6x6_250 (250 marcadores únicos)
- Las mediciones son relativas a la cámara
- La precisión depende de la calibración y condiciones de iluminación
- Los ángulos de Euler pueden sufrir gimbal lock en ciertas orientaciones
