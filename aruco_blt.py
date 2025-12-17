#!/usr/bin/env python3
"""
Robot Seguidor de Códigos ArUco - VERSIÓN FINAL
Basado en código funcional de detección ARUco con distancia y rotación
Raspberry Pi 4 + Arduino + HC-05 Bluetooth
Solo 2 motores (M1 y M2)

IMPORTANTE: Ejecutar primero ~/mantener_bluetooth.sh en otra terminal
"""

import cv2
import numpy as np
import serial
import time
import sys
import platform
from datetime import datetime

# Detectar si estamos en Raspberry Pi
IS_RASPBERRY_PI = platform.machine().startswith('arm') or platform.machine().startswith('aarch64')

# Intentar importar picamera2 si estamos en RPi
PICAMERA2_AVAILABLE = False
if IS_RASPBERRY_PI:
    try:
        from picamera2 import Picamera2
        PICAMERA2_AVAILABLE = True
        print("✓ Usando picamera2 para Raspberry Pi")
    except ImportError:
        print("⚠ picamera2 no disponible, usando OpenCV")
        PICAMERA2_AVAILABLE = False

# ============== CONFIGURACIÓN ==============

# Bluetooth
BLUETOOTH_PORT = '/dev/rfcomm0'
BAUD_RATE = 9600

# Configuración del marcador ARUco
ARUCO_DICT = cv2.aruco.DICT_6X6_250
MARKER_SIZE = 0.05  # Tamaño del marcador en metros (5 cm)

# Parámetros de calibración de cámara
CAMERA_MATRIX = np.array([
    [800, 0, 320],
    [0, 800, 240],
    [0, 0, 1]
], dtype=float)

DIST_COEFFS = np.zeros((5, 1))

# Parámetros de control del robot (AJUSTABLES)
DISTANCIA_OBJETIVO = 30.0       # Distancia objetivo en cm
DISTANCIA_TOLERANCIA = 5.0      # Tolerancia de distancia (±5 cm)
DISTANCIA_MUY_CERCA = 15.0      # Muy cerca - detener
DISTANCIA_MUY_LEJOS = 60.0      # Muy lejos - avanzar despacio

ANGULO_YAW_TOLERANCIA = 15.0    # Tolerancia de ángulo yaw (±15°)
VELOCIDAD_ENVIO = 0.2           # Segundos entre comandos

# Comandos
CMD_AVANZAR = 'F'
CMD_RETROCEDER = 'B'
CMD_IZQUIERDA = 'L'
CMD_DERECHA = 'R'
CMD_DETENER = 'S'
CMD_DESPACIO = 'G'
CMD_TEST = 'T'

# ============== FUNCIONES DE CÁMARA ==============

def initialize_camera_picamera2():
    """Inicializa la cámara usando picamera2 (Raspberry Pi)"""
    try:
        print("Inicializando cámara con picamera2...")
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(
            main={"size": (640, 480), "format": "RGB888"}
        )
        picam2.configure(config)
        picam2.start()
        print("✓ Cámara Raspberry Pi inicializada correctamente")
        return picam2
    except Exception as e:
        print(f"✗ Error al inicializar picamera2: {e}")
        return None


def initialize_camera_opencv():
    """Inicializa la cámara con OpenCV"""
    backends = [
        (cv2.CAP_V4L2, "V4L2 (Linux)"),
        (cv2.CAP_ANY, "Auto"),
        (0, "Default")
    ]

    print("Intentando acceder a la cámara con OpenCV...")

    for backend, name in backends:
        print(f"  Probando backend: {name}...")
        try:
            if isinstance(backend, int) and backend == 0:
                cap = cv2.VideoCapture(0)
            else:
                cap = cv2.VideoCapture(0, backend)

            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    print(f"  ✓ Cámara inicializada con {name}")
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    return cap
                else:
                    cap.release()
            else:
                cap.release()
        except Exception as e:
            print(f"  Error con {name}: {e}")
            continue

    return None


def initialize_camera():
    """Inicializa la cámara según la plataforma"""
    if PICAMERA2_AVAILABLE:
        cam = initialize_camera_picamera2()
        if cam is not None:
            return cam, True

    cam = initialize_camera_opencv()
    if cam is not None:
        return cam, False

    return None, False


# ============== FUNCIONES DE GEOMETRÍA ==============

def rotation_matrix_to_euler_angles(R):
    """Convierte matriz de rotación a ángulos de Euler"""
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0

    return np.degrees([x, y, z])


def draw_axis(img, camera_matrix, dist_coeffs, rvec, tvec, length=0.03):
    """Dibuja los ejes X, Y, Z del marcador"""
    axis_points = np.float32([
        [0, 0, 0],
        [length, 0, 0],
        [0, length, 0],
        [0, 0, length]
    ])

    image_points, _ = cv2.projectPoints(
        axis_points, rvec, tvec, camera_matrix, dist_coeffs
    )

    image_points = image_points.astype(int)
    origin = tuple(image_points[0].ravel())

    img = cv2.line(img, origin, tuple(image_points[1].ravel()), (0, 0, 255), 3)
    img = cv2.line(img, origin, tuple(image_points[2].ravel()), (0, 255, 0), 3)
    img = cv2.line(img, origin, tuple(image_points[3].ravel()), (255, 0, 0), 3)

    return img


# ============== CLASE PRINCIPAL ==============

class RobotArucoTracker:
    def __init__(self):
        self.bluetooth = None
        self.cam = None
        self.is_picamera = False
        self.detector = None
        self.ultimo_comando = None
        self.tiempo_ultimo_comando = 0
        self.modo_debug = True
        self.aruco_detectado = False
        self.tiempo_sin_aruco = time.time()
        
    def verificar_conexion_bluetooth(self):
        """Verifica que /dev/rfcomm0 exista"""
        print("=" * 50)
        print("Verificando conexión Bluetooth...")
        print("=" * 50)
        
        import os
        if not os.path.exists(BLUETOOTH_PORT):
            print(f"✗ ERROR: {BLUETOOTH_PORT} no existe")
            print()
            print("SOLUCIÓN:")
            print("1. Abre otra terminal")
            print("2. Ejecuta: ~/mantener_bluetooth.sh")
            print("3. Espera a que diga 'CONEXIÓN LISTA'")
            print("4. Vuelve aquí y ejecuta este script de nuevo")
            print()
            return False
        
        print(f"✓ {BLUETOOTH_PORT} existe")
        return True
    
    def conectar_bluetooth(self):
        """Conecta al Arduino via Bluetooth"""
        print("\nConectando al puerto serial Bluetooth...")
        
        try:
            self.bluetooth = serial.Serial(
                BLUETOOTH_PORT,
                BAUD_RATE,
                timeout=1
            )
            time.sleep(1)
            
            print("✓ Conexión serial establecida")
            
            # Comando de prueba
            self.bluetooth.write(b'S')
            time.sleep(0.5)
            
            if self.bluetooth.in_waiting > 0:
                respuesta = self.bluetooth.read(self.bluetooth.in_waiting).decode(errors='ignore')
                print(f"✓ Arduino respondió: {respuesta.strip()}")
            else:
                print("✓ Puerto serial funcional")
            
            return True
            
        except Exception as e:
            print(f"✗ Error al conectar: {e}")
            print("Verifica que ~/mantener_bluetooth.sh esté corriendo")
            return False
    
    def inicializar_camara(self):
        """Inicializa la cámara"""
        print("\nInicializando cámara...")
        
        self.cam, self.is_picamera = initialize_camera()
        
        if self.cam is None:
            print("\n❌ Error: No se pudo acceder a la cámara")
            if IS_RASPBERRY_PI:
                print("\nSoluciones:")
                print("1. sudo apt install -y python3-picamera2")
                print("2. sudo raspi-config → Interface Options → Camera → Enable")
                print("3. Reinicia la Raspberry Pi")
            return False
        
        print("✓ Cámara inicializada correctamente")
        return True
    
    def inicializar_aruco(self):
        """Inicializa el detector de ArUco"""
        print("\nInicializando detector ArUco...")
        
        try:
            aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
            aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
            
            print(f"✓ Detector ArUco listo (6X6_250)")
            print(f"  Tamaño marcador: {MARKER_SIZE * 100} cm")
            return True
            
        except Exception as e:
            print(f"✗ Error al inicializar ArUco: {e}")
            return False
    
    def enviar_comando(self, comando):
        """Envía un comando al Arduino"""
        try:
            tiempo_actual = time.time()
            if (comando == self.ultimo_comando and 
                tiempo_actual - self.tiempo_ultimo_comando < VELOCIDAD_ENVIO):
                return True
            
            self.bluetooth.write(comando.encode())
            self.bluetooth.flush()
            
            self.ultimo_comando = comando
            self.tiempo_ultimo_comando = tiempo_actual
            
            if self.modo_debug:
                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                comandos_str = {
                    'F': '↑ AVANZAR',
                    'B': '↓ RETROCEDER',
                    'L': '← IZQUIERDA',
                    'R': '→ DERECHA',
                    'S': '■ DETENER',
                    'G': '↑ DESPACIO'
                }
                print(f"[{timestamp}] {comandos_str.get(comando, comando)}")
            
            return True
            
        except Exception as e:
            print(f"✗ Error al enviar: {e}")
            return False
    
    def calcular_comando(self, distancia, yaw):
        """
        Determina comando basándose en distancia y ángulo yaw
        
        Args:
            distancia: Distancia al marcador en cm
            yaw: Ángulo de rotación en grados
        
        Returns:
            (comando, descripción, color)
        """
        
        # 1. PRIORIDAD: Distancia muy cerca → DETENER
        if distancia < DISTANCIA_MUY_CERCA:
            return CMD_DETENER, f"Muy cerca ({distancia:.1f}cm)", (0, 0, 255)
        
        # 2. ORIENTACIÓN: Corregir ángulo primero
        # Si está desalineado más de la tolerancia, girar
        if abs(yaw) > ANGULO_YAW_TOLERANCIA:
            if yaw > 0:
                return CMD_DERECHA, f"Girar DER ({yaw:.1f}°)", (255, 165, 0)
            else:
                return CMD_IZQUIERDA, f"Girar IZQ ({yaw:.1f}°)", (255, 165, 0)
        
        # 3. DISTANCIA: Ajustar distancia al objetivo
        diff_distancia = distancia - DISTANCIA_OBJETIVO
        
        # Muy lejos → Avanzar despacio
        if distancia > DISTANCIA_MUY_LEJOS:
            return CMD_DESPACIO, f"Muy lejos ({distancia:.1f}cm)", (255, 255, 0)
        
        # Lejos del objetivo → Avanzar
        elif diff_distancia > DISTANCIA_TOLERANCIA:
            return CMD_AVANZAR, f"Acercarse ({distancia:.1f}cm)", (0, 255, 0)
        
        # Cerca del objetivo → Retroceder
        elif diff_distancia < -DISTANCIA_TOLERANCIA:
            return CMD_RETROCEDER, f"Alejarse ({distancia:.1f}cm)", (200, 100, 255)
        
        # En posición objetivo → Detener
        else:
            return CMD_DETENER, f"Posición OK ({distancia:.1f}cm)", (0, 255, 0)
    
    def procesar_frame(self, frame):
        """Procesa un frame para detectar ArUco y decidir acción"""
        
        # Convertir a escala de grises
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detectar marcadores
        corners, ids, rejected = self.detector.detectMarkers(gray)
        
        comando = CMD_DETENER
        info_texto = "Buscando ArUco..."
        color_info = (0, 165, 255)
        
        if ids is not None and len(ids) > 0:
            # ArUco detectado
            self.aruco_detectado = True
            self.tiempo_sin_aruco = time.time()
            
            # Dibujar marcadores
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            # Estimar pose
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, MARKER_SIZE, CAMERA_MATRIX, DIST_COEFFS
            )
            
            # Procesar primer marcador
            i = 0
            rvec = rvecs[i][0]
            tvec = tvecs[i][0]
            marker_id = int(ids[i][0])
            
            # Dibujar ejes
            frame = draw_axis(frame, CAMERA_MATRIX, DIST_COEFFS, rvec, tvec)
            
            # Calcular distancia
            distancia = np.linalg.norm(tvec) * 100  # en cm
            
            # Calcular rotación
            rotation_matrix, _ = cv2.Rodrigues(rvec)
            roll, pitch, yaw = rotation_matrix_to_euler_angles(rotation_matrix)
            
            # Calcular comando basado en distancia y yaw
            comando, info_texto, color_info = self.calcular_comando(distancia, yaw)
            
            # Dibujar información en pantalla
            corner = corners[i][0][0]
            x, y = int(corner[0]), int(corner[1])
            
            # Info básica
            cv2.putText(frame, f"ID: {marker_id}", (x, y - 100),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Dist: {distancia:.1f} cm", (x, y - 75),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # Rotación
            cv2.putText(frame, f"Yaw: {yaw:.1f}deg", (x, y - 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 100), 2)
            cv2.putText(frame, f"Pitch: {pitch:.1f}deg", (x, y - 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 100), 2)
            cv2.putText(frame, f"Roll: {roll:.1f}deg", (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 100), 2)
            
            # Línea horizontal de referencia
            alto, ancho = frame.shape[:2]
            cv2.line(frame, (ancho//2 - 50, alto//2), 
                    (ancho//2 + 50, alto//2), (0, 255, 0), 2)
            cv2.circle(frame, (ancho//2, alto//2), 5, (0, 0, 255), -1)
            
        else:
            # Sin ArUco
            if self.aruco_detectado:
                tiempo_perdido = time.time() - self.tiempo_sin_aruco
                if tiempo_perdido < 1.0:
                    # Poco tiempo sin verlo
                    comando = CMD_DETENER
                    info_texto = "ArUco perdido - Esperando..."
                    color_info = (0, 165, 255)
                else:
                    # Mucho tiempo sin verlo
                    self.aruco_detectado = False
                    comando = CMD_DETENER
                    info_texto = "Sin ArUco - DETENIDO"
                    color_info = (0, 0, 255)
        
        # Estado en pantalla
        alto = frame.shape[0]
        cv2.putText(frame, info_texto, (10, alto - 50),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, color_info, 2)
        cv2.putText(frame, f"CMD: {comando}", (10, alto - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, color_info, 2)
        
        # Parámetros de configuración
        cv2.putText(frame, f"Dist. Objetivo: {DISTANCIA_OBJETIVO:.0f}cm", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, f"Tolerancia Yaw: +/-{ANGULO_YAW_TOLERANCIA:.0f}deg", (10, 50),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return frame, comando
    
    def ejecutar(self):
        """Bucle principal"""
        print("\n" + "=" * 60)
        print("    ROBOT SEGUIDOR DE ARUCO - SISTEMA INICIADO")
        print("=" * 60)
        print("Controles:")
        print("  Q - Salir")
        print("  D - Toggle debug")
        print("  T - Test de motores")
        print("  ESPACIO - Detener emergencia")
        print("  C - Mostrar info de calibración")
        print("=" * 60)
        print(f"\nParámetros actuales:")
        print(f"  Distancia objetivo: {DISTANCIA_OBJETIVO} cm (±{DISTANCIA_TOLERANCIA} cm)")
        print(f"  Tolerancia angular: ±{ANGULO_YAW_TOLERANCIA}°")
        print(f"  Distancia muy cerca: < {DISTANCIA_MUY_CERCA} cm")
        print(f"  Distancia muy lejos: > {DISTANCIA_MUY_LEJOS} cm")
        print("=" * 60 + "\n")
        
        fps_counter = 0
        fps_start = time.time()
        fps_actual = 0
        
        try:
            while True:
                # Leer frame
                if self.is_picamera:
                    frame = self.cam.capture_array()
                    ret = frame is not None
                    if ret:
                        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                else:
                    ret, frame = self.cam.read()
                
                if not ret or frame is None:
                    print("✗ Error al leer frame")
                    break
                
                # Procesar frame
                frame_procesado, comando = self.procesar_frame(frame)
                
                # Enviar comando
                self.enviar_comando(comando)
                
                # FPS
                fps_counter += 1
                if fps_counter >= 30:
                    fps_actual = fps_counter / (time.time() - fps_start)
                    fps_counter = 0
                    fps_start = time.time()
                
                cv2.putText(frame_procesado, f"FPS: {fps_actual:.1f}", 
                           (frame_procesado.shape[1] - 120, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # Mostrar
                cv2.imshow('Robot ArUco Tracker', frame_procesado)
                
                # Teclas
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q') or key == ord('Q'):
                    print("\nSaliendo...")
                    break
                elif key == ord('d') or key == ord('D'):
                    self.modo_debug = not self.modo_debug
                    print(f"Debug: {'ON' if self.modo_debug else 'OFF'}")
                elif key == ord('t') or key == ord('T'):
                    print("Test de motores...")
                    self.enviar_comando(CMD_TEST)
                elif key == 32:  # ESPACIO
                    print("¡DETENCIÓN DE EMERGENCIA!")
                    for _ in range(5):
                        self.enviar_comando(CMD_DETENER)
                        time.sleep(0.05)
                elif key == ord('c') or key == ord('C'):
                    print("\n=== INFO CALIBRACIÓN ===")
                    print(f"Matriz cámara:\n{CAMERA_MATRIX}")
                    print(f"Coef. distorsión: {DIST_COEFFS.ravel()}")
                    print("========================\n")
                
        except KeyboardInterrupt:
            print("\n\nInterrumpido (Ctrl+C)")
        
        finally:
            self.limpiar()
    
    def limpiar(self):
        """Limpia recursos"""
        print("\nLimpiando recursos...")
        
        if self.bluetooth and self.bluetooth.is_open:
            print("Deteniendo robot...")
            try:
                for _ in range(5):
                    self.bluetooth.write(CMD_DETENER.encode())
                    time.sleep(0.1)
            except:
                pass
            self.bluetooth.close()
            print("✓ Bluetooth cerrado")
        
        if self.cam:
            if self.is_picamera:
                self.cam.stop()
            else:
                self.cam.release()
            print("✓ Cámara liberada")
        
        cv2.destroyAllWindows()
        print("✓ Limpieza completa")
        print("\n¡Hasta pronto!\n")


# ============== FUNCIÓN PRINCIPAL ==============

def main():
    print("\n" + "=" * 60)
    print("  ROBOT SEGUIDOR DE ARUCO CON DISTANCIA Y ROTACIÓN")
    print("  Raspberry Pi 4 + Arduino + HC-05 + Detección 3D")
    print("=" * 60 + "\n")
    
    robot = RobotArucoTracker()
    
    # Inicializar componentes
    if not robot.verificar_conexion_bluetooth():
        sys.exit(1)
    
    if not robot.conectar_bluetooth():
        sys.exit(1)
    
    if not robot.inicializar_camara():
        sys.exit(1)
    
    if not robot.inicializar_aruco():
        sys.exit(1)
    
    # Ejecutar
    robot.ejecutar()


if __name__ == "__main__":
    main()