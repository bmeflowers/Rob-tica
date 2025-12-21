#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Seguidor de Códigos ArUco - VERSIÓN HEADLESS CORREGIDA
Raspberry Pi 4 + Arduino + HC-05 Bluetooth
Solo 2 motores (M1 y M2)
Sin display - para ejecución automática
CORREGIDO: Keepalive activo para mantener conexión Bluetooth
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
        print("OK - Usando picamera2 para Raspberry Pi")
    except ImportError:
        print("AVISO - picamera2 no disponible, usando OpenCV")
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
VELOCIDAD_ENVIO = 1.0           # ⚡ CAMBIADO: Enviar cada 1 segundo (KEEPALIVE)

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
        print("OK - Cámara Raspberry Pi inicializada correctamente")
        return picam2
    except Exception as e:
        print(f"ERROR - Error al inicializar picamera2: {e}")
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
                    print(f"  OK - Cámara inicializada con {name}")
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
        self.frames_procesados = 0
        self.ultimo_reporte = time.time()

    def verificar_conexion_bluetooth(self):
        """Verifica que /dev/rfcomm0 exista y reintenta si es necesario"""
        print("=" * 50)
        print("Verificando conexión Bluetooth...")
        print("=" * 50)
        
        import os
        
        max_intentos = 10
        for intento in range(1, max_intentos + 1):
            if os.path.exists(BLUETOOTH_PORT):
                print(f"OK - {BLUETOOTH_PORT} existe")
                return True
            
            print(f"Intento {intento}/{max_intentos}: Esperando {BLUETOOTH_PORT}...")
            time.sleep(2)
        
        print(f"\nERROR: {BLUETOOTH_PORT} no existe después de {max_intentos} intentos")
        print("\nEl servicio de Bluetooth puede no estar listo todavía.")
        print("El sistema intentará reiniciar automáticamente.")
        return False
    
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
            
            print("OK - Conexión serial establecida")
            
            # Comando de prueba
            self.bluetooth.write(b'S')
            time.sleep(0.5)
            
            if self.bluetooth.in_waiting > 0:
                respuesta = self.bluetooth.read(self.bluetooth.in_waiting).decode(errors='ignore')
                print(f"OK - Arduino respondió: {respuesta.strip()}")
            else:
                print("OK - Puerto serial funcional")
            
            return True
            
        except Exception as e:
            print(f"ERROR - Error al conectar: {e}")
            print("Verifica que el script de Bluetooth esté corriendo")
            return False
    
    def inicializar_camara(self):
        """Inicializa la cámara"""
        print("\nInicializando cámara...")
        
        self.cam, self.is_picamera = initialize_camera()
        
        if self.cam is None:
            print("\nERROR: No se pudo acceder a la cámara")
            if IS_RASPBERRY_PI:
                print("\nSoluciones:")
                print("1. sudo apt install -y python3-picamera2")
                print("2. sudo raspi-config -> Interface Options -> Camera -> Enable")
                print("3. Reinicia la Raspberry Pi")
            return False
        
        print("OK - Cámara inicializada correctamente")
        return True
    
    def inicializar_aruco(self):
        """Inicializa el detector de ArUco"""
        print("\nInicializando detector ArUco...")
        
        try:
            aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
            aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
            
            print(f"OK - Detector ArUco listo (6X6_250)")
            print(f"  Tamaño marcador: {MARKER_SIZE * 100} cm")
            return True
            
        except Exception as e:
            print(f"ERROR - Error al inicializar ArUco: {e}")
            return False
    
    def enviar_comando(self, comando):
        """
        Envía un comando al Arduino
        ⚡ CORREGIDO: Ahora SIEMPRE envía aunque sea el mismo comando
        Esto mantiene la conexión Bluetooth activa (KEEPALIVE)
        """
        try:
            tiempo_actual = time.time()
            
            # ⚡ CAMBIADO: Ahora SÍ envía comandos repetidos después de VELOCIDAD_ENVIO
            # Esto previene que el HC-05 cierre la conexión por inactividad
            if (comando == self.ultimo_comando and 
                tiempo_actual - self.tiempo_ultimo_comando < VELOCIDAD_ENVIO):
                return True  # Aún no es tiempo de reenviar
            
            # Enviar comando
            self.bluetooth.write(comando.encode())
            self.bluetooth.flush()
            
            # ⚡ NUEVO: Vaciar buffer de respuesta del Arduino
            # Esto previene que el buffer se llene y bloquee la comunicación
            if self.bluetooth.in_waiting > 0:
                self.bluetooth.read(self.bluetooth.in_waiting)
            
            self.ultimo_comando = comando
            self.tiempo_ultimo_comando = tiempo_actual
            
            # Log periódico cada 30 frames
            if self.modo_debug and self.frames_procesados % 30 == 0:
                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                comandos_str = {
                    'F': 'AVANZAR',
                    'B': 'RETROCEDER',
                    'L': 'IZQUIERDA',
                    'R': 'DERECHA',
                    'S': 'DETENER',
                    'G': 'DESPACIO'
                }
                print(f"[{timestamp}] CMD: {comandos_str.get(comando, comando)} (Keepalive activo)")
            
            return True
            
        except Exception as e:
            print(f"ERROR - Error al enviar: {e}")
            print("⚠ La conexión Bluetooth puede haberse perdido")
            return False
    
    def calcular_comando(self, distancia, yaw):
        """
        Determina comando basándose en distancia y ángulo yaw
        
        Args:
            distancia: Distancia al marcador en cm
            yaw: Ángulo de rotación en grados
        
        Returns:
            comando
        """
        
        # 1. PRIORIDAD: Distancia muy cerca -> DETENER
        if distancia < DISTANCIA_MUY_CERCA:
            return CMD_DETENER
        
        # 2. ORIENTACIÓN: Corregir ángulo primero
        if abs(yaw) > ANGULO_YAW_TOLERANCIA:
            if yaw > 0:
                return CMD_DERECHA
            else:
                return CMD_IZQUIERDA
        
        # 3. DISTANCIA: Ajustar distancia al objetivo
        diff_distancia = distancia - DISTANCIA_OBJETIVO
        
        # Muy lejos -> Avanzar despacio
        if distancia > DISTANCIA_MUY_LEJOS:
            return CMD_DESPACIO
        
        # Lejos del objetivo -> Avanzar
        elif diff_distancia > DISTANCIA_TOLERANCIA:
            return CMD_AVANZAR
        
        # Cerca del objetivo -> Retroceder
        elif diff_distancia < -DISTANCIA_TOLERANCIA:
            return CMD_RETROCEDER
        
        # En posición objetivo -> Detener
        else:
            return CMD_DETENER
    
    def procesar_frame(self, frame):
        """Procesa un frame para detectar ArUco y decidir acción"""
        
        self.frames_procesados += 1
        
        # Convertir a escala de grises
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detectar marcadores
        corners, ids, rejected = self.detector.detectMarkers(gray)
        
        comando = CMD_DETENER
        
        if ids is not None and len(ids) > 0:
            # ArUco detectado
            if not self.aruco_detectado:
                print("\n>>> ArUco DETECTADO <<<")
                self.aruco_detectado = True
            
            self.tiempo_sin_aruco = time.time()
            
            # Estimar pose
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, MARKER_SIZE, CAMERA_MATRIX, DIST_COEFFS
            )
            
            # Procesar primer marcador
            i = 0
            rvec = rvecs[i][0]
            tvec = tvecs[i][0]
            marker_id = int(ids[i][0])
            
            # Calcular distancia
            distancia = np.linalg.norm(tvec) * 100  # en cm
            
            # Calcular rotación
            rotation_matrix, _ = cv2.Rodrigues(rvec)
            roll, pitch, yaw = rotation_matrix_to_euler_angles(rotation_matrix)
            
            # Calcular comando basado en distancia y yaw
            comando = self.calcular_comando(distancia, yaw)
            
            # Log periódico cada 30 frames
            if self.frames_procesados % 30 == 0:
                print(f"  ID: {marker_id} | Dist: {distancia:.1f}cm | Yaw: {yaw:.1f}° | CMD: {comando}")
            
        else:
            # Sin ArUco
            if self.aruco_detectado:
                tiempo_perdido = time.time() - self.tiempo_sin_aruco
                if tiempo_perdido > 1.0:
                    # Mucho tiempo sin verlo
                    print("\n>>> ArUco PERDIDO - Robot detenido <<<")
                    self.aruco_detectado = False
                    comando = CMD_DETENER
        
        return comando
    
    def ejecutar(self):
        """Bucle principal HEADLESS (sin display)"""
        print("\n" + "=" * 60)
        print("    ROBOT SEGUIDOR DE ARUCO - MODO HEADLESS")
        print("=" * 60)
        print("Sin interfaz gráfica - Ejecución automática")
        print("Presiona Ctrl+C para detener")
        print("=" * 60)
        print(f"\nParámetros actuales:")
        print(f"  Distancia objetivo: {DISTANCIA_OBJETIVO} cm (+/-{DISTANCIA_TOLERANCIA} cm)")
        print(f"  Tolerancia angular: +/-{ANGULO_YAW_TOLERANCIA}°")
        print(f"  Distancia muy cerca: < {DISTANCIA_MUY_CERCA} cm")
        print(f"  Distancia muy lejos: > {DISTANCIA_MUY_LEJOS} cm")
        print(f"  ⚡ Keepalive Bluetooth: Cada {VELOCIDAD_ENVIO}s")
        print("=" * 60 + "\n")
        
        fps_counter = 0
        fps_start = time.time()
        
        try:
            print(">>> INICIANDO SEGUIMIENTO DE ARUCO <<<")
            print(">>> KEEPALIVE ACTIVO - Bluetooth siempre enviando <<<\n")
            
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
                    print("ERROR - Error al leer frame de cámara")
                    break
                
                # Procesar frame
                comando = self.procesar_frame(frame)
                
                # ⚡ IMPORTANTE: Enviar comando SIEMPRE (incluso si es 'S' repetido)
                # Esto mantiene la conexión Bluetooth activa
                self.enviar_comando(comando)
                
                # Reporte periódico (cada 5 segundos)
                if time.time() - self.ultimo_reporte > 5.0:
                    fps = self.frames_procesados / (time.time() - fps_start)
                    print(f"\n[ESTADÍSTICAS] FPS: {fps:.1f} | Frames procesados: {self.frames_procesados}")
                    if self.aruco_detectado:
                        print("               Estado: SIGUIENDO ARUCO")
                    else:
                        print("               Estado: BUSCANDO ARUCO")
                    print("               Bluetooth: KEEPALIVE ACTIVO")
                    self.ultimo_reporte = time.time()
                
                fps_counter += 1
                
        except KeyboardInterrupt:
            print("\n\n>>> Interrumpido por usuario (Ctrl+C) <<<")
        
        finally:
            self.limpiar()
    
    def limpiar(self):
        """Limpia recursos"""
        print("\n>>> Limpiando recursos <<<")
        
        if self.bluetooth and self.bluetooth.is_open:
            print("Deteniendo robot...")
            try:
                for _ in range(5):
                    self.bluetooth.write(CMD_DETENER.encode())
                    time.sleep(0.1)
            except:
                pass
            self.bluetooth.close()
            print("OK - Bluetooth cerrado")
        
        if self.cam:
            if self.is_picamera:
                self.cam.stop()
            else:
                self.cam.release()
            print("OK - Cámara liberada")
        
        print("OK - Limpieza completa")
        print("\n>>> Sistema apagado correctamente <<<\n")


# ============== FUNCIÓN PRINCIPAL ==============

def main():
    print("\n" + "=" * 60)
    print("  ROBOT SEGUIDOR DE ARUCO - VERSIÓN HEADLESS CORREGIDA")
    print("  Raspberry Pi 4 + Arduino + HC-05 + Detección 3D")
    print("  ⚡ KEEPALIVE ACTIVO - Mantiene Bluetooth vivo")
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