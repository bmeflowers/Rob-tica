#!/usr/bin/env python3
"""
ARUco Marker Follower - Raspberry Pi
Detecta marcadores ARUco y envía comandos al Arduino para seguirlos
"""

import cv2
import numpy as np
import sys
import platform
import serial
import time

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

# ===== CONFIGURACIÓN =====
ARUCO_DICT = cv2.aruco.DICT_6X6_250
MARKER_SIZE = 0.05  # Tamaño del marcador en metros (5 cm)

# Puerto serial del Arduino (ajusta según tu sistema)
ARDUINO_PORT = '/dev/ttyUSB0'  # o '/dev/ttyACM0' en RPi
ARDUINO_BAUDRATE = 9600

# Parámetros de calibración de cámara
CAMERA_MATRIX = np.array([
    [800, 0, 320],
    [0, 800, 240],
    [0, 0, 1]
], dtype=float)

DIST_COEFFS = np.zeros((5, 1))

# ===== UMBRALES DE CONTROL =====
# Distancia objetivo en centímetros
TARGET_DISTANCE = 50.0  # Mantener a 50 cm del marcador
DISTANCE_TOLERANCE = 10.0  # ±10 cm es aceptable

# Ángulo de visión (yaw) para considerar centrado
YAW_TOLERANCE = 15.0  # ±15 grados

# Resolución de la cámara
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_CENTER_X = FRAME_WIDTH // 2


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


def initialize_camera_picamera2():
    """Inicializa cámara Raspberry Pi"""
    try:
        print("Inicializando cámara con picamera2...")
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(
            main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"}
        )
        picam2.configure(config)
        picam2.start()
        print("✓ Cámara Raspberry Pi inicializada")
        return picam2
    except Exception as e:
        print(f"✗ Error al inicializar picamera2: {e}")
        return None


def initialize_camera_opencv():
    """Inicializa cámara con OpenCV"""
    backends = [
        (cv2.CAP_V4L2, "V4L2"),
        (cv2.CAP_ANY, "Auto"),
        (0, "Default")
    ]

    for backend, name in backends:
        try:
            if isinstance(backend, int):
                cap = cv2.VideoCapture(0)
            else:
                cap = cv2.VideoCapture(0, backend)

            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    print(f"✓ Cámara inicializada con {name}")
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
                    return cap
                cap.release()
        except Exception as e:
            continue
    return None


def initialize_camera():
    """Inicializa cámara según plataforma"""
    if PICAMERA2_AVAILABLE:
        cam = initialize_camera_picamera2()
        if cam is not None:
            return cam, True

    cam = initialize_camera_opencv()
    if cam is not None:
        return cam, False

    return None, False


def initialize_arduino():
    """Inicializa comunicación serial con Arduino"""
    try:
        ser = serial.Serial(ARDUINO_PORT, ARDUINO_BAUDRATE, timeout=1)
        time.sleep(2)  # Esperar reset del Arduino
        print(f"✓ Conectado al Arduino en {ARDUINO_PORT}")
        return ser
    except serial.SerialException as e:
        print(f"✗ Error al conectar con Arduino: {e}")
        print("\nVerifica:")
        print("  1. Puerto correcto (ls /dev/tty* | grep -E 'USB|ACM')")
        print("  2. Permisos: sudo usermod -a -G dialout $USER")
        return None


def send_command(ser, cmd):
    """Envía comando al Arduino"""
    if ser and ser.is_open:
        try:
            ser.write(f"{cmd}\n".encode())
            ser.flush()
        except Exception as e:
            print(f"Error enviando comando: {e}")


def calculate_control_command(distance, yaw, marker_center_x):
    """
    Calcula el comando a enviar según posición del marcador
    
    Lógica:
    1. Si está muy lejos -> ADELANTE
    2. Si está muy cerca -> ATRÁS
    3. Si está a buena distancia:
       - Si yaw indica que está a la izquierda -> GIRAR IZQUIERDA
       - Si yaw indica que está a la derecha -> GIRAR DERECHA
       - Si está centrado -> STOP (objetivo alcanzado)
    """
    
    # Calcular error de distancia
    distance_error = distance - TARGET_DISTANCE
    
    # Determinar comando
    if distance > TARGET_DISTANCE + DISTANCE_TOLERANCE:
        # Muy lejos, avanzar
        return 'F', "Avanzando (lejos)"
    
    elif distance < TARGET_DISTANCE - DISTANCE_TOLERANCE:
        # Muy cerca, retroceder
        return 'B', "Retrocediendo (cerca)"
    
    else:
        # Distancia correcta, verificar orientación
        # Calculamos desviación en X (posición horizontal en imagen)
        x_error = marker_center_x - FRAME_CENTER_X
        
        # Si el marcador está a la izquierda (x_error negativo)
        if x_error < -50 or yaw > YAW_TOLERANCE:
            return 'L', "Girando izquierda"
        
        # Si el marcador está a la derecha (x_error positivo)
        elif x_error > 50 or yaw < -YAW_TOLERANCE:
            return 'R', "Girando derecha"
        
        else:
            # Centrado y a distancia correcta
            return 'S', "Objetivo alcanzado"


def draw_status(frame, cmd, description, distance, yaw):
    """Dibuja información de estado en el frame"""
    cv2.putText(frame, f"Comando: {cmd} - {description}", 
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(frame, f"Distancia: {distance:.1f} cm (Target: {TARGET_DISTANCE:.0f})", 
                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    cv2.putText(frame, f"Yaw: {yaw:.1f} deg", 
                (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 100), 2)
    
    # Línea de referencia central
    cv2.line(frame, (FRAME_CENTER_X, 0), (FRAME_CENTER_X, FRAME_HEIGHT), 
             (0, 255, 255), 2)


def main():
    # Inicializar cámara
    cam, is_picamera = initialize_camera()
    if cam is None:
        print("❌ No se pudo inicializar la cámara")
        sys.exit(1)

    # Inicializar Arduino
    arduino = initialize_arduino()
    if arduino is None:
        print("❌ No se pudo conectar con Arduino")
        cam.stop() if is_picamera else cam.release()
        sys.exit(1)

    # Inicializar detector ARUco
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    print("\n" + "="*50)
    print("SISTEMA DE SEGUIMIENTO ARUCO INICIADO")
    print("="*50)
    print(f"Distancia objetivo: {TARGET_DISTANCE} cm (±{DISTANCE_TOLERANCE} cm)")
    print(f"Tolerancia de ángulo: ±{YAW_TOLERANCE}°")
    print("\nComandos:")
    print("  'q' - Salir")
    print("  's' - Detener motores manualmente")
    print("="*50 + "\n")

    last_command = None
    no_marker_counter = 0

    try:
        while True:
            # Leer frame
            if is_picamera:
                frame = cam.capture_array()
                ret = frame is not None
                if ret:
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            else:
                ret, frame = cam.read()

            if not ret or frame is None:
                print("Error leyendo frame")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detectar marcadores
            corners, ids, rejected = detector.detectMarkers(gray)

            if ids is not None:
                no_marker_counter = 0
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                # Estimar pose
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, MARKER_SIZE, CAMERA_MATRIX, DIST_COEFFS
                )

                # Usar el primer marcador detectado
                rvec = rvecs[0][0]
                tvec = tvecs[0][0]
                marker_id = int(ids[0][0])

                # Calcular distancia
                distance = np.linalg.norm(tvec) * 100

                # Calcular ángulos
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                roll, pitch, yaw = rotation_matrix_to_euler_angles(rotation_matrix)

                # Calcular centro del marcador en imagen
                marker_corners = corners[0][0]
                marker_center_x = np.mean(marker_corners[:, 0])
                marker_center_y = np.mean(marker_corners[:, 1])

                # Dibujar centro del marcador
                cv2.circle(frame, (int(marker_center_x), int(marker_center_y)), 
                          5, (0, 0, 255), -1)

                # Calcular comando
                cmd, description = calculate_control_command(
                    distance, yaw, marker_center_x
                )

                # Enviar comando solo si cambió
                if cmd != last_command:
                    send_command(arduino, cmd)
                    last_command = cmd
                    print(f"ID:{marker_id} | Dist:{distance:.1f}cm | "
                          f"Yaw:{yaw:.1f}° | CMD:{cmd} ({description})")

                # Dibujar estado
                draw_status(frame, cmd, description, distance, yaw)

            else:
                # No se detectó marcador
                no_marker_counter += 1
                if no_marker_counter > 30:  # ~1 segundo sin detección
                    if last_command != 'S':
                        send_command(arduino, 'S')
                        last_command = 'S'
                        print("⚠ Marcador perdido - DETENIENDO")
                
                cv2.putText(frame, "BUSCANDO MARCADOR...", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            # Mostrar frame
            cv2.imshow('ARUco Follower', frame)

            # Manejo de teclas
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                send_command(arduino, 'S')
                print("⏹ Detenido manualmente")

    except KeyboardInterrupt:
        print("\n⚠ Interrupción del usuario")

    finally:
        # Detener motores antes de salir
        send_command(arduino, 'S')
        print("\n🛑 Deteniendo motores...")
        
        # Liberar recursos
        if is_picamera:
            cam.stop()
        else:
            cam.release()
        
        if arduino:
            arduino.close()
        
        cv2.destroyAllWindows()
        print("✓ Sistema finalizado correctamente")


if __name__ == "__main__":
    main()