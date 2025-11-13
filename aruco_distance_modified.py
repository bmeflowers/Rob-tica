#!/usr/bin/env python3
"""
ARUco Marker Distance & Rotation Follower (Raspberry Pi / PC)
- Detecta marcadores ArUco, estima pose y controla motores vía Arduino por Serie
- Envia al Arduino comandos "M <izq> <der>" (-255..255) y "S" para detener

Requisitos:
  pip install opencv-contrib-python pyserial numpy
  (en Raspberry) sudo apt install python3-picamera2

Ejemplos:
  python3 aruco_follow.py --port /dev/ttyACM0 --baud 115200 --desired 30
  python3 aruco_follow.py --port /dev/rfcomm0 --baud 115200 --kturn 3 --klinear 2
  python3 aruco_follow.py --calib camera_calibration.npz
"""

import cv2
import numpy as np
import sys
import time
import platform
import argparse
from collections import deque

# ===== Detectar Raspberry Pi =====
IS_RASPBERRY_PI = platform.machine().startswith('arm') or platform.machine().startswith('aarch64')

# ===== Intentar picamera2 =====
PICAMERA2_AVAILABLE = False
if IS_RASPBERRY_PI:
    try:
        from picamera2 import Picamera2
        PICAMERA2_AVAILABLE = True
        print("✓ Usando picamera2 para Raspberry Pi")
    except Exception as e:
        print(f"⚠ picamera2 no disponible ({e}), usando OpenCV")
        PICAMERA2_AVAILABLE = False

# ======== Utilidades Serie ========
try:
    import serial
except Exception:
    serial = None  # por si no está instalado

ser = None
LAST_SENT = ""          # último comando enviado
LAST_SEND_TS = 0.0

def serial_init(port, baud):
    global ser
    if serial is None:
        print("⚠ pyserial no está instalado. Ejecuta: pip install pyserial")
        return None
    try:
        ser = serial.Serial(port, baud, timeout=0.01)
        time.sleep(2.0)  # esperar reset de Arduino
        print(f"✓ Serie abierta en {port} @ {baud}")
        return ser
    except Exception as e:
        print(f"✗ No se pudo abrir serie en {port}: {e}")
        ser = None
        return None

def send_line(s: str, throttle_ms=0):
    """Envía una línea por serie con opcional limitador de frecuencia."""
    global LAST_SENT, LAST_SEND_TS
    now = time.time() * 1000.0
    if throttle_ms > 0 and (now - LAST_SEND_TS) < throttle_ms and s == LAST_SENT:
        return  # no spamear el mismo comando
    if ser and ser.writable():
        try:
            ser.write((s.strip() + "\n").encode("utf-8"))
            LAST_SENT = s
            LAST_SEND_TS = now
        except Exception as e:
            print(f"\n[ERR serial] {e}")

# ======== ArUco / Geometría ========
def rotation_matrix_to_euler_angles(R):
    """Convierte R a (roll, pitch, yaw) en grados."""
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])  # Roll
        y = np.arctan2(-R[2, 0], sy)      # Pitch
        z = np.arctan2(R[1, 0], R[0, 0])  # Yaw
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0
    return np.degrees([x, y, z])

def draw_axis(img, camera_matrix, dist_coeffs, rvec, tvec, length=0.03):
    """Dibuja ejes X,Y,Z del marcador."""
    axis_points = np.float32([
        [0, 0, 0],
        [length, 0, 0],
        [0, length, 0],
        [0, 0, length],
    ])
    image_points, _ = cv2.projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs)
    image_points = image_points.astype(int)
    origin = tuple(image_points[0].ravel())
    img = cv2.line(img, origin, tuple(image_points[1].ravel()), (0, 0, 255), 3)   # X rojo
    img = cv2.line(img, origin, tuple(image_points[2].ravel()), (0, 255, 0), 3)   # Y verde
    img = cv2.line(img, origin, tuple(image_points[3].ravel()), (255, 0, 0), 3)   # Z azul
    return img

# ======== Cámara ========
def initialize_camera_picamera2(res=(640, 480)):
    try:
        print("Inicializando cámara con picamera2...")
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(main={"size": res, "format": "RGB888"})
        picam2.configure(config)
        picam2.start()
        print("✓ Cámara Raspberry Pi inicializada")
        return picam2
    except Exception as e:
        print(f"✗ Error picamera2: {e}")
        return None

def initialize_camera_opencv(index=0, res=(640, 480)):
    print("Intentando acceder a la cámara con OpenCV...")
    backends = [(cv2.CAP_V4L2, "V4L2 (Linux)"), (cv2.CAP_ANY, "Auto"), (0, "Default")]
    for backend, name in backends:
        print(f"  Probando backend: {name}...")
        try:
            cap = cv2.VideoCapture(index if backend == 0 else index, backend) if isinstance(backend, int) else cv2.VideoCapture(index)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, res[0])
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, res[1])
                ret, frame = cap.read()
                if ret and frame is not None:
                    print(f"  ✓ Cámara inicializada con {name}")
                    return cap
            cap.release()
        except Exception as e:
            print(f"  Error con {name}: {e}")
    return None

def initialize_camera(res=(640, 480)):
    if PICAMERA2_AVAILABLE:
        cam = initialize_camera_picamera2(res)
        if cam is not None:
            return cam, True
    cam = initialize_camera_opencv(0, res)
    if cam is not None:
        return cam, False
    return None, False

# ======== Principal ========
def parse_args():
    parser = argparse.ArgumentParser(description="ArUco follower con control serie a Arduino")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Puerto serie (USB: /dev/ttyACM0 o /dev/ttyUSB0, BT: /dev/rfcomm0, Windows: COM3)")
    parser.add_argument("--baud", type=int, default=115200, help="Baudios serie (Arduino USB típico 115200, HC-05 9600 si lo configuraste así)")
    parser.add_argument("--desired", type=float, default=30.0, help="Distancia objetivo al marcador (cm)")
    parser.add_argument("--dead_yaw", type=float, default=3.0, help="Zona muerta de yaw (grados)")
    parser.add_argument("--dead_dist", type=float, default=2.0, help="Zona muerta de distancia (cm)")
    parser.add_argument("--kturn", type=float, default=4.0, help="Ganancia proporcional de giro (deg -> PWM)")
    parser.add_argument("--klinear", type=float, default=3.0, help="Ganancia proporcional lineal (cm -> PWM)")
    parser.add_argument("--marksize", type=float, default=0.05, help="Tamaño del marcador (m), p.ej. 0.05 = 5 cm")
    parser.add_argument("--calib", default="", help="Ruta a camera_calibration.npz para usar CAMERA_MATRIX y DIST_COEFFS")
    parser.add_argument("--dict", default="DICT_6X6_250", help="Diccionario ArUco (p.ej. DICT_4X4_50, DICT_5X5_100, DICT_6X6_250, DICT_7X7_1000, DICT_ARUCO_ORIGINAL)")
    parser.add_argument("--flip_sign_turn", action="store_true", help="Invierte el signo de giro si gira al revés")
    parser.add_argument("--flip_sign_lin", action="store_true", help="Invierte el signo lineal si avanza/retrocede al revés")
    parser.add_argument("--res", default="640x480", help="Resolución cámara WxH, p.ej. 640x480")
    return parser.parse_args()

def load_calibration(path_npz):
    if not path_npz:
        return None, None
    try:
        data = np.load(path_npz)
        K = data["camera_matrix"]
        D = data["dist_coeffs"]
        print("✓ Calibración cargada de:", path_npz)
        return K, D
    except Exception as e:
        print(f"⚠ No se pudo cargar calibración '{path_npz}': {e}")
        return None, None

def aruco_dictionary_from_name(name):
    name = name.strip()
    if not hasattr(cv2.aruco, name):
        print(f"⚠ Diccionario '{name}' no existe, usando DICT_6X6_250")
        name = "DICT_6X6_250"
    return getattr(cv2.aruco, name)

def clamp_pwm(v, lo=-255, hi=255, dead=10):
    v = max(min(int(round(v)), hi), lo)
    if abs(v) < dead:
        v = 0
    return v

def main():
    args = parse_args()

    # Parse resolución
    try:
        w, h = [int(x) for x in args.res.lower().split("x")]
        RES = (w, h)
    except Exception:
        RES = (640, 480)

    # Calibración por defecto (aprox)
    CAMERA_MATRIX = np.array([[800, 0, 320],
                              [0, 800, 240],
                              [0,   0,   1]], dtype=float)
    DIST_COEFFS = np.zeros((5, 1))

    # Intentar cargar calibración real si se proporciona
    K, D = load_calibration(args.calib)
    if K is not None and D is not None:
        CAMERA_MATRIX, DIST_COEFFS = K.astype(float), D.astype(float)

    # Serie
    serial_init(args.port, args.baud)

    # Cámara
    cam, is_picamera = initialize_camera(RES)
    if cam is None:
        print("\n❌ Error: No se pudo acceder a la cámara")
        print("Sugerencias:")
        if IS_RASPBERRY_PI:
            print("  sudo apt install -y python3-picamera2")
            print("  sudo raspi-config  (Interface Options > Camera > Enable)")
        else:
            print("  Verifica /dev/video* y permisos (grupo video)")
        sys.exit(1)

    # ArUco
    ARUCO_DICT_ID = aruco_dictionary_from_name(args.dict)
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    print("Sistema de seguimiento ArUco iniciado")
    print(f"Tamaño marcador: {args.marksize*100:.0f} cm | Dist objetivo: {args.desired:.1f} cm")
    print("Presiona 'q' para salir, 'c' para info de calibración.")
    print("Imprime un marcador: https://chev.me/arucogen/")

    # Control
    yaw_hist  = deque(maxlen=5)
    dist_hist = deque(maxlen=5)
    last_seen_ts = 0.0
    STOP_THROTTLE_MS = 200  # no spamear 'S' más de 5/s
    SEND_THROTTLE_MS = 40   # ~25 Hz máximo para 'M'

    try:
        while True:
            # Capturar frame
            if is_picamera:
                frame = cam.capture_array()
                if frame is None:
                    print("Error de frame (picamera2)")
                    break
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            else:
                ret, frame = cam.read()
                if not ret or frame is None:
                    print("Error: No se pudo leer el frame")
                    break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = detector.detectMarkers(gray)

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, args.marksize, CAMERA_MATRIX, DIST_COEFFS
                )

                # Usar el primer marcador detectado (o podrías elegir por ID)
                rvec = rvecs[0][0]
                tvec = tvecs[0][0]

                # Dibujar ejes y overlays
                frame = draw_axis(frame, CAMERA_MATRIX, DIST_COEFFS, rvec, tvec)

                # Distancia (cm)
                distance = float(np.linalg.norm(tvec) * 100.0)

                # Yaw
                R, _ = cv2.Rodrigues(rvec)
                roll, pitch, yaw = rotation_matrix_to_euler_angles(R)

                # Históricos (filtro mediana simple)
                yaw_hist.append(yaw)
                dist_hist.append(distance)
                yaw_f  = float(np.median(yaw_hist))
                dist_f = float(np.median(dist_hist))

                # Control proporcional
                # Giro: si el signo está invertido para tu robot, usa --flip_sign_turn
                turn_cmd = (-args.kturn * yaw_f) if not args.flip_sign_turn else (args.kturn * yaw_f)

                # Lineal: error de distancia (positivo si estás lejos). flip para invertir sentido
                dist_err = (dist_f - args.desired)
                base = (-args.klinear * dist_err) if not args.flip_sign_lin else (args.klinear * dist_err)

                # Zonas muertas
                if abs(yaw_f) <= args.dead_yaw:
                    turn_cmd = 0.0
                if abs(dist_err) <= args.dead_dist:
                    base = 0.0

                # Diferencial
                left  = base - turn_cmd
                right = base + turn_cmd

                l_cmd = clamp_pwm(left)
                r_cmd = clamp_pwm(right)

                # Enviar al Arduino
                send_line(f"M {l_cmd} {r_cmd}", throttle_ms=SEND_THROTTLE_MS)

                # Dibujos e info
                marker_id_val = int(ids[0][0])
                c0 = corners[0][0][0]
                x0, y0 = int(c0[0]), int(c0[1])

                cv2.putText(frame, f"ID: {marker_id_val}", (x0, y0 - 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                cv2.putText(frame, f"Dist: {dist_f:.1f} cm", (x0, y0 - 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
                cv2.putText(frame, f"Yaw: {yaw_f:.1f} deg", (x0, y0 - 36), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,200,100), 2)
                cv2.putText(frame, f"M {l_cmd:+4d} {r_cmd:+4d}", (x0, y0 - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200,200,255), 2)

                print(f"\rID:{marker_id_val} | Dist:{dist_f:5.1f}cm | Yaw:{yaw_f:6.1f}° -> M {l_cmd:+4d} {r_cmd:+4d}", end="", flush=True)
                last_seen_ts = time.time()

            else:
                # No marcador: enviar STOP con limitador
                send_line("S", throttle_ms=STOP_THROTTLE_MS)
                # Overlay de estado
                cv2.putText(frame, "Sin marcador: STOP", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

            # Ayuda en pantalla
            cv2.putText(frame, "q: Salir | c: Info Calibracion", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
            cv2.imshow("ARUco Follower", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('c'):
                print("\n\n=== INFORMACION DE CALIBRACION ===")
                print("Para mejor precisión, ejecuta: python camera_calibration.py")
                print("Esto generará un archivo camera_calibration.npz que puedes pasar con --calib")
                print(f"Matriz de cámara actual:\n{CAMERA_MATRIX}")
                print(f"Coeficientes de distorsión:\n{DIST_COEFFS.ravel()}")
                print("=====================================\n")

    finally:
        # Liberar
        try:
            if ser:
                send_line("S")
        except Exception:
            pass
        if is_picamera and cam:
            try: cam.stop()
            except Exception: pass
        elif cam:
            cam.release()
        cv2.destroyAllWindows()
        print("\nPrograma finalizado.")

if __name__ == "__main__":
    main()
