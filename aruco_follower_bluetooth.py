#!/usr/bin/env python3
"""
ARUco Marker Follower (Raspberry Pi)
CONTROL VIA BLUETOOTH SERIAL HC-05 -> Arduino UNO R3

Comandos enviados:
    F = Adelante
    B = Atrás
    L = Izquierda
    R = Derecha
    S = Stop

Puerto Bluetooth esperado:
    /dev/rfcomm0

Pasos previos en Raspberry:
    bluetoothctl -> pair / trust / connect
    sudo rfcomm bind rfcomm0 XX:XX:XX:XX:XX:XX 1
"""

import cv2
import numpy as np
import sys
import time
import platform
import argparse
from collections import deque

# ========== SERIAL BLUETOOTH ==========
import serial
ser = None
LAST_SENT = ""
LAST_SEND_TS = 0.0

def serial_init(port="/dev/rfcomm0", baud=9600):
    """Inicializa el puerto Bluetooth como serial."""
    global ser
    try:
        ser = serial.Serial(port, baud, timeout=0.01)
        time.sleep(2)
        print(f"✓ Bluetooth serial abierto en {port} @ {baud}")
        return ser
    except Exception as e:
        print(f"✗ Error abriendo puerto {port}: {e}")
        ser = None
        return None

def send_cmd_letter(letter: str, throttle_ms=60):
    """
    Envía F/B/L/R/S al Arduino por Bluetooth (HC-05).
    throttle_ms evita repetir el mismo comando demasiado rápido.
    """
    global LAST_SENT, LAST_SEND_TS

    letter = letter.strip().upper()
    if letter not in ["F", "B", "L", "R", "S"]:
        return

    now = time.time() * 1000.0
    if throttle_ms > 0 and (letter == LAST_SENT) and ((now - LAST_SEND_TS) < throttle_ms):
        return  # no spamear

    if ser:
        try:
            ser.write(letter.encode("utf-8"))
            LAST_SENT = letter
            LAST_SEND_TS = now
            print(f"\r[BT CMD] {letter}   ", end="", flush=True)
        except:
            print("\n[ERR] Fallo enviando por Bluetooth")


# ========== DETECCIÓN DE RASPBERRY Y CÁMARA ==========
IS_RASPBERRY_PI = platform.machine().startswith('arm') or platform.machine().startswith('aarch64')

# intentar picamera2 si existe
PICAMERA2_AVAILABLE = False
if IS_RASPBERRY_PI:
    try:
        from picamera2 import Picamera2
        PICAMERA2_AVAILABLE = True
        print("✓ Usando picamera2")
    except:
        print("⚠ No se pudo cargar picamera2, intentando OpenCV.")


# ========== UTILIDADES DE ARUCO ==========
def rotation_matrix_to_euler_angles(R):
    """Convierte matriz de rotación a roll/pitch/yaw (°)."""
    sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2,1], R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
    return np.degrees([x, y, z])


def draw_axis(img, K, D, rvec, tvec, length=0.03):
    """Dibuja ejes X/Y/Z sobre el marcador."""
    axis = np.float32([
        [0,0,0],
        [length,0,0],
        [0,length,0],
        [0,0,length]
    ])
    pts,_ = cv2.projectPoints(axis, rvec, tvec, K, D)
    pts = pts.astype(int)
    o = tuple(pts[0].ravel())
    img = cv2.line(img, o, tuple(pts[1].ravel()), (0,0,255), 3)
    img = cv2.line(img, o, tuple(pts[2].ravel()), (0,255,0), 3)
    img = cv2.line(img, o, tuple(pts[3].ravel()), (255,0,0), 3)
    return img


# ========== CÁMARA ==========
def initialize_camera():
    if PICAMERA2_AVAILABLE:
        try:
            picam = Picamera2()
            cfg = picam.create_preview_configuration(main={"size": (640,480), "format": "RGB888"})
            picam.configure(cfg)
            picam.start()
            return picam, True
        except:
            pass

    cap = cv2.VideoCapture(0)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        return cap, False
    
    return None, False


# ========== ARGUMENTOS ==========
def parse_args():
    parser = argparse.ArgumentParser(description="ARUco follower via Bluetooth")
    parser.add_argument("--desired", type=float, default=30.0, help="Distancia objetivo (cm)")
    parser.add_argument("--dead_yaw", type=float, default=5.0, help="Zona muerta yaw (°)")
    parser.add_argument("--dead_dist", type=float, default=5.0, help="Zona muerta distancia (cm)")
    parser.add_argument("--marksize", type=float, default=0.05, help="Tamaño marcador (m)")
    parser.add_argument("--calib", default="")
    parser.add_argument("--dict", default="DICT_6X6_250")
    return parser.parse_args()


# ========== CARGA DE CALIBRACIÓN ==========
def load_calibration(path):
    if not path:
        return None, None
    try:
        data = np.load(path)
        return data["camera_matrix"], data["dist_coeffs"]
    except:
        return None, None


# ========== MAIN ==========
def main():
    args = parse_args()

    # calibración inicial aproximada
    K = np.array([[800,0,320],[0,800,240],[0,0,1]], dtype=float)
    D = np.zeros((5,1))

    CK, CD = load_calibration(args.calib)
    if CK is not None:
        K = CK.astype(float)
        D = CD.astype(float)

    # INICIALIZAR BLUETOOTH
    serial_init("/dev/rfcomm0", 9600)

    # INICIALIZAR CÁMARA
    cam, is_picam = initialize_camera()
    if cam is None:
        print("❌ No se pudo abrir la cámara.")
        sys.exit(1)

    # Configurar ARUco
    ARUCO_DICT = getattr(cv2.aruco, args.dict)
    dict_aruco = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dict_aruco, params)

    yaw_hist = deque(maxlen=5)
    dist_hist = deque(maxlen=5)
    STOP_THROTTLE_MS = 200

    print("Sistema ARUco iniciado (envío Bluetooth F/B/L/R/S)")
    print("Presiona 'q' para salir.")

    try:
        while True:
            # capturar frame
            if is_picam:
                frame = cam.capture_array()
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            else:
                ret, frame = cam.read()
                if not ret:
                    continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)

            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, args.marksize, K, D
                )

                rvec = rvecs[0][0]
                tvec = tvecs[0][0]

                frame = draw_axis(frame, K, D, rvec, tvec)

                # distancia
                dist = float(np.linalg.norm(tvec) * 100.0)

                # yaw
                R,_ = cv2.Rodrigues(rvec)
                roll, pitch, yaw = rotation_matrix_to_euler_angles(R)

                yaw_hist.append(yaw)
                dist_hist.append(dist)

                yaw_f = float(np.median(yaw_hist))
                dist_f = float(np.median(dist_hist))

                # ===== DECISIÓN DE MOVIMIENTO =====
                cmd = "S"

                # 1) corregir yaw
                if abs(yaw_f) > args.dead_yaw:
                    if yaw_f > 0:
                        cmd = "R"   # derecha
                    else:
                        cmd = "L"   # izquierda
                else:
                    # 2) corregir distancia
                    err = dist_f - args.desired
                    if err > args.dead_dist:
                        cmd = "F"
                    elif err < -args.dead_dist:
                        cmd = "B"
                    else:
                        cmd = "S"

                send_cmd_letter(cmd)

                # overlays
                cv2.putText(frame, f"Dist:{dist_f:.1f}cm", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(0,255,255),2)
                cv2.putText(frame, f"Yaw:{yaw_f:.1f}deg", (10,55), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(200,200,255),2)
                cv2.putText(frame, f"CMD:{cmd}", (10,80), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(255,0,0),2)

                print(f"\rID:{ids[0][0]} | Dist:{dist_f:.1f} | Yaw:{yaw_f:.1f} -> CMD {cmd}", end="", flush=True)

            else:
                # si no hay marcador → STOP ocasional
                send_cmd_letter("S", throttle_ms=STOP_THROTTLE_MS)
                cv2.putText(frame, "Sin marcador: STOP", (10,30), cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,0,255),2)

            cv2.imshow("ARUco Bluetooth Control", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        try:
            send_cmd_letter("S")
        except:
            pass

        if is_picam:
            cam.stop()
        else:
            cam.release()

        cv2.destroyAllWindows()
        print("\nPrograma finalizado.")


if __name__ == "__main__":
    main()
