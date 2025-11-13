#!/usr/bin/env python3
"""
ARUco Marker Distance and Rotation Detection in Real-time
Detecta marcadores ARUco y calcula distancia y rotación usando la cámara del portátil
"""

import cv2
import numpy as np
import sys

# Configuración del marcador ARUco
ARUCO_DICT = cv2.aruco.DICT_6X6_250
MARKER_SIZE = 0.05  # Tamaño del marcador en metros (5 cm)

# Parámetros de calibración de cámara (valores aproximados para cámaras de portátil)
# Para mejor precisión, ejecuta camera_calibration.py primero
CAMERA_MATRIX = np.array([
    [800, 0, 320],
    [0, 800, 240],
    [0, 0, 1]
], dtype=float)

DIST_COEFFS = np.zeros((5, 1))  # Asumiendo sin distorsión


def rotation_matrix_to_euler_angles(R):
    """
    Convierte una matriz de rotación a ángulos de Euler (roll, pitch, yaw)
    Retorna ángulos en grados
    """
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)

    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])  # Roll
        y = np.arctan2(-R[2, 0], sy)       # Pitch
        z = np.arctan2(R[1, 0], R[0, 0])  # Yaw
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0

    return np.degrees([x, y, z])


def draw_axis(img, camera_matrix, dist_coeffs, rvec, tvec, length=0.03):
    """
    Dibuja los ejes X, Y, Z del marcador para visualizar la orientación
    """
    # Definir puntos de los ejes en 3D
    axis_points = np.float32([
        [0, 0, 0],           # Origen
        [length, 0, 0],      # Eje X (rojo)
        [0, length, 0],      # Eje Y (verde)
        [0, 0, length]       # Eje Z (azul)
    ])

    # Proyectar puntos 3D a 2D
    image_points, _ = cv2.projectPoints(
        axis_points, rvec, tvec, camera_matrix, dist_coeffs
    )

    image_points = image_points.astype(int)
    origin = tuple(image_points[0].ravel())

    # Dibujar ejes con colores RGB
    img = cv2.line(img, origin, tuple(image_points[1].ravel()), (0, 0, 255), 3)  # X - Rojo
    img = cv2.line(img, origin, tuple(image_points[2].ravel()), (0, 255, 0), 3)  # Y - Verde
    img = cv2.line(img, origin, tuple(image_points[3].ravel()), (255, 0, 0), 3)  # Z - Azul

    return img


def initialize_camera():
    """
    Intenta inicializar la cámara con diferentes backends
    Retorna el objeto VideoCapture o None si falla
    """
    # Lista de backends a probar en orden de preferencia
    backends = [
        (cv2.CAP_V4L2, "V4L2 (Linux)"),
        (cv2.CAP_ANY, "Auto"),
        (0, "Default")
    ]

    print("Intentando acceder a la cámara...")

    for backend, name in backends:
        print(f"  Probando backend: {name}...")
        try:
            if isinstance(backend, int) and backend == 0:
                cap = cv2.VideoCapture(0)
            else:
                cap = cv2.VideoCapture(0, backend)

            if cap.isOpened():
                # Probar leer un frame
                ret, frame = cap.read()
                if ret and frame is not None:
                    print(f"  ✓ Cámara inicializada con {name}")
                    # Configurar resolución
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


def main():
    # Inicializar captura de video
    cap = initialize_camera()

    if cap is None:
        print("\n❌ Error: No se pudo acceder a la cámara")
        print("\nPosibles soluciones:")
        print("1. Verifica que la cámara esté conectada:")
        print("   ls /dev/video*")
        print("2. Verifica permisos:")
        print("   sudo usermod -a -G video $USER")
        print("   (luego cierra sesión y vuelve a entrar)")
        print("3. Prueba con otro programa:")
        print("   raspistill -o test.jpg  # Para cámara oficial RPi")
        print("   ffmpeg -f v4l2 -i /dev/video0 -frames 1 test.jpg  # Para USB")
        sys.exit(1)

    # Inicializar detector ARUco
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    print("Sistema de detección ARUco iniciado")
    print(f"Tamaño del marcador configurado: {MARKER_SIZE * 100} cm")
    print("Presiona 'q' para salir, 'c' para calibrar automáticamente")
    print("\nImprime un marcador desde: https://chev.me/arucogen/")

    # Variables para calibración automática
    auto_calibrate = False

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Error: No se pudo leer el frame")
            break

        # Convertir a escala de grises
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detectar marcadores
        corners, ids, rejected = detector.detectMarkers(gray)

        # Si se detectaron marcadores
        if ids is not None:
            # Dibujar marcadores detectados
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Estimar pose de cada marcador
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, MARKER_SIZE, CAMERA_MATRIX, DIST_COEFFS
            )

            for i, marker_id in enumerate(ids):
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]

                # Dibujar ejes del marcador
                frame = draw_axis(frame, CAMERA_MATRIX, DIST_COEFFS, rvec, tvec)

                # Calcular distancia (en centímetros)
                distance = np.linalg.norm(tvec) * 100

                # Convertir vector de rotación a matriz de rotación
                rotation_matrix, _ = cv2.Rodrigues(rvec)

                # Obtener ángulos de Euler
                roll, pitch, yaw = rotation_matrix_to_euler_angles(rotation_matrix)

                # Mostrar información en pantalla
                marker_id_val = int(marker_id[0])
                corner = corners[i][0][0]  # Primera esquina del marcador

                # Información de distancia
                cv2.putText(
                    frame,
                    f"ID: {marker_id_val}",
                    (int(corner[0]), int(corner[1]) - 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    frame,
                    f"Dist: {distance:.1f} cm",
                    (int(corner[0]), int(corner[1]) - 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2
                )

                # Información de rotación
                cv2.putText(
                    frame,
                    f"Roll: {roll:.1f}deg",
                    (int(corner[0]), int(corner[1]) - 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 200, 100),
                    2
                )

                cv2.putText(
                    frame,
                    f"Pitch: {pitch:.1f}deg",
                    (int(corner[0]), int(corner[1]) - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 200, 100),
                    2
                )

                cv2.putText(
                    frame,
                    f"Yaw: {yaw:.1f}deg",
                    (int(corner[0]), int(corner[1])),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 200, 100),
                    2
                )

                # Imprimir en consola
                print(f"\rID:{marker_id_val} | Dist:{distance:.1f}cm | "
                      f"Roll:{roll:.1f}° Pitch:{pitch:.1f}° Yaw:{yaw:.1f}°",
                      end="", flush=True)

        # Mostrar instrucciones en pantalla
        cv2.putText(
            frame,
            "q: Salir | c: Info Calibracion",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )

        # Mostrar frame
        cv2.imshow('ARUco Distance & Rotation Detector', frame)

        # Manejo de teclas
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('c'):
            print("\n\n=== INFORMACION DE CALIBRACION ===")
            print("Para mejor precisión, ejecuta: python camera_calibration.py")
            print("Esto generará un archivo camera_calibration.npz")
            print(f"Matriz de cámara actual:\n{CAMERA_MATRIX}")
            print(f"Coeficientes de distorsión:\n{DIST_COEFFS.ravel()}")
            print("=====================================\n")

    # Liberar recursos
    cap.release()
    cv2.destroyAllWindows()
    print("\n\nPrograma finalizado")


if __name__ == "__main__":
    main()
