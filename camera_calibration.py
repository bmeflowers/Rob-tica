#!/usr/bin/env python3
"""
Calibración de cámara usando patrón de tablero de ajedrez
Genera parámetros precisos para la detección ARUco
"""

import cv2
import numpy as np
import glob
import os

# Configuración del patrón de calibración (tablero de ajedrez)
CHECKERBOARD = (6, 9)  # Número de esquinas internas (filas, columnas)
SQUARE_SIZE = 0.025    # Tamaño de cada cuadrado en metros (2.5 cm)

# Criterios de terminación para refinamiento de esquinas
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def capture_calibration_images():
    """
    Captura imágenes del patrón de calibración desde la cámara
    """
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: No se pudo acceder a la cámara")
        return False

    # Crear directorio para imágenes de calibración
    os.makedirs("calibration_images", exist_ok=True)

    print("\n=== CAPTURA DE IMAGENES DE CALIBRACION ===")
    print(f"Patrón requerido: Tablero de ajedrez {CHECKERBOARD[0]}x{CHECKERBOARD[1]} esquinas internas")
    print("\nInstrucciones:")
    print("1. Imprime un tablero de ajedrez desde:")
    print("   https://markhedleyjones.com/projects/calibration-checkerboard-collection")
    print("2. Muestra el patrón a la cámara desde diferentes ángulos y distancias")
    print("3. Presiona ESPACIO para capturar (necesitas ~15-20 imágenes)")
    print("4. Presiona 'q' cuando tengas suficientes imágenes\n")

    image_count = 0

    while True:
        ret, frame = cap.read()

        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Buscar el patrón de ajedrez
        ret_chess, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

        display_frame = frame.copy()

        if ret_chess:
            # Dibujar las esquinas encontradas
            cv2.drawChessboardCorners(display_frame, CHECKERBOARD, corners, ret_chess)
            cv2.putText(display_frame, "Patron detectado - ESPACIO para capturar",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(display_frame, "Buscando patron...",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.putText(display_frame, f"Imagenes capturadas: {image_count}",
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow('Calibracion - Captura de imagenes', display_frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord(' ') and ret_chess:
            # Guardar imagen
            filename = f"calibration_images/calib_{image_count:02d}.jpg"
            cv2.imwrite(filename, frame)
            image_count += 1
            print(f"Imagen {image_count} capturada")

        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    if image_count < 10:
        print(f"\nAdvertencia: Solo se capturaron {image_count} imágenes.")
        print("Se recomiendan al menos 15 imágenes para una buena calibración.")
        return False

    return True


def calibrate_camera():
    """
    Realiza la calibración de la cámara usando las imágenes capturadas
    """
    # Preparar puntos 3D del objeto
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    # Arrays para almacenar puntos
    objpoints = []  # Puntos 3D en el mundo real
    imgpoints = []  # Puntos 2D en la imagen

    # Cargar imágenes
    images = glob.glob('calibration_images/*.jpg')

    if not images:
        print("Error: No se encontraron imágenes de calibración")
        return None

    print(f"\n=== PROCESANDO {len(images)} IMAGENES ===")

    img_shape = None

    for idx, fname in enumerate(images):
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if img_shape is None:
            img_shape = gray.shape[::-1]

        # Buscar esquinas del tablero
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

        if ret:
            objpoints.append(objp)

            # Refinar posiciones de las esquinas
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners_refined)

            print(f"Imagen {idx + 1}/{len(images)}: OK")
        else:
            print(f"Imagen {idx + 1}/{len(images)}: Patrón no detectado")

    if len(objpoints) < 10:
        print(f"\nError: Solo se detectó el patrón en {len(objpoints)} imágenes.")
        print("Se necesitan al menos 10 imágenes válidas.")
        return None

    # Calibrar cámara
    print("\n=== CALIBRANDO CAMARA ===")
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, img_shape, None, None
    )

    if not ret:
        print("Error: Fallo en la calibración")
        return None

    # Calcular error de reproyección
    total_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i],
                                          camera_matrix, dist_coeffs)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        total_error += error

    mean_error = total_error / len(objpoints)

    print("\n=== RESULTADOS DE CALIBRACION ===")
    print(f"\nMatriz de camara:\n{camera_matrix}")
    print(f"\nCoeficientes de distorsion:\n{dist_coeffs.ravel()}")
    print(f"\nError medio de reproyeccion: {mean_error:.4f} pixels")

    if mean_error < 1.0:
        print("Calibracion EXCELENTE")
    elif mean_error < 2.0:
        print("Calibracion BUENA")
    else:
        print("Calibracion ACEPTABLE (considera capturar mas imagenes)")

    # Guardar resultados
    np.savez('camera_calibration.npz',
             camera_matrix=camera_matrix,
             dist_coeffs=dist_coeffs,
             rvecs=rvecs,
             tvecs=tvecs,
             mean_error=mean_error)

    print("\nParametros guardados en: camera_calibration.npz")

    # Generar código para usar en el script principal
    print("\n=== CODIGO PARA USAR EN EL SCRIPT PRINCIPAL ===")
    print("\n# Cargar calibracion")
    print("calib_data = np.load('camera_calibration.npz')")
    print("CAMERA_MATRIX = calib_data['camera_matrix']")
    print("DIST_COEFFS = calib_data['dist_coeffs']")
    print("\n============================================")

    return camera_matrix, dist_coeffs


def test_calibration():
    """
    Prueba la calibración en tiempo real
    """
    if not os.path.exists('camera_calibration.npz'):
        print("Error: Archivo de calibración no encontrado")
        return

    # Cargar calibración
    calib_data = np.load('camera_calibration.npz')
    camera_matrix = calib_data['camera_matrix']
    dist_coeffs = calib_data['dist_coeffs']

    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: No se pudo acceder a la cámara")
        return

    print("\n=== PRUEBA DE CALIBRACION ===")
    print("Mostrando comparacion: Original vs Undistorted")
    print("Presiona 'q' para salir\n")

    while True:
        ret, frame = cap.read()

        if not ret:
            break

        # Aplicar corrección de distorsión
        h, w = frame.shape[:2]
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            camera_matrix, dist_coeffs, (w, h), 1, (w, h)
        )

        undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs,
                                     None, new_camera_matrix)

        # Combinar imágenes
        combined = np.hstack((frame, undistorted))

        cv2.putText(combined, "Original", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(combined, "Corregida", (w + 10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow('Test de Calibracion', combined)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def main():
    print("=== CALIBRACION DE CAMARA ===")
    print("\nOpciones:")
    print("1. Capturar nuevas imagenes y calibrar")
    print("2. Calibrar usando imagenes existentes")
    print("3. Probar calibracion existente")
    print("4. Salir")

    choice = input("\nSelecciona una opcion (1-4): ")

    if choice == '1':
        if capture_calibration_images():
            calibrate_camera()
            print("\nDeseas probar la calibracion? (s/n): ")
            if input().lower() == 's':
                test_calibration()
    elif choice == '2':
        calibrate_camera()
    elif choice == '3':
        test_calibration()
    else:
        print("Saliendo...")


if __name__ == "__main__":
    main()
