#!/usr/bin/env python3
"""
Script de diagnóstico de cámara para Raspberry Pi
Ayuda a identificar problemas con el acceso a la cámara
"""

import cv2
import sys
import subprocess
import os
import platform

# Detectar si estamos en Raspberry Pi
IS_RASPBERRY_PI = platform.machine().startswith('arm') or platform.machine().startswith('aarch64')


def check_picamera2():
    """Verifica si picamera2 está instalado y funciona"""
    print("=" * 60)
    print("1. VERIFICANDO PICAMERA2 (Raspberry Pi)")
    print("=" * 60)

    if not IS_RASPBERRY_PI:
        print("⊘ No es una Raspberry Pi, saltando verificación de picamera2")
        return False

    try:
        from picamera2 import Picamera2
        print("✓ picamera2 está instalado")

        # Intentar inicializar
        try:
            print("\nIntentando inicializar cámara con picamera2...")
            picam2 = Picamera2()
            config = picam2.create_preview_configuration(
                main={"size": (640, 480), "format": "RGB888"}
            )
            picam2.configure(config)
            picam2.start()

            # Capturar un frame de prueba
            frame = picam2.capture_array()
            if frame is not None:
                height, width = frame.shape[:2]
                print(f"✓ picamera2 FUNCIONA correctamente")
                print(f"  Resolución: {width}x{height}")
                picam2.stop()
                return True
            else:
                print("✗ No se pudo capturar frame con picamera2")
                picam2.stop()
                return False

        except Exception as e:
            print(f"✗ Error al usar picamera2: {e}")
            return False

    except ImportError:
        print("✗ picamera2 NO está instalado")
        print("  Instalar con: sudo apt install -y python3-picamera2")
        return False


def check_camera_devices():
    """Verifica dispositivos de cámara disponibles en el sistema"""
    print("\n" + "=" * 60)
    print("2. VERIFICANDO DISPOSITIVOS DE CÁMARA")
    print("=" * 60)

    try:
        result = subprocess.run(['ls', '-l', '/dev/video*'],
                              capture_output=True, text=True)
        if result.returncode == 0:
            print("✓ Dispositivos de video encontrados:")
            print(result.stdout)
        else:
            print("✗ No se encontraron dispositivos /dev/video*")
            print("  ¿Está la cámara conectada?")
            return False
    except Exception as e:
        print(f"✗ Error al buscar dispositivos: {e}")
        return False

    return True


def check_user_permissions():
    """Verifica si el usuario tiene permisos para acceder a la cámara"""
    print("\n" + "=" * 60)
    print("3. VERIFICANDO PERMISOS DE USUARIO")
    print("=" * 60)

    try:
        result = subprocess.run(['groups'], capture_output=True, text=True)
        groups = result.stdout.strip()
        print(f"Grupos del usuario: {groups}")

        if 'video' in groups:
            print("✓ Usuario pertenece al grupo 'video'")
            return True
        else:
            print("✗ Usuario NO pertenece al grupo 'video'")
            print("  Solución: sudo usermod -a -G video $USER")
            print("  Luego cierra sesión y vuelve a entrar")
            return False
    except Exception as e:
        print(f"✗ Error al verificar grupos: {e}")
        return False


def test_opencv_backends():
    """Prueba diferentes backends de OpenCV"""
    print("\n" + "=" * 60)
    print("4. PROBANDO BACKENDS DE OPENCV")
    print("=" * 60)

    backends = [
        (cv2.CAP_V4L2, "V4L2", "Video4Linux2 - Nativo de Linux"),
        (cv2.CAP_GSTREAMER, "GStreamer", "Framework multimedia"),
        (cv2.CAP_FFMPEG, "FFmpeg", "Framework multimedia"),
        (cv2.CAP_ANY, "CAP_ANY", "Detección automática"),
    ]

    working_backends = []

    for backend_id, name, description in backends:
        print(f"\nProbando {name} ({description})...")
        try:
            cap = cv2.VideoCapture(0, backend_id)

            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    height, width = frame.shape[:2]
                    print(f"  ✓ {name} FUNCIONA")
                    print(f"    Resolución: {width}x{height}")
                    working_backends.append((backend_id, name))
                    cap.release()
                else:
                    print(f"  ✗ {name} abre pero no puede leer frames")
                    cap.release()
            else:
                print(f"  ✗ {name} no puede abrir la cámara")
                cap.release()
        except Exception as e:
            print(f"  ✗ {name} error: {e}")

    return working_backends


def test_simple_capture():
    """Prueba captura simple sin especificar backend"""
    print("\n" + "=" * 60)
    print("5. PRUEBA DE CAPTURA SIMPLE (OpenCV)")
    print("=" * 60)

    print("\nIntentando cv2.VideoCapture(0)...")
    try:
        cap = cv2.VideoCapture(0)

        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None:
                height, width = frame.shape[:2]
                print(f"✓ Captura exitosa: {width}x{height}")
                print(f"  Backend usado: {cap.getBackendName()}")
                cap.release()
                return True
            else:
                print("✗ Abrió pero no pudo leer frames")
                cap.release()
        else:
            print("✗ No pudo abrir la cámara")
            cap.release()
    except Exception as e:
        print(f"✗ Error: {e}")

    return False


def check_v4l2_utils():
    """Verifica si v4l2-utils está instalado"""
    print("\n" + "=" * 60)
    print("6. VERIFICANDO HERRAMIENTAS DEL SISTEMA")
    print("=" * 60)

    try:
        result = subprocess.run(['which', 'v4l2-ctl'],
                              capture_output=True, text=True)
        if result.returncode == 0:
            print("✓ v4l2-ctl está instalado")

            # Mostrar información de la cámara
            result = subprocess.run(['v4l2-ctl', '--list-devices'],
                                  capture_output=True, text=True)
            print("\nDispositivos de video:")
            print(result.stdout)

            # Mostrar formatos soportados
            result = subprocess.run(['v4l2-ctl', '-d', '/dev/video0', '--list-formats-ext'],
                                  capture_output=True, text=True)
            print("\nFormatos soportados por /dev/video0:")
            print(result.stdout[:500])  # Primeras líneas

        else:
            print("✗ v4l2-ctl no está instalado")
            print("  Instalar: sudo apt install v4l-utils")
    except Exception as e:
        print(f"✗ Error: {e}")


def main():
    print("\n")
    print("╔" + "=" * 58 + "╗")
    print("║" + " " * 10 + "DIAGNÓSTICO DE CÁMARA - RASPBERRY PI" + " " * 11 + "║")
    print("╚" + "=" * 58 + "╝")
    print()

    # Ejecutar diagnósticos
    picamera2_works = check_picamera2()
    devices_ok = check_camera_devices()
    permissions_ok = check_user_permissions()

    if not devices_ok:
        print("\n⚠️  No se detectaron cámaras. Verifica la conexión.")
        sys.exit(1)

    check_v4l2_utils()
    working_backends = test_opencv_backends()
    simple_works = test_simple_capture()

    # Resumen
    print("\n" + "=" * 60)
    print("RESUMEN Y RECOMENDACIONES")
    print("=" * 60)

    if IS_RASPBERRY_PI:
        if picamera2_works:
            print("\n✓✓ RECOMENDACIÓN: picamera2 funciona perfectamente")
            print("   Tu proyecto está configurado para usar picamera2 automáticamente")
            print("   Puedes ejecutar: python3 aruco_distance_rotation.py")
        else:
            print("\n⚠️  picamera2 no está funcionando")
            print("   Soluciones:")
            print("   1. Instala picamera2: sudo apt install -y python3-picamera2")
            print("   2. Habilita la cámara: sudo raspi-config")
            print("      Selecciona: Interface Options > Camera > Enable")
            print("   3. Reinicia la Raspberry Pi")
            print("\n   El sistema intentará usar OpenCV como alternativa...")

    if working_backends:
        print("\n✓ Backends que funcionan:")
        for backend_id, name in working_backends:
            print(f"  - {name}")
        print("\nRecomendación: Usa el primer backend de la lista")
    else:
        print("\n✗ No se encontró ningún backend funcional")
        print("\nSoluciones sugeridas:")
        print("1. Verifica que la cámara esté conectada correctamente")
        print("2. Reinicia la Raspberry Pi")
        print("3. Si usas cámara oficial RPi:")
        print("   - Habilita la cámara: sudo raspi-config")
        print("   - Selecciona: Interface Options > Camera > Enable")
        print("4. Si usas cámara USB:")
        print("   - Prueba otro puerto USB")
        print("   - Verifica con: lsusb")

    if not permissions_ok:
        print("\n⚠️  ACCIÓN REQUERIDA: Añadir usuario al grupo video")
        print("    sudo usermod -a -G video $USER")
        print("    Luego cierra sesión y vuelve a entrar")

    if simple_works:
        print("\n✓ La cámara está funcionando correctamente")
        print("  Puedes ejecutar: python3 aruco_distance_rotation.py")
    else:
        print("\n✗ Hay problemas con el acceso a la cámara")
        print("  Revisa las soluciones sugeridas arriba")

    print("\n" + "=" * 60 + "\n")


if __name__ == "__main__":
    main()
