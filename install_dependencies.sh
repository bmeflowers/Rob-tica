#!/bin/bash

# Script de instalación automática de dependencias
# Detecta la plataforma y ajusta las dependencias según sea necesario

set -e  # Salir si hay errores

echo "==================================="
echo "Instalador de dependencias ARUco"
echo "==================================="

# Detectar si estamos en Raspberry Pi
is_raspberry_pi() {
    if [ -f /proc/device-tree/model ] && grep -q "Raspberry Pi" /proc/device-tree/model; then
        return 0
    elif uname -m | grep -qE "armv7l|aarch64"; then
        return 0
    else
        return 1
    fi
}

# Verificar si Python está instalado
if ! command -v python3 &> /dev/null; then
    echo "Error: Python3 no está instalado"
    exit 1
fi

echo "Python version: $(python3 --version)"
echo "Plataforma: $(uname -m)"
echo ""

if is_raspberry_pi; then
    echo "✓ Raspberry Pi detectada"
    echo "Usando instalación optimizada para ARM..."
    echo ""

    # Verificar si opencv ya está instalado desde apt
    if python3 -c "import cv2" 2>/dev/null; then
        echo "✓ OpenCV ya está instalado"
        CV_VERSION=$(python3 -c "import cv2; print(cv2.__version__)" 2>/dev/null)
        echo "  Versión actual: $CV_VERSION"
        echo ""
        read -p "¿Deseas reinstalar OpenCV? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            echo "Saltando instalación de OpenCV..."
            SKIP_OPENCV=true
        fi
    fi

    if [ "$SKIP_OPENCV" != true ]; then
        echo "Opción 1: Instalar desde repositorios del sistema (recomendado, más rápido)"
        echo "Opción 2: Instalar opencv-python-headless vía pip (más lento)"
        echo ""
        read -p "Selecciona opción (1/2) [1]: " INSTALL_OPTION
        INSTALL_OPTION=${INSTALL_OPTION:-1}

        if [ "$INSTALL_OPTION" = "1" ]; then
            echo ""
            echo "Instalando OpenCV desde repositorios del sistema..."
            sudo apt update
            sudo apt install -y python3-opencv python3-numpy
            echo "✓ OpenCV instalado desde apt"
        else
            echo ""
            echo "Instalando dependencias del sistema necesarias..."
            sudo apt update
            sudo apt install -y python3-dev build-essential cmake pkg-config \
                libjpeg-dev libpng-dev libavcodec-dev libavformat-dev \
                libswscale-dev libatlas-base-dev gfortran

            echo ""
            echo "Instalando numpy..."
            pip3 install --upgrade pip
            pip3 install "numpy>=1.24.0"

            echo ""
            echo "Instalando opencv-python-headless (esto puede tardar varios minutos)..."
            pip3 install opencv-python-headless
            echo "✓ OpenCV instalado vía pip"
        fi
    fi

    # Instalar otras dependencias si hay
    if [ -f "requirements_rpi.txt" ]; then
        echo ""
        echo "Instalando dependencias adicionales..."
        pip3 install -r requirements_rpi.txt
    fi

else
    echo "✓ Sistema de escritorio detectado"
    echo "Instalando desde requirements.txt..."
    echo ""

    # Actualizar pip
    pip3 install --upgrade pip

    # Instalar desde requirements.txt normal
    pip3 install -r requirements.txt
    echo "✓ Dependencias instaladas"
fi

echo ""
echo "==================================="
echo "Verificando instalación..."
echo "==================================="

# Verificar que OpenCV funciona
if python3 -c "import cv2; import numpy as np; print('OpenCV version:', cv2.__version__); print('NumPy version:', np.__version__)" 2>/dev/null; then
    echo "✓ Instalación exitosa"
    echo ""
    echo "Puedes ejecutar tu proyecto con:"
    echo "  python3 aruco_distance_rotation.py"
else
    echo "✗ Error en la instalación"
    echo "Por favor, verifica los mensajes de error arriba"
    exit 1
fi

echo "==================================="
