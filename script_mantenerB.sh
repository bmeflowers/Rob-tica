#!/bin/bash

# ============================================
# SCRIPT PARA MANTENER CONEXIÓN BLUETOOTH
# ============================================

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# CONFIGURACIÓN - CAMBIAR SEGÚN TU HC-05
MAC_HC05="98:D3:31:FB:XX:XX"  # ← CAMBIAR ESTO
PUERTO="/dev/rfcomm0"
CANAL=1

# ============================================

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  GESTOR DE CONEXIÓN BLUETOOTH HC-05${NC}"
echo -e "${BLUE}========================================${NC}"

# Función para limpiar al salir
cleanup() {
    echo ""
    echo -e "${YELLOW}Cerrando conexión...${NC}"
    echo "S" > $PUERTO 2>/dev/null  # Detener robot
    sudo rfcomm release $PUERTO 2>/dev/null
    sudo pkill -9 rfcomm 2>/dev/null
    echo -e "${GREEN}✓ Conexión cerrada${NC}"
    exit 0
}

# Capturar Ctrl+C
trap cleanup SIGINT SIGTERM

# Verificar que el Bluetooth está activo
echo "1. Verificando servicio Bluetooth..."
if systemctl is-active --quiet bluetooth; then
    echo -e "${GREEN}✓ Bluetooth activo${NC}"
else
    echo -e "${YELLOW}⚠ Bluetooth inactivo. Iniciando...${NC}"
    sudo systemctl start bluetooth
    sleep 2
fi

# Limpiar conexiones previas
echo ""
echo "2. Limpiando conexiones previas..."
sudo pkill -9 rfcomm 2>/dev/null
sudo rfcomm release $PUERTO 2>/dev/null
sleep 1
echo -e "${GREEN}✓ Limpieza completada${NC}"

# Conectar
echo ""
echo "3. Conectando al HC-05..."
echo "   MAC: $MAC_HC05"
echo "   Puerto: $PUERTO"

# Ejecutar rfcomm connect en segundo plano
sudo rfcomm connect $PUERTO $MAC_HC05 $CANAL > /tmp/rfcomm.log 2>&1 &
RFCOMM_PID=$!

# Esperar a que se cree el dispositivo
echo -n "   Esperando"
for i in {1..15}; do
    if [ -e $PUERTO ]; then
        echo ""
        echo -e "${GREEN}✓ Conexión establecida${NC}"
        sleep 1
        break
    fi
    echo -n "."
    sleep 1
done

# Verificar que se creó
if [ ! -e $PUERTO ]; then
    echo ""
    echo -e "${RED}✗ Error: No se pudo crear $PUERTO${NC}"
    echo ""
    echo "Log de rfcomm:"
    cat /tmp/rfcomm.log
    echo ""
    echo "Verifica:"
    echo "  - HC-05 encendido (LED parpadeando)"
    echo "  - MAC correcta: $MAC_HC05"
    echo "  - Bluetooth emparejado: bluetoothctl paired-devices"
    cleanup
fi

# Dar permisos
sudo chmod 666 $PUERTO

# Verificar permisos
echo ""
echo "4. Verificando permisos..."
ls -l $PUERTO
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Permisos OK${NC}"
fi

# Prueba de conexión
echo ""
echo "5. Probando conexión..."
echo "S" > $PUERTO 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Puede enviar comandos${NC}"
else
    echo -e "${RED}✗ No puede enviar comandos${NC}"
    cleanup
fi

# Información
echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}✓✓✓ CONEXIÓN LISTA ✓✓✓${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo "Dispositivo: $PUERTO"
echo "Estado: Conectado y funcionando"
echo ""
echo "Comandos disponibles:"
echo "  F - Avanzar"
echo "  B - Retroceder"
echo "  L - Girar izquierda"
echo "  R - Girar derecha"
echo "  S - Detener"
echo "  G - Avanzar despacio"
echo "  T - Test de motores"
echo ""
echo "Presiona Ctrl+C para cerrar la conexión"
echo -e "${BLUE}========================================${NC}"

# Mantener el script corriendo
echo ""
echo "Conexión activa. Esperando..."
echo "(Puedes abrir otra terminal para usar el robot)"
echo ""

# Loop infinito para mantener el script vivo
while true; do
    # Verificar que rfcomm sigue corriendo
    if ! ps -p $RFCOMM_PID > /dev/null 2>&1; then
        echo -e "${RED}✗ Proceso rfcomm terminó inesperadamente${NC}"
        echo "Intentando reconectar..."
        
        sudo rfcomm connect $PUERTO $MAC_HC05 $CANAL > /tmp/rfcomm.log 2>&1 &
        RFCOMM_PID=$!
        
        sleep 5
        
        if [ ! -e $PUERTO ]; then
            echo -e "${RED}✗ No se pudo reconectar${NC}"
            cleanup
        fi
        
        sudo chmod 666 $PUERTO
        echo -e "${GREEN}✓ Reconectado${NC}"
    fi
    
    sleep 5
done