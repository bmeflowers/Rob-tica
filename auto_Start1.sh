#!/bin/bash

# ============================================
# AUTO-INICIO DEL ROBOT ARUCO FOLLOWER
# Versión Optimizada para Systemd
# ============================================

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# ============================================
# CONFIGURACIÓN
# ============================================

# MAC del HC-05 - CÁMBIALA POR LA TUYA
MAC_HC05="00:23:10:00:D5:81"

# Rutas
PYTHON_SCRIPT="$HOME/Desktop/Proyecto Robotica/Rob-tica/aruco_blt2.py"
LOG_DIR="$HOME/robot_logs"
LOG_FILE="$LOG_DIR/robot_$(date +%Y%m%d_%H%M%S).log"
PUERTO="/dev/rfcomm0"
CANAL=1

# ============================================

# Crear directorio de logs
mkdir -p "$LOG_DIR"

# Función de log
log() {
    echo -e "${1}" | tee -a "$LOG_FILE"
}

# Función de limpieza
cleanup() {
    log "\n${YELLOW}========================================${NC}"
    log "${YELLOW}Apagando sistema del robot...${NC}"
    log "${YELLOW}========================================${NC}"
    
    # Detener el robot
    if [ -e "$PUERTO" ]; then
        echo "S" > "$PUERTO" 2>/dev/null
        log "✓ Robot detenido"
    fi
    
    # Matar proceso Python
    sudo pkill -9 -f "aruco_blt2.py" 2>/dev/null
    log "✓ Proceso Python terminado"
    
    # Cerrar Bluetooth
    sudo rfcomm release "$PUERTO" 2>/dev/null
    sudo pkill -9 rfcomm 2>/dev/null
    log "✓ Bluetooth desconectado"
    
    log "${GREEN}Sistema apagado correctamente${NC}"
    exit 0
}

# Capturar señales de terminación
trap cleanup SIGINT SIGTERM EXIT

# ============================================
# INICIO
# ============================================

log "${BLUE}============================================${NC}"
log "${BLUE}   ROBOT ARUCO FOLLOWER - AUTO INICIO${NC}"
log "${BLUE}============================================${NC}"
log "Fecha: $(date '+%Y-%m-%d %H:%M:%S')"
log "Usuario: $(whoami)"
log "MAC HC-05: $MAC_HC05"
log "${BLUE}============================================${NC}\n"

# ============================================
# FASE 1: VERIFICAR ARCHIVOS
# ============================================

log "${YELLOW}Fase 1: Verificando archivos...${NC}"

if [ ! -f "$PYTHON_SCRIPT" ]; then
    log "${RED}✗ ERROR: No se encuentra $PYTHON_SCRIPT${NC}"
    exit 1
fi
log "${GREEN}✓ Script Python encontrado${NC}\n"

# ============================================
# FASE 2: VERIFICAR BLUETOOTH
# ============================================

log "${YELLOW}Fase 2: Verificando Bluetooth...${NC}"

# Verificar servicio Bluetooth
if ! systemctl is-active --quiet bluetooth; then
    log "${YELLOW}⚠ Iniciando servicio Bluetooth...${NC}"
    sudo systemctl start bluetooth
    sleep 3
fi
log "${GREEN}✓ Servicio Bluetooth activo${NC}"

# Verificar que el HC-05 está emparejado
if ! bluetoothctl paired-devices | grep -q "$MAC_HC05"; then
    log "${RED}✗ ERROR: HC-05 no está emparejado${NC}"
    log "   Ejecuta: bluetoothctl"
    log "   Luego: pair $MAC_HC05"
    exit 1
fi
log "${GREEN}✓ HC-05 emparejado${NC}\n"

# ============================================
# FASE 3: LIMPIAR CONEXIONES PREVIAS
# ============================================

log "${YELLOW}Fase 3: Limpiando conexiones previas...${NC}"

sudo pkill -9 rfcomm 2>/dev/null
sudo rfcomm release "$PUERTO" 2>/dev/null
sleep 2

log "${GREEN}✓ Limpieza completada${NC}\n"

# ============================================
# FASE 4: ESTABLECER CONEXIÓN BLUETOOTH
# ============================================

log "${YELLOW}Fase 4: Conectando al HC-05...${NC}"
log "   MAC: $MAC_HC05"
log "   Puerto: $PUERTO"

# Conectar en segundo plano
sudo rfcomm connect "$PUERTO" "$MAC_HC05" "$CANAL" > "$LOG_DIR/rfcomm.log" 2>&1 &
RFCOMM_PID=$!

log "   PID rfcomm: $RFCOMM_PID"
log "   Esperando conexión"

# Esperar a que se cree el puerto
TIMEOUT=0
MAX_TIMEOUT=30

while [ $TIMEOUT -lt $MAX_TIMEOUT ]; do
    if [ -e "$PUERTO" ]; then
        log "${GREEN}✓ Puerto creado: $PUERTO${NC}"
        break
    fi
    echo -n "."
    sleep 1
    TIMEOUT=$((TIMEOUT + 1))
done
echo ""

# Verificar creación del puerto
if [ ! -e "$PUERTO" ]; then
    log "${RED}✗ ERROR: No se pudo crear $PUERTO${NC}"
    log "\nLog de rfcomm:"
    cat "$LOG_DIR/rfcomm.log"
    log "\nVerifica:"
    log "  1. HC-05 encendido (LED parpadeando)"
    log "  2. MAC correcta: $MAC_HC05"
    log "  3. HC-05 emparejado: bluetoothctl paired-devices"
    exit 1
fi

# Ajustar permisos
sleep 2
sudo chmod 666 "$PUERTO"

# Verificar permisos
if [ ! -w "$PUERTO" ]; then
    log "${RED}✗ ERROR: Sin permisos de escritura en $PUERTO${NC}"
    exit 1
fi
log "${GREEN}✓ Permisos configurados${NC}"

# ============================================
# FASE 5: PROBAR CONEXIÓN
# ============================================

log "\n${YELLOW}Fase 5: Probando comunicación...${NC}"

# Enviar comando de prueba
echo "S" > "$PUERTO" 2>/dev/null
if [ $? -eq 0 ]; then
    log "${GREEN}✓ Comunicación exitosa${NC}"
else
    log "${RED}✗ ERROR: No se puede enviar comandos${NC}"
    exit 1
fi

sleep 1

# ============================================
# FASE 6: INICIAR PYTHON
# ============================================

log "\n${YELLOW}Fase 6: Iniciando detección ArUco...${NC}\n"
log "${BLUE}============================================${NC}"
log "${GREEN}      ✓✓✓ SISTEMA INICIADO ✓✓✓${NC}"
log "${BLUE}============================================${NC}"
log "Robot operativo - Buscando marcador ArUco"
log "Log: $LOG_FILE"
log "${BLUE}============================================${NC}\n"

# Cambiar al directorio del script
cd "$(dirname "$PYTHON_SCRIPT")"

# Ejecutar Python con output redirigido
python3 "$PYTHON_SCRIPT" 2>&1 | tee -a "$LOG_FILE"

# Si Python termina, hacer cleanup
PYTHON_EXIT_CODE=$?
log "\n${YELLOW}Programa Python terminó con código: $PYTHON_EXIT_CODE${NC}"

cleanup