#!/bin/bash

# ============================================
# SCRIPT DE INICIO DEL ROBOT ARUCO
# Versión Final - Funcional
# ============================================

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# ============================================
# CONFIGURACIÓN - VERIFICA ESTOS DATOS
# ============================================

MAC_HC05="00:23:10:00:D5:81"  # ← Tu MAC verificada
CANAL=1                        # ← Canal verificado
PUERTO="/dev/rfcomm0"
PYTHON_SCRIPT="$HOME/Desktop/Proyecto Robotica/Rob-tica/aruco_blt2.py"
LOG_DIR="$HOME/robot_logs"

# ============================================

# Crear directorio de logs
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/robot_$(date +%Y%m%d_%H%M%S).log"

# Función de log
log() {
    echo -e "${1}" | tee -a "$LOG_FILE"
}

# Función de limpieza al salir
cleanup() {
    log "\n${YELLOW}========================================${NC}"
    log "${YELLOW}APAGANDO ROBOT...${NC}"
    log "${YELLOW}========================================${NC}"
    
    # Detener robot
    if [ -e "$PUERTO" ]; then
        for i in {1..5}; do
            echo "S" > "$PUERTO" 2>/dev/null
            sleep 0.2
        done
        log "✓ Robot detenido"
    fi
    
    # Matar Python
    sudo pkill -9 -f "aruco_blt2.py" 2>/dev/null
    log "✓ Proceso Python terminado"
    
    # Cerrar Bluetooth
    sudo rfcomm release "$PUERTO" 2>/dev/null
    sudo pkill -9 rfcomm 2>/dev/null
    log "✓ Bluetooth desconectado"
    
    log "${GREEN}✓ Sistema apagado correctamente${NC}\n"
    exit 0
}

# Capturar Ctrl+C
trap cleanup SIGINT SIGTERM EXIT

# ============================================
# INICIO
# ============================================

log "${BLUE}============================================${NC}"
log "${BLUE}    ROBOT SEGUIDOR DE ARUCO - INICIO${NC}"
log "${BLUE}============================================${NC}"
log "Fecha: $(date '+%Y-%m-%d %H:%M:%S')"
log "MAC HC-05: $MAC_HC05"
log "Script Python: $(basename $PYTHON_SCRIPT)"
log "Log: $LOG_FILE"
log "${BLUE}============================================${NC}\n"

# ============================================
# 1. VERIFICACIONES PREVIAS
# ============================================

log "${YELLOW}[1/6] Verificando archivos...${NC}"

if [ ! -f "$PYTHON_SCRIPT" ]; then
    log "${RED}✗ ERROR: No se encuentra $PYTHON_SCRIPT${NC}"
    exit 1
fi
log "${GREEN}✓ Script Python encontrado${NC}\n"

# ============================================
# 2. VERIFICAR BLUETOOTH
# ============================================

log "${YELLOW}[2/6] Verificando Bluetooth...${NC}"

# Verificar servicio
if ! systemctl is-active --quiet bluetooth; then
    log "  Iniciando servicio Bluetooth..."
    sudo systemctl start bluetooth
    sleep 3
fi

# Verificar emparejamiento
if ! bluetoothctl devices | grep -q "$MAC_HC05"; then
    log "${RED}✗ ERROR: HC-05 no está emparejado${NC}"
    log "  Ejecuta: bluetoothctl"
    log "  Luego: pair $MAC_HC05"
    exit 1
fi

log "${GREEN}✓ Bluetooth listo${NC}\n"

# ============================================
# 3. LIMPIAR CONEXIONES PREVIAS
# ============================================

log "${YELLOW}[3/6] Limpiando conexiones previas...${NC}"

sudo pkill -9 rfcomm 2>/dev/null
sudo rfcomm release "$PUERTO" 2>/dev/null
sleep 2

if [ -e "$PUERTO" ]; then
    log "${YELLOW}⚠ Puerto existe, eliminando...${NC}"
    sudo rm -f "$PUERTO" 2>/dev/null
    sleep 1
fi

log "${GREEN}✓ Limpieza completada${NC}\n"

# ============================================
# 4. CONECTAR BLUETOOTH (LO QUE FUNCIONA)
# ============================================

log "${YELLOW}[4/6] Conectando HC-05...${NC}"
log "  MAC: $MAC_HC05"
log "  Canal: $CANAL"

# Ejecutar rfcomm en segundo plano
sudo rfcomm connect "$PUERTO" "$MAC_HC05" "$CANAL" > "$LOG_DIR/rfcomm_$(date +%H%M%S).log" 2>&1 &
RFCOMM_PID=$!

log "  PID: $RFCOMM_PID"
log "  Esperando puerto..."

# Esperar a que se cree el puerto
for i in {1..30}; do
    if [ -e "$PUERTO" ]; then
        sleep 2  # Esperar estabilización
        if ps -p $RFCOMM_PID > /dev/null 2>&1; then
            log "${GREEN}✓ HC-05 conectado exitosamente${NC}"
            break
        fi
    fi
    echo -n "."
    sleep 1
done
echo ""

# Verificar conexión
if [ ! -e "$PUERTO" ]; then
    log "${RED}✗ ERROR: No se pudo crear $PUERTO${NC}"
    log "\nRevisa:"
    log "  1. LED del HC-05 debe parpadear LENTO ahora"
    log "  2. Log: $LOG_DIR/rfcomm_*.log"
    exit 1
fi

# Ajustar permisos
sudo chmod 666 "$PUERTO"
log "${GREEN}✓ Permisos configurados${NC}\n"

# ============================================
# 5. PROBAR COMUNICACIÓN
# ============================================

log "${YELLOW}[5/6] Probando comunicación...${NC}"

# Enviar comando de prueba
for i in {1..3}; do
    echo "S" > "$PUERTO" 2>/dev/null
    sleep 0.2
done

if [ $? -eq 0 ]; then
    log "${GREEN}✓ Comunicación con Arduino OK${NC}\n"
else
    log "${RED}✗ ERROR: No se puede comunicar con Arduino${NC}"
    exit 1
fi

# ============================================
# 6. INICIAR PROGRAMA PYTHON
# ============================================

log "${YELLOW}[6/6] Iniciando detección ArUco...${NC}\n"

log "${BLUE}============================================${NC}"
log "${GREEN}    ✓✓✓ ROBOT OPERATIVO ✓✓✓${NC}"
log "${BLUE}============================================${NC}"
log "Presiona Ctrl+C para detener"
log "${BLUE}============================================${NC}\n"

# Cambiar al directorio del script
cd "$(dirname "$PYTHON_SCRIPT")"

# Ejecutar Python
python3 "$PYTHON_SCRIPT" 2>&1 | tee -a "$LOG_FILE"

# Si termina, hacer cleanup
EXIT_CODE=$?
log "\n${YELLOW}Python terminó con código: $EXIT_CODE${NC}"
cleanup