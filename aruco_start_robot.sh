#!/bin/bash

# ============================================
# AUTO-INICIO DEL ROBOT ARUCO FOLLOWER
# Script que integra:
# 1. Tu script de conexión Bluetooth existente
# 2. Tu código Python de detección ArUco
# ============================================

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# ============================================
# CONFIGURACIÓN - AJUSTAR SEGÚN TU SISTEMA
# ============================================

# Ruta a tu script de Bluetooth (mantener_bluetooth.sh)
BLUETOOTH_SCRIPT="$HOME/Desktop/Proyecto Robotica/Rob-tica/script_mantenerB3.sh"

# Ruta a tu código Python de ArUco
PYTHON_SCRIPT="$HOME/Desktop/Proyecto Robotica/Rob-tica/aruco_blt.py"

# Directorio de logs
LOG_DIR="$HOME/robot_logs"
LOG_FILE="$LOG_DIR/robot_$(date +%Y%m%d_%H%M%S).log"

# Puerto Bluetooth
PUERTO="/dev/rfcomm0"

# ============================================

# Crear directorio de logs
mkdir -p "$LOG_DIR"

# Función de log
log() {
    echo -e "${1}" | tee -a "$LOG_FILE"
}

# Función de limpieza
cleanup() {
    log "${YELLOW}========================================${NC}"
    log "${YELLOW}Apagando sistema del robot...${NC}"
    log "${YELLOW}========================================${NC}"
    
    # Detener el robot
    if [ -e "$PUERTO" ]; then
        echo "S" > "$PUERTO" 2>/dev/null
        log "✓ Robot detenido"
    fi
    
    # Matar proceso Python
    sudo pkill -9 -f "aruco_btl.py" 2>/dev/null
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
# INICIO DEL SISTEMA
# ============================================

log "${BLUE}============================================${NC}"
log "${BLUE}   ROBOT ARUCO FOLLOWER - AUTO INICIO${NC}"
log "${BLUE}============================================${NC}"
log "Fecha: $(date '+%Y-%m-%d %H:%M:%S')"
log "Usuario: $(whoami)"
log "Directorio: $(pwd)"
log "${BLUE}============================================${NC}"
log ""

# ============================================
# FASE 1: ESPERAR A QUE EL SISTEMA ESTÉ LISTO
# ============================================

log "${YELLOW}Fase 1: Esperando inicialización del sistema...${NC}"
log "  (Esperando 15 segundos para que todo esté listo)"

for i in {15..1}; do
    echo -n "."
    sleep 1
done
echo ""

log "${GREEN}✓ Sistema listo${NC}"
log ""

# ============================================
# FASE 2: VERIFICAR ARCHIVOS
# ============================================

log "${YELLOW}Fase 2: Verificando archivos necesarios...${NC}"

# Verificar script de Bluetooth
if [ ! -f "$BLUETOOTH_SCRIPT" ]; then
    log "${RED}✗ ERROR: No se encuentra el script de Bluetooth${NC}"
    log "   Buscado en: $BLUETOOTH_SCRIPT"
    log ""
    log "Solución:"
    log "  1. Verifica que el archivo existe"
    log "  2. O cambia la ruta en este script (línea BLUETOOTH_SCRIPT)"
    exit 1
fi
log "${GREEN}✓ Script de Bluetooth encontrado${NC}"
log "   Ubicación: $BLUETOOTH_SCRIPT"

# Verificar script Python
if [ ! -f "$PYTHON_SCRIPT" ]; then
    log "${RED}✗ ERROR: No se encuentra el código Python${NC}"
    log "   Buscado en: $PYTHON_SCRIPT"
    log ""
    log "Solución:"
    log "  1. Verifica que el archivo existe"
    log "  2. O cambia la ruta en este script (línea PYTHON_SCRIPT)"
    exit 1
fi
log "${GREEN}✓ Código Python encontrado${NC}"
log "   Ubicación: $PYTHON_SCRIPT"
log ""

# ============================================
# FASE 3: INICIAR CONEXIÓN BLUETOOTH
# ============================================

log "${YELLOW}Fase 3: Iniciando conexión Bluetooth...${NC}"
log "  Ejecutando: $BLUETOOTH_SCRIPT"
log ""

# Dar permisos de ejecución si no los tiene
chmod +x "$BLUETOOTH_SCRIPT" 2>/dev/null

# Ejecutar script de Bluetooth en segundo plano
"$BLUETOOTH_SCRIPT" > "$LOG_DIR/bluetooth_$(date +%Y%m%d_%H%M%S).log" 2>&1 &
BLUETOOTH_PID=$!

log "  PID del proceso Bluetooth: $BLUETOOTH_PID"
log "  Esperando a que se establezca la conexión..."

# Esperar a que se cree el puerto /dev/rfcomm0
TIMEOUT=0
MAX_TIMEOUT=30

while [ $TIMEOUT -lt $MAX_TIMEOUT ]; do
    if [ -e "$PUERTO" ]; then
        log "${GREEN}✓ Puerto Bluetooth creado: $PUERTO${NC}"
        sleep 2  # Esperar un poco más para estabilizar
        break
    fi
    
    echo -n "."
    sleep 1
    TIMEOUT=$((TIMEOUT + 1))
done
echo ""

# Verificar que se creó el puerto
if [ ! -e "$PUERTO" ]; then
    log "${RED}✗ ERROR: No se pudo crear $PUERTO después de $MAX_TIMEOUT segundos${NC}"
    log ""
    log "Posibles causas:"
    log "  1. HC-05 no está encendido"
    log "  2. HC-05 no está emparejado"
    log "  3. MAC incorrecta en mantener_bluetooth.sh"
    log ""
    log "Log del Bluetooth:"
    cat "$LOG_DIR/bluetooth_"*.log 2>/dev/null | tail -20
    exit 1
fi

# Verificar permisos del puerto
if [ -w "$PUERTO" ]; then
    log "${GREEN}✓ Puerto Bluetooth accesible y con permisos de escritura${NC}"
else
    log "${YELLOW}⚠ Ajustando permisos del puerto...${NC}"
    sudo chmod 666 "$PUERTO"
    sleep 1
    if [ -w "$PUERTO" ]; then
        log "${GREEN}✓ Permisos ajustados correctamente${NC}"
    else
        log "${RED}✗ ERROR: No se pudieron ajustar los permisos${NC}"
        exit 1
    fi
fi

log ""

# ============================================
# FASE 4: PROBAR CONEXIÓN BLUETOOTH
# ============================================

log "${YELLOW}Fase 4: Probando comunicación con Arduino...${NC}"

# Enviar comando de prueba (detener)
echo "S" > "$PUERTO" 2>/dev/null
if [ $? -eq 0 ]; then
    log "${GREEN}✓ Comando enviado correctamente${NC}"
    sleep 1
else
    log "${RED}✗ ERROR: No se pudo enviar comando${NC}"
    exit 1
fi

log ""

# ============================================
# FASE 5: INICIAR PROGRAMA PYTHON
# ============================================

log "${YELLOW}Fase 5: Iniciando detección de ArUco...${NC}"
log ""
log "${BLUE}============================================${NC}"
log "${GREEN}      ✓✓✓ SISTEMA INICIADO ✓✓✓${NC}"
log "${BLUE}============================================${NC}"
log "Robot operativo y buscando marcador ArUco"
log "Presiona Ctrl+C para detener"
log "${BLUE}============================================${NC}"
log ""

# Cambiar al directorio del script Python
cd "$(dirname "$PYTHON_SCRIPT")"

# Ejecutar el programa Python
# El output se guarda en el log principal
python3 "$(basename "$PYTHON_SCRIPT")" 2>&1 | tee -a "$LOG_FILE"

# Si el programa Python termina, ejecutar cleanup
log ""
log "${YELLOW}Programa Python terminado${NC}"
cleanup