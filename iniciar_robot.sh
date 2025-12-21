#!/bin/bash

# ============================================
# SCRIPT DE INICIO DEL ROBOT ARUCO
# Versión Corregida - Con Keepalive y Watchdog
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

# Variables globales
RFCOMM_PID=""
WATCHDOG_PID=""

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
    
    # Matar watchdog primero
    if [ ! -z "$WATCHDOG_PID" ]; then
        kill $WATCHDOG_PID 2>/dev/null
        log "✓ Watchdog detenido"
    fi
    
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
}

# ⚡ CORREGIDO: Solo capturar interrupciones del usuario
trap cleanup SIGINT SIGTERM

# ============================================
# INICIO
# ============================================

log "${BLUE}============================================${NC}"
log "${BLUE}    ROBOT SEGUIDOR DE ARUCO - INICIO${NC}"
log "${BLUE}============================================${NC}"
log "Fecha: $(date '+%Y-%m-%d %H:%M:%S')"
log "MAC HC-05: $MAC_HC05"
log "Script Python: $(basename "$PYTHON_SCRIPT")"
log "Log: $LOG_FILE"
log "${BLUE}============================================${NC}\n"

# ============================================
# 1. VERIFICACIONES PREVIAS
# ============================================

log "${YELLOW}[1/7] Verificando archivos...${NC}"

if [ ! -f "$PYTHON_SCRIPT" ]; then
    log "${RED}✗ ERROR: No se encuentra $PYTHON_SCRIPT${NC}"
    exit 1
fi

# ⚡ NUEVO: Verificar sintaxis de Python
log "  Verificando sintaxis Python..."
if ! python3 -m py_compile "$PYTHON_SCRIPT" 2>/dev/null; then
    log "${RED}✗ ERROR: El script Python tiene errores de sintaxis${NC}"
    log "  Ejecuta: python3 '$PYTHON_SCRIPT'"
    exit 1
fi

log "${GREEN}✓ Script Python válido${NC}\n"

# ============================================
# 2. VERIFICAR BLUETOOTH
# ============================================

log "${YELLOW}[2/7] Verificando Bluetooth...${NC}"

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

log "${YELLOW}[3/7] Limpiando conexiones previas...${NC}"

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
# 4. CONECTAR BLUETOOTH
# ============================================

log "${YELLOW}[4/7] Conectando HC-05...${NC}"
log "  MAC: $MAC_HC05"
log "  Canal: $CANAL"

# Ejecutar rfcomm en segundo plano
sudo rfcomm connect "$PUERTO" "$MAC_HC05" "$CANAL" > "$LOG_DIR/rfcomm_$(date +%H%M%S).log" 2>&1 &
RFCOMM_PID=$!

log "  PID rfcomm: $RFCOMM_PID"
log "  Esperando puerto..."

# Esperar a que se cree el puerto
CONECTADO=false
for i in {1..30}; do
    if [ -e "$PUERTO" ]; then
        sleep 2  # Esperar estabilización
        if ps -p $RFCOMM_PID > /dev/null 2>&1; then
            log "${GREEN}✓ HC-05 conectado exitosamente${NC}"
            CONECTADO=true
            break
        fi
    fi
    echo -n "."
    sleep 1
done
echo ""

# Verificar conexión
if [ "$CONECTADO" = false ]; then
    log "${RED}✗ ERROR: No se pudo crear $PUERTO${NC}"
    log "\nRevisa:"
    log "  1. LED del HC-05 debe parpadear LENTO ahora"
    log "  2. Log: $LOG_DIR/rfcomm_*.log"
    log "  3. cat $LOG_DIR/rfcomm_*.log"
    exit 1
fi

# Ajustar permisos
sudo chmod 666 "$PUERTO"
log "${GREEN}✓ Permisos configurados${NC}\n"

# ============================================
# 5. PROBAR COMUNICACIÓN
# ============================================

log "${YELLOW}[5/7] Probando comunicación...${NC}"

# Enviar comando de prueba
COMUNICACION_OK=false
for i in {1..3}; do
    if echo "S" > "$PUERTO" 2>/dev/null; then
        COMUNICACION_OK=true
    fi
    sleep 0.2
done

if [ "$COMUNICACION_OK" = true ]; then
    log "${GREEN}✓ Comunicación con Arduino OK${NC}\n"
else
    log "${RED}✗ ERROR: No se puede comunicar con Arduino${NC}"
    cleanup
    exit 1
fi

# ============================================
# 6. WATCHDOG DE BLUETOOTH
# ============================================

log "${YELLOW}[6/7] Iniciando watchdog de Bluetooth...${NC}"

# ⚡ NUEVO: Watchdog que reinicia rfcomm si muere
(
    while true; do
        # Verificar si rfcomm sigue vivo
        if ! ps -p $RFCOMM_PID > /dev/null 2>&1; then
            echo "$(date '+%H:%M:%S') - ⚠ rfcomm murió (PID: $RFCOMM_PID), reconectando..." >> "$LOG_FILE"
            
            # Limpiar puerto
            sudo rfcomm release "$PUERTO" 2>/dev/null
            sleep 1
            
            # Reconectar
            sudo rfcomm connect "$PUERTO" "$MAC_HC05" "$CANAL" >> "$LOG_DIR/rfcomm_$(date +%H%M%S).log" 2>&1 &
            RFCOMM_PID=$!
            
            # Esperar conexión
            for j in {1..10}; do
                if [ -e "$PUERTO" ]; then
                    sudo chmod 666 "$PUERTO"
                    echo "$(date '+%H:%M:%S') - ✓ rfcomm reconectado (PID: $RFCOMM_PID)" >> "$LOG_FILE"
                    break
                fi
                sleep 1
            done
        fi
        
        # Verificar cada 5 segundos
        sleep 5
    done
) &
WATCHDOG_PID=$!

log "  PID watchdog: $WATCHDOG_PID"
log "${GREEN}✓ Watchdog activo - Reiniciará Bluetooth si se cae${NC}\n"

# ============================================
# 7. INICIAR PROGRAMA PYTHON
# ============================================

log "${YELLOW}[7/7] Iniciando detección ArUco...${NC}\n"

log "${BLUE}============================================${NC}"
log "${GREEN}    ✓✓✓ ROBOT OPERATIVO ✓✓✓${NC}"
log "${BLUE}============================================${NC}"
log "⚡ Keepalive activo en Python"
log "⚡ Watchdog monitoreando Bluetooth"
log "Presiona Ctrl+C para detener"
log "${BLUE}============================================${NC}\n"

# Cambiar al directorio del script
cd "$(dirname "$PYTHON_SCRIPT")"

# ⚡ CORREGIDO: Ejecutar Python y capturar su código de salida
python3 "$PYTHON_SCRIPT" 2>&1 | tee -a "$LOG_FILE"
EXIT_CODE=$?

# ⚡ IMPORTANTE: Solo mostrar error si Python falló
if [ $EXIT_CODE -ne 0 ] && [ $EXIT_CODE -ne 130 ]; then
    log "\n${RED}✗ Python terminó con error (código: $EXIT_CODE)${NC}"
    log "${YELLOW}Revisa el log: $LOG_FILE${NC}"
fi

# Hacer cleanup manual (ya que no hay trap EXIT)
cleanup
exit $EXIT_CODE