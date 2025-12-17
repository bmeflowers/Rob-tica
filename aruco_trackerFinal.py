#!/usr/bin/env python3
"""
Robot Seguidor de Códigos ArUco - VERSIÓN FINAL
Raspberry Pi 4 + Arduino + HC-05 Bluetooth
Solo 2 motores (M1 y M2)

IMPORTANTE: Ejecutar primero ~/mantener_bluetooth.sh en otra terminal
"""

import cv2
import serial
import time
import numpy as np
from datetime import datetime
import sys

# ============== CONFIGURACIÓN ==============

# Puerto Bluetooth (debe estar ya conectado con mantener_bluetooth.sh)
BLUETOOTH_PORT = '/dev/rfcomm0'
BAUD_RATE = 9600

# Configuración de cámara
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Configuración ArUco
ARUCO_DICT = cv2.aruco.DICT_6X6_250

# Parámetros de control del robot
CENTRO_TOLERANCIA = 70       # Pixeles de tolerancia para "centrado"
AREA_MINIMA_CERCA = 10000    # Área para considerar "muy cerca" → DETENER
AREA_MINIMA_LEJOS = 1500     # Área para considerar "muy lejos" → DESPACIO
VELOCIDAD_ENVIO = 0.2        # Segundos entre comandos

# Comandos
CMD_AVANZAR = 'F'
CMD_RETROCEDER = 'B'
CMD_IZQUIERDA = 'L'
CMD_DERECHA = 'R'
CMD_DETENER = 'S'
CMD_DESPACIO = 'G'
CMD_TEST = 'T'

# ============== CLASE PRINCIPAL ==============

class RobotArucoTracker:
    def __init__(self):
        self.bluetooth = None
        self.cap = None
        self.detector = None
        self.ultimo_comando = None
        self.tiempo_ultimo_comando = 0
        self.modo_debug = True
        self.aruco_detectado = False
        self.tiempo_sin_aruco = 0
        
    def verificar_conexion_bluetooth(self):
        """Verifica que /dev/rfcomm0 exista y esté listo"""
        print("=" * 50)
        print("Verificando conexión Bluetooth...")
        print("=" * 50)
        
        import os
        if not os.path.exists(BLUETOOTH_PORT):
            print(f"✗ ERROR: {BLUETOOTH_PORT} no existe")
            print()
            print("SOLUCIÓN:")
            print("1. Abre otra terminal")
            print("2. Ejecuta: ~/mantener_bluetooth.sh")
            print("3. Espera a que diga 'CONEXIÓN LISTA'")
            print("4. Vuelve aquí y ejecuta este script de nuevo")
            print()
            return False
        
        print(f"✓ {BLUETOOTH_PORT} existe")
        return True
    
    def conectar_bluetooth(self):
        """Conecta al módulo HC-05 via puerto serial"""
        print("\nConectando al puerto serial...")
        
        try:
            self.bluetooth = serial.Serial(
                BLUETOOTH_PORT,
                BAUD_RATE,
                timeout=1
            )
            time.sleep(1)
            
            print("✓ Conexión serial establecida")
            
            # Enviar comando de prueba
            print("Enviando comando de prueba...")
            self.bluetooth.write(b'S')
            time.sleep(0.5)
            
            # Verificar respuesta
            if self.bluetooth.in_waiting > 0:
                respuesta = self.bluetooth.read(self.bluetooth.in_waiting).decode(errors='ignore')
                print(f"✓ Arduino respondió: {respuesta.strip()}")
            else:
                print("✓ Puerto serial funcional (sin respuesta)")
            
            return True
            
        except serial.SerialException as e:
            print(f"✗ Error al conectar: {e}")
            print()
            print("Verifica que ~/mantener_bluetooth.sh esté corriendo")
            return False
    
    def inicializar_camara(self):
        """Inicializa la cámara"""
        print("\nInicializando cámara...")
        
        try:
            self.cap = cv2.VideoCapture(CAMERA_INDEX)
            
            if not self.cap.isOpened():
                print("✗ No se pudo abrir la cámara")
                return False
            
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
            
            # Verificar lectura
            ret, frame = self.cap.read()
            if not ret:
                print("✗ No se pudo leer de la cámara")
                return False
            
            print(f"✓ Cámara inicializada ({FRAME_WIDTH}x{FRAME_HEIGHT})")
            return True
            
        except Exception as e:
            print(f"✗ Error al inicializar cámara: {e}")
            return False
    
    def inicializar_aruco(self):
        """Inicializa el detector de ArUco"""
        print("\nInicializando detector ArUco...")
        
        try:
            aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
            aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
            
            print(f"✓ Detector ArUco listo (Diccionario: 6X6_250)")
            return True
            
        except Exception as e:
            print(f"✗ Error al inicializar ArUco: {e}")
            return False
    
    def enviar_comando(self, comando):
        """Envía un comando al Arduino"""
        try:
            # Control de velocidad de envío
            tiempo_actual = time.time()
            if (comando == self.ultimo_comando and 
                tiempo_actual - self.tiempo_ultimo_comando < VELOCIDAD_ENVIO):
                return True
            
            # Enviar comando
            self.bluetooth.write(comando.encode())
            self.bluetooth.flush()
            
            self.ultimo_comando = comando
            self.tiempo_ultimo_comando = tiempo_actual
            
            # Debug
            if self.modo_debug:
                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                comandos_str = {
                    'F': '↑ AVANZAR',
                    'B': '↓ RETROCEDER',
                    'L': '← IZQUIERDA',
                    'R': '→ DERECHA',
                    'S': '■ DETENER',
                    'G': '↑ DESPACIO'
                }
                print(f"[{timestamp}] {comandos_str.get(comando, comando)}")
            
            return True
            
        except Exception as e:
            print(f"✗ Error al enviar comando: {e}")
            return False
    
    def calcular_comando(self, centro_x, centro_y, ancho_frame, alto_frame, area):
        """Determina qué comando enviar según posición y tamaño del ArUco"""
        centro_frame_x = ancho_frame / 2
        
        # Muy cerca → DETENER
        if area > AREA_MINIMA_CERCA:
            return CMD_DETENER, "Muy cerca", (0, 0, 255)
        
        # Muy lejos → DESPACIO
        if area < AREA_MINIMA_LEJOS:
            return CMD_DESPACIO, "Muy lejos", (255, 165, 0)
        
        # Descentrado izquierda → GIRAR IZQUIERDA
        if centro_x < centro_frame_x - CENTRO_TOLERANCIA:
            return CMD_IZQUIERDA, "Girando IZQ", (255, 255, 0)
        
        # Descentrado derecha → GIRAR DERECHA
        elif centro_x > centro_frame_x + CENTRO_TOLERANCIA:
            return CMD_DERECHA, "Girando DER", (255, 255, 0)
        
        # Centrado → AVANZAR
        else:
            return CMD_AVANZAR, "Centrado OK", (0, 255, 0)
    
    def procesar_frame(self, frame):
        """Procesa un frame para detectar ArUco y decidir acción"""
        alto, ancho = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)
        
        comando = CMD_DETENER
        info_texto = "Buscando ArUco..."
        color_info = (0, 165, 255)  # Naranja
        
        if ids is not None and len(ids) > 0:
            # ArUco detectado
            self.aruco_detectado = True
            self.tiempo_sin_aruco = time.time()
            
            # Dibujar marcadores
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            # Procesar primer marcador
            esquinas = corners[0][0]
            centro_x = int(np.mean(esquinas[:, 0]))
            centro_y = int(np.mean(esquinas[:, 1]))
            area = cv2.contourArea(esquinas)
            
            # Dibujar centro y línea de referencia
            cv2.circle(frame, (centro_x, centro_y), 10, (0, 255, 0), -1)
            cv2.line(frame, (int(ancho/2), 0), (int(ancho/2), alto), (255, 0, 0), 2)
            
            # Dibujar zona de tolerancia
            izq = int(ancho/2 - CENTRO_TOLERANCIA)
            der = int(ancho/2 + CENTRO_TOLERANCIA)
            cv2.line(frame, (izq, 0), (izq, alto), (255, 255, 0), 1)
            cv2.line(frame, (der, 0), (der, alto), (255, 255, 0), 1)
            
            # Calcular comando
            comando, info_texto, color_info = self.calcular_comando(
                centro_x, centro_y, ancho, alto, area
            )
            
            # Información en pantalla
            cv2.putText(frame, f"ID: {ids[0][0]}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(frame, f"Area: {int(area)}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(frame, f"Pos X: {centro_x}", (10, 85),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
        else:
            # Sin ArUco
            if self.aruco_detectado:
                # Acabamos de perder el ArUco
                tiempo_perdido = time.time() - self.tiempo_sin_aruco
                if tiempo_perdido < 1.0:
                    # Poco tiempo sin verlo, seguir avanzando
                    comando = CMD_AVANZAR
                    info_texto = "ArUco perdido (avanzando)"
                    color_info = (0, 165, 255)
                else:
                    # Mucho tiempo sin verlo, detenerse
                    self.aruco_detectado = False
                    comando = CMD_DETENER
                    info_texto = "Sin ArUco - DETENIDO"
                    color_info = (0, 0, 255)
        
        # Mostrar estado en pantalla
        cv2.putText(frame, info_texto, (10, alto - 50),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_info, 2)
        cv2.putText(frame, f"CMD: {comando}", (10, alto - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_info, 2)
        
        return frame, comando
    
    def ejecutar(self):
        """Bucle principal de ejecución"""
        print("\n" + "=" * 50)
        print("ROBOT ARUCO TRACKER - INICIADO")
        print("=" * 50)
        print("Controles:")
        print("  Q - Salir")
        print("  D - Toggle debug")
        print("  T - Test de motores")
        print("  ESPACIO - Detener emergencia")
        print("=" * 50 + "\n")
        
        # Contador de FPS
        fps_counter = 0
        fps_start = time.time()
        fps_actual = 0
        
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("✗ Error al leer frame")
                    break
                
                # Procesar frame
                frame_procesado, comando = self.procesar_frame(frame)
                
                # Enviar comando
                self.enviar_comando(comando)
                
                # Calcular FPS
                fps_counter += 1
                if fps_counter >= 30:
                    fps_actual = fps_counter / (time.time() - fps_start)
                    fps_counter = 0
                    fps_start = time.time()
                
                # Mostrar FPS
                cv2.putText(frame_procesado, f"FPS: {fps_actual:.1f}", 
                           (FRAME_WIDTH - 120, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # Mostrar frame
                cv2.imshow('Robot ArUco Tracker', frame_procesado)
                
                # Manejo de teclas
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q') or key == ord('Q'):
                    print("\nSaliendo...")
                    break
                elif key == ord('d') or key == ord('D'):
                    self.modo_debug = not self.modo_debug
                    print(f"Debug: {'ON' if self.modo_debug else 'OFF'}")
                elif key == ord('t') or key == ord('T'):
                    print("Ejecutando test de motores...")
                    self.enviar_comando(CMD_TEST)
                    time.sleep(0.5)
                elif key == 32:  # ESPACIO
                    print("¡DETENCIÓN DE EMERGENCIA!")
                    for _ in range(5):
                        self.enviar_comando(CMD_DETENER)
                        time.sleep(0.05)
                
        except KeyboardInterrupt:
            print("\n\nInterrumpido por usuario (Ctrl+C)")
        
        finally:
            self.limpiar()
    
    def limpiar(self):
        """Limpia recursos y detiene el robot"""
        print("\nLimpiando recursos...")
        
        if self.bluetooth and self.bluetooth.is_open:
            print("Deteniendo robot...")
            try:
                for _ in range(5):
                    self.bluetooth.write(CMD_DETENER.encode())
                    time.sleep(0.1)
            except:
                pass
            
            self.bluetooth.close()
            print("✓ Bluetooth cerrado")
        
        if self.cap:
            self.cap.release()
            print("✓ Cámara liberada")
        
        cv2.destroyAllWindows()
        print("✓ Ventanas cerradas")
        print("\n¡Hasta pronto!\n")

# ============== FUNCIÓN PRINCIPAL ==============

def main():
    print("\n" + "=" * 50)
    print("  ROBOT SEGUIDOR DE ARUCO")
    print("  Raspberry Pi 4 + Arduino + HC-05")
    print("=" * 50 + "\n")
    
    robot = RobotArucoTracker()
    
    # Verificar y conectar componentes
    if not robot.verificar_conexion_bluetooth():
        sys.exit(1)
    
    if not robot.conectar_bluetooth():
        sys.exit(1)
    
    if not robot.inicializar_camara():
        sys.exit(1)
    
    if not robot.inicializar_aruco():
        sys.exit(1)
    
    # Ejecutar bucle principal
    robot.ejecutar()

if __name__ == "__main__":
    main()