#include <AFMotor.h>

// ==== MOTORES (Adafruit Motor Shield v1) ====
AF_DCMotor motor1(1, MOTOR12_1KHZ); // M1
AF_DCMotor motor2(2, MOTOR12_1KHZ); // M2

// ==== VELOCIDADES ====
uint8_t speed_fwd  = 180; // velocidad moderada para seguimiento
uint8_t speed_turn = 160; // giros más suaves
uint8_t speed_back = 160; // retroceso moderado

// ==== TIMEOUT PARA SEGURIDAD ====
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 1000; // 1 segundo sin comandos = stop

// ==== FUNCIONES DE MOVIMIENTO ====
void adelante() {
  motor1.setSpeed(speed_fwd);
  motor2.setSpeed(speed_fwd);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
}

void atras() {
  motor1.setSpeed(speed_back);
  motor2.setSpeed(speed_back);
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
}

void izquierda() {
  motor1.setSpeed(speed_turn);
  motor2.setSpeed(speed_turn);
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
}

void derecha() {
  motor1.setSpeed(speed_turn);
  motor2.setSpeed(speed_turn);
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
}

void stopAll() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
}

// ==== SETUP ====
void setup() {
  // Motores detenidos al arrancar
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  delay(500);

  Serial.begin(9600);
  
  Serial.println(F("========================================"));
  Serial.println(F("  SISTEMA ARUCO FOLLOWER - ARDUINO"));
  Serial.println(F("========================================"));
  Serial.println(F("Esperando comandos desde Raspberry Pi..."));
  Serial.println(F("Comandos: F=Adelante, B=Atras, L=Izq, R=Der, S=Stop"));
  Serial.println(F("========================================"));
  
  lastCommandTime = millis();
}

// ==== LOOP ====
void loop() {
  // Verificar timeout de seguridad
  if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
    stopAll();
    // No imprimir repetidamente para no saturar
  }
  
  // Procesar comandos desde Serial (Raspberry Pi)
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    // Actualizar tiempo del último comando
    lastCommandTime = millis();
    
    // Procesar comando
    switch (cmd) {
      case 'F':
      case 'f':
        adelante();
        Serial.println(F("-> ADELANTE"));
        break;
        
      case 'B':
      case 'b':
        atras();
        Serial.println(F("-> ATRAS"));
        break;
        
      case 'L':
      case 'l':
        izquierda();
        Serial.println(F("-> IZQUIERDA"));
        break;
        
      case 'R':
      case 'r':
        derecha();
        Serial.println(F("-> DERECHA"));
        break;
        
      case 'S':
      case 's':
        stopAll();
        Serial.println(F("-> STOP"));
        break;
        
      // Comandos de velocidad (0-9) compatibilidad
      case '0': case '1': case '2': case '3': case '4':
      case '5': case '6': case '7': case '8': case '9':
        {
          int nivel = cmd - '0';
          speed_fwd  = map(nivel, 0, 9, 80, 255);
          speed_back = speed_fwd;
          speed_turn = map(nivel, 0, 9, 80, 200); // giros más lentos
          Serial.print(F("Velocidad ajustada a: "));
          Serial.println(speed_fwd);
        }
        break;
        
      case '\n':
      case '\r':
        // Ignorar saltos de línea
        break;
        
      default:
        Serial.print(F("Comando desconocido: "));
        Serial.println(cmd);
        break;
    }
  }
}