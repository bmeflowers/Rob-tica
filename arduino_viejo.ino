#include <AFMotor.h>
#include <SoftwareSerial.h>

// ==== BLUETOOTH (HC-05 / HC-06) ====
const uint8_t BT_RX_PIN = 9;  // Arduino recibe (TX del HC-05)
const uint8_t BT_TX_PIN = 10; // Arduino transmite (RX del HC-05) -> usar divisor de voltaje
SoftwareSerial BT(BT_RX_PIN, BT_TX_PIN);

// ==== MOTORES (Adafruit Motor Shield v1) ====
AF_DCMotor motor1(1, MOTOR12_1KHZ); // M1
AF_DCMotor motor2(2, MOTOR12_1KHZ); // M2

// ==== VELOCIDADES ====
uint8_t speed_fwd  = 200; // adelante
uint8_t speed_turn = 200; // giros
uint8_t speed_back = 200; // atrás
const uint8_t SPEED_MIN = 80;
const uint8_t SPEED_MAX = 255;

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

// ==== AJUSTE DE VELOCIDAD CON SLIDER (0–9) ====
void ajustarVelocidad(char c) {
  if (c >= '0' && c <= '9') {
    int nivel = c - '0';
    speed_fwd  = map(nivel, 0, 9, SPEED_MIN, SPEED_MAX);
    speed_back = speed_fwd;
    speed_turn = speed_fwd;
    BT.print("Velocidad: ");
    BT.println(speed_fwd);
  }
}

// ==== SETUP ====
void setup() {
  // Motores detenidos al arrancar
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  delay(500);

  Serial.begin(9600);
  BT.begin(9600); // baud del HC-05
  BT.println(F("Bluetooth RC listo. Usa F,B,L,R,S o el slider (0–9)."));
  Serial.println(F("Bluetooth RC listo."));
}

// ==== LOOP ====
void loop() {
  if (BT.available()) {
    char cmd = BT.read();
    Serial.println(cmd);

    switch (cmd) {
      case 'F': case 'f': adelante();  BT.println(F("Adelante")); break;
      case 'B': case 'b': atras();     BT.println(F("Atras"));    break;
      case 'L': case 'l': izquierda(); BT.println(F("Izquierda"));break;
      case 'R': case 'r': derecha();   BT.println(F("Derecha"));  break;
      case 'S': case 's': stopAll();   BT.println(F("Stop"));     break;
      default: ajustarVelocidad(cmd); break;
    }
  }
}