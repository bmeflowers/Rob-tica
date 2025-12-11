#include <AFMotor.h>
#include <SoftwareSerial.h>

// ==== BLUETOOTH (HC-05 / HC-06) ====
const uint8_t BT_RX_PIN = 9;   // Arduino RX  <- TX del HC-05
const uint8_t BT_TX_PIN = 10;  // Arduino TX  -> RX del HC-05 (usar divisor de voltaje)
SoftwareSerial BT(BT_RX_PIN, BT_TX_PIN);

// ==== MOTORES (Adafruit Motor Shield v1) ====
AF_DCMotor motor1(1, MOTOR12_1KHZ); // M1 (lado izquierdo, por ejemplo)
AF_DCMotor motor2(2, MOTOR12_1KHZ); // M2 (lado derecho)

// ==== VELOCIDADES POR DEFECTO ====
uint8_t speed_base = 180;           // velocidad base si se usan F/B/L/R
const uint8_t SPEED_MIN = 60;
const uint8_t SPEED_MAX = 255;

// ==== WATCHDOG ====
const unsigned long COMMAND_TIMEOUT_MS = 500; // si no hay comandos por 0.5s -> STOP
unsigned long last_command_ms = 0;

// ==== BUFFER DE ENTRADA (para líneas tipo "M -200 180" o "V200") ====
static const uint8_t CMD_BUF_SZ = 32;
char cmd_buf[CMD_BUF_SZ];
uint8_t cmd_len = 0;

// ---------------- MOVIMIENTO BÁSICO ----------------
void stopAll() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
}

void adelante() {
  motor1.setSpeed(speed_base);
  motor2.setSpeed(speed_base);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
}

void atras() {
  motor1.setSpeed(speed_base);
  motor2.setSpeed(speed_base);
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
}

void izquierda() {
  motor1.setSpeed(speed_base);
  motor2.setSpeed(speed_base);
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
}

void derecha() {
  motor1.setSpeed(speed_base);
  motor2.setSpeed(speed_base);
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
}

// ----- CONTROL DIFERENCIAL: M izq der  (rangos -255..255) -----
void setMotorSigned(AF_DCMotor &m, int val) {
  int spd = abs(val);
  if (spd > 255) spd = 255;
  if (spd == 0) {
    m.run(RELEASE);
  } else {
    m.setSpeed(spd);
    m.run(val >= 0 ? FORWARD : BACKWARD);
  }
}

void cmdDifferential(int left, int right) {
  setMotorSigned(motor1, left);
  setMotorSigned(motor2, right);
}

// ---------------- ENTRADA SERIE UNIFICADA ----------------
// Lee de Serial (USB) y de BT (HC-05). Devuelve -1 si no hay nada.
int readOneByteUnified() {
  if (Serial.available()) return Serial.read();
  if (BT.available())     return BT.read();
  return -1;
}

// Envía respuesta a ambos (útil para debug)
void printUnified(const String &s) {
  Serial.println(s);
  BT.println(s);
}

// Procesa línea completa en cmd_buf (terminada en '\n')
void processLine(char *line) {
  // Elimina CR/LF finales
  size_t n = strlen(line);
  while (n && (line[n-1] == '\r' || line[n-1] == '\n')) line[--n] = 0;

  if (n == 0) return;

  // Comandos de una letra compatibles con tu app BT
  if (n == 1) {
    char c = line[0];
    switch (c) {
      case 'F': case 'f': adelante();  printUnified(F("OK Adelante"));  break;
      case 'B': case 'b': atras();     printUnified(F("OK Atras"));     break;
      case 'L': case 'l': izquierda(); printUnified(F("OK Izquierda")); break;
      case 'R': case 'r': derecha();   printUnified(F("OK Derecha"));   break;
      case 'S': case 's': stopAll();   printUnified(F("OK Stop"));      break;
      case '0'...'9':
        // compat: slider 0..9 -> map a 60..255
        speed_base = map(c - '0', 0, 9, SPEED_MIN, SPEED_MAX);
        printUnified(String(F("OK Velocidad base: ")) + speed_base);
        break;
      default:
        printUnified(F("ERR Comando desconocido"));
        break;
    }
    return;
  }

  // Comandos extendidos:
  // V<num>   -> fija velocidad base 0..255 (ej: "V200")
  // M l r    -> control diferencial signed -255..255 (ej: "M -180 200")
  // S        -> stop (ya cubierto arriba, pero permitido en línea también)
  if (line[0] == 'V') {
    int v = atoi(line + 1);
    v = constrain(v, 0, 255);
    speed_base = v;
    printUnified(String(F("OK Velocidad base: ")) + speed_base);
    return;
  }

  if (line[0] == 'M') {
    int l = 0, r = 0;
    // Formato: "M <izq> <der>"
    if (sscanf(line + 1, "%d %d", &l, &r) == 2) {
      l = constrain(l, -255, 255);
      r = constrain(r, -255, 255);
      cmdDifferential(l, r);
      // No spamear demasiado:
      // printUnified(String(F("OK M ")) + l + " " + r);
      return;
    } else {
      printUnified(F("ERR Formato M invalido. Use: M <izq> <der>"));
      return;
    }
  }

  if (line[0] == 'S' || line[0] == 's') {
    stopAll();
    printUnified(F("OK Stop"));
    return;
  }

  printUnified(F("ERR Comando no reconocido"));
}

void setup() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  delay(300);

  Serial.begin(115200);   // para USB
  BT.begin(9600);         // baud típico HC-05

  printUnified(F("Robot listo. Comandos: F/B/L/R/S, 0..9, V<0..255>, M <izq> <der>"));

  last_command_ms = millis();
}

void loop() {
  int b = readOneByteUnified();
  if (b >= 0) {
    last_command_ms = millis();

    // Acumular hasta '\n' (también aceptar comandos de 1 char sin LF)
    if (b == '\n') {
      cmd_buf[cmd_len] = 0;
      processLine(cmd_buf);
      cmd_len = 0;
    } else if (cmd_len + 1 < CMD_BUF_SZ) {
      cmd_buf[cmd_len++] = (char)b;
      // Modo compat: si llega comando de 1 char (F/L/...), procesarlo sin esperar '\n'
      if (cmd_len == 1 &&
          (cmd_buf[0]=='F'||cmd_buf[0]=='f'||cmd_buf[0]=='B'||cmd_buf[0]=='b'||
           cmd_buf[0]=='L'||cmd_buf[0]=='l'||cmd_buf[0]=='R'||cmd_buf[0]=='r'||
           cmd_buf[0]=='S'||cmd_buf[0]=='s'||
           (cmd_buf[0]>='0' && cmd_buf[0]<='9'))) {
        cmd_buf[1] = 0;
        processLine(cmd_buf);
        cmd_len = 0;
      }
    } else {
      // Overflow -> reset buffer
      cmd_len = 0;
    }
  }

  // Watchdog: si no hay comandos recientes, detente
  if (millis() - last_command_ms > COMMAND_TIMEOUT_MS) {
    stopAll();
  }
}
