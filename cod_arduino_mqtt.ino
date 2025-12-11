#include <SPI.h>
#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <AFMotor.h>

// ==== CONFIG WIFI ====
char ssid[] = "TU_SSID";       // <-- CAMBIA
char pass[] = "TU_PASSWORD";   // <-- CAMBIA

// IP del Raspberry Pi (broker MQTT)
IPAddress mqttServer(192, 168, 1, 50);  // <-- CAMBIA a la IP real del RPi
const uint16_t mqttPort = 1883;
const char* mqttTopic = "robotica/move";

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

// ==== WIFI & MQTT ====
WiFiClient wifiClient;
PubSubClient client(wifiClient);

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

// ==== MQTT: callback cuando llega un mensaje ====
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Copiamos el payload en un string
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  Serial.print("MQTT [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println(msg);

  // Esperamos algo como "1 F" o "1 S"
  msg.trim();
  if (msg.length() == 0) return;

  // Buscamos espacio
  int spaceIndex = msg.indexOf(' ');
  char cmd = 0;

  if (spaceIndex >= 0 && spaceIndex < (int)msg.length() - 1) {
    // Ej: "1 F"
    cmd = msg.charAt(spaceIndex + 1);
  } else {
    // Si no hay espacio, usamos el primer carácter
    cmd = msg.charAt(0);
  }

  cmd = toupper(cmd);

  // Actualizamos tiempo del último comando
  lastCommandTime = millis();

  switch (cmd) {
    case 'F':
      adelante();
      Serial.println("-> ADELANTE");
      break;
    case 'B':
      atras();
      Serial.println("-> ATRAS");
      break;
    case 'L':
      izquierda();
      Serial.println("-> IZQUIERDA");
      break;
    case 'R':
      derecha();
      Serial.println("-> DERECHA");
      break;
    case 'S':
    default:
      stopAll();
      Serial.println("-> STOP/DEFAULT");
      break;
  }
}

// ==== Conexión WiFi ====
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.print("Conectando a WiFi: ");
  Serial.println(ssid);

  WiFi.disconnect();
  delay(1000);

  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nWiFi conectado!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

// ==== Conexión MQTT ====
void connectMQTT() {
  client.setServer(mqttServer, mqttPort);
  client.setCallback(mqttCallback);

  while (!client.connected()) {
    Serial.print("Conectando a MQTT...");
    String clientId = "ArduinoRobot-";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      Serial.println("conectado!");
      client.subscribe(mqttTopic);
      Serial.print("Suscrito a: ");
      Serial.println(mqttTopic);
    } else {
      Serial.print("falló, rc=");
      Serial.print(client.state());
      Serial.println(" intentando en 2 segundos...");
      delay(2000);
    }
  }
}

// ==== SETUP ====
void setup() {
  Serial.begin(9600);

  // Motores detenidos al arrancar
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  delay(500);

  Serial.println("========================================");
  Serial.println("  SISTEMA ARUCO FOLLOWER - ARDUINO MQTT");
  Serial.println("========================================");

  connectWiFi();
  connectMQTT();

  lastCommandTime = millis();
}

// ==== LOOP ====
void loop() {
  // Revisión WiFi y MQTT
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }
  if (!client.connected()) {
    connectMQTT();
  }
  client.loop();  // procesa mensajes MQTT

  // Timeout de seguridad
  if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
    stopAll();
  }
}
