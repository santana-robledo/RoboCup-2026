#include <I2Cdev.h>
#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include <Adafruit_VL53L0X.h>
#include <esp_now.h>
#include <WiFi.h>
/* 
Forma de enviar los datos
M,0.0,0.0,0.0,0,0,G
   X   Y   t  P C modo
     G=global, L=Local
*/
/*
MAC_A: D8:13:2A:7D:92:80
MAC_B: B8:D6:1A:6B:29:C4
*/

///////////////////////// Banderas de configuración ////////////

bool usarPiso = true;
bool usarDistancia = true;
bool porteriaAzul = true;
char ROBOT_ID = 'A';

////////////////////////////// Maquina de Estados ////////////////

enum EstadoRobot {
  JUGAR,
  CONFIG,
  LINEA,
  EVADIR,
  DETENER,
};

EstadoRobot estadoActual = JUGAR;
EstadoRobot estadoAnterior = JUGAR;

////////////// Conexión ESP32 /////////////////

uint8_t robotB_mac[] = {0xD8, 0x13, 0x2A, 0x7D, 0x92, 0x80};
uint8_t robotA_mac[] = {0xB8, 0xD6, 0x1A, 0x6B, 0x29, 0xC4};
uint8_t* peerAddress;
bool usarComunicacion = true;

////////////////////////////////////////////////
const int UMBRAL_LINEA = 2000;

int contadorLinea[12] = {0};
const int LINEA_CONFIRMADA = 2;

////////// Sensores de distancia
const int NUM_SENSORES = 5;
#define SDA_PIN 21
#define SCL_PIN 22
#define MUX_ADDR 0x74
#define VL53_ADDR 0x29

Adafruit_VL53L0X sensor0 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor2 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor3 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor4 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor5 = Adafruit_VL53L0X();

bool activo0 = false;
bool activo2 = false;
bool activo3 = false;
bool activo4 = false;
bool activo5 = false;

int dist[NUM_SENSORES];
const float VELOCIDAD_EVASION = 0.4;

const int distanciaSegura[NUM_SENSORES] = {
  60,   // canal 2
  60,   // canal 3
  60,   // canal 4
  60,   // canal 5
  60    // canal 6
};
const float sensorX[5] = {
   -0.9,  // Frontal izquierdo
   0.9,   // Atras
   -0.9,   // Frontal derecho
   0.0,  // Lateral derecho
   0.0    // Lateral izquierdo
};

const float sensorY[5] = {
   0.0, // canal 2
   0.0, // canal 3
   0.0, // canal 4
   0.9, // canal 5
  -0.9 // canal 5
};

void selectMuxChannel(uint8_t channel) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delay(50);
}

bool existeVL53() {
  Wire.beginTransmission(VL53_ADDR);
  return Wire.endTransmission() == 0;
}

void iniciarSensor(uint8_t canal, Adafruit_VL53L0X &sensor, bool &activo) {
  selectMuxChannel(canal);

  if (!existeVL53()) {
    activo = false;
    return;
  }

  if (sensor.begin()) {
    activo = true;
  } else {
    activo = false;
  }

  delay(200);
}

int leerSensor(uint8_t canal, Adafruit_VL53L0X &sensor, bool activo) {
  if (!activo) return -1;

  selectMuxChannel(canal);

  VL53L0X_RangingMeasurementData_t measure;
  sensor.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter;   // Devuelve distancia en mm
  } else {
    return -1;                        // Fuera de rango
  }
}


///// Sensores de piso
float pisoFiltrado[12];
const float ALPHA_PISO = 0.6;
int umbral_parada = 2000;
int contadorEstadoLinea=0;

const int umbral[12] = {
  400,   // L1
  400,   // L2
  400,  // L3
  400,  // L4
  400,   // L5
  400,   // L6
  400,   // L7
  400,   // L8
  400,   // L9
  400,   // L10
  400,   // L11
  400    // L12
};

bool parLinea[6];
int paresLineaActivos = 0;
int sensoresLineaActivos = 0;

float pisoX[12] = {
  -0.25, -0.25,  0.0,  0.0,
   0.25,  0.25,  0.0,  0.0,
  -0.25, -0.25, -0.25, -0.25
};

float pisoY[12] = {
  -0.25, -0.25,  0.25,  0.25,
   0.0,   0.0,  -0.25, -0.25,
   0.25,  0.25,  0.0,   0.0
};

unsigned long lastDebug = 0;

float evadeX = 0.0;
float evadeY = 0.0;
float pisoUx = 0.0;
float pisoUy = 0.0;
int piso[12];
bool linea[12];

float L = 0.09, R = 0.029;

float Ux = 0.0;
float Uy = 0.0;
float Ut = 0.0;

float Ux_cmd = 0.0;
float Uy_cmd = 0.0;

float v1 = 0.0;
float v2 = 0.0;
float v3 = 0.0;

float vmax_A = (300 * 2 * PI / 60) * R;
float vmax_B = (300 * 2 * PI / 60) * R;
float vmax_C = (300 * 2 * PI / 60) * R;

float kv_A = 255.0 / vmax_A;
float kv_B = 255.0 / vmax_B;
float kv_C = 255.0 / vmax_C;

int estadoPelota = 0;
int estadoPelotaPrev = -1;

int patada = 0;
int patada_prev = 0;

unsigned long tiempoPatada = 0;
bool pateando = false;

int cilindro = 0;

// ================= IMU =================

BNO080 bno080;

float theta_f = 0.0;
float theta_offset = 0.0;

// Gyro
float gx_dps = 0.0;
float gy_dps = 0.0;
float gz_dps = 0.0;

float gx_rad = 0.0;
float gy_rad = 0.0;
float gz_rad = 0.0;

// Bias
float bias_x = 0.0;
float bias_y = 0.0;
float bias_z = 0.0;

// Ángulos
float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

float dt = 0.0;
unsigned long lastTime = 0;

int pwm_a = 0;
int pwm_b = 0;
int pwm_c = 0;

float wa = 0.0;
float wb = 0.0;
float wc = 0.0;

float Ut_pid = 0.0;

float Kp = 3;
float Ki = 1;

float setpoint = 0.0;

float error = 0.0;
float error_int = 0.0;

char modo = 'G';

int sw1 = 0;
int sw2 = 0;
int sw3 = 0;
int sw1_prev = 0;

// ===== MOTOR A =====
#define IN1   5
#define IN2   17
#define ENA   18

// ===== MOTOR B =====
#define IN3   4
#define IN4   2
#define ENB   16

// ===== MOTOR C =====
#define IN5   15
#define IN6   13
#define ENC   14

// ===== CILINDRO =====
#define IN7   19
#define IN8   23

// ================= SWITCHES =================
#define SW1 39
#define SW2 32
#define SW3 35

// ===== SENSOR PELOTA =====
#define SENSOR_PELOTA 33

// ===== MULTIPLEXOR =====
#define SIG_MUX 34
#define S0 25
#define S1 26
#define S2 27
#define S3 12

// ===== I2C =====
#define SDA_IMU 21
#define SCL_IMU 22

// ===== PWM =====
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

struct Config {
  int sw1;
  int sw2;
  int sw3;
};

Config config;
Config configAnterior;

const char* nombreEstado(EstadoRobot estado) {
  switch (estado) {
    case JUGAR:   return "JUGAR";
    case CONFIG:  return "CONFIG";
    case LINEA:   return "LINEA";
    case EVADIR:  return "EVADIR";
    case DETENER: return "DETENER";
    default:      return "DESCONOCIDO";
  }
}

void disableAll() {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(0);
  Wire.endTransmission();
}

void seleccionarCanal(int canal) {
  digitalWrite(S0, canal & 0x01);
  digitalWrite(S1, (canal >> 1) & 0x01);
  digitalWrite(S2, (canal >> 2) & 0x01);
  digitalWrite(S3, (canal >> 3) & 0x01);
}

int leerMux(int canal) {
  seleccionarCanal(canal);
  delayMicroseconds(100);

  int v1 = analogRead(SIG_MUX);
  int v2 = analogRead(SIG_MUX);

  return (v1 + v2) / 2;
}

void setup() {
  pinMode(SENSOR_PELOTA, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);

  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(SIG_MUX, INPUT);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, LOW);
  digitalWrite(IN7, LOW);
  digitalWrite(IN8, LOW);

  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENC, PWM_FREQ, PWM_RESOLUTION);

  stopMotorA();
  stopMotorB();
  stopMotorC();

  Serial.begin(115200);
  Serial.setTimeout(10);

  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);
  pinMode(SW3, INPUT);
  delay(50);

  config.sw1 = digitalRead(SW1);
  config.sw2 = digitalRead(SW2);
  config.sw3 = digitalRead(SW3);
  configAnterior = config;

  //Serial.print("C,");
  //Serial.print(config.sw1);
  //Serial.print(",");
  // Serial.print(config.sw2);
  //Serial.print(",");
  //Serial.println(config.sw3);

  // ===================== IMU (VERSIÓN MEJORADA) =====================
Serial.println();
Serial.println("=================================");
Serial.println("INICIANDO BNO080");
Serial.println("=================================");

Wire.end();
delay(300);

Wire.begin(SDA_IMU, SCL_IMU);
Wire.setClock(100000);   // Prueba primero a 100 kHz

bool imuOK = false;

for (int intento = 1; intento <= 8; intento++) {

  Serial.print("Intento ");
  Serial.print(intento);
  Serial.print(" de inicializar BNO080... ");

  if (bno080.begin()) {
    imuOK = true;
    Serial.println("ÉXITO");
    break;
  }

  Serial.println("Fallo");
  delay(600);
}

if (!imuOK) {

  Serial.println("BNO080 NO DETECTADO");

} else {

  Serial.println("BNO080 DETECTADO");

  // Reinicio interno
  bno080.softReset();
  delay(500);

  // Volver a habilitar reportes después del reset
  bno080.enableGameRotationVector(50);
  delay(200);

  bno080.enableGyro(50);
  delay(200);

  Serial.println("Esperando primeros datos del BNO080...");

  unsigned long t0 = millis();
  bool datosOK = false;

  while (millis() - t0 < 15000) {

    if (bno080.dataAvailable()) {

      float yaw0 = bno080.getYaw();

      if (!isnan(yaw0) && !isinf(yaw0)) {

        Serial.print("Primer yaw recibido: ");
        Serial.println(yaw0, 6);

        theta_offset = yaw0;
        yaw = 0.0;
        theta_f = 0.0;

        datosOK = true;
        break;
      }
      else {
        Serial.println("Yaw NAN recibido");
      }
    }

    delay(30);
  }

  if (datosOK) {
    Serial.println("✅ BNO080 LISTO");
  }
  else {
    Serial.println("❌ BNO080 DETECTADO PERO SIN DATOS VALIDOS");
  }
}

Serial.println("=================================");

    if (usarDistancia) {
     //Serial.println("Iniciando sensores de distancia...");

    iniciarSensor(0, sensor0, activo0);
    iniciarSensor(2, sensor2, activo2);
    iniciarSensor(3, sensor3, activo3);
    iniciarSensor(4, sensor4, activo4);
    iniciarSensor(5, sensor5, activo5);

   //Serial.println();
   //Serial.println("Resumen:");

  /*if (activo0) Serial.println("Canal 0 OK");
  else Serial.println("Canal 0 NO DETECTADO");

  if (activo2) Serial.println("Canal 2 OK");
  else Serial.println("Canal 2 NO DETECTADO");

  if (activo3) Serial.println("Canal 3 OK");
  else Serial.println("Canal 3 NO DETECTADO");

  if (activo4) Serial.println("Canal 4 OK");
  else Serial.println("Canal 4 NO DETECTADO");

  if (activo5) Serial.println("Canal 5 OK");
  else Serial.println("Canal 5 NO DETECTADO");*/

  // Serial.println();
  }

  if (ROBOT_ID == 'A') {
    peerAddress = robotB_mac;
  } else {
    peerAddress = robotA_mac;
  }

  lastTime = micros();

  //Serial.println("Robot listo");

  for (int i = 0; i < 12; i++) {
    pisoFiltrado[i] = leerMux(i);
  }
}

void loop() {
  config.sw1 = digitalRead(SW1);
  config.sw2 = digitalRead(SW2);
  config.sw3 = digitalRead(SW3);

  bool cambioSwitches =
    config.sw1 != configAnterior.sw1 ||
    config.sw2 != configAnterior.sw2 ||
    config.sw3 != configAnterior.sw3;

  ////////////////////////////// Lectura de ángulo ////////////////////
  unsigned long currentTime = micros();
  dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;

  if (dt <= 0) dt = 0.0001;

  if (bno080.dataAvailable()) {
      float yaw_raw = bno080.getYaw();

      theta_f = atan2(
        sin(yaw_raw - theta_offset),
        cos(yaw_raw - theta_offset)
      );

      yaw = theta_f;
    }

  /////////////// Calculo PID //////////////////////////////////////
  error = setpoint - theta_f;
  error = atan2(sin(error), cos(error));

  error_int += error * dt;
  error_int = constrain(error_int, -2.0, 2.0);

  Ut_pid = Kp * error + Ki * error_int;
  Ut_pid = constrain(Ut_pid, -4.0, 4.0);
  ///////////////////////////////////////////////////////////////////

  //////////////////////////// Lectura Raspberry Pi //////////////////
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    char buffer[50];
    input.toCharArray(buffer, 50);

    char* token = strtok(buffer, ",");

    token = strtok(NULL, ",");
    if (token) Ux = atof(token);

    token = strtok(NULL, ",");
    if (token) Uy = atof(token);

    token = strtok(NULL, ",");
    if (token) {
      float valor_t = atof(token);
      Ut = valor_t;
      setpoint = valor_t;
      setpoint = atan2(sin(setpoint), cos(setpoint));
    }

    token = strtok(NULL, ",");
    if (token) patada = atoi(token);

    token = strtok(NULL, ",");
    if (token) cilindro = atoi(token);

    token = strtok(NULL, ",");
    if (token) {
      char modo_nuevo = token[0];

      if (modo_nuevo != modo) {
        error_int = 0.0;
        modo = modo_nuevo;
      }
    }

    Ux = constrain(Ux, -1.0, 1.0);
    Uy = constrain(Uy, -1.0, 1.0);
    Ut = constrain(Ut, -1.0, 1.0);
  }

  Ux_cmd = Ux;
  Uy_cmd = Uy;

  ////////////////////////////// Lectura de distancia //////////////////
   if (usarDistancia) {
    dist[0]=leerSensor(0, sensor0, activo0);
    dist[1]=leerSensor(2, sensor2, activo2);
    dist[2]=leerSensor(3, sensor3, activo3);
    dist[3]=leerSensor(4, sensor4, activo4);
    dist[4]=leerSensor(5, sensor5, activo5);
  }

  ////////////////////////////// Lectura de piso ///////////////////////
  for (int i = 0; i < 12; i++) {
    int lectura = leerMux(i);
    pisoFiltrado[i] = ALPHA_PISO * lectura + (1.0 - ALPHA_PISO) * pisoFiltrado[i];
    piso[i] = (int)pisoFiltrado[i];
    bool lecturaActual = (piso[i] < umbral[i]);

if (lecturaActual) {
  if (contadorLinea[i] < LINEA_CONFIRMADA) {
    contadorLinea[i]++;
  }
} else {
  contadorLinea[i] = 0;
}

linea[i] = (contadorLinea[i] >= LINEA_CONFIRMADA);
  }

  parLinea[0] = linea[0] && linea[1];
  parLinea[1] = linea[2] && linea[3];
  parLinea[2] = linea[4] && linea[5];
  parLinea[3] = linea[6] && linea[7];
  parLinea[4] = linea[8] && linea[9];
  parLinea[5] = linea[10] && linea[11];

  /////////////////////////// Decisión de estado ///////////////////////
  bool hayLinea = false;

  if (usarPiso) {
    for (int i = 0; i < 6; i++) {
      if (parLinea[i]) {
        hayLinea = true;
        break;
      }
    }
  }

  if (usarDistancia) {
    evadeX = 0;
    evadeY = 0;

    for (int i = 0; i < 5; i++) {
      if (dist[i] > 0 && dist[i] < distanciaSegura[i]) {
        evadeX += sensorX[i];
        evadeY += sensorY[i];
      }
    }

    Ux_cmd += evadeX;
    Uy_cmd += evadeY;
  }
  estadoPelota = digitalRead(SENSOR_PELOTA);
  if (estadoPelota != estadoPelotaPrev) {
    Serial.print("P,");
    Serial.println(estadoPelota);
    estadoPelotaPrev = estadoPelota;
  }

  if (cambioSwitches) {
    estadoActual = CONFIG;
  }
  else if (hayLinea) {
  contadorEstadoLinea++;
  } else {
    contadorEstadoLinea = 0;
  }
  if (contadorEstadoLinea > 1) estadoActual = LINEA;
  if (!hayLinea && contadorEstadoLinea == 0) estadoActual = JUGAR;

  bool cambioEstado = (estadoActual != estadoAnterior);
  if (cambioEstado) {
    estadoAnterior = estadoActual;
  }

  ////////////////////////////// Debug /////////////////////////////////
  if (millis() - lastDebug >= 100) {
    lastDebug = millis();
    debugRobot();
    //imprimirDistancias();
    //imprimirPiso();
  }

  ////////////////////////// Máquina de estados ////////////////////////
  switch (estadoActual) {

  case JUGAR:

  if (patada == 1 && patada_prev == 0 && !pateando) {
    digitalWrite(IN7, LOW);
    digitalWrite(IN8, HIGH);
    tiempoPatada = millis();
    pateando = true;
  }

  if (pateando) {
    if (millis() - tiempoPatada >= 120) {
      digitalWrite(IN7, LOW);
      digitalWrite(IN8, LOW);
      pateando = false;
      patada = 0;
    }

    patada_prev = patada;
    break;
  }

  if (cilindro == 1) {
    digitalWrite(IN7, HIGH);
    digitalWrite(IN8, LOW);
  } else {
    digitalWrite(IN7, LOW);
    digitalWrite(IN8, LOW);
  }

  break;

    case EVADIR:
      stopMotorA();
      stopMotorB();
      stopMotorC();
      break;

    case DETENER:
      stopMotorA();
      stopMotorB();
      stopMotorC();
      break;

        case LINEA:
  Ux_cmd = 0.0;
  Uy_cmd = 0.0;
  Ut = 0.0;
  Ut_pid = 0.0;
  error_int = 0.0;

  for (int i = 0; i < 12; i++) {
    if (linea[i]) {
      Ux_cmd += pisoX[i] * 3.0;   // Factor fuerte
      Uy_cmd += pisoY[i] * 3.0;
    }
  }

  Ux_cmd = constrain(Ux_cmd, -1.0, 1.0);
  Uy_cmd = constrain(Uy_cmd, -1.0, 1.0);
  break;

case CONFIG:

  if (cambioEstado) {

    sw1 = digitalRead(SW1);
    sw2 = digitalRead(SW2);
    sw3 = digitalRead(SW3);

    porteriaAzul = sw2;

    stopMotorA();
    stopMotorB();
    stopMotorC();

    configAnterior = config;
  }

  if (config.sw1 == HIGH && sw1_prev == LOW) {
    theta_offset = bno080.getYaw();
    error_int = 0.0;
  }

  sw1_prev = config.sw1;
  break;
  }
  

  ///////////////////////////// Cinématica /////////////////////////////
  if (modo == 'G') {
    v1 = (sin(theta_f) * Ux_cmd)
       - (cos(theta_f) * Uy_cmd)
       + (L * Ut_pid);

    v2 = (cos(theta_f + PI / 6) * Ux_cmd)
       + (sin(theta_f + PI / 6) * Uy_cmd)
       + (L * Ut_pid);

    v3 = (-sin(theta_f + PI / 3) * Ux_cmd)
       + (cos(theta_f + PI / 3) * Uy_cmd)
       + (L * Ut_pid);
  }
  else if (modo == 'L') {
    v1 = (-Uy_cmd + Ut);

    v2 = ((0.866 * Ux_cmd)
       + (0.5 * Uy_cmd)
       + Ut);

    v3 = ((-0.866 * Ux_cmd)
       + (0.5 * Uy_cmd)
       + Ut);
  }

  //////////////////////// Asignación de velocidades //////////////////
  wa = v1 / R;
  wb = v2 / R;
  wc = v3 / R;

  float va = wa * R;
  float vb = wb * R;
  float vc = wc * R;

  pwm_a = mapPWM_linear(va, kv_A);
  pwm_b = mapPWM_linear(vb, kv_B);
  pwm_c = mapPWM_linear(vc, kv_C);

  (wa > 0) ? motorA_forward(pwm_a) :
  (wa < 0) ? motorA_backward(pwm_a) :
             stopMotorA();

  (wb > 0) ? motorB_forward(pwm_b) :
  (wb < 0) ? motorB_backward(pwm_b) :
             stopMotorB();

  (wc > 0) ? motorC_forward(pwm_c) :
  (wc < 0) ? motorC_backward(pwm_c) :
             stopMotorC();
}

void stopMotorA() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  ledcWrite(ENA, 0);
}

void motorA_forward(int PWM) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  ledcWrite(ENA, PWM);
}

void motorA_backward(int PWM) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  ledcWrite(ENA, PWM);
}

void stopMotorB() {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(ENB, 0);
}

void motorB_forward(int PWM) {
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(ENB, PWM);
}

void motorB_backward(int PWM) {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(ENB, PWM);
}

void stopMotorC() {
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, LOW);
  ledcWrite(ENC, 0);
}

void motorC_forward(int PWM) {
  digitalWrite(IN5, HIGH);
  digitalWrite(IN6, LOW);
  ledcWrite(ENC, PWM);
}

void motorC_backward(int PWM) {
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, HIGH);
  ledcWrite(ENC, PWM);
}

int mapPWM_linear(float v, float kv) {
  if (isnan(v) || isinf(v)) {
    return 0;        
  }
  float pwm = kv * v;
  pwm = constrain(pwm, -255, 255);
  return abs((int)pwm);
}

void debugRobot() {
  Serial.print("Estado: ");
  Serial.print(nombreEstado(estadoActual));
  Serial.print(" | Yaw: "); Serial.print(yaw, 2);
  Serial.print(" | Ux: "); Serial.print(Ux_cmd, 2);
  Serial.print(" | Uy: "); Serial.print(Uy_cmd, 2);
  Serial.print(" | Ut: "); Serial.print(Ut, 2);
  Serial.print(" | PWM_A: "); Serial.print(pwm_a);
  Serial.print(" | PWM_B: "); Serial.print(pwm_b);
  Serial.print(" | PWM_C: "); Serial.print(pwm_c);
  Serial.print(" | SW1: "); Serial.print(config.sw1);
  Serial.print(" | SW2: "); Serial.print(config.sw2);
  Serial.print(" | SW3: "); Serial.println(config.sw3);
}

void imprimirDistancias() {
  Serial.print("Canal 0: "); Serial.print(dist[0]); Serial.println(" mm");
  Serial.print("Canal 2: "); Serial.print(dist[1]); Serial.println(" mm");
  Serial.print("Canal 3: "); Serial.print(dist[2]); Serial.println(" mm");
  Serial.print("Canal 4: "); Serial.print(dist[3]); Serial.println(" mm");
  Serial.print("Canal 5: "); Serial.print(dist[4]); Serial.println(" mm");
  Serial.println("----------------");
}
void imprimirPiso() {

  for (int i = 0; i < 12; i++) {
    Serial.print("L");
    Serial.print(i+1);
    Serial.print(":");
    Serial.print(piso[i]);
    Serial.print(" ");
  }

  Serial.print(" | Activos: ");

  bool alguno = false;

  for (int i = 0; i < 12; i++) {
    if (linea[i]) {
      Serial.print("L");
      Serial.print(i+1);
      Serial.print(" ");
      alguno = true;
    }
  }

  if (!alguno) {
    Serial.print("Ninguno");
  }

  Serial.println();
}