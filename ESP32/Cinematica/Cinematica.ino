#include "I2Cdev.h"
#include <MPU6050.h>
#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include <VL53L0X.h>
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

bool usarBNO080 = true;
bool serPortero = false;
bool usarPiso = true;
bool usarDistancia = false;
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

const int DISTANCIA_PORTERO = 150;
const int UMBRAL_LINEA = 2000;

////////// Sensores de distancia
#define MUX1 0x70
#define MUX2 0x74
#define NUM_SENSORES 11

VL53L0X sensores[NUM_SENSORES];

int dist[NUM_SENSORES];
const int distanciaSegura[NUM_SENSORES] = {
  60,   // S1
  30,   // S2
  50,   // S3
  50,   // S4
  50,   // S5
  30,   // S6
  30,   // S7
  90,   // S8
  100,  // S9
  100,  // S10
  60    // S11
};

const uint8_t muxSensor[NUM_SENSORES] = {
  MUX1, MUX1, MUX1, MUX1, MUX1,
  MUX2, MUX2, MUX2, MUX2, MUX2, MUX2
};

const uint8_t canalSensor[NUM_SENSORES] = {
  0, 1, 2, 3, 4,
  0, 1, 2, 3, 4, 5
};

const float FUERZA_EVASION = 0.3;

///// Sensores de piso
float pisoFiltrado[12];
const float ALPHA_PISO = 0.6;
int umbral_parada = 2000;

const int umbral[12] = {
  900,   // L0
  200,   // L1
  1450,  // L2
  1250,  // L3
  500,   // L4
  620,   // L5
  380,   // L6
  700,   // L7
  500,   // L8
  950,   // L9
  350,   // L10
  200    // L11
};

bool parLinea[6];
int paresLineaActivos = 0;
int sensoresLineaActivos = 0;

float pisoX[12] = {
  -0.5, -0.5,  0.0,  0.0,
   1.0,  1.0,  0.0,  0.0,
  -0.5, -0.5, -1.0, -1.0
};

float pisoY[12] = {
  -0.5, -0.5,  1.0,  1.0,
   0.0,  0.0, -1.0, -1.0,
   0.5,  0.5,  0.0,  0.0
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

MPU6050 mpu;
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
#define IN7   23
#define IN8   19

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
  Wire.beginTransmission(MUX1);
  Wire.write(0);
  Wire.endTransmission();

  Wire.beginTransmission(MUX2);
  Wire.write(0);
  Wire.endTransmission();
}

void tcaSelect(uint8_t mux, uint8_t canal) {
  disableAll();

  Wire.beginTransmission(mux);
  Wire.write(1 << canal);
  Wire.endTransmission();

  delay(2);
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

  Serial.print("C,");
  Serial.print(config.sw1);
  Serial.print(",");
  Serial.print(config.sw2);
  Serial.print(",");
  Serial.println(config.sw3);

  Wire.begin(SDA_IMU, SCL_IMU);
  Wire.setClock(100000);

if (usarBNO080) {
  Serial.println("Iniciando BNO080...");

  if (!bno080.begin()) {
    Serial.println("BNO080 no detectado");
    while (1);
  }

  Serial.println("BNO080 detectado");

  bno080.enableGameRotationVector(50);
  Serial.println("Game Rotation Vector habilitado");

  delay(3000);

  unsigned long t0 = millis();
  bool datosOK = false;

  while (millis() - t0 < 5000) {
    if (bno080.dataAvailable()) {
      datosOK = true;
      break;
    }
    delay(10);
  }

  if (!datosOK) {
    Serial.println("BNO080 sin datos");
    while (1);
  }

  theta_offset = bno080.getYaw();
  Serial.println("BNO080 listo");
}
  else {
    Serial.println("Iniciando MPU6050...");

    mpu.initialize();

    if (!mpu.testConnection()) {
      Serial.println("Error MPU6050");
      while (1);
    }

    Serial.println("Calibrando gyro...");

    long sumX = 0;
    long sumY = 0;
    long sumZ = 0;

    for (int i = 0; i < 3000; i++) {
      int16_t gx, gy, gz;
      mpu.getRotation(&gx, &gy, &gz);

      sumX += gx;
      sumY += gy;
      sumZ += gz;

      delay(2);
    }

    bias_x = sumX / 3000.0;
    bias_y = sumY / 3000.0;
    bias_z = sumZ / 3000.0;
    lastTime = micros();

    Serial.println("MPU6050 listo");
  }

  if (usarDistancia) {
    for (int i = 0; i < NUM_SENSORES; i++) {
      tcaSelect(muxSensor[i], canalSensor[i]);

      if (!sensores[i].init()) {
        Serial.print("Error sensor ");
        Serial.println(i + 1);
        while (1);
      }

      sensores[i].setTimeout(500);
      sensores[i].startContinuous();
    }

    disableAll();
  }

  if (ROBOT_ID == 'A') {
    peerAddress = robotB_mac;
  } else {
    peerAddress = robotA_mac;
  }

  lastTime = micros();

  Serial.println("Robot listo");

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

  if (usarBNO080) {
    if (bno080.dataAvailable()) {
      float yaw_raw = bno080.getYaw();

      theta_f = atan2(
        sin(yaw_raw - theta_offset),
        cos(yaw_raw - theta_offset)
      );

      yaw = theta_f;
    }
  } else {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    gx_dps = (gx - bias_x) / 131.0;
    gy_dps = (gy - bias_y) / 131.0;
    gz_dps = (gz - bias_z) / 131.0;

    gx_rad = gx_dps * PI / 180.0;
    gy_rad = gy_dps * PI / 180.0;
    gz_rad = gz_dps * PI / 180.0;

    float accel_roll = atan2(ay, az);
    float accel_pitch = atan2(-ax, sqrt(ay * ay + az * az));

    roll = 0.98 * (roll + gx_rad * dt) + 0.02 * accel_roll;
    pitch = 0.98 * (pitch + gy_rad * dt) + 0.02 * accel_pitch;

    yaw += gz_rad * dt;
    yaw = atan2(sin(yaw), cos(yaw));

    theta_f = yaw;
  }
  /////////////////////////////////////////////////////////////////

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
    for (int i = 0; i < NUM_SENSORES; i++) {
      tcaSelect(muxSensor[i], canalSensor[i]);

      uint16_t lectura = sensores[i].readRangeContinuousMillimeters();

      if (sensores[i].timeoutOccurred()) {
        dist[i] = -1;
      } else {
        dist[i] = lectura;
      }
    }
  }

  ////////////////////////////// Lectura de piso ///////////////////////
  for (int i = 0; i < 12; i++) {
    int lectura = leerMux(i);
    pisoFiltrado[i] = ALPHA_PISO * lectura + (1.0 - ALPHA_PISO) * pisoFiltrado[i];
    piso[i] = (int)pisoFiltrado[i];
    linea[i] = piso[i] < umbral[i];
  }

  parLinea[0] = linea[0] || linea[1];
  parLinea[1] = linea[2] || linea[3];
  parLinea[2] = linea[4] || linea[5];
  parLinea[3] = linea[6] || linea[7];
  parLinea[4] = linea[8] || linea[9];
  parLinea[5] = linea[10] || linea[11];

  /////////////////////////// Decisión de estado ///////////////////////
  bool hayLinea = false;
  bool hayObstaculo = false;

  if (usarPiso) {
    for (int i = 0; i < 6; i++) {
      if (parLinea[i]) {
        hayLinea = true;
        break;
      }
    }
  }

  if (usarDistancia) {
    for (int i = 0; i < NUM_SENSORES; i++) {
      if (dist[i] > 0 && dist[i] < distanciaSegura[i]) {
        hayObstaculo = true;
        break;
      }
    }
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
    estadoActual = LINEA;
  }
  else if (hayObstaculo) {
    estadoActual = DETENER;
  }
  else {
    estadoActual = JUGAR;
  }

  bool cambioEstado = (estadoActual != estadoAnterior);
  if (cambioEstado) {
    estadoAnterior = estadoActual;
  }

  ////////////////////////////// Debug /////////////////////////////////
  if (millis() - lastDebug >= 100) {
    lastDebug = millis();
    //debugRobot();
    //imprimirDistancias();
    imprimirPiso();
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

      if (pateando && millis() - tiempoPatada >= 120) {
        digitalWrite(IN7, LOW);
        digitalWrite(IN8, LOW);
        pateando = false;
        patada = 0;
      }

      patada_prev = patada;

      if (usarDistancia && serPortero && dist[5] > 0) {
        float errorDist = DISTANCIA_PORTERO - dist[5];

        if (abs(errorDist) > 20) {
          float correccion = errorDist * 0.009;
          correccion = constrain(correccion, -0.6, 0.6);
          Ux_cmd += correccion;
        }
      }
      break;

    case EVADIR:
      Ux_cmd = 0;
      Uy_cmd = 0;
      Ut = 0;
      Ut_pid = 0;
      break;

    case DETENER:
      Ux_cmd = 0;
      Uy_cmd = 0;
      Ut = 0;
      Ut_pid = 0;
      break;

    case LINEA:
      Ux_cmd = 0;
      Uy_cmd = 0;

      for (int i = 0; i < 12; i++) {
        if (piso[i] > umbral_parada) {
          estadoActual = DETENER;
          Ux_cmd = 0;
          Uy_cmd = 0;
          Ut = 0;
          Ut_pid = 0;
          break;
        }

        if (linea[i]) {
          Ux_cmd += pisoX[i];
          Uy_cmd += pisoY[i];
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

        serPortero = sw1;
        porteriaAzul = sw2;

        stopMotorA();
        stopMotorB();
        stopMotorC();

        configAnterior = config;
      }

      Ux_cmd = 0;
      Uy_cmd = 0;
      Ut = 0;
      Ut_pid = 0;
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
  for (int i = 0; i < NUM_SENSORES; i++) {
    Serial.print("S");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(dist[i]);
    Serial.print(" ");

    if (i < NUM_SENSORES - 1) Serial.print("| ");
  }
  Serial.println();
}

void imprimirPiso() {
  for (int i = 0; i < 12; i++) {
    Serial.print("L");
    Serial.print(i);
    Serial.print(":");
    Serial.print(piso[i]);
    Serial.print(" ");
  }
  Serial.println();
}
