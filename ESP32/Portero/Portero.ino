#include <I2Cdev.h>
#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include <Adafruit_VL53L0X.h>
#include <esp_now.h>
#include <WiFi.h>

/* 
Forma de enviar los datos:
M,0.0,0.0,0.0,0,0,G
   X   Y   t  P C modo

P = pateador
C = cilindro
G = global
L = local
*/

bool usarPiso = false;
bool usarDistancia = true;
bool porteriaAzul = true;
char ROBOT_ID = 'A';

const int DISTANCIA_COLISION = 50;
const float VELOCIDAD_EVASION = 0.4;

unsigned long lastDist = 0;
const unsigned long INTERVALO_DIST = 120;

enum EstadoRobot {
  JUGAR,
  CONFIG,
  LINEA,
  EVADIR,
  DETENER,
};

EstadoRobot estadoActual = JUGAR;
EstadoRobot estadoAnterior = JUGAR;

uint8_t robotB_mac[] = {0xD8, 0x13, 0x2A, 0x7D, 0x92, 0x80};
uint8_t robotA_mac[] = {0xB8, 0xD6, 0x1A, 0x6B, 0x29, 0xC4};
uint8_t* peerAddress;
bool usarComunicacion = true;

const int UMBRAL_LINEA = 2000;

int contadorLinea[12] = {0};
const int LINEA_CONFIRMADA = 1;

#define MUX1 0x70
#define VL53_ADDR 0x29
#define NUM_SENSORES 4

Adafruit_VL53L0X sensor2 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor3 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor4 = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor5 = Adafruit_VL53L0X();

bool activo2 = false;
bool activo3 = false;
bool activo4 = false;
bool activo5 = false;

int dist[NUM_SENSORES];

void selectMuxChannel(uint8_t channel) {
  Wire.beginTransmission(MUX1);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delay(5);
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

void leerSensorDist(uint8_t canal, Adafruit_VL53L0X &sensor, bool activo, int &destino) {
  if (!activo) {
    destino = -1;
    return;
  }

  selectMuxChannel(canal);

  VL53L0X_RangingMeasurementData_t measure;
  sensor.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) {
    destino = measure.RangeMilliMeter;
  } else {
    destino = 8191;
  }
}

// ===== SENSORES DE PISO =====

float pisoFiltrado[12];
const float ALPHA_PISO = 1.0;
int umbral_parada = 2000;

const int umbral[12] = {
  0,
  0,
  500,
  500,
  500,
  500,
  500,
  500,
  0,
  0,
  500,
  500
};

bool parLinea[6];

float pisoX[12] = {
  -0.35, -0.35,  0.0,  0.0,
   0.35,  0.35,  0.0,  0.0,
  -0.35, -0.35, -0.35, -0.35
};

float pisoY[12] = {
  -0.35, -0.35,  0.35,  0.35,
   0.0,   0.0,  -0.35, -0.35,
   0.35,  0.35,  0.0,   0.0
};

unsigned long lastDebug = 0;

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

// ===== IMU =====

BNO080 bno080;

float theta_f = 0.0;
float theta_offset = 0.0;

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

float Kp = 2;
float Ki = 0.8;

float setpoint = 0.0;

float error = 0.0;
float error_int = 0.0;

char modo = 'G';

int sw1 = 0;
int sw2 = 0;
int sw3 = 0;
int sw1_prev = 0;

// ===== MOTOR A =====
#define IN1   17
#define IN2   5
#define ENA   18

// ===== MOTOR B =====
#define IN3   2
#define IN4   4
#define ENB   16

// ===== MOTOR C =====
#define IN5   15
#define IN6   13
#define ENC   14

// ===== PATEADOR / CILINDRO =====
#define IN7   19
#define IN8   23

// ===== SWITCHES =====
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

// ===================== IMU =====================
Serial.println();
Serial.println("=================================");
Serial.println("INICIANDO BNO080");
Serial.println("=================================");

Wire.begin(SDA_IMU, SCL_IMU);
Wire.setClock(100000);

bool imuOK = false;

for (int intento = 1; intento <= 5; intento++) {

  Serial.print("Intento ");
  Serial.print(intento);
  Serial.print("... ");

  if (bno080.begin()) {
    imuOK = true;
    Serial.println("OK");
    break;
  }

  Serial.println("FALLO");
  delay(500);
}

if (!imuOK) {

  Serial.println("ERROR: BNO080 NO DETECTADO");

} else {

  Serial.println("BNO080 DETECTADO");

  // EXACTAMENTE como en el programa mínimo
  bno080.enableGameRotationVector(50);
  delay(200);

  Serial.println("Esperando datos...");

  unsigned long inicio = millis();
  bool datosOK = false;

  while (millis() - inicio < 5000) {

    if (bno080.dataAvailable()) {

      float yaw0 = bno080.getYaw();

      Serial.print("Yaw inicial: ");
      Serial.println(yaw0, 6);

      if (!isnan(yaw0) && !isinf(yaw0)) {

        theta_offset = yaw0;
        theta_f = 0.0;
        yaw = 0.0;

        datosOK = true;
        break;
      }
    }

    delay(20);
  }

  if (datosOK) {
    Serial.println("BNO080 LISTO");
  } else {
    Serial.println("BNO080 SIN DATOS");
  }
}

Serial.println("=================================");

Serial.println("=================================");

  if (usarDistancia) {
    iniciarSensor(2, sensor2, activo2);
    iniciarSensor(3, sensor3, activo3);
    iniciarSensor(4, sensor4, activo4);
    iniciarSensor(5, sensor5, activo5);

    if (activo2) Serial.println("Canal 2 OK");
    if (activo3) Serial.println("Canal 3 OK");
    if (activo4) Serial.println("Canal 4 OK");
    if (activo5) Serial.println("Canal 5 OK");
  }

  if (ROBOT_ID == 'A') {
    peerAddress = robotB_mac;
  } else {
    peerAddress = robotA_mac;
  }

  lastTime = micros();

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

  error = setpoint - theta_f;
  error = atan2(sin(error), cos(error));

  error_int += error * dt;
  error_int = constrain(error_int, -2.0, 2.0);

  Ut_pid = Kp * error + Ki * error_int;
  Ut_pid = constrain(Ut_pid, -4.0, 4.0);

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

  if (usarDistancia && millis() - lastDist >= INTERVALO_DIST) {
    lastDist = millis();

    leerSensorDist(2, sensor2, activo2, dist[0]);
    leerSensorDist(3, sensor3, activo3, dist[1]);
    leerSensorDist(4, sensor4, activo4, dist[2]);
    leerSensorDist(5, sensor5, activo5, dist[3]);
  }

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

  parLinea[0] = linea[0] || linea[1];
  parLinea[1] = linea[2] || linea[3];
  parLinea[2] = linea[4] || linea[5];
  parLinea[3] = linea[6] || linea[7];
  parLinea[4] = linea[8] || linea[9];
  parLinea[5] = linea[10] || linea[11];

  bool hayLinea = false;

  if (usarPiso) {
    for (int i = 0; i < 6; i++) {
      if (parLinea[i]) {
        hayLinea = true;
        break;
      }
    }
  }

  estadoPelota = digitalRead(SENSOR_PELOTA);
  if (estadoPelota != estadoPelotaPrev) {
    estadoPelotaPrev = estadoPelota;
  }

  if (cambioSwitches) {
    estadoActual = CONFIG;
  }
  else if (hayLinea) {
    estadoActual = LINEA;
  }
  else {
    estadoActual = JUGAR;
  }

  bool cambioEstado = (estadoActual != estadoAnterior);
  if (cambioEstado) {
    estadoAnterior = estadoActual;
  }

  if (millis() - lastDebug >= 100) {
    lastDebug = millis();
    //imprimirPiso();
    debugRobot();
  }

  switch (estadoActual) {

    case JUGAR:

    // ===== PATEADOR P =====
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

    // ===== CILINDRO C =====
    if (cilindro == 1) {
      digitalWrite(IN7, HIGH);
      digitalWrite(IN8, LOW);
    } else {
      digitalWrite(IN7, LOW);
      digitalWrite(IN8, LOW);
    }

    patada_prev = patada;

    // ===== ANTICHOQUE SOLO EN JUGAR =====
    if (usarDistancia && dist[0] > 0 && dist[0] < DISTANCIA_COLISION) {
      Ux_cmd = -VELOCIDAD_EVASION;
      Uy_cmd = 0;
    }
    else if (usarDistancia && dist[2] > 0 && dist[2] < DISTANCIA_COLISION) {
      Ux_cmd = VELOCIDAD_EVASION;
      Uy_cmd = 0;
    }
    else if (usarDistancia && dist[3] > 0 && dist[3] < DISTANCIA_COLISION) {
      Ux_cmd = 0;
      Uy_cmd = -VELOCIDAD_EVASION;
    }
    else if (usarDistancia && dist[1] > 0 && dist[1] < DISTANCIA_COLISION) {
      Ux_cmd = 0;
      Uy_cmd = VELOCIDAD_EVASION;
    }

    break;

    case EVADIR:
      Ux_cmd = 0;
      Uy_cmd = 0;
      Ut = 0;
      Ut_pid = 0;
      digitalWrite(IN7, LOW);
      digitalWrite(IN8, LOW);
      break;

    case DETENER:
      Ux_cmd = 0;
      Uy_cmd = 0;
      Ut = 0;
      Ut_pid = 0;
      digitalWrite(IN7, LOW);
      digitalWrite(IN8, LOW);
      break;

    case LINEA:
      Ux_cmd = 0;
      Uy_cmd = 0;
      Ut = 0;
      Ut_pid = 0;
      error_int = 0;
      digitalWrite(IN7, LOW);
      digitalWrite(IN8, LOW);

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
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(ENB, PWM);
}

void motorB_backward(int PWM) {
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
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
  Serial.print(" | P: "); Serial.print(patada);
  Serial.print(" | C: "); Serial.print(cilindro);
  Serial.print(" | PWM_A: "); Serial.print(pwm_a);
  Serial.print(" | PWM_B: "); Serial.print(pwm_b);
  Serial.print(" | PWM_C: "); Serial.println(pwm_c);
}

void imprimirDistancias() {
  Serial.print("Frente: "); Serial.print(dist[0]); Serial.println(" mm");
  Serial.print("Derecha: "); Serial.print(dist[1]); Serial.println(" mm");
  Serial.print("Atras: "); Serial.print(dist[2]); Serial.println(" mm");
  Serial.print("Izquierda: "); Serial.print(dist[3]); Serial.println(" mm");
  Serial.println("----------------");
}

void imprimirPiso() {
  for (int i = 0; i < 12; i++) {
    Serial.print("L");
    Serial.print(i + 1);
    Serial.print(":");
    Serial.print(piso[i]);
    Serial.print(" ");
  }

  Serial.print(" | Activos: ");

  bool alguno = false;

  for (int i = 0; i < 12; i++) {
    if (linea[i]) {
      Serial.print("L");
      Serial.print(i + 1);
      Serial.print(" ");
      alguno = true;
    }
  }

  if (!alguno) {
    Serial.print("Ninguno");
  }

  Serial.println();
}