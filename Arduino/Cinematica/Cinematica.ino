#include "I2Cdev.h"
#include <MPU6050.h>
#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include <VL53L0X.h>

/* 
Forma de enviar los datos
M,0.0,0.0,0.0,0,0,G
   X   Y   t  P C modo
     G=global, L=Local
*/

bool usarBNO080 = false;

#define TCAADDR 0x70

VL53L0X vl53_1;
VL53L0X vl53_2;
VL53L0X vl53_3;

unsigned long lastDebug = 0;

float evadeX = 0.0;
float evadeY = 0.0;

const int DISTANCIA_SEGURA = 120;

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

// Angulos
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

int dist1 = 0;
int dist2 = 0;
int dist3 = 0;

char modo = 'G';

int sw1=0;
int sw2=0;
int sw3=0;

// ===== MOTOR A =====
#define ENA   14
#define IN1   15
#define IN2   13

// ===== MOTOR B =====
#define ENB   16
#define IN3   2
#define IN4   4

// ===== MOTOR C =====
#define ENC   18
#define IN5   5
#define IN6   17

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
#define MUX_S0 12
#define MUX_S1 27
#define MUX_S2 26
#define MUX_EN 25

// ===== I2C =====
#define SDA_IMU 21
#define SCL_IMU 22

// ===== PWM =====
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

void tcaSelect(uint8_t i) {

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();

  delay(5);
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);

  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);
  pinMode(SW3, INPUT);
  delay(50);

  struct Config {
  int sw1;
  int sw2;
  int sw3; };
  Config config;

  config.sw1 = digitalRead(SW1);
  config.sw2 = digitalRead(SW2);
  config.sw3 = digitalRead(SW3);

  Serial.print("C,");
  Serial.print(config.sw1);
  Serial.print(",");
  Serial.print(config.sw2);
  Serial.print(",");
  Serial.println(config.sw3);

  Wire.begin(SDA_IMU, SCL_IMU);
  Wire.setClock(400000);

  if (usarBNO080) {

    //Serial.println("Iniciando BNO080...");

    if (!bno080.begin()) {

      //Serial.println("BNO080 no detectado");

      while (1);
    }

    bno080.enableRotationVector(50);

    delay(2000);

    while (!bno080.dataAvailable()) {

      delay(10);
    }

    theta_offset = bno080.getYaw();

    //Serial.print("Offset inicial: ");
    //Serial.println(theta_offset * 180.0 / PI);

    //Serial.println("BNO080 listo");
  }

  else {

    //Serial.println("Iniciando MPU6050...");

    mpu.initialize();

    if (!mpu.testConnection()) {

      //Serial.println("Error MPU6050");

      while (1);
    }

    //Serial.println("Calibrando gyro...");

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

    //Serial.println("MPU6050 listo");
  }

  tcaSelect(1);

  if (!vl53_1.init()) {

    Serial.println("VL53 #1 no detectado");

    while (1);
  }

  vl53_1.setTimeout(100);
  vl53_1.startContinuous();

  tcaSelect(2);

  if (!vl53_2.init()) {

    Serial.println("VL53 #2 no detectado");

    while (1);
  }

  vl53_2.setTimeout(100);
  vl53_2.startContinuous();

  tcaSelect(3);

  if (!vl53_3.init()) {

    Serial.println("VL53 #3 no detectado");

    while (1);
  }

  vl53_3.setTimeout(100);
  vl53_3.startContinuous();

  pinMode(SENSOR_PELOTA, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);

  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);

  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);

  pinMode(SIG_MUX, INPUT);

  pinMode(MUX_EN, OUTPUT);

  digitalWrite(MUX_EN, LOW);

  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENC, PWM_FREQ, PWM_RESOLUTION);

  stopMotorA();
  stopMotorB();
  stopMotorC();

  lastTime = micros();

  Serial.println("Robot listo");
}

void loop() {

  evadeX = 0.0;
  evadeY = 0.0;

  estadoPelota = digitalRead(SENSOR_PELOTA);

  if (estadoPelota != estadoPelotaPrev) {

    Serial.print("P,");
    Serial.println(estadoPelota);

    estadoPelotaPrev = estadoPelota;
  }

  tcaSelect(1);

  dist1 = vl53_1.readRangeContinuousMillimeters();

  if (vl53_1.timeoutOccurred()) dist1 = -1;

  tcaSelect(2);

  dist2 = vl53_2.readRangeContinuousMillimeters();

  if (vl53_2.timeoutOccurred()) dist2 = -1;

  tcaSelect(3);

  dist3 = vl53_3.readRangeContinuousMillimeters();

  if (vl53_3.timeoutOccurred()) dist3 = -1;

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
  }

  else {

    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    mpu.getMotion6(
      &ax, &ay, &az,
      &gx, &gy, &gz
    );

    gx_dps = (gx - bias_x) / 131.0;
    gy_dps = (gy - bias_y) / 131.0;
    gz_dps = (gz - bias_z) / 131.0;

    gx_rad = gx_dps * PI / 180.0;
    gy_rad = gy_dps * PI / 180.0;
    gz_rad = gz_dps * PI / 180.0;

    float accel_roll =
      atan2(ay, az);

    float accel_pitch =
      atan2(-ax, sqrt(ay * ay + az * az));

    roll =
      0.98 * (roll + gx_rad * dt) +
      0.02 * accel_roll;

    pitch =
      0.98 * (pitch + gy_rad * dt) +
      0.02 * accel_pitch;

    yaw += gz_rad * dt;

    yaw = atan2(sin(yaw), cos(yaw));

    theta_f = yaw;
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

    char *token = strtok(buffer, ",");

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

  if (!pateando) {

    if (cilindro == 1) {

      digitalWrite(IN7, LOW);
      digitalWrite(IN8, HIGH);

    } else {

      digitalWrite(IN7, LOW);
      digitalWrite(IN8, LOW);
    }
  }

  // D2 = FRENTE -> ir hacia atras
  if (dist2 > 30 && dist2 < DISTANCIA_SEGURA) {

    float fuerza = (DISTANCIA_SEGURA - dist2) / (float)DISTANCIA_SEGURA;

    evadeX -= fuerza;
  }

  // D1 = IZQUIERDA -> ir derecha
  if (dist1 > 30 && dist1 < DISTANCIA_SEGURA) {

    float fuerza = (DISTANCIA_SEGURA - dist1) / (float)DISTANCIA_SEGURA;

    evadeY -= fuerza;
  }

  // D3 = DERECHA -> ir izquierda
  if (dist3 > 30 && dist3 < DISTANCIA_SEGURA) {

    float fuerza = (DISTANCIA_SEGURA - dist3) / (float)DISTANCIA_SEGURA;

    evadeY += fuerza;
  }

  Ux_cmd = Ux + evadeX;
  Uy_cmd = Uy + evadeY;

  Ux_cmd = constrain(Ux_cmd, -1.0, 1.0);
  Uy_cmd = constrain(Uy_cmd, -1.0, 1.0);

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

  if (millis() - lastDebug >= 100) {

    lastDebug = millis();

    debugRobot();
  }
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

  return abs(pwm);
}

void debugRobot() {

  Serial.print("Ux: ");Serial.print(Ux_cmd, 2);
  Serial.print(" | Uy: ");Serial.print(Uy_cmd, 2);
  Serial.print(" | Ut: ");Serial.print(Ut, 2);
  Serial.print(" | Yaw: ");Serial.print(yaw, 2);
  Serial.print(" | D1: ");Serial.print(dist1);
  Serial.print(" | D2: ");Serial.print(dist2);
  Serial.print(" | D3: ");Serial.print(dist3);
  Serial.print(" | PWM_A: ");Serial.print(pwm_a);
  Serial.print(" | PWM_B: ");Serial.print(pwm_b);
  Serial.print(" | PWM_C: ");Serial.print(pwm_c);
  Serial.print(" | SW1: "); Serial.print(sw1);
Serial.print(" | SW2: "); Serial.print(sw2);
Serial.print(" | SW3: "); Serial.println(sw3);
}
