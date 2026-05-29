#include "I2Cdev.h"
#include <MPU6050.h>
#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

bool usarBNO080 = true;

/* 
Forma de enviar los datos
M,0.0,0.0,0.0,0,0,G
   X   Y   t  P C modo
     G=global, L=Local
 */

// ================= ROBOT =================
float L = 0.09, R = 0.029;
float Ux = 0.0, Uy = 0.0, Ut = 0.0;
float v1 = 0.0, v2 = 0.0, v3 = 0.0;

// Velocidad máxima
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
int cilindro = 0;

unsigned long tiempoPatada = 0;
bool pateando = false;

MPU6050 mpu;
BNO080 bno080;

// ================= IMU =================
float theta_f = 0.0;
float wz = 0.0;
float bias_z = 0.0;
float dt = 0.0;

unsigned long lastTime = 0;

int pwm_a = 0, pwm_b = 0, pwm_c = 0;

float Ut_pid = 0.0;

// ======================================================
// ==================== PINES ESP32 =====================
// ======================================================

// ===== MOTOR A =====
#define ENA   14
#define IN1   13
#define IN2   15

// ===== MOTOR B =====
#define ENB   16
#define IN3   4
#define IN4   2

// ===== MOTOR C =====
#define ENC   18
#define IN5   5
#define IN6   17

// ===== CILINDRO =====
#define IN7   19
#define IN8   23

// ===== SENSOR PELOTA =====
#define SENSOR_PELOTA 33

// ===== MULTIPLEXOR =====
#define SIG_MUX 34
#define MUX_S0 12
#define MUX_S1 27
#define MUX_S2 26
#define MUX_EN 25

// ===== SWITCHES =====
#define SWITCH_1 39
#define SWITCH_2 32
#define SWITCH_3 35

// ===== I2C =====
#define SDA_IMU 21
#define SCL_IMU 22

// ===== PWM ESP32 =====
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

float wa = 0.0, wb = 0.0, wc = 0.0;

// ================= PID =================
float Kp = 3;
float Ki = 1;
float Kd = 0.0;

float setpoint = 0.0;

float error = 0.0;
float error_int = 0.0;

char modo = 'G';

void setup() {

  Ux = 0.0;
  Uy = 0.0;
  Ut = 0.0;

  Serial.begin(115200);
  Serial.setTimeout(10);

  // ================= I2C ESP32 =================

  Wire.begin(SDA_IMU, SCL_IMU);
  Wire.setClock(400000);

  // ================= IMU =================

  if (usarBNO080) {

    Serial.println("Iniciando BNO080...");

    if (!bno080.begin()) {

      Serial.println("BNO080 no detectado");

      while (1);
    }

    bno080.enableRotationVector(50);

    Serial.println("BNO080 listo");
  }

  else {

    Serial.println("Iniciando MPU6050...");

    mpu.initialize();

    if (!mpu.testConnection()) {

      Serial.println("Error MPU6050");

      while (1);
    }

    long sum = 0;

    for (int i = 0; i < 2000; i++) {

      int16_t gx, gy, gz;

      mpu.getRotation(&gx, &gy, &gz);

      sum += gz;

      delay(2);
    }

    bias_z = sum / 2000.0;

    Serial.println("MPU6050 listo");
  }

  // ================= PINES =================

  pinMode(SENSOR_PELOTA, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);

  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);

  // Multiplexor
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);

  pinMode(SIG_MUX, INPUT);

  pinMode(MUX_EN, OUTPUT);

  digitalWrite(MUX_EN, LOW);

  // Switches
  pinMode(SWITCH_1, INPUT);
  pinMode(SWITCH_2, INPUT);
  pinMode(SWITCH_3, INPUT);

  // ================= PWM ESP32 =================

  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENC, PWM_FREQ, PWM_RESOLUTION);

  stopMotorA();
  stopMotorB();
  stopMotorC();

  patada = 0;

  lastTime = micros();
}

void loop() {

  // ================= SENSOR PELOTA =================

  estadoPelota = digitalRead(SENSOR_PELOTA);

  if (estadoPelota != estadoPelotaPrev) {

    Serial.print("P,");
    Serial.println(estadoPelota);

    estadoPelotaPrev = estadoPelota;
  }

  // ================= PATADA =================

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

  // ================= TIEMPO =================

  unsigned long currentTime = micros();

  dt = (currentTime - lastTime) / 1000000.0;

  lastTime = currentTime;

  if (dt <= 0) dt = 0.0001;

  // ================= IMU =================

  if (usarBNO080) {

    if (bno080.dataAvailable()) {

      theta_f = bno080.getYaw();
    }
  }

  else {

    int16_t gx, gy, gz;

    mpu.getRotation(&gx, &gy, &gz);

    wz = ((gz - bias_z) / 131.0) * PI / 180.0;

    theta_f += wz * dt;

    theta_f = atan2(sin(theta_f), cos(theta_f));
  }

  // ================= PID =================

  error = setpoint - theta_f;

  error = atan2(sin(error), cos(error));

  error_int += error * dt;

  error_int = constrain(error_int, -2.0, 2.0);

  Ut_pid = Kp * error + Ki * error_int;

  Ut_pid = constrain(Ut_pid, -4.0, 4.0);

  // ================= SERIAL =================

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

  // ================= CILINDRO =================

  if (cilindro == 1) {

    digitalWrite(IN7, LOW);
    digitalWrite(IN8, HIGH);

  } else {

    digitalWrite(IN7, LOW);
    digitalWrite(IN8, LOW);
  }

  // ================= CINEMATICA =================

  if (modo == 'G') {

    v1 = (sin(theta_f) * Ux) - (cos(theta_f) * Uy) + (L * Ut_pid);

    v2 = (cos(theta_f + PI / 6) * Ux) +
         (sin(theta_f + PI / 6) * Uy) +
         (L * Ut_pid);

    v3 = (-sin(theta_f + PI / 3) * Ux) +
         (cos(theta_f + PI / 3) * Uy) +
         (L * Ut_pid);
  }

  else if (modo == 'L') {

    v1 = (-Uy + (L * Ut));

    v2 = ((0.866 * Ux) + (0.5 * Uy) + (Ut * L));

    v3 = ((-0.866 * Ux) + (0.5 * Uy) + (Ut * L));
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

  // ================= MOTORES =================

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

// ======================================================
// ================= FUNCIONES MOTORES ==================
// ======================================================

void stopMotorA() {

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  analogWrite(ENA, 0);
}

void motorA_forward(int PWM) {

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  analogWrite(ENA, PWM);
}

void motorA_backward(int PWM) {

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  analogWrite(ENA, PWM);
}

void stopMotorB() {

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  analogWrite(ENB, 0);
}

void motorB_forward(int PWM) {

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENB, PWM);
}

void motorB_backward(int PWM) {

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENB, PWM);
}

void stopMotorC() {

  digitalWrite(IN5, LOW);
  digitalWrite(IN6, LOW);

  analogWrite(ENC, 0);
}

void motorC_forward(int PWM) {

  digitalWrite(IN5, HIGH);
  digitalWrite(IN6, LOW);

  analogWrite(ENC, PWM);
}

void motorC_backward(int PWM) {

  digitalWrite(IN5, LOW);
  digitalWrite(IN6, HIGH);

  analogWrite(ENC, PWM);
}

int mapPWM_linear(float v, float kv) {

  float pwm = kv * v;

  pwm = constrain(pwm, -255, 255);

  return abs(pwm);
}
