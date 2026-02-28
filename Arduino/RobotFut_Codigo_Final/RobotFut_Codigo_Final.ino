/**********************************************************************
 * ROBOT OMNIDIRECCIONAL 3 RUEDAS - ROBOCUP
 * Arduino Mega + IMU BNO080 + Control PI de orientación
 *
 * Protocolo Serial:
 *   M,Ux,Uy,Ut,Patada,Cilindro,Kp,Ki
 *
 **********************************************************************/

#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"

// =====================================================
// ================= CONSTANTES FÍSICAS =================
// =====================================================

const float MAX_RAD_A = 130 * 2 * PI / 60;
const float MAX_RAD_B = 170 * 2 * PI / 60;
const float MAX_RAD_C = 170 * 2 * PI / 60;

float L = 0.09;
float R = 0.029;

// =====================================================
// ============ CORRECCIÓN SENTIDO MOTORES ==============
// =====================================================
// Ajusta signos si algún motor gira al revés (solo signos)
int DIR_A = -1;   // A invertido (como ya viste)
int DIR_B =  1;   // B normal (según tu última prueba en rotación)
int DIR_C =  1;   // C normal

// =====================================================
// ===================== IMU ===========================
// =====================================================

BNO080 myIMU;

#define PIN_INT 2
#define PIN_RST 4
#define IMU_ADDR 0x4B

float theta_f = 0;
float setpoint = 0.0;

// =====================================================
// ====================== PID ==========================
// =====================================================

float Kp = 20;
float Ki = 5;
float Kd = 0.2;

float error = 0.0;
float error_int = 0.0;
float error_der = 0.0;

float Ut_pid = 0.0;

// =====================================================
// ================== TIEMPO (dt) ======================
// =====================================================

unsigned long lastTime = 0;
float dt = 0.0;

// =====================================================
// ================= MOVIMIENTO ROBOT ==================
// =====================================================

float Ux = 0.0;
float Uy = 0.0;
float Ut = 0.0;

float v1 = 0.0, v2 = 0.0, v3 = 0.0;
float wa = 0.0, wb = 0.0, wc = 0.0;

int pwm_a = 0, pwm_b = 0, pwm_c = 0;

// =====================================================
// ================= ACTUADORES ========================
// =====================================================

int patada = 0;
int cilindro = 0;
bool pateando = false;
unsigned long tiempoPatada = 0;

#define SENSOR_PELOTA 9
int estadoPelota = 0;
int estadoPelotaPrev = -1;

// =====================================================
// ================== PINES MOTORES ====================
// =====================================================

// Motor A
#define IN1 23
#define IN2 22
#define ENA 5

// Motor B
#define IN3 24
#define IN4 25
#define ENB 6

// Motor C
#define IN5 26
#define IN6 27
#define ENC 7

// Cilindro (rodillo)
#define IN7 28
#define IN8 29
#define END 3

// Pateador
#define IN9 31
#define IN10 30
#define ENE 8

int s0, s1, s2, s3, s4, s5; //variables donde guardamos valores de sensores de linea
// =====================================================
// ===================== SETUP =========================
// =====================================================

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN5, OUTPUT); pinMode(IN6, OUTPUT); pinMode(ENC, OUTPUT);
  pinMode(IN7, OUTPUT); pinMode(IN8, OUTPUT); pinMode(END, OUTPUT);
  pinMode(IN9, OUTPUT); pinMode(IN10, OUTPUT); pinMode(ENE, OUTPUT);
  pinMode(SENSOR_PELOTA, INPUT);

  Wire.begin();
  Wire.setClock(400000);

  if (!myIMU.begin(IMU_ADDR, Wire, PIN_INT)) {
    Serial.println("Error IMU");
    while (1);
  }

  myIMU.enableRotationVector(50);

  lastTime = micros();
}

// =====================================================
// ======================= LOOP ========================
// =====================================================

void loop() {
  calcularTiempo();
  leerPelota();
  leerIMU();
  controlOrientacion();
  leerSerial();
  controlActuadores();
  cinematica();
  aplicarMotores();
  //debugPrint(); // limitado a 5 Hz
  //leerSensores();

}

// =====================================================
// ================= FUNCIONES =========================
// =====================================================


void leerSensores() {
  s0 = analogRead(A0); //Frente derecho
  s1 = analogRead(A1); //Frente izquierdo
  s2 = analogRead(A2); //Lateral derecho superior
  s3 = analogRead(A3); //Lateral derecho inferior
  s4 = analogRead(A4); //Lateral izquierdo superior
  s5 = analogRead(A5); //Lateral izquierdo inferior

  if (s0 < 500 || s1 < 500) {   // línea al frente
    Ux = -0.4;
    Uy = 0;
    Ut = 0;
  }

  else if (s2 < 500 || s3 < 500) { // línea derecha
    Ux = 0;
    Uy = -0.4;
    Ut = 0;
  }

  else if (s4 < 500 || s5 < 500) { // línea izquierda
    Ux = 0;
    Uy = 0.4;
    Ut = 0;
  }

  else if (s0 < 500 && s1 < 500 && s2 < 500 && s3 < 500 && s4 < 500 && s5 < 500) { // Levantan el Robot
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    digitalWrite(IN5, LOW); digitalWrite(IN6, LOW);
    digitalWrite(IN7, LOW); digitalWrite(IN8, LOW);
    digitalWrite(IN9, LOW); digitalWrite(IN10, LOW);


}

}


void calcularTiempo() {
  unsigned long currentTime = micros();
  dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;
  if (dt <= 0) dt = 0.0001;
}

void leerPelota() {
  estadoPelota = digitalRead(SENSOR_PELOTA);
  if (estadoPelota != estadoPelotaPrev) {
    Serial.print("P,");
    Serial.println(estadoPelota);
    estadoPelotaPrev = estadoPelota;
  }
}

void leerIMU() {
  if (myIMU.dataAvailable()) {
    theta_f = myIMU.getYaw();
  }
}

void controlOrientacion() {
  error = setpoint - theta_f;
  error = atan2(sin(error), cos(error));

  error_int += error * dt;
  error_int = constrain(error_int, -2.0, 2.0);

  Ut_pid = Kp * error + Ki * error_int;
  Ut_pid = constrain(Ut_pid, -4.0, 4.0);
}

// Protocolo: M,Ux,Uy,Ut,Patada,Cilindro,Kp,Ki
void leerSerial() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    char buffer[60];
    input.toCharArray(buffer, 60);

    char *token = strtok(buffer, ",");
    token = strtok(NULL, ","); if (token) Ux = atof(token);
    token = strtok(NULL, ","); if (token) Uy = atof(token);
    token = strtok(NULL, ","); if (token) Ut = atof(token);
    token = strtok(NULL, ","); if (token) patada = atoi(token);
    token = strtok(NULL, ","); if (token) cilindro = atoi(token);
    token = strtok(NULL, ","); if (token) Kp = atof(token);
    token = strtok(NULL, ","); if (token) Ki = atof(token);
  }
}

void controlActuadores() {

  // Patada 120 ms
  if (patada == 1 && !pateando) {
    digitalWrite(IN9, LOW);
    digitalWrite(IN10, HIGH);
    analogWrite(ENE, 255);
    tiempoPatada = millis();
    pateando = true;
  }

  if (pateando && millis() - tiempoPatada >= 120) {
    digitalWrite(IN9, LOW);
    digitalWrite(IN10, LOW);
    analogWrite(ENE, 0);
    pateando = false;
    patada = 0;
  }

  // Rodillo/cilindro
  if (cilindro == 1) {
    digitalWrite(IN7, LOW);
    digitalWrite(IN8, HIGH);
    analogWrite(END, 255);
  } else {
    digitalWrite(IN7, LOW);
    digitalWrite(IN8, LOW);
    analogWrite(END, 0);
  }
}

void cinematica() {

  Ut_pid = Ut;   // modo prueba
  theta_f = 0;   // modo prueba

  // Cinemática omni (según tu modelo actual)
  v1 = (sin(theta_f) * Ux) - (cos(theta_f) * Uy) + (L * Ut_pid);
  v2 = (cos(theta_f + PI / 6) * Ux) + (sin(theta_f + PI / 6) * Uy) + (L * Ut_pid);
  v3 = (-sin(theta_f + PI / 3) * Ux) + (cos(theta_f + PI / 3) * Uy) + (L * Ut_pid);

  // ====== AQUÍ ESTÁ EL CAMBIO CLAVE ======
  // Intercambiamos B y C porque tu "B física" corresponde al modelo de v3
  wa = v2 / R;   // ✅ A toma v2
  wb = v1 / R;   // ✅ B toma v1
  wc = v3 / R;   // C igual

  // Corrección de sentido por motor
  wa *= DIR_A;
  wb *= DIR_B;
  wc *= DIR_C;

  pwm_a = mapPWM(wa, MAX_RAD_A);
  pwm_b = mapPWM(wb, MAX_RAD_B);
  pwm_c = mapPWM(wc, MAX_RAD_C);
}

void aplicarMotores() {

  if (wa > 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else if (wa < 0) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); }
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); }
  analogWrite(ENA, pwm_a);

  if (wb > 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else if (wb < 0) { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); }
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); }
  analogWrite(ENB, pwm_b);

  if (wc > 0) { digitalWrite(IN5, HIGH); digitalWrite(IN6, LOW); }
  else if (wc < 0) { digitalWrite(IN5, LOW); digitalWrite(IN6, HIGH); }
  else { digitalWrite(IN5, LOW); digitalWrite(IN6, LOW); }
  analogWrite(ENC, pwm_c);
}

// Debug limitado (5 Hz) para no saturar Serial
void debugPrint() {
  static unsigned long last = 0;
  if (millis() - last < 200) return;
  last = millis();

  Serial.print("Ux: "); Serial.print(Ux, 2);
  Serial.print(" Uy: "); Serial.print(Uy, 2);
  Serial.print(" Ut: "); Serial.print(Ut, 2);

  Serial.print(" | PWM A: "); Serial.print(pwm_a);
  Serial.print(" PWM B: "); Serial.print(pwm_b);
  Serial.print(" PWM C: "); Serial.println(pwm_c);
}

int mapPWM(float w, float max_rads) {
  float p = w / max_rads;
  p = constrain(p, -1.0, 1.0);
  return (int)(fabs(p) * 255);
}
