#include "I2Cdev.h"
#include <MPU6050.h>
#include <Wire.h>

// ===== CONSTANTES =====

/* 
Forma de enviar los datos
M,0.0,0.0,0.0,0,0
X    Y   t  P C
 */

float kA = 0.6;
float kB = 1.0;
float kC = 1.0;


const float MAX_RAD_A = 300 * 2 * PI / 60;
const float MAX_RAD_B = 300 * 2 * PI / 60;
const float MAX_RAD_C = 300 * 2 * PI / 60;

int patada = 0;
int patada_prev = 0;
int cilindro = 0;

unsigned long tiempoPatada = 0;
bool pateando = false;

// ===== IMU =====
MPU6050 mpu;
float theta_f = 0.0;
float wz = 0.0;
float bias_z = 0.0;
float dt = 0.0;
unsigned long lastTime = 0;

// ===== ROBOT =====
float L = 0.09, R = 0.029;
float Ux = 0.0, Uy = 0.0, Ut = 0.0;
float v1 = 0.0, v2 = 0.0, v3 = 0.0;
float wa = 0.0, wb = 0.0, wc = 0.0;

// ===== PID =====
float Kp = 8;
float Ki = 0;
float Kd = 0.0;

float setpoint = 0.0;
float error = 0.0;
float error_prev = 0.0;
float error_int = 0.0;
float Ut_pid = 0.0;

bool firstRun = true;

//////////////  DERECHO ////////////////
// ===== MOTOR A =====
#define IN1  52
#define IN2  49
#define ENA   4

// ===== MOTOR B Left =====
#define IN3   39// Morado Izquierdo IN1
#define IN4  26 // Gris Izquierdo IN2    
#define ENB   8// Azul Izquierdo

//////////////  IZQUIERDO ////////////////
// ===== MOTOR C =====
#define IN5 28
#define IN6  31
#define ENC 7

// ===== MOTOR Cilindro =====
#define IN7 51 // 
#define IN8 53 // 
#define END 6  // 

// ===== Pateador =====
#define RELE 12  //
#define SENSOR_PELOTA 33

int estadoPelotaPrev = -1;
unsigned long lastSerialTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("Error MPU6050");
    while (1);
  }

  // Calibración
  long sum = 0;
  for (int i = 0; i < 2000; i++) {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    sum += gz;
    delay(2);
  }
  bias_z = sum / 2000.0;

  pinMode(SENSOR_PELOTA, INPUT_PULLUP);
  pinMode(RELE, OUTPUT);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN5, OUTPUT); pinMode(IN6, OUTPUT); pinMode(ENC, OUTPUT);
  pinMode(IN7, OUTPUT);pinMode(IN8, OUTPUT);pinMode(END, OUTPUT);

  lastTime = micros();
}

void loop() {

  // ===== TIMEOUT =====
  /*
  if (millis() - lastSerialTime > 500) {
    Ux = Uy = Ut = 0;
    error_int = 0;
    cilindro = 0;
    patada = 0;
    digitalWrite(RELE, LOW);
    stopAll();
  }*/

  // ===== SENSOR PELOTA =====
  /*
  int estadoPelota = digitalRead(SENSOR_PELOTA);
  if (estadoPelota != estadoPelotaPrev) {
    Serial.print("P,");
    Serial.println(estadoPelota);
    estadoPelotaPrev = estadoPelota;
  }*/

  // ===== TIEMPO =====
  unsigned long currentTime = micros();
  dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;
  if (dt <= 0) dt = 0.0001;

  // ===== IMU =====
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  float gz_corregido = gz - bias_z;

  // Zona muerta para eliminar drift
  if (abs(gz_corregido) < 50) {
    gz_corregido = 0;
  }

  wz = (gz_corregido / 131.0) * PI / 180.0;
  theta_f += wz * dt;
  theta_f = atan2(sin(theta_f), cos(theta_f));

  // ===== SETPOINT INICIAL =====
  if (firstRun) {
    setpoint = theta_f;
    firstRun = false;
  }

  // ===== PID =====
  error = setpoint - theta_f;
  error = atan2(sin(error), cos(error));

  // Zona muerta para evitar oscilaciones cerca del objetivo
  if (abs(error) < 0.02) {
    error = 0;
    error_int = 0;
  }

  error_int += error * dt;
  error_int = constrain(error_int, -1.0, 1.0);

  float derivative = (error - error_prev) / dt;

  Ut_pid = Kp * error + Ki * error_int + Kd * derivative;
  Ut_pid = constrain(Ut_pid, -30.0, 30.0);

  error_prev = error;

  // ===== SERIAL =====
  if (Serial.available() > 0) {
    lastSerialTime = millis();

    String input = Serial.readStringUntil('\n');
    char buffer[60];
    input.toCharArray(buffer, 60);

    char *token = strtok(buffer, ",");
    token = strtok(NULL, ","); if(token) Ux = atof(token);
    token = strtok(NULL, ","); if(token) Uy = atof(token);
    token = strtok(NULL, ","); if(token) Ut = atof(token);
    token = strtok(NULL, ","); if(token) patada = atoi(token);
    token = strtok(NULL, ","); if(token) cilindro = atoi(token);
    Ux = constrain(Ux, -7.0, 7.0);
    Uy = constrain(Uy, -3.0,3.0);
    Ut = constrain(Ut, -7.0, 7.0);
    float scale = MAX_RAD_A / 7.0;
    Ux *= scale;
    Uy *= scale;
    Ut *= scale;
  }



  // ===== PATEADOR (ANTI-REBOTE) =====
  if (patada == 1 && patada_prev == 0 && !pateando) {
    digitalWrite(RELE, HIGH);
    tiempoPatada = millis();
    pateando = true;
  }

  if (pateando && millis() - tiempoPatada >= 120) {
    digitalWrite(RELE, LOW);
    pateando = false;
    patada = 0;
  }

  patada_prev = patada;

  // ===== RODILLO =====
  if (cilindro == 1) {
    digitalWrite(IN7, LOW);
    digitalWrite(IN8, HIGH);
    analogWrite(END, 255);
  } else {
    digitalWrite(IN7, LOW);
    digitalWrite(IN8, LOW);
    analogWrite(END, 0);
  }

  // ===== CONTROL FINAL =====
  float Ut_total = Ut_pid;

  v1 = (sin(theta_f) * Ux) - (cos(theta_f) * Uy) + (L * Ut_total);
  v2 = (cos(theta_f + PI/6) * Ux) + (sin(theta_f + PI/6) * Uy) + (L * Ut_total);
  v3 = (-sin(theta_f + PI/3) * Ux) + (cos(theta_f + PI/3) * Uy) + (L * Ut_total);

  wa = (v1 / R) * kA;
  wb = (v2 / R) * kB;
  wc = (v3 / R) * kC;

  float max_w = max(max(abs(wa), abs(wb)), abs(wc));

  if (max_w > MAX_RAD_A) {
      float scale = MAX_RAD_A / max_w;
      wa *= scale;
      wb *= scale;
      wc *= scale;
  }

  int pwm_a = mapPWM(wa, MAX_RAD_A);
  int pwm_b = mapPWM(wb, MAX_RAD_B);
  int pwm_c = mapPWM(wc, MAX_RAD_C);

  // ===== MOTORES =====
  (wa > 0) ? motorA_forward(pwm_a) : (wa < 0) ? motorA_backward(pwm_a) : stopMotorA();
  (wb > 0) ? motorB_forward(pwm_b) : (wb < 0) ? motorB_backward(pwm_b) : stopMotorB();
  (wc > 0) ? motorC_forward(pwm_c) : (wc < 0) ? motorC_backward(pwm_c) : stopMotorC();
  
  Serial.print("Theta: "); Serial.print(theta_f);
  Serial.print(" Ux: "); Serial.print(Ux);
  Serial.print(" Uy: "); Serial.print(Uy);
  Serial.print(" Ut: "); Serial.print(Ut);
  //Serial.print(" Pelota: "); Serial.print(estadoPelota);
  Serial.print(" Patada: "); Serial.print(patada);
  Serial.print(" Rodillo: "); Serial.print(cilindro);
  Serial.print(" Ut_pid: "); Serial.print(Ut_pid);
  Serial.print(" Ut_total: "); Serial.print(Ut_total);
  Serial.print(" Error: "); Serial.println(error);
}

// ===== FUNCIONES =====
void stopAll(){
  stopMotorA(); stopMotorB(); stopMotorC();
  digitalWrite(IN7, LOW);
  digitalWrite(IN8, LOW);
  analogWrite(END, 0);
}

void stopMotorA(){ digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA,0);}
void motorA_forward(int PWM){ digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW); analogWrite(ENA,PWM);}
void motorA_backward(int PWM){ digitalWrite(IN1,LOW); digitalWrite(IN2,HIGH); analogWrite(ENA,PWM);}

void stopMotorB(){ digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(ENB,0);}
void motorB_forward(int PWM){ digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW); analogWrite(ENB,PWM);}
void motorB_backward(int PWM){ digitalWrite(IN3,LOW); digitalWrite(IN4,HIGH); analogWrite(ENB,PWM);}

void stopMotorC(){ digitalWrite(IN5, LOW); digitalWrite(IN6, LOW); analogWrite(ENC,0);}
void motorC_forward(int PWM){ digitalWrite(IN5,HIGH); digitalWrite(IN6,LOW); analogWrite(ENC,PWM);}
void motorC_backward(int PWM){ digitalWrite(IN5,LOW); digitalWrite(IN6,HIGH); analogWrite(ENC,PWM);}

int mapPWM(float w, float max_rads){
  float p = constrain(w / max_rads, -1.0, 1.0);
  return abs(p) * 255;
}
