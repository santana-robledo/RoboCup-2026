#include "I2Cdev.h"
#include <MPU6050.h>
#include <Wire.h>

/* 
Forma de enviar los datos
M,0.0,0.0,0.0,0,0,3.0,0.0
X    Y   t  P C  Kp  Ki
 */

//Constantes de velocidad máxima
const float MAX_RAD_A = 130 * 2 * PI / 60;
const float MAX_RAD_B = 170 * 2 * PI / 60;
const float MAX_RAD_C = 170 * 2 * PI / 60;

int estadoPelota = 0;
unsigned long lastSerialTime = 0;
int estadoPelotaPrev = -1;

int patada = 0;
int patada_prev = 0;
int cilindro = 0;

unsigned long tiempoPatada = 0;
bool pateando = false;

MPU6050 mpu;

//Variables IMU
float theta_f = 0.0;
float wz = 0.0;
float bias_z = 0.0;
float dt = 0.0;

unsigned long lastTime = 0;

//Robot
float L = 0.09, R = 0.029;
float Ux = 0.0, Uy = 0.0, Ut = 0.0;
float v1 = 0.0, v2 = 0.0, v3 = 0.0;

int pwm_a = 0, pwm_b = 0, pwm_c = 0;

float Ut_pid = 0.0;

//////////////  DERECHO ////////////////
// ===== MOTOR A =====
#define IN1  52
#define IN2  49
#define ENA   4

// ===== MOTOR B Left =====
#define IN3   26// Morado Izquierdo IN1
#define IN4  39 // Gris Izquierdo IN2    
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


float wa = 0.0, wb = 0.0, wc = 0.0;

//PID
float Kp = 3;
float Ki = 1;
float Kd = 0.0;

float setpoint = 0.0;

float error = 0.0;
float error_int = 0.0;

void setup() {

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

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

  pinMode(SENSOR_PELOTA, INPUT_PULLUP);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN5, OUTPUT); pinMode(IN6, OUTPUT); pinMode(ENC, OUTPUT);

  pinMode(IN7, OUTPUT); pinMode(IN8, OUTPUT); pinMode(END, OUTPUT);
  pinMode(RELE, OUTPUT);
  digitalWrite(RELE, LOW);
  stopMotorA();
  stopMotorB();
  stopMotorC();
  patada=0;

  lastTime = micros();
}

void loop() {
    if (millis() - lastSerialTime > 500) {
      Ux = 0;
      Uy = 0;
      Ut = 0;
      Ut_pid=0;
      error_int=0;
      cilindro=0;
      patada=0;
      digitalWrite(RELE, LOW);
      stopMotorA();
      stopMotorB();
      stopMotorC();
  }
  estadoPelota = digitalRead(SENSOR_PELOTA);
  //Serial.print("P,");Serial.println(estadoPelota);

  if(patada == 1 && patada_prev == 0 && !pateando){
    digitalWrite(RELE, HIGH);
    tiempoPatada = millis();
    pateando = true;
  }

  //Tiempo de activación
  if(pateando && millis() - tiempoPatada >= 120){
    digitalWrite(RELE, LOW);
    pateando = false;
  }

  //Guardar estado anterior
  patada_prev = patada;

  //Tiempo
  unsigned long currentTime = micros();
  dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;
  if (dt <= 0) dt = 0.0001;

  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  wz = ((gz - bias_z) / 131.0) * PI / 180.0;
  theta_f += wz * dt;
  theta_f = atan2(sin(theta_f), cos(theta_f));

  //PID
  error = setpoint - theta_f;
  error = atan2(sin(error), cos(error));

  error_int += error * dt;
  error_int = constrain(error_int, -2.0, 2.0);

  Ut_pid = Kp * error + Ki * error_int;
  Ut_pid = constrain(Ut_pid, -4.0, 4.0);

  //Serial
  if (Serial.available() > 0) {
    lastSerialTime = millis(); 
    String input = Serial.readStringUntil('\n');
    char buffer[50];
    input.toCharArray(buffer, 50);

    char *token = strtok(buffer, ",");
    token = strtok(NULL, ","); if(token) Ux = atof(token);
    token = strtok(NULL, ","); if(token) Uy = atof(token);
    token = strtok(NULL, ","); if(token) Ut = atof(token);
    token = strtok(NULL, ","); if(token) patada = atoi(token);
    token = strtok(NULL, ","); if(token) cilindro = atoi(token);
    //token = strtok(NULL, ","); if(token) Kp = atof(token);
    //token = strtok(NULL, ","); if(token) Ki = atof(token);
  }

  //Pateador
  if(patada == 1 && !pateando){
    digitalWrite(RELE, HIGH);
    tiempoPatada = millis();
    pateando = true;
  }

  if(pateando && millis() - tiempoPatada >= 120){
    digitalWrite(RELE, LOW);
    pateando = false;
    patada = 0;
  }

  //Cilindro
  if(cilindro == 1){
    digitalWrite(IN7, LOW);
    digitalWrite(IN8, HIGH);
    analogWrite(END, 255);
  } else {
    digitalWrite(IN7, LOW);
    digitalWrite(IN8, LOW);
    analogWrite(END, 0);
  }

  //Cinemática
  //Ut_pid=Ut;
  //theta_f=0;
  v1 = (sin(theta_f) * Ux) - (cos(theta_f) * Uy) + (L * Ut_pid);
  v2 = (cos(theta_f + PI/6) * Ux) + (sin(theta_f + PI/6) * Uy) + (L * Ut_pid);
  v3 = (-sin(theta_f + PI/3) * Ux) + (cos(theta_f + PI/3) * Uy) + (L * Ut_pid);

  wa = v1 / R;
  wb = v2 / R;
  wc = v3 / R;

  pwm_a = mapPWM(wa, MAX_RAD_A);
  pwm_b = mapPWM(wb, MAX_RAD_B);
  pwm_c = mapPWM(wc, MAX_RAD_C);

  //Motores
  (wa > 0) ? motorA_forward(pwm_a) : (wa < 0) ? motorA_backward(pwm_a) : stopMotorA();
  (wb > 0) ? motorB_forward(pwm_b) : (wb < 0) ? motorB_backward(pwm_b) : stopMotorB();
  (wc > 0) ? motorC_forward(pwm_c) : (wc < 0) ? motorC_backward(pwm_c) : stopMotorC();

  //Debug
//Debug
/*
  Serial.print("Theta: "); Serial.print(theta_f);
  Serial.print(" PWM A: "); Serial.print(pwm_a);
  Serial.print(" PWM B: "); Serial.print(pwm_b);
  Serial.print(" PWM C: "); Serial.print(pwm_c);
  Serial.print(" Pelota: "); Serial.print(estadoPelota);
  Serial.print(" Patada: "); Serial.print(patada);
  Serial.print(" Rodillo: "); Serial.println(cilindro);*/
}

//Funciones motores (igual que tu código)
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
