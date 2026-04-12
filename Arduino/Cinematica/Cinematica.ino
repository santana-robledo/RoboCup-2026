#include "I2Cdev.h"
#include <MPU6050.h>
#include <Wire.h>

/* 
Forma de enviar los datos
M,0.0,0.0,0.0,0,0
  Ux  Uy  Ut  P C
*/

// Constantes de velocidad máxima
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

// Variables IMU
float theta_f = 0.0;
float wz = 0.0;
float bias_z = 0.0;
float dt = 0.0;

unsigned long lastTime = 0;

// Robot
float L = 0.09, R = 0.029;
float Ux = 0.0, Uy = 0.0, Ut = 0.0;
float v1 = 0.0, v2 = 0.0, v3 = 0.0;

int pwm_a = 0, pwm_b = 0, pwm_c = 0;

float Ut_pid = 0.0;

// ===== MOTOR A (ATRÁS) =====
#define IN1  52
#define IN2  49
#define ENA   4

// ===== MOTOR B (código) = MOTOR C (físico) - FRENTE DERECHA =====
#define IN3  26
#define IN4  39
#define ENB   8

// ===== MOTOR C (código) = MOTOR B (físico) - FRENTE IZQUIERDA =====
#define IN5 28
#define IN6 31
#define ENC 7

// ===== Cilindro =====
#define IN7 51
#define IN8 53
#define END 6

// ===== Pateador =====
#define RELE 12
#define SENSOR_PELOTA 33

float wa = 0.0, wb = 0.0, wc = 0.0;

// PID para orientación
float Kp = 3.0;
float Ki = 1.0;
float Kd = 0.0;

float setpoint = 0.0;
float error = 0.0;
float error_int = 0.0;

// Control de estado
bool robot_activo = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("Error MPU6050");
    while (1);
  }

  // Calibrar giroscopio
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
  patada = 0;

  lastTime = micros();
  
  Serial.println("Robot iniciado - Frente opuesto a Motor A");
}

void loop() {
  // ===== TIMEOUT: Parar si no hay comandos =====
  if (millis() - lastSerialTime > 500) {
    Ux = 0;
    Uy = 0;
    Ut = 0;
    Ut_pid = 0;
    error_int = 0;
    cilindro = 0;
    patada = 0;
    robot_activo = false;
    digitalWrite(RELE, LOW);
    stopMotorA();
    stopMotorB();
    stopMotorC();
  }

  // ===== SENSOR PELOTA =====
  estadoPelota = digitalRead(SENSOR_PELOTA);
  
  if (estadoPelota != estadoPelotaPrev) {
    Serial.print("P,");
    Serial.println(estadoPelota);
    estadoPelotaPrev = estadoPelota;
  }

  // ===== PATEADOR =====
  if (patada == 1 && patada_prev == 0 && !pateando) {
    digitalWrite(RELE, HIGH);
    tiempoPatada = millis();
    pateando = true;
  }

  if (pateando && millis() - tiempoPatada >= 120) {
    digitalWrite(RELE, LOW);
    pateando = false;
  }

  patada_prev = patada;

  // ===== TIEMPO =====
  unsigned long currentTime = micros();
  dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;
  if (dt <= 0) dt = 0.0001;

  // ===== IMU =====
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  wz = ((gz - bias_z) / 131.0) * PI / 180.0;
  theta_f += wz * dt;
  theta_f = atan2(sin(theta_f), cos(theta_f));

  // ===== LEER SERIAL =====
  if (Serial.available() > 0) {
    lastSerialTime = millis();
    String input = Serial.readStringUntil('\n');
    char buffer[50];
    input.toCharArray(buffer, 50);

    char *token = strtok(buffer, ",");
    
    if (token && token[0] == 'M') {
      float nuevo_Ux = 0, nuevo_Uy = 0, nuevo_Ut = 0;
      
      token = strtok(NULL, ","); if (token) nuevo_Ux = atof(token);
      token = strtok(NULL, ","); if (token) nuevo_Uy = atof(token);
      token = strtok(NULL, ","); if (token) nuevo_Ut = atof(token);
      token = strtok(NULL, ","); if (token) patada = atoi(token);
      token = strtok(NULL, ","); if (token) cilindro = atoi(token);

      // Detectar transición parado -> movimiento
      bool nuevo_activo = (abs(nuevo_Ux) > 0.01 || abs(nuevo_Uy) > 0.01 || abs(nuevo_Ut) > 0.01);
      
      if (nuevo_activo && !robot_activo) {
        setpoint = theta_f;
        error_int = 0;
      }
      
      robot_activo = nuevo_activo;
      Ux = nuevo_Ux;
      Uy = nuevo_Uy;
      Ut = nuevo_Ut;
    }
    else if (token && token[0] == 'R') {
      theta_f = 0;
      setpoint = 0;
      error_int = 0;
      Serial.println("OK,RESET");
    }
  }

  // ===== CILINDRO =====
  if (cilindro == 1) {
    digitalWrite(IN7, LOW);
    digitalWrite(IN8, HIGH);
    analogWrite(END, 255);
  } else {
    digitalWrite(IN7, LOW);
    digitalWrite(IN8, LOW);
    analogWrite(END, 0);
  }

  // ===== PID DE ORIENTACIÓN =====
  if (abs(Ut) < 0.01) {
    error = setpoint - theta_f;
    error = atan2(sin(error), cos(error));
    
    error_int += error * dt;
    error_int = constrain(error_int, -2.0, 2.0);
    
    Ut_pid = Kp * error + Ki * error_int;
    Ut_pid = constrain(Ut_pid, -4.0, 4.0);
  } else {
    Ut_pid = Ut;
    setpoint = theta_f;
    error_int = 0;
  }

  // ===== CINEMÁTICA CORREGIDA =====
  // Motor A: ATRÁS (perpendicular al avance)
  // - No contribuye a Ux (avance)
  // - Contribuye a Uy (lateral)
  // - Contribuye a rotación
  v1 = Uy + (L * Ut_pid);
  
  // Motor B (código) = Motor C (físico): FRENTE-DERECHA
  // INTERCAMBIADO: usar ecuación de C original
  v2 = -0.5 * Uy - 0.866 * Ux + (L * Ut_pid);
  
  // Motor C (código) = Motor B (físico): FRENTE-IZQUIERDA  
  // INTERCAMBIADO: usar ecuación de B original
  v3 = -0.5 * Uy + 0.866 * Ux + (L * Ut_pid);

  // Calcular velocidades angulares
  wa = v1 / R;
  wb = v2 / R;
  wc = v3 / R;

  // Mapear a PWM
  pwm_a = mapPWM(wa, MAX_RAD_A);
  pwm_b = mapPWM(wb, MAX_RAD_B);
  pwm_c = mapPWM(wc, MAX_RAD_C);

  // ===== COMANDAR MOTORES (con deadband) =====
  if (wa > 0.1) motorA_forward(pwm_a);
  else if (wa < -0.1) motorA_backward(pwm_a);
  else stopMotorA();

  if (wb > 0.1) motorB_forward(pwm_b);
  else if (wb < -0.1) motorB_backward(pwm_b);
  else stopMotorB();

  if (wc > 0.1) motorC_forward(pwm_c);
  else if (wc < -0.1) motorC_backward(pwm_c);
  else stopMotorC();
}

// ===== FUNCIONES DE MOTORES =====
void stopMotorA() { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA, 0); }
void motorA_forward(int PWM) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, PWM); }
void motorA_backward(int PWM) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, PWM); }

void stopMotorB() { digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(ENB, 0); }
void motorB_forward(int PWM) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, PWM); }
void motorB_backward(int PWM) { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(ENB, PWM); }

void stopMotorC() { digitalWrite(IN5, LOW); digitalWrite(IN6, LOW); analogWrite(ENC, 0); }
void motorC_forward(int PWM) { digitalWrite(IN5, HIGH); digitalWrite(IN6, LOW); analogWrite(ENC, PWM); }
void motorC_backward(int PWM) { digitalWrite(IN5, LOW); digitalWrite(IN6, HIGH); analogWrite(ENC, PWM); }

int mapPWM(float w, float max_rads) {
  float p = constrain(w / max_rads, -1.0, 1.0);
  return abs(p) * 255;
}
