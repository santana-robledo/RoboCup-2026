/**********************************************************************
 * ROBOT OMNIDIRECCIONAL 3 RUEDAS - ROBOCUP
 * Arduino Mega + IMU BNO080 + Control PI de orientación
 *
 * Protocolo Serial:
 *   M,Ux,Uy,Ut,Patada,Cilindro,Kp,Ki
 *
 **********************************************************************/

#include <Wire.h> // Comunicación I2C
#include "SparkFun_BNO080_Arduino_Library.h" // Librería IMU

// =====================================================
// ================= CONSTANTES FÍSICAS =================
// =====================================================

// Conversión de RPM a rad/s: ω = RPM * 2π / 60
const float MAX_RAD_A = 130 * 2 * PI / 60;
const float MAX_RAD_B = 170 * 2 * PI / 60;
const float MAX_RAD_C = 170 * 2 * PI / 60;

// Geometría del robot
float L = 0.09;   // Distancia centro-rueda (m)
float R = 0.029;  // Radio rueda (m)

// =====================================================
// ===================== IMU ===========================
// =====================================================

BNO080 myIMU;

#define PIN_INT 2
#define PIN_RST 4
#define IMU_ADDR 0x4B

// Variables de orientación
float theta_f = 0;      // Yaw actual (rad)
float setpoint = 0.0;   // Ángulo objetivo (rad)

// =====================================================
// ====================== PID ==========================
// =====================================================

float Kp = 20;
float Ki = 5;
float Kd = 0.2; // Declarado pero no usado actualmente

float error = 0.0;
float error_int = 0.0;
float error_der = 0.0;

float Ut_pid = 0.0;     // Velocidad angular corregida por PI

// =====================================================
// ================== TIEMPO (dt) ======================
// =====================================================

unsigned long lastTime = 0;
float dt = 0.0;

// =====================================================
// ================= MOVIMIENTO ROBOT ==================
// =====================================================

float Ux = 0.0;   // Velocidad lineal eje X (m/s)
float Uy = 0.0;   // Velocidad lineal eje Y (m/s)
float Ut = 0.0;   // Velocidad angular deseada (rad/s)

float v1 = 0.0, v2 = 0.0, v3 = 0.0; // Velocidades lineales en ruedas
float wa = 0.0, wb = 0.0, wc = 0.0; // Velocidades angulares ruedas

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

// Cilindro
#define IN7 28
#define IN8 29
#define END 3

// Pateador
#define IN9 31
#define IN10 30
#define ENE 8

// =====================================================
// ===================== SETUP =========================
// =====================================================

void setup() {

  Serial.begin(115200);

  // Configuración de pines motores
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN5, OUTPUT); pinMode(IN6, OUTPUT); pinMode(ENC, OUTPUT);
  pinMode(IN7, OUTPUT); pinMode(IN8, OUTPUT); pinMode(END, OUTPUT);
  pinMode(IN9, OUTPUT); pinMode(IN10, OUTPUT); pinMode(ENE, OUTPUT);
  pinMode(SENSOR_PELOTA, INPUT);

  // Inicialización I2C
  Wire.begin();
  Wire.setClock(400000);

  // Inicialización IMU
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
  debugPrint();
}

// =====================================================
// ================= FUNCIONES =========================
// =====================================================

// ---------------- Tiempo ----------------
// Calcula dt en segundos usando micros()
void calcularTiempo() {
  unsigned long currentTime = micros();
  dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;
  if (dt <= 0) dt = 0.0001;
}

// ================= Sensores =========================

// ---------------- Pelota ----------------
// Solo imprime cuando cambia estado
void leerPelota() {
  estadoPelota = digitalRead(SENSOR_PELOTA);
  if (estadoPelota != estadoPelotaPrev) {
    Serial.print("P,");
    Serial.println(estadoPelota);
    estadoPelotaPrev = estadoPelota;
  }
}

// ---------------- IMU ----------------
// Obtiene yaw en radianes
void leerIMU() {
  if (myIMU.dataAvailable()) {
    theta_f = myIMU.getYaw();
  }
}

// ---------------- Sensores de piso ----------------
/*
void SensorPiso(){

}
*/

// ---------------- Sensores de obstáculos ----------------
/*
void SensorObstaculos(){
  
}
*/


// ---------------- Switch para cambio de porteria ----------------



// ===================================================

// ---------------- Control PI ----------------
// Normaliza error angular con atan2
void controlOrientacion() {

  error = setpoint - theta_f;

  // Normalización angular [-π, π]
  error = atan2(sin(error), cos(error));

  // Integral
  error_int += error * dt;
  error_int = constrain(error_int, -2.0, 2.0);

  // Control PI
  Ut_pid = Kp * error + Ki * error_int;
  Ut_pid = constrain(Ut_pid, -4.0, 4.0);
}

// ---------------- Serial ----------------
// Parsea CSV
void leerSerial() {
  if (Serial.available()) {

    String input = Serial.readStringUntil('\n');
    char buffer[50];
    input.toCharArray(buffer, 50);

    char *token = strtok(buffer, ",");
    token = strtok(NULL, ","); if(token) Ux = atof(token);
    token = strtok(NULL, ","); if(token) Uy = atof(token);
    token = strtok(NULL, ","); if(token) Ut = atof(token);
    token = strtok(NULL, ","); if(token) patada = atoi(token);
    token = strtok(NULL, ","); if(token) cilindro = atoi(token);
    token = strtok(NULL, ","); if(token) Kp = atof(token);
    token = strtok(NULL, ","); if(token) Ki = atof(token);
  }
}

// ---------------- Actuadores ----------------
// Patada temporizada 120 ms
void controlActuadores() {

  if(patada == 1 && !pateando){
    digitalWrite(IN9, LOW);
    digitalWrite(IN10, HIGH);
    analogWrite(ENE, 255);
    tiempoPatada = millis();
    pateando = true;
  }

  if(pateando && millis() - tiempoPatada >= 120){
    digitalWrite(IN9, LOW);
    digitalWrite(IN10, LOW);
    analogWrite(ENE, 0);
    pateando = false;
    patada = 0;
  }

  if(cilindro == 1){
    digitalWrite(IN7, LOW);
    digitalWrite(IN8, HIGH);
    analogWrite(END, 255);
  }
  else{
    digitalWrite(IN7, LOW);
    digitalWrite(IN8, LOW);
    analogWrite(END, 0);
  }
}

// ---------------- Cinemática Omni ----------------
// Convierte velocidades del robot a ruedas
void cinematica() {

  Ut_pid = Ut;      // Modo prueba
  theta_f = 0;      // Modo prueba

  // Transformación cinemática (ruedas separadas 120°)
  v1 = (sin(theta_f) * Ux) - (cos(theta_f) * Uy) + (L * Ut_pid);
  v2 = (cos(theta_f + PI/6) * Ux) + (sin(theta_f + PI/6) * Uy) + (L * Ut_pid);
  v3 = (-sin(theta_f + PI/3) * Ux) + (cos(theta_f + PI/3) * Uy) + (L * Ut_pid);

  wa = v1 / R;
  wb = v2 / R;
  wc = v3 / R;

  pwm_a = mapPWM(wa, MAX_RAD_A);
  pwm_b = mapPWM(wb, MAX_RAD_B);
  pwm_c = mapPWM(wc, MAX_RAD_C);
}

// ---------------- Motores ----------------
void aplicarMotores() {

  if (wa > 0) { digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW); }
  else if (wa < 0) { digitalWrite(IN1,LOW); digitalWrite(IN2,HIGH); }
  else { digitalWrite(IN1,LOW); digitalWrite(IN2,LOW); }
  analogWrite(ENA,pwm_a);

  if (wb > 0) { digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW); }
  else if (wb < 0) { digitalWrite(IN3,LOW); digitalWrite(IN4,HIGH); }
  else { digitalWrite(IN3,LOW); digitalWrite(IN4,LOW); }
  analogWrite(ENB,pwm_b);

  if (wc > 0) { digitalWrite(IN5,HIGH); digitalWrite(IN6,LOW); }
  else if (wc < 0) { digitalWrite(IN5,LOW); digitalWrite(IN6,HIGH); }
  else { digitalWrite(IN5,LOW); digitalWrite(IN6,LOW); }
  analogWrite(ENC,pwm_c);
}

// ---------------- Debug ----------------
void debugPrint() {
  Serial.print("Yaw(rad): "); Serial.print(theta_f);
  Serial.print(" PWM A: "); Serial.print(pwm_a);
  Serial.print(" PWM B: "); Serial.print(pwm_b);
  Serial.print(" PWM C: "); Serial.println(pwm_c);
}

// ---------------- Map PWM ----------------
int mapPWM(float velocidad_angular, float max_rads){
  float porcentaje = velocidad_angular / max_rads;
  porcentaje = constrain(porcentaje, -1.0, 1.0);
  return abs(porcentaje) * 255;
}