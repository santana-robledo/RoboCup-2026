/**********************************************************************
 * ROBOT OMNIDIRECCIONAL 3 RUEDAS - ROBOCUP
 * Arduino Mega + IMU BNO080 + Control PI de orientación
 *
 * Protocolo Serial:
 *   M,Ux,Uy,Ut,Patada,Cilindro,Kp,Ki
 *   M,0.0,0.0,0.0,0,0,N
 *
 *   o enviar solo:
 *   N  -> Sin PID angular
 *   P  -> Usar PID angular
 *   L  -> Mantener último ángulo
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
// ============ CALIBRACIÓN DE MOTORES ==================
// =====================================================

float GAIN_A = 1.00;
float GAIN_B = 1.00;
float GAIN_C = 1.00;

// =====================================================
// ===================== IMU ============================
// =====================================================

BNO080 myIMU;

#define PIN_INT 2
#define PIN_RST 4
#define IMU_ADDR 0x4B

float theta_f = 0;
float setpoint = 0.0;
float yaw_offset = 0;
bool imu_calibrada = false;
unsigned long imu_start_time = 0;

// =====================================================
// ====================== PID ===========================
// =====================================================

float Kp = 3.1;
float Ki = 0.5;
float Kd = 0;

float error = 0.0;
float error_int = 0.0;
float Ut_pid = 0.0;

// =====================================================
// ================== TIEMPO (dt) =======================
// =====================================================

unsigned long lastTime = 0;
float dt = 0.0;

// =====================================================
// ================= MOVIMIENTO ROBOT ===================
// =====================================================

float Ux = 0.0;
float Uy = 0.0;
float Ut = 0.0;

float v1 = 0.0, v2 = 0.0, v3 = 0.0;
float wa = 0.0, wb = 0.0, wc = 0.0;

int pwm_a = 0, pwm_b = 0, pwm_c = 0;

// =====================================================
// ================== ESTRATEGIA ========================
// =====================================================

char modo = 'N';             // N, P, L
float ultimo_setpoint = 0;

// =====================================================
// ================= FILTRO VELOCIDAD ===================
// =====================================================

float Ux_prev = 0;
float Uy_prev = 0;
float alpha = 0.6;   // suavizado

// =====================================================
// ================= WATCHDOG ===========================
// =====================================================

unsigned long lastSerialTime = 0;

// ================= ACTUADORES =========================

int patada = 0;
int cilindro = 0;
bool pateando = false;
unsigned long tiempoPatada = 0;

#define SENSOR_PELOTA 9
int estadoPelota = 0;
int estadoPelotaPrev = -1;

// ================== PINES MOTORES =====================

// ===== MOTOR A Derecho =====
#define IN1 23
#define IN2 22
#define ENA 5

// ===== MOTOR B Izquierdo =====
#define IN3 24
#define IN4 25
#define ENB 6

// ===== MOTOR C =====
#define IN5 27
#define IN6 26
#define ENC 7

// ===== MOTOR Rodillo =====
#define IN7 28
#define IN8 29
#define END 3

// ===== Pateador =====
#define IN9 31
#define IN10 30
#define ENE 8

// =====================================================
// ===================== SETUP =========================
// =====================================================

void setup() {

  imu_start_time = millis();
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
// ======================= LOOP =========================
// =====================================================

void loop() {

  calcularTiempo();
  leerIMU();
  leerSerial();

  // Watchdog comunicación
  /*if (millis() - lastSerialTime > 200) {
    Ux = 0;
    Uy = 0;
    Ut = 0;
  } */

  controlOrientacion();
  cinematica();
  aplicarMotores();
  //DebugPrint();

}

// =====================================================
// ================= FUNCIONES ==========================

void calcularTiempo() {
  unsigned long currentTime = micros();
  dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;
  if (dt <= 0) dt = 0.0001;
}

void leerIMU() {

  if (myIMU.dataAvailable()) {

    float yaw_raw = myIMU.getYaw();

    if (!imu_calibrada && millis() - imu_start_time > 1500) {
      yaw_offset = yaw_raw;
      imu_calibrada = true;
    }

    if (imu_calibrada) {
      theta_f = yaw_raw - yaw_offset;
      theta_f = atan2(sin(theta_f), cos(theta_f));
    }
  }
}

// M,Ux,Uy,Ut,Patada,Cilindro,Kp,Ki
void leerSerial() {

  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();  // elimina \r y espacios

  if (input.length() == 1) {
    modo = input.charAt(0);
    return;
  }

  if (input.charAt(0) != 'M') return;

  char buffer[80];
  input.toCharArray(buffer, 80);

  char *token = strtok(buffer, ",");

  token = strtok(NULL, ","); if (token) Ux = atof(token);
  token = strtok(NULL, ","); if (token) Uy = atof(token);
  token = strtok(NULL, ","); if (token) Ut = atof(token);
  token = strtok(NULL, ","); if (token) patada = atoi(token);
  token = strtok(NULL, ","); if (token) cilindro = atoi(token);
  token = strtok(NULL, ","); if (token) modo = token[0];

  lastSerialTime = millis();
}

void controlOrientacion() {

  switch (modo) {

    case 'N':   // sin PID
      Ut_pid = 0;
      error_int = 0;
      setpoint = theta_f; 
      break;

    case 'P':   // PID activo
      error = setpoint - theta_f;
      error = atan2(sin(error), cos(error));

      error_int += error * dt;
      error_int = constrain(error_int, -2.0, 2.0);

      Ut_pid = Kp * error + Ki * error_int;
      Ut_pid = constrain(Ut_pid, -4.0, 4.0);

      ultimo_setpoint = setpoint;
      break;

    case 'L':   // mantener último ángulo
      error = ultimo_setpoint - theta_f;
      error = atan2(sin(error), cos(error));

      Ut_pid = Kp * error;
      Ut_pid = constrain(Ut_pid, -4.0, 4.0);
      break;
  }
}

void cinematica() {

   //===== FILTRO DE VELOCIDAD =====
if (Ux == 0 && Uy == 0) {
    Ux_prev = 0;
    Uy_prev = 0;
}
else {
    Ux = alpha * Ux + (1 - alpha) * Ux_prev;
    Uy = alpha * Uy + (1 - alpha) * Uy_prev;

    Ux_prev = Ux;
    Uy_prev = Uy;
    }


  // ===== TRANSFORMACIÓN CAMPO GLOBAL =====
  float Ux_robot = cos(theta_f) * Ux + sin(theta_f) * Uy;
  float Uy_robot = -sin(theta_f) * Ux + cos(theta_f) * Uy;

  float Ut_total = (modo == 'N') ? Ut : Ut_pid;

  v1 = (sin(theta_f) * Ux_robot) - (cos(theta_f) * Uy_robot) + (L * Ut_total);
  v2 = (cos(theta_f + PI/6) * Ux_robot) + (sin(theta_f + PI/6) * Uy_robot) + (L * Ut_total);
  v3 = (-sin(theta_f + PI/3) * Ux_robot) + (cos(theta_f + PI/3) * Uy_robot) + (L * Ut_total);

/*
  v1 = (-Uy_robot) + (L * Ut_total);
  v2 = ( (sqrt(3)/2)*Ux_robot + 0.5*Uy_robot ) + (L * Ut_total);
  v3 = ( (-sqrt(3)/2)*Ux_robot + 0.5*Uy_robot ) + (L * Ut_total);
*/

  wa = v1 / R;
  wb = v2 / R;
  wc = v3 / R;

  if (abs(wa) < 0.2) wa = 0;
  if (abs(wb) < 0.2) wb = 0;
  if (abs(wc) < 0.2) wc = 0;

// ===== LIMITADOR GLOBAL PROPORCIONAL =====
float max_w = max(abs(wa), max(abs(wb), abs(wc)));

// usamos el menor máximo como referencia global
float max_limit = min(MAX_RAD_A, min(MAX_RAD_B, MAX_RAD_C));

if (max_w > max_limit) {
  float scale = max_limit / max_w;
  wa *= scale;
  wb *= scale;
  wc *= scale;
}

  pwm_a = mapPWM(wa, MAX_RAD_A);
  pwm_b = mapPWM(wb, MAX_RAD_B);
  pwm_c = mapPWM(wc, MAX_RAD_C);

}
void aplicarMotores() {

// ===== MOTOR A =====
if (pwm_a == 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
}
else if (wa > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, pwm_a);
}
else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, pwm_a);
}

// ===== MOTOR B =====
if (pwm_b == 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
}
else if (wb > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, pwm_b);
}
else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, pwm_b);
}

// ===== MOTOR C =====
if (pwm_c == 0) {
    digitalWrite(IN5, LOW);
    digitalWrite(IN6, LOW);
    analogWrite(ENC, 0);
}
else if (wc > 0) {
    digitalWrite(IN6, HIGH);
    digitalWrite(IN5, LOW);
    analogWrite(ENC, pwm_c);
}
else {
    digitalWrite(IN6, LOW);
    digitalWrite(IN5, HIGH);
    analogWrite(ENC, pwm_c);
}
}

int mapPWM(float w, float max_rads) {

  // normalizamos velocidad (-1 a 1)
  float p = w / max_rads;
  p = constrain(p, -1.0, 1.0);

  // Escala lógica 0–7
  float escala_7 = fabs(p) * 7.0;

  // Mapear 0–7 → 0–255
  int pwm = (int)(escala_7 * (255.0 / 7.0));

  pwm = constrain(pwm, 0, 255);

  return pwm;
}

void DebugPrint(){
  Serial.print("Theta(rad): "); Serial.print(theta_f);
  //Serial.print(" Ux:"); Serial.print(Ux);
  //Serial.print(" Uy:"); Serial.print(Uy);
  //Serial.print(" Ut:"); Serial.print(Ut);
  //Serial.print(" Kp: "); Serial.print(Kp);
  //Serial.print(" Ki: "); Serial.print(Ki);
  Serial.print(" Pwm_a: "); Serial.print(pwm_a);
  Serial.print(" Pwm_b: "); Serial.print(pwm_b);
  Serial.print(" Pwm_c: "); Serial.print(pwm_c);
  Serial.print(" Cilindro: ");Serial.print(cilindro);
  Serial.print(" EstadoPelota: ");Serial.print(estadoPelota);
  Serial.print(" wa: "); Serial.print(wa);
  Serial.print(" wb: "); Serial.print(wb);
  Serial.print(" wc: "); Serial.print(wc);
  Serial.print(" modo: "); Serial.print(modo);
  Serial.print(" Patada: ");Serial.println(patada);
}
