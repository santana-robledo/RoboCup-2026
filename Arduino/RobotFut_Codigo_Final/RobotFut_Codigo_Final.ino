/**********************************************************************
 * ROBOT OMNIDIRECCIONAL 3 RUEDAS
 *
 * Protocolo Serial:
 *   M,Ux,Uy,Ut,Patada,Cilindro,Kp,Ki
 *   M,0.0,0.0,0.0,0,0,N
 *
 *   N  -> Sin PID angular
 *   P  -> Usar PID angular
 *   L  -> Mantener último ángulo
 *
 **********************************************************************/

#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"

// ================= velocidad máxima permitida de cada motor. =================

const float MAX_RAD_A = 130 * 2 * PI / 60;
const float MAX_RAD_B = 170 * 2 * PI / 60;
const float MAX_RAD_C = 170 * 2 * PI / 60;

float L = 0.09; // distancia al centro del robot
float R = 0.029; // radio rueda

// ============ Ganancias de motores ==================

float GAIN_A = 1.00;
float GAIN_B = 1.00;
float GAIN_C = 1.00;

// ===================== IMU ============================

BNO080 myIMU;

#define PIN_INT 2
#define PIN_RST 4
#define IMU_ADDR 0x4B

float theta_f = 0; // radio rueda
float setpoint = 0.0; // referencia para PID
float yaw_offset = 0; // salida PID para rotación
bool imu_calibrada = false;
unsigned long imu_start_time = 0;

// ====================== Variables de PID ===========================

float Kp = 3.1;
float Ki = 0.5;
float Kd = 0;

float error = 0.0;
float error_int = 0.0;
float Ut_pid = 0.0;

// ================== TIEMPO (dt) =======================

unsigned long lastTime = 0;
float dt = 0.0;

// ================= Variables para velocidades de ruedas ==========

float Ux = 0.0;
float Uy = 0.0;
float Ut = 0.0;

float v1 = 0.0, v2 = 0.0, v3 = 0.0; // velocidades lineales ruedas
float wa = 0.0, wb = 0.0, wc = 0.0; // rad/s ruedas

int pwm_a = 0, pwm_b = 0, pwm_c = 0; //PWM final enviado a los motores.

// ================== Modelo Angular ========================

char modo = 'N';             // N, P, L control angular
float ultimo_setpoint = 0;

// ================= FILTRO VELOCIDAD ===================

float Ux_prev = 0;
float Uy_prev = 0;
float alpha = 0.6;   // suavizado

// ================= Controlamos datos del serial ===========================

unsigned long lastSerialTime = 0;
unsigned long lastCommandTime=0;

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
#define IN1 23 // Morado Derecho
#define IN2 22 // Azul Derecho
#define ENA 5  // Gris Derecho

// ===== MOTOR B Izquierdo =====
#define IN3 24 // Morado Izquierdo IN1
#define IN4 25 // Gris Izquierdo IN2    
#define ENB 6  // Azul Izquierdo

// ===== MOTOR C Derecho =====
#define IN5 27 // Verde Derecho IN3
#define IN6 26 // Amarillo Derecho IN4
#define ENC 7  // Naranja Derecho

// ===== MOTOR Rodillo =====
#define IN7 28 // Negro Izquierd
#define IN8 29 // Blanco Izquierdo
#define END 3  // Cafe Izquierdo

// ===== Pateador =====
#define IN9 31 // 
#define IN10 30 // 
#define ENE 8  //

void setup() {

  //Inicializa arduino, pines y comunicación 12C
  imu_start_time = millis();
  Serial.begin(115200); 

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN5, OUTPUT); pinMode(IN6, OUTPUT); pinMode(ENC, OUTPUT);
  pinMode(IN7, OUTPUT); pinMode(IN8, OUTPUT); pinMode(END, OUTPUT);
  pinMode(IN9, OUTPUT); pinMode(IN10, OUTPUT); pinMode(ENE, OUTPUT);
  pinMode(SENSOR_PELOTA, INPUT);
  Ux = 0.0;
  Uy = 0.0;
  Ut = 0.0;


  Wire.begin();
  Wire.setClock(400000);

  if (!myIMU.begin(IMU_ADDR, Wire, PIN_INT)) {
    Serial.println("Error IMU");
    while (1);
  }

  myIMU.enableRotationVector(50);
  lastTime = micros();
}

void loop() {

  calcularTiempo();
  leerIMU();
  leerSerial();

  // seguridad si se pierde comunicación.
  if (millis() - lastSerialTime > 200) {
    Ux = 0;
    Uy = 0;
    Ut = 0;
    Ut_pid=0;
    error_int=0;
    modo='N';
  }

  float Ut_total = obtenerUtTotal();
  
  if(Ux==0 && Uy==0 && Ut_total==0){ //Freno activo
    pwm_a=pwm_b=pwm_c=0;
  }

  controlOrientacion();
  cinematica();
  aplicarMotores();
  //DebugPrint();

}

// ================= FUNCIONES ==========================

void calcularTiempo() { //Calcula el tiempo transcurrido (dt) entre iteraciones del loop()
  unsigned long currentTime = micros();
  dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;
  if (dt <= 0) dt = 0.0001; //Evita división por cero si el tiempo es muy pequeño.
}

void leerIMU() {

  if (myIMU.dataAvailable()) {//Lee la IMU BNO080 para obtener la orientación del robot (yaw).

    float yaw_raw = myIMU.getYaw();

    if (!imu_calibrada && millis() - imu_start_time > 1500) {
      yaw_offset = yaw_raw; //Aplica un offset para normalizar el ángulo respecto al inicio.
      imu_calibrada = true; //bandera de imu calibrada
    }

    if (imu_calibrada) {
      theta_f = yaw_raw - yaw_offset; //Ahora calculamos error respecto a ese angulo
      theta_f = atan2(sin(theta_f), cos(theta_f)); //Convertimos el angulo al rango [−π,π] 
    }
  }
}

// M,Ux,Uy,Ut,Patada,Cilindro,Kp,Ki
void leerSerial() {

  if (!Serial.available()) return; //Si no hay serial no regresamos nada

  String input = Serial.readStringUntil('\n');
  input.trim();  // elimina \r y espacios

  if (input.length() == 1) {
    modo = input.charAt(0);
    return;
  }

  if (input.charAt(0) != 'M') return; //Si no inicia con M no regresamos nada

  char buffer[80];
  input.toCharArray(buffer, 80);

  char *token = strtok(buffer, ",");

  //Tomamos los datos del serial
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
      error = setpoint - theta_f; //Calculamos error
      error = atan2(sin(error), cos(error)); //Acotamos error

      error_int += error * dt; //Sumamos el error integral
      error_int = constrain(error_int, -2.0, 2.0); //Lo acotamos

      Ut_pid = Kp * error + Ki * error_int; //Sumamos acciones de control
      Ut_pid = constrain(Ut_pid, -4.0, 4.0); //Acotamos

      ultimo_setpoint = setpoint;//Actualizamos setpoint
      break;

    case 'L':   // mantener último ángulo
      error = ultimo_setpoint - theta_f; //Actualizamos el angulo objetivo
      error = atan2(sin(error), cos(error)); //Acotamos

      Ut_pid = Kp * error; //Proporcional
      Ut_pid = constrain(Ut_pid, -4.0, 4.0);//Acotamos
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


  
  float Ux_robot = cos(theta_f) * Ux + sin(theta_f) * Uy;
  float Uy_robot = -sin(theta_f) * Ux + cos(theta_f) * Uy;
  float Ut_total = (modo == 'N') ? Ut : Ut_pid;
/*
  v1 = (sin(theta_f) * Ux_robot) - (cos(theta_f) * Uy_robot) + (L * Ut_total);
  v2 = (cos(theta_f + PI/6) * Ux_robot) + (sin(theta_f + PI/6) * Uy_robot) + (L * Ut_total);
  v3 = (-sin(theta_f + PI/3) * Ux_robot) + (cos(theta_f + PI/3) * Uy_robot) + (L * Ut_total);
*/

  //Cinematica
  v1 = (-Uy_robot) + (L * Ut_total);
  v2 = ( (sqrt(3)/2)*Ux_robot + 0.5*Uy_robot ) + (L * Ut_total);
  v3 = ( (-sqrt(3)/2)*Ux_robot + 0.5*Uy_robot ) + (L * Ut_total);

  // Transforma las velocidades del campo global a velocidades de rueda
  wa = v1 / R;
  wb = v2 / R;
  wc = v3 / R;

  //Definimos sentido de giro
  if (abs(wa) < 0.2) wa = 0;
  if (abs(wb) < 0.2) wb = 0;
  if (abs(wc) < 0.2) wc = 0;

// Calcula la mayor velocidad absoluta entre las tres ruedas.
float max_w = max(abs(wa), max(abs(wb), abs(wc)));

// Toma el límite más pequeño de las tres ruedas según sus capacidades físicas.
float max_limit = min(MAX_RAD_A, min(MAX_RAD_B, MAX_RAD_C));

if (max_w > max_limit) { //Si alguna rueda está por encima del límite permitido, se escala proporcionalmente para que todas las ruedas respeten el máximo
  float scale = max_limit / max_w;
  wa *= scale;
  wb *= scale;
  wc *= scale;
}

  pwm_a = mapPWM(wa, MAX_RAD_A);
  pwm_b = mapPWM(wb, MAX_RAD_B);
  pwm_c = mapPWM(wc, MAX_RAD_C);

}
void aplicarMotores() { //Envía señales PWM y dirección a cada motor según

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

int mapPWM(float w, float max_rads) { //Mapear PWM

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

float obtenerUtTotal(){
  if(modo=='N') return Ut;
  else return Ut_pid;
}
