#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

bool leerMPU = false;

float yaw = 0;
unsigned long lastTime = 0;

//////////////  DERECHO ////////////////
// ===== MOTOR A =====
#define IN1  49
#define IN2  52
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
int sensor=33;

float wa = 0.0;
float wb = 0.0;
float wc = 0.0;
bool leerSensor = false;

String comando = "";

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  pinMode(ENC, OUTPUT);

  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);
  pinMode(END, OUTPUT);

  pinMode(RELE, OUTPUT);
  digitalWrite(RELE, LOW);
  

  stopMotorA();
  stopMotorB();
  stopMotorC();
  CilindroOFF();
  PateadorOff();
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("Error MPU6050");
  } else {
    Serial.println("MPU6050 listo");
  }

  lastTime = millis();

  Serial.println("Listo. Manda comando");
}

void loop() {

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {  // comando completo
      comando.trim();

      if (comando.length() == 0) return;

      Serial.println(comando);

      char motor = comando.charAt(0);
      int pwm = 0;

      if (comando.length() > 1) {
        pwm = comando.substring(1).toInt();
        pwm = constrain(pwm, -255, 255);
      }

      switch(motor){

        case 'A':
          if(pwm > 0) motorA_forward(pwm);
          else if(pwm < 0) motorA_backward(abs(pwm));
          else stopMotorA();
          break;

        case 'B':
          if(pwm > 0) motorB_forward(pwm);
          else if(pwm < 0) motorB_backward(abs(pwm));
          else stopMotorB();
          break;

        case 'C':
          if(pwm > 0) motorC_forward(pwm);
          else if(pwm < 0) motorC_backward(abs(pwm));
          else stopMotorC();
          break;

        case 'S':
          stopMotorA();
          stopMotorB();
          stopMotorC();
          CilindroOFF();
          break;
        
        case 'Q':
          PateadorON();
          break;
        
        case 'W':
          PateadorOff();
          break;

        case 'E':
          leerSensor = true;
          leerMPU = false;
          break;

        case 'F':
          leerSensor = false;
          leerMPU = false;
          break;

        case 'D':
          if(pwm > 0) CilindroON(pwm);
          else if(pwm < 0) CilindroON(abs(pwm));
          else CilindroOFF();
          break;
        
        case 'Z':
          leerMPU = true;
          leerSensor = false;
          break;

        case 'X':
          leerMPU = false;
          leerSensor = false;
          break;

        default:
          stopMotorA();
          stopMotorB();
          stopMotorC();
          break;
      }

      comando = "";  // limpiar buffer

    } else {
      comando += c;  // acumular caracteres
    }
  }
  if (leerSensor) {
  int sensorValue = digitalRead(sensor);
  Serial.println(sensorValue);
  delay(50);
}

if (leerMPU) {

  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  // rad/s
  float gz_rad = (gz / 131.0) * (PI / 180.0);

  // integrar yaw
  yaw += gz_rad * dt;

  Serial.println(yaw);
}
}

// ================= FUNCIONES =================

// ---- STOP ----
void stopMotorA() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
}

void stopMotorB() {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

// ---- MOTOR A ----
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

// ---- MOTOR B ----
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

// ---- MOTOR C ----
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

void moverFrente(int pwm){
  pwm = constrain(pwm, 0, 255);

  stopMotorA();
  motorB_forward(pwm);
  motorC_forward(pwm);
}

void moverAtras(int pwm){
  pwm = constrain(pwm, 0, 255);

  stopMotorA();
  motorB_backward(pwm);
  motorC_backward(pwm);
}

void GirarDerecha(int pwm){
  pwm = constrain(pwm, 0, 255);

  motorA_backward(pwm);
  motorB_forward(pwm/2);
  motorC_forward(pwm/2);
}

void GirarIzquierda(int pwm){
  pwm = constrain(pwm, 0, 255);

  motorA_forward(pwm);
  motorB_backward(pwm/2);
  motorC_backward(pwm/2);
}

void PateadorON(){
  digitalWrite(RELE, HIGH);
}

void PateadorOff(){ 
  digitalWrite(RELE, LOW);
}

void CilindroOFF(){   
  digitalWrite(IN7, LOW);
  digitalWrite(IN8, LOW);
  analogWrite(END, 0);
}

void CilindroON(int pwm){
  pwm = constrain(pwm, 0, 255);   
  digitalWrite(IN7, LOW);
  digitalWrite(IN8, HIGH);
  analogWrite(END, pwm);
}
