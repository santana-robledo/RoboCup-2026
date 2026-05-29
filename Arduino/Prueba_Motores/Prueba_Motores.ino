#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include <MPU6050.h>
#include <VL53L0X.h>

VL53L0X vl53_1;
VL53L0X vl53_2;
VL53L0X vl53_3;

bool usarBNO080 = false;
bool usarDistancia= false;

BNO080 bno080;
MPU6050 mpu;

bool modoDebug = false;

float yaw = 0;

String comando = "";

unsigned long lastTime = 0;
unsigned long lastDebug = 0;

#define TCAADDR 0x70

#define SDA_BNO 21
#define SCL_BNO 22

// ================= MOTORES =================
#define ENA 14
#define IN1 13
#define IN2 15

#define ENB 16
#define IN3 4
#define IN4 2

#define ENC 18
#define IN5 5
#define IN6 17

// ================= PATEADOR =================
#define IN7 19
#define IN8 23

// ================= SENSORES =================
#define PELOTA 33

// ================= MUX =================
#define SIG_MUX 34
#define S0 12
#define S1 27
#define S2 26
#define S3 25

// ================= SWITCHES =================
#define SW1 39
#define SW2 32
#define SW3 35

// ================= PWM =================
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

// =====================================================

void seleccionarCanal(int canal) {

  digitalWrite(S0, canal & 0x01);
  digitalWrite(S1, (canal >> 1) & 0x01);
  digitalWrite(S2, (canal >> 2) & 0x01);
  digitalWrite(S3, (canal >> 3) & 0x01);
}

// =====================================================

void tcaSelect(uint8_t i) {

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

// =====================================================

int leerMux(int canal) {

  seleccionarCanal(canal);

  delayMicroseconds(100);

  int v1 = analogRead(SIG_MUX);
  int v2 = analogRead(SIG_MUX);

  return (v1 + v2) / 2;
}

// ===================== SETUP =====================

void setup() {

  Serial.begin(115200);

  Wire.begin(SDA_BNO, SCL_BNO);
  Wire.setClock(400000);

  delay(100);

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

      Serial.println("MPU6050 no detectado");

      while (1);
    }

    lastTime = millis();
    Serial.println("MPU6050 listo");
  }

  // ================= VL53 #1 =================

// ================= VL53 =================

if (usarDistancia) {

  // ================= VL53 #1 =================

  tcaSelect(1);

  if (!vl53_1.init()) {

    Serial.println("VL53 #1 no detectado");

    while (1);
  }

  vl53_1.setTimeout(50);
  vl53_1.startContinuous();

  Serial.println("VL53 #1 listo");

  // ================= VL53 #2 =================

  tcaSelect(2);

  if (!vl53_2.init()) {

    Serial.println("VL53 #2 no detectado");

    while (1);
  }

  vl53_2.setTimeout(50);
  vl53_2.startContinuous();

  Serial.println("VL53 #2 listo");

  // ================= VL53 #3 =================

  tcaSelect(3);

  if (!vl53_3.init()) {

    Serial.println("VL53 #3 no detectado");

    while (1);
  }

  vl53_3.setTimeout(50);
  vl53_3.startContinuous();

  Serial.println("VL53 #3 listo");
}

  // ================= PINES =================

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);

  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);

  pinMode(PELOTA, INPUT);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  pinMode(SIG_MUX, INPUT);

  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);
  pinMode(SW3, INPUT);

  // ================= PWM =================

  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENC, PWM_FREQ, PWM_RESOLUTION);

  stopMotorA();
  stopMotorB();
  stopMotorC();
  stopMotorD();

  Serial.println("Listo");
}

// ===================== LOOP =====================

void loop() {

  // ================= SERIAL =================

  while (Serial.available()) {

    char c = Serial.read();

    if (c == '\n' || c == '\r') {

      comando.trim();

      if (comando.length() == 0) {

        comando = "";
        break;
      }
      Serial.println(comando);

      char motor = comando.charAt(0);

      int pwm = 0;

      if (comando.length() > 1) {

        pwm = constrain(comando.substring(1).toInt(), -255, 255);
      }

      switch (motor) {

        case 'A':

          if (pwm > 0)
            motorA_forward(pwm);

          else if (pwm < 0)
            motorA_backward(abs(pwm));

          else
            stopMotorA();

          break;

        case 'B':

          if (pwm > 0)
            motorB_forward(pwm);

          else if (pwm < 0)
            motorB_backward(abs(pwm));

          else
            stopMotorB();

          break;

        case 'C':

          if (pwm > 0)
            motorC_forward(pwm);

          else if (pwm < 0)
            motorC_backward(abs(pwm));

          else
            stopMotorC();

          break;

        case 'D':

          if (pwm > 100)
            cilindro_on();

          else if (pwm < -100)
            pateador_on();

          else if (pwm > 0)
            ambos_on();

          else
            stopMotorD();

          break;

        case 'E':

          modoDebug = true;
          break;

        case 'F':

          modoDebug = false;
          break;

        case 'S':

          stopMotorA();
          stopMotorB();
          stopMotorC();
          stopMotorD();

          break;
      }

      comando = "";
    }

    else {

      comando += c;
    }
  }

  // ================= DEBUG =================

  if (modoDebug && millis() - lastDebug >= 50) {

    lastDebug = millis();

    // ===== YAW =====

    if (usarBNO080) {

      if (bno080.dataAvailable()) {

        yaw = bno080.getYaw() * 180.0 / PI;
      }
    }

    else{
      int16_t gx, gy, gz;

      mpu.getRotation(&gx, &gy, &gz);

      unsigned long currentTime = micros();

      float dt = (currentTime - lastTime) / 1000000.0;

      lastTime = currentTime;

      float wz = (gz / 131.0) * PI / 180.0;

      yaw += wz * dt;

      yaw = atan2(sin(yaw), cos(yaw));
    }

    // ===== VL53 =====

int dist1 = -1;
int dist2 = -1;
int dist3 = -1;

if (usarDistancia) {

  tcaSelect(1);

  dist1 = vl53_1.readRangeContinuousMillimeters();

  if (vl53_1.timeoutOccurred())
    dist1 = -1;

  tcaSelect(2);

  dist2 = vl53_2.readRangeContinuousMillimeters();

  if (vl53_2.timeoutOccurred())
    dist2 = -1;

  tcaSelect(3);

  dist3 = vl53_3.readRangeContinuousMillimeters();

  if (vl53_3.timeoutOccurred())
    dist3 = -1;
}

    // ===== MUX =====

    int s1 = leerMux(0);
    int s2 = leerMux(1);
    int s3 = leerMux(2);

    int sw1 = digitalRead(SW1);
    int sw2 = digitalRead(SW2);
    int sw3 = digitalRead(SW3);

    // ===== PRINT =====

    Serial.print("Yaw: ");
    Serial.print(yaw);

    Serial.print(" | D1: ");
    Serial.print(dist1);

    Serial.print(" | D2: ");
    Serial.print(dist2);

    Serial.print(" | D3: ");
    Serial.print(dist3);

    Serial.print(" | S1: ");
    Serial.print(s1);

    Serial.print(" | S2: ");
    Serial.print(s2);

    Serial.print(" | S3: ");
    Serial.print(s3);

    Serial.print(" | SW1: ");
    Serial.print(sw1);

    Serial.print(" | SW2: ");
    Serial.print(sw2);

    Serial.print(" | SW3: ");
    Serial.println(sw3);
  }
}

// ===================== FUNCIONES =====================

void stopMotorA() {

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  ledcWrite(ENA, 0);
}

void stopMotorB() {

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  ledcWrite(ENB, 0);
}

void stopMotorC() {

  digitalWrite(IN5, LOW);
  digitalWrite(IN6, LOW);

  ledcWrite(ENC, 0);
}

void stopMotorD() {

  digitalWrite(IN7, LOW);
  digitalWrite(IN8, LOW);
}

void motorA_forward(int p) {

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  ledcWrite(ENA, p);
}

void motorA_backward(int p) {

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  ledcWrite(ENA, p);
}

void motorB_forward(int p) {

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  ledcWrite(ENB, p);
}

void motorB_backward(int p) {

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  ledcWrite(ENB, p);
}

void motorC_forward(int p) {

  digitalWrite(IN5, HIGH);
  digitalWrite(IN6, LOW);

  ledcWrite(ENC, p);
}

void motorC_backward(int p) {

  digitalWrite(IN5, LOW);
  digitalWrite(IN6, HIGH);

  ledcWrite(ENC, p);
}

void cilindro_on() {

  digitalWrite(IN7, HIGH);
  digitalWrite(IN8, LOW);
}

void pateador_on() {

  digitalWrite(IN7, LOW);
  digitalWrite(IN8, HIGH);
}

void ambos_on() {

  digitalWrite(IN7, HIGH);
  digitalWrite(IN8, HIGH);
}
