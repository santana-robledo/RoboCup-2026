#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include <MPU6050.h>
#include <VL53L0X.h>

#define MUX1 0x70
#define MUX2 0x74

#define NUM_SENSORES 11

VL53L0X sensores[NUM_SENSORES];

const uint8_t muxSensor[NUM_SENSORES] = {
  MUX1, MUX1, MUX1, MUX1, MUX1,
  MUX2, MUX2, MUX2, MUX2, MUX2, MUX2
};

const uint8_t canalSensor[NUM_SENSORES] = {
  0, 1, 2, 3, 4,
  0, 1, 2, 3, 4, 5
};

// ------------------------
bool usarBNO080 = false;
bool usarDistancia = true;
// ------------------------

BNO080 bno080;
MPU6050 mpu;

bool modoDebug = false;
bool modoLecturaIR = false;
bool modoLecturaDistancia = false;

float yaw = 0.0;
String comando = "";

unsigned long lastTime = 0;
unsigned long lastDebug = 0;
unsigned long lastIR = 0;
unsigned long lastDistancia = 0;

// ===== Multiplexor analógico =====
#define SIG_MUX 34
#define S0 25
#define S1 26
#define S2 27
#define S3 12

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

// ================= SWITCHES =================
#define SW1 39
#define SW2 32
#define SW3 35

// ================= PWM =================
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

// ================= PROTOTIPOS =================
void disableAll();
void tcaSelect(uint8_t mux, uint8_t canal);
void seleccionarCanal(int canal);
int leerSensor(int canal);

void stopMotorA();
void stopMotorB();
void stopMotorC();
void stopMotorD();

void motorA_forward(int p);
void motorA_backward(int p);
void motorB_forward(int p);
void motorB_backward(int p);
void motorC_forward(int p);
void motorC_backward(int p);

void cilindro_on();
void pateador_on();
void ambos_on();

void initVL53L0X();
void initIMU();

void imprimirIR();
void imprimirDistancias();

// =====================================================
void disableAll() {
  Wire.beginTransmission(MUX1);
  Wire.write(0);
  Wire.endTransmission();

  Wire.beginTransmission(MUX2);
  Wire.write(0);
  Wire.endTransmission();

  delay(2);
}

void tcaSelect(uint8_t mux, uint8_t canal) {
  disableAll();

  Wire.beginTransmission(mux);
  Wire.write(1 << canal);
  Wire.endTransmission();

  delay(2);
}

void seleccionarCanal(int canal) {
  digitalWrite(S0, canal & 0x01);
  digitalWrite(S1, (canal >> 1) & 0x01);
  digitalWrite(S2, (canal >> 2) & 0x01);
  digitalWrite(S3, (canal >> 3) & 0x01);
}

int leerSensor(int canal) {
  seleccionarCanal(canal);
  delayMicroseconds(20);
  return analogRead(SIG_MUX);
}

void initVL53L0X() {
  if (!usarDistancia) return;

  Serial.println("Inicializando 11 sensores VL53L0X...");

  disableAll();
  delay(20);

  for (int i = 0; i < NUM_SENSORES; i++) {
    tcaSelect(muxSensor[i], canalSensor[i]);

    if (sensores[i].init()) {
      sensores[i].setTimeout(200);
      sensores[i].startContinuous();

      Serial.print("Sensor ");
      Serial.print(i + 1);
      Serial.println(" OK");
    } else {
      Serial.print("Sensor ");
      Serial.print(i + 1);
      Serial.println(" ERROR");
    }

    delay(10);
  }

  disableAll();
  Serial.println("Inicializacion terminada.");
}

void initIMU() {
  disableAll();
  delay(20);

  if (usarBNO080) {
    Serial.println("Iniciando BNO080...");

    if (!bno080.begin()) {
      Serial.println("BNO080 no detectado");
      while (1);
    }

    bno080.enableRotationVector(50);
    Serial.println("BNO080 listo");
  } else {
    Serial.println("Iniciando MPU6050...");

    mpu.initialize();

    if (!mpu.testConnection()) {
      Serial.println("MPU6050 no detectado");
      while (1);
    }

    lastTime = micros();
    Serial.println("MPU6050 listo");
  }
}

void imprimirIR() {
  int sensoresIR[12];

  for (int i = 0; i < 12; i++) {
    sensoresIR[i] = leerSensor(i);
  }

  for (int i = 0; i < 12; i++) {
    Serial.print("S");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(sensoresIR[i]);

    if (i < 11) Serial.print("   ");
  }

  Serial.println();
}

void imprimirDistancias() {
  for (int i = 0; i < NUM_SENSORES; i++) {
    tcaSelect(muxSensor[i], canalSensor[i]);

    uint16_t distancia = sensores[i].readRangeContinuousMillimeters();

    Serial.print("S");
    Serial.print(i + 1);
    Serial.print(": ");

    if (sensores[i].timeoutOccurred()) {
      Serial.print("ERR");
    } else {
      Serial.print(distancia);
      Serial.print(" mm");
    }

    if (i < NUM_SENSORES - 1) Serial.print(" | ");
  }

  Serial.println();
  disableAll();
}

void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);
  Wire.setClock(100000);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  pinMode(SIG_MUX, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);

  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);

  pinMode(PELOTA, INPUT);

  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);
  pinMode(SW3, INPUT);

  delay(1000);

  initVL53L0X();
  initIMU();

  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENC, PWM_FREQ, PWM_RESOLUTION);

  stopMotorA();
  stopMotorB();
  stopMotorC();
  stopMotorD();

  Serial.println("Listo");
}

void loop() {
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
          if (pwm > 0) motorA_forward(pwm);
          else if (pwm < 0) motorA_backward(abs(pwm));
          else stopMotorA();
          break;

        case 'B':
          if (pwm > 0) motorB_forward(pwm);
          else if (pwm < 0) motorB_backward(abs(pwm));
          else stopMotorB();
          break;

        case 'C':
          if (pwm > 0) motorC_forward(pwm);
          else if (pwm < 0) motorC_backward(abs(pwm));
          else stopMotorC();
          break;

        case 'D':
          if (pwm > 100) cilindro_on();
          else if (pwm < -100) pateador_on();
          else if (pwm > 0) ambos_on();
          else stopMotorD();
          break;

        case 'E':
          modoDebug = true;
          modoLecturaIR = false;
          modoLecturaDistancia = false;
          break;

        case 'F':
          modoDebug = false;
          modoLecturaIR = false;
          modoLecturaDistancia = false;
          break;

        case 'S':
          stopMotorA();
          stopMotorB();
          stopMotorC();
          stopMotorD();
          break;

        case 'L':
          modoLecturaDistancia = true;
          modoLecturaIR = false;
          modoDebug = false;
          break;

        case 'I':
          modoLecturaIR = true;
          modoLecturaDistancia = false;
          modoDebug = false;
          break;

        default:
          Serial.println("Comando no reconocido");
          break;
      }

      comando = "";
    } else {
      comando += c;
    }
  }

  if (modoDebug && millis() - lastDebug >= 50) {
    lastDebug = millis();

    if (usarBNO080) {
      if (bno080.dataAvailable()) {
        yaw = bno080.getYaw() * 180.0 / PI;
      }
    } else {
      int16_t gx, gy, gz;
      mpu.getRotation(&gx, &gy, &gz);

      unsigned long currentTime = micros();
      float dt = (currentTime - lastTime) / 1000000.0;
      lastTime = currentTime;

      float wz = (gz / 131.0) * PI / 180.0;
      yaw += wz * dt;
      yaw = atan2(sin(yaw), cos(yaw));
    }

    int s1 = leerSensor(0);
    int s2 = leerSensor(1);
    int s3 = leerSensor(2);

    int sw1 = digitalRead(SW1);
    int sw2 = digitalRead(SW2);
    int sw3 = digitalRead(SW3);
    int pelota = digitalRead(PELOTA);

    Serial.print("Yaw: ");
    Serial.print(yaw);

    Serial.print(" | S1: ");
    Serial.print(s1);

    Serial.print(" | S2: ");
    Serial.print(s2);

    Serial.print(" | S3: ");
    Serial.print(s3);

    Serial.print(" | Pelota: ");
    Serial.print(pelota);

    Serial.print(" | SW1: ");
    Serial.print(sw1);

    Serial.print(" | SW2: ");
    Serial.print(sw2);

    Serial.print(" | SW3: ");
    Serial.println(sw3);
  }

  if (modoLecturaIR && millis() - lastIR >= 100) {
    lastIR = millis();
    imprimirIR();
  }

  if (modoLecturaDistancia && millis() - lastDistancia >= 100) {
    lastDistancia = millis();
    imprimirDistancias();
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