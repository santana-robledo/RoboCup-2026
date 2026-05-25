#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

BNO080 bno080;

bool modoDebug = false;

float yaw = 0;

String comando = "";

//////////////// BNO080 //////////////////

#define SDA_BNO 21
#define SCL_BNO 22

////////////// PUENTE H 1 //////////////

// ===== MOTOR A =====
#define ENA 14
#define IN1 13
#define IN2 15

// ===== MOTOR B =====
#define ENB 16
#define IN3 4
#define IN4 2

////////////// PUENTE H 2 //////////////

// ===== MOTOR C =====
#define ENC 18
#define IN5 5
#define IN6 17

// ===== CILINDRO =====
#define IN7 19
#define IN8 23

////////////// SENSOR PELOTA //////////////

#define PELOTA 33

////////////// MULTIPLEXOR //////////////

// SIG
#define SIG_MUX 34

// SELECT
#define S0 12
#define S1 27
#define S2 26

// ENABLE
#define MUX_EN 25

////////////// SWITCHES //////////////

#define SW1 39
#define SW2 32
#define SW3 35

////////////// PWM ESP32 //////////////

#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

//////////////////////////////////////////////////////////

void setup() {

  Serial.begin(115200);

  ////////////////// I2C //////////////////

  Wire.begin(SDA_BNO, SCL_BNO);

  delay(100);

  ////////////////// BNO080 //////////////////

  Serial.println("Iniciando BNO080...");

  if (!bno080.begin()) {

    Serial.println("BNO080 no detectado");

    while (1);
  }

  // Rotation Vector
  bno080.enableRotationVector(50);

  Serial.println("BNO080 listo");

  ////////////////// PINES //////////////////

  // MOTOR A
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // MOTOR B
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // MOTOR C
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);

  // CILINDRO
  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);

  // PELOTA
  pinMode(PELOTA, INPUT);

  // MULTIPLEXOR
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);

  pinMode(SIG_MUX, INPUT);

  pinMode(MUX_EN, OUTPUT);

  digitalWrite(MUX_EN, LOW);

  // SWITCHES
  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);
  pinMode(SW3, INPUT);

  ////////////////// PWM //////////////////

  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENC, PWM_FREQ, PWM_RESOLUTION);

  ////////////////// ESTADO INICIAL //////////////////

  stopMotorA();
  stopMotorB();
  stopMotorC();
  stopMotorD();

  Serial.println("Listo");
}

//////////////////////////////////////////////////////////

void loop() {

  while (Serial.available()) {

    char c = Serial.read();

    if (c == '\n') {

      comando.trim();

      if (comando.length() == 0)
        return;

      Serial.println(comando);

      char motor = comando.charAt(0);

      int pwm = 0;

      if (comando.length() > 1) {

        pwm = comando.substring(1).toInt();

        pwm = constrain(pwm, -255, 255);
      }

      switch (motor) {

        ////////////////// MOTOR A //////////////////

        case 'A':

          if (pwm > 0)
            motorA_forward(pwm);

          else if (pwm < 0)
            motorA_backward(abs(pwm));

          else
            stopMotorA();

          break;

        ////////////////// MOTOR B //////////////////

        case 'B':

          if (pwm > 0)
            motorB_forward(pwm);

          else if (pwm < 0)
            motorB_backward(abs(pwm));

          else
            stopMotorB();

          break;

        ////////////////// MOTOR C //////////////////

        case 'C':

          if (pwm > 0)
            motorC_forward(pwm);

          else if (pwm < 0)
            motorC_backward(abs(pwm));

          else
            stopMotorC();

          break;

        ////////////////// MOTOR D //////////////////

        case 'D':

          // D200  -> cilindro
          if (pwm > 100) {

            cilindro_on();
          }

          // D-200 -> pateador
          else if (pwm < -100) {

            pateador_on();
          }

          // D50 -> ambos
          else if (pwm > 0 && pwm <= 100) {

            ambos_on();
          }

          // D0 -> apagar
          else {

            stopMotorD();
          }

          break;

        ////////////////// STOP //////////////////

        case 'S':

          stopMotorA();
          stopMotorB();
          stopMotorC();
          stopMotorD();

          break;

        ////////////////// DEBUG //////////////////

        case 'E':

          modoDebug = true;
          break;

        case 'F':

          modoDebug = false;
          break;

        ////////////////// DEFAULT //////////////////

        default:

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

  ////////////////// DEBUG //////////////////

  if (modoDebug) {
    int pelota = digitalRead(PELOTA);
    int sw1 = digitalRead(SW1);
    int sw2 = digitalRead(SW2);
    int sw3 = digitalRead(SW3);

    ////////////////// BNO080 //////////////////

    if (bno080.dataAvailable()) {

      yaw = bno080.getYaw() * 180.0 / PI;
    }

    int s1, s2, s3, s4, s5;

    // SENSOR 1
    seleccionarCanal(0);
    delayMicroseconds(20);
    s1 = analogRead(SIG_MUX);

    // SENSOR 2
    seleccionarCanal(1);
    delayMicroseconds(20);
    s2 = analogRead(SIG_MUX);

    // SENSOR 3
    seleccionarCanal(2);
    delayMicroseconds(20);
    s3 = analogRead(SIG_MUX);

    // SENSOR 4
    seleccionarCanal(3);
    delayMicroseconds(20);
    s4 = analogRead(SIG_MUX);

    // SENSOR 5
    seleccionarCanal(4);
    delayMicroseconds(20);
    s5 = analogRead(SIG_MUX);

    ////////////////// IMPRESION //////////////////

    Serial.print("Yaw: ");
    Serial.print(yaw);

    Serial.print(" | Pelota: ");
    Serial.print(pelota);

    Serial.print(" | SW1: ");
    Serial.print(sw1);

    Serial.print(" | SW2: ");
    Serial.print(sw2);

    Serial.print(" | SW3: ");
    Serial.print(sw3);

    Serial.print(" | S1: ");
    Serial.print(s1);

    Serial.print(" | S2: ");
    Serial.print(s2);

    Serial.print(" | S3: ");
    Serial.print(s3);

    Serial.print(" | S4: ");
    Serial.print(s4);

    Serial.print(" | S5: ");
    Serial.println(s5);

    delay(100);
  }
}

void seleccionarCanal(int canal) {

  digitalWrite(S0, canal & 0x01);
  digitalWrite(S1, (canal >> 1) & 0x01);
  digitalWrite(S2, (canal >> 2) & 0x01);
}

//////////////// STOP //////////////////

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

void stopMotorC() {

  digitalWrite(IN5, LOW);
  digitalWrite(IN6, LOW);

  analogWrite(ENC, 0);
}

//////////////// MOTOR A //////////////////

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

//////////////// MOTOR B //////////////////

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

//////////////// MOTOR C //////////////////

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

//////////////// MOTOR D //////////////////

void stopMotorD() {

  digitalWrite(IN7, LOW);
  digitalWrite(IN8, LOW);
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
