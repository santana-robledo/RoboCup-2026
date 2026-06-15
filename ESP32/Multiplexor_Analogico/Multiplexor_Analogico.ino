#define S0 25
#define S1 26
#define S2 27
#define S3 12
#define SIG 34

#define IN1   15
#define IN2   13

void setup() {

  Serial.begin(115200);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
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

  return analogRead(SIG);
}

void loop() {

  int sensores[12];

  for (int i = 0; i < 12; i++) {
    sensores[i] = leerSensor(i);
  }

  for (int i = 0; i < 12; i++) {
    Serial.print("S");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(sensores[i]);

    if (i < 11)
      Serial.print("   ");
  }

  Serial.println();

  delay(100);
}