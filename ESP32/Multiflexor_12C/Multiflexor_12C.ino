#include <Wire.h>
#include <VL53L0X.h>

#define MUX1 0x70
#define MUX2 0x74

#define NUM_SENSORES 11

#define IN1   15
#define IN2   13

VL53L0X sensores[NUM_SENSORES];

// Sensores 1-5 en MUX1 canales 0-4
// Sensores 6-11 en MUX2 canales 0-5
const uint8_t muxSensor[NUM_SENSORES] = {
  MUX1, MUX1, MUX1, MUX1, MUX1,
  MUX2, MUX2, MUX2, MUX2, MUX2, MUX2
};

const uint8_t canalSensor[NUM_SENSORES] = {
  0, 1, 2, 3, 4,
  0, 1, 2, 3, 4, 5
};

void disableAll() {

  Wire.beginTransmission(MUX1);
  Wire.write(0);
  Wire.endTransmission();

  Wire.beginTransmission(MUX2);
  Wire.write(0);
  Wire.endTransmission();
}

void tcaSelect(uint8_t mux, uint8_t canal) {

  disableAll();

  Wire.beginTransmission(mux);
  Wire.write(1 << canal);
  Wire.endTransmission();

  delay(2);
}

void setup() {

  Serial.begin(115200);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  Wire.begin(21, 22);
  Wire.setClock(100000);

  delay(1000);

  Serial.println();
  Serial.println("Inicializando 11 sensores VL53L0X...");

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
  }

  Serial.println("Inicializacion terminada.");
}

void loop() {

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

    if (i < NUM_SENSORES - 1)
      Serial.print(" | ");
  }

  Serial.println();

  delay(100);
}