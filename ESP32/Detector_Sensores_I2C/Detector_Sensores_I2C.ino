#include <Wire.h>

#define MUX1 0x70
#define MUX2 0x74

void disableAll(uint8_t mux) {
  Wire.beginTransmission(mux);
  Wire.write(0);
  Wire.endTransmission();
}

void tcaSelect(uint8_t mux, uint8_t channel) {
  disableAll(MUX1);
  disableAll(MUX2);

  Wire.beginTransmission(mux);
  Wire.write(1 << channel);
  Wire.endTransmission();

  delay(5);
}

bool sensorExiste() {
  Wire.beginTransmission(0x29);
  return (Wire.endTransmission() == 0);
}

void scanMux(uint8_t mux, const char* nombre) {

  Serial.println();
  Serial.print("Escaneando ");
  Serial.println(nombre);

  for (int ch = 0; ch < 8; ch++) {

    tcaSelect(mux, ch);

    Serial.print(nombre);
    Serial.print(" Canal ");
    Serial.print(ch);

    if (sensorExiste())
      Serial.println(" -> VL53L0X detectado");
    else
      Serial.println(" -> VACIO");
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  delay(1000);

  scanMux(MUX1, "MUX1");
  scanMux(MUX2, "MUX2");
}

void loop() {
}