#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h" 

BNO080 myIMU;

#define PIN_INT 2
#define PIN_RST 4
#define IMU_ADDR 0x4B // o 0x4B

void setup() {
  Serial.begin(115200); 
  while(!Serial);

  Serial.println("Iniciando sistema anti-congelamiento...");

  Wire.begin();
  Wire.setClock(400000); 

  // --- LA SOLUCIÓN AL CONGELAMIENTO ---
  // Esto evita que el Arduino se quede pegado si el sensor falla un instante
  Wire.setWireTimeout(3000, true); 
  // ------------------------------------

  // Iniciar sensor
  if (myIMU.begin(IMU_ADDR, Wire, PIN_INT) == false) {
    Serial.println("Error de conexión con BNO080.");
    while(1);
  }

  myIMU.enableRotationVector(50); 
  Serial.println("Sensor listo. Mueve el sensor.");
  
  pinMode(13, OUTPUT); // Usaremos el LED 13 como testigo
}

void loop() {
  // Verificamos datos
  if (myIMU.dataAvailable() == true) {

    digitalWrite(13, !digitalRead(13)); 

    float yaw = myIMU.getYaw();
    float anguloZ = (yaw * 180.0) / PI; 
    
    if (anguloZ < 0) anguloZ += 360.0;

    Serial.print("Z: ");
    Serial.println(yaw, 1);
  }
  
  // Pequeña pausa para no saturar el puerto serie si va muy rápido
  // (Aunque dataAvailable ya regula esto, ayuda a la estabilidad)
  //delay(1); 
}
