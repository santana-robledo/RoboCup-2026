/*
Lectura analogica con filtro de Kalman
*/
float alpha=0.5;
int pin=A0;
float valorfiltrado=0;
String color = "";

//Umbrales
int NEGRO_MAX=300;
int VERDE_MAX=600;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  valorfiltrado = analogRead(pin);
}

void loop() {
  int sensorValue = analogRead(pin);
  valorfiltrado= valorfiltrado + alpha* (sensorValue - valorfiltrado);

if (valorfiltrado >= 0 && valorfiltrado <= NEGRO_MAX) {
    color = "NEGRO";
  } 
  else if (valorfiltrado > NEGRO_MAX && valorfiltrado <= VERDE_MAX) {
    color = "VERDE";
  } 
  else {
    color = "BLANCO";
  }
}

  Serial.print("Valor: ");
  Serial.print(sensorValue);
  Serial.print(" | Filtrado: ");
  Serial.print(valorfiltrado);
  Serial.print(" Color: ");
  Serial.println(color);
  delay(1); 
}
