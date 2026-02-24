
// ===== MOTOR A Right =====
#define IN1 23 // Morado Derecho
#define IN2 22 // Azul Derecho
#define ENA 5  // Gris Derecho

// ===== MOTOR B Left =====
#define IN3 24 // Morado Izquierdo IN1
#define IN4 25 // Gris Izquierdo IN2    
#define ENB 6  // Azul Izquierdo

// ===== MOTOR C =====
#define IN5 26 // Verde Derecho IN3
#define IN6 27 // Amarillo Derecho IN4
#define ENC 7  // Naranja Derecho

// ===== MOTOR Cilindro =====
#define IN7 28 // 
#define IN8 29 // 
#define END 9  // 

// ===== Pateador =====
#define IN9 30 // 
#define IN10 31 // 
#define ENE 8  //

float wa = 0.0;
float wb = 0.0;
float wc = 0.0;

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

  pinMode(IN9, OUTPUT);
  pinMode(IN10, OUTPUT);
  pinMode(ENE, OUTPUT);

  stopMotorA();
  stopMotorB();
  stopMotorC();
  CilindroOFF();
  PateadorOff();

  Serial.println("Listo. Manda comando");

}

void loop() {
  /*
  if (analogRead(A0) > 120) {
    Serial.print("BLANCO A0");
  } else {
    Serial.print("NEGRO A0");
  }

   if (analogRead(A1) > 120) {
    Serial.print("BLANCO A1");
  } else {
    Serial.print("NEGRO A1");
  }

   if (analogRead(A2) > 120) {
    Serial.print("BLANCOA2");
  } else {
    Serial.print("NEGRO A2");
  }

   if (analogRead(A3) > 120) {
    Serial.print("BLANCO A3");
  } else {
    Serial.print("NEGRO A3");
  }

   if (analogRead(A4) > 120) {
    Serial.print("BLANCO A4");
  } else {
    Serial.print("NEGRO A4");
  }

   if (analogRead(A5) > 120) {
    Serial.print("BLANCO A5");
  } else {
    Serial.print("NEGRO A5");
  }
*/
  if (Serial.available()) {

    String comando = Serial.readStringUntil('\n');
    comando.trim();
    Serial.println(comando);

    if (comando.length() == 0) return;

    char motor = comando.charAt(0);
    int pwm = 0;

    if (comando.length() > 1) {
      pwm = comando.substring(1).toInt();
      pwm = constrain(pwm, -255, 255);
    }

    switch(motor){

      case 'A':   // Motor A
        if(pwm > 0) motorA_forward(pwm);
        else if(pwm < 0) motorA_backward(abs(pwm));
        else stopMotorA();
        break;

      case 'B':   // Motor B
        if(pwm > 0) motorB_forward(pwm);
        else if(pwm < 0) motorB_backward(abs(pwm));
        else stopMotorB();
        break;

      case 'C':   // Motor C
        if(pwm > 0) motorC_forward(pwm);
        else if(pwm < 0) motorC_backward(abs(pwm));
        else stopMotorC();
        break;

      case 'S':   // STOP todo
        stopMotorA();
        stopMotorB();
        stopMotorC();
        break;
      
      case 'Q':
        moverFrente(pwm);
        break;
      
      case 'W':
        moverAtras(pwm);
        break;

      case 'E':
        GirarDerecha(pwm);
        break;

      case 'R':
        GirarIzquierda(pwm);
        break;

      case 'T':
        if(pwm > 0) CilindroON(pwm);
        else if(pwm < 0) CilindroON(abs(pwm));
        else CilindroOFF();
        break;

      case 'Y':
        if(pwm > 0) PateadorON(pwm);
        else if(pwm < 0) PateadorON(abs(pwm));
        else PateadorOff();
        break;

      default:
        stopMotorA();
        stopMotorB();
        stopMotorC();
        break;
    }
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
  delay(2000);
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

void PateadorON(int pwm){
  pwm = constrain(pwm, 0, 255);
  digitalWrite(IN9, LOW);
  digitalWrite(IN10, HIGH);
  analogWrite(ENE, pwm);
  
}

void PateadorOff(){ 
  digitalWrite(IN9, LOW);
  digitalWrite(IN10, LOW);
  analogWrite(ENE, 0);
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
