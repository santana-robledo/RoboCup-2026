#include "I2Cdev.h" //Para comunicación 12C
#include "SparkFun_BNO080_Arduino_Library.h" //Para IMU

/* 
0.0,0.0,0.0,3.0,0.0
 */
 //Constantes de velocidad máxima, RPM a radianes
const float MAX_RAD_A = 130 * 2 * PI / 60; //13.61 
const float MAX_RAD_B = 170 * 2 * PI / 60; //17.80 rad/s
const float MAX_RAD_C = 170 * 2 * PI / 60; //17.80 rad/s


BNO080 myIMU; //Objeto IMU
#define PIN_INT 2 //Pin de interrupción
#define PIN_RST 4 //Pin de reset
#define IMU_ADDR 0x4B // o 0x4B

//Variables de orientación
float rawTheta = 0.0;
float theta = 0.0;     // ángulo acumulado en radianes
float wz = 0.0;        // velocidad angular en rad/s
float bias = 0.0;      // bias del giroscopio
float dt = 0.0;        //tiempo

unsigned long lastTime = 0;

float yawAngle = 0;

//Variables del robot
float L = 0.09, R = 0.029; //Distancia centro-rueda, radio de la rueda
//Velocidades del robot
float Ux = 0.0, Uy = 0.0, Ut = 0.0;
float v1 = 0.0, v2 = 0.0, v3 = 0.0;
int pwm_a = 0, pwm_b = 0, pwm_c = 0;

float alpha = 0.3;
static float theta_f = 0;
unsigned long now = 0;
float Ut_pid = 0.0; //salida del PID
float theta_deg=0.0; //ángulo actual en grados

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
#define END 3  // 

// ===== Pateador =====
#define IN9 30 // 
#define IN10 31 // 
#define ENE 8  //

float wa = 0.0, wb = 0.0, wc = 0.0;
/////// Ganancias PID para orientación /////
float Kp = 20;
float Ki = 5;
float Kd = 0.2;

float setpoint = 0.0; //Angulo al que queremos apuntar

//Variables internas del PID
float error = 0.0;
float error_prev = 0.0;
float error_int = 0.0;
float error_der = 0.0;

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.setTimeout(20);

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
  
  CilindroOFF();
  PateadorOff();
  stopMotorA(); 
  stopMotorB(); 
  stopMotorC();

  //Inicialización I2C
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);//400 kHz

  if (myIMU.begin(IMU_ADDR, Wire, PIN_INT) == false) { //Inicializar IMU
    Serial.println("Error de conexión con BNO080.");
    while(1);
  }
  myIMU.enableRotationVector(50);//Activar medición, lee cada 50 ms

 
  lastTime = micros();
}

void loop() {
  unsigned long currentTime = micros(); //tiempo actual
  dt = (currentTime - lastTime) / 1000000.0; //Calcula cuánto tiempo pasó desde la última iteración en segundos
  lastTime = currentTime; //Actualiza tiempo
  if (dt <= 0) dt = 0.0001; //Evitar división entre 0

  if (myIMU.dataAvailable() == true)
  {
    theta_f = myIMU.getYaw();//Obtiene el angulo
  }

  if (isnan(error_int)) error_int = 0; //Protege variable de ser corrompida
  if (isnan(error)) error = 0; //Protege variable de ser corrompida
  error = setpoint - theta_f; //Calculamos error en angulo para PID en radianes
  error = atan2(sin(error), cos(error)); //Normalizar error angular

  error_int += error * dt; //Acumula error
  error_int = constrain(error_int, -2.0, 2.0);//limitamos error (π/180)

  Ut_pid = Kp*error + Ki*error_int ; // Calcula velocidad angular deseada.
  Ut_pid = constrain(Ut_pid, -4.0, 4.0);
  
  // =================== Lectura serial ===================
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    char inputBuffer[50];
    input.toCharArray(inputBuffer, 50);

    char *token = strtok(inputBuffer, ",");
    if(token != NULL) Ux = atof(token);
    token = strtok(NULL, ",");
    if(token != NULL) Uy = atof(token);
    token = strtok(NULL, ",");
    if(token != NULL) Ut = atof(token);
    token = strtok(NULL, ",");
    if(token != NULL) Kp = atof(token);
    token = strtok(NULL, ",");
    if(token != NULL) Ki = atof(token);
  }

  // =================== Cinemática ===================

  //Ut_pid=Ut; //Solo para pruebas de cinematica
  //theta_f=0; //Solo para pruebas de cinematica
  v1 = (sin(theta_f) * Ux) - (cos(theta_f) * Uy) + (L * Ut_pid);
  v2 = (cos(theta_f + PI/6) * Ux) + (sin(theta_f + PI/6) * Uy) + (L * Ut_pid);
  v3 = (-sin(theta_f + PI/3) * Ux) + (cos(theta_f + PI/3) * Uy) + (L * Ut_pid);

  //Convertimos a velocidad angular
  //v=ωR Entonces ω=v/R
  wa = v1 / R;
  wb = v2 / R;
  wc = v3 / R;

  /*
  Serial.print("Ut_pid: "); Serial.println(Ut_pid);Serial.print("error: "); Serial.println(error);
  Serial.print("dt: "); Serial.println(dt);Serial.print(" wa: "); Serial.print(wa);Serial.print(" wb: "); Serial.print(wb);Serial.print(" wc: "); Serial.println(wc);
  */

  //Convertir a PWM
  pwm_a = mapPWM(wa, MAX_RAD_A);
  pwm_b = mapPWM(wb, MAX_RAD_B);
  pwm_c = mapPWM(wc, MAX_RAD_C);

  // =================== Motor control ===================
  if (wa > 0) motorA_forward(pwm_a);
  else if (wa < 0) motorA_backward(pwm_a);
  else stopMotorA();

  if (wb > 0) motorB_forward(pwm_b);
  else if (wb < 0) motorB_backward(pwm_b);
  else stopMotorB();

  if (wc > 0) motorC_forward(pwm_c);
  else if (wc < 0) motorC_backward(pwm_c);
  else stopMotorC();

  // =================== Debug ===================
  Serial.print("Theta(rad): "); Serial.print(theta_f);
  //Serial.print(" Ux:"); Serial.print(Ux);
  //Serial.print(" Uy:"); Serial.print(Uy);
  //Serial.print(" Ut:"); Serial.print(Ut);
  //Serial.print(" Kp: "); Serial.print(Kp);
  //Serial.print(" Ki: "); Serial.print(Ki);
  Serial.print(" Pwm_a: "); Serial.print(pwm_a);
  Serial.print(" Pwm_b: "); Serial.print(pwm_b);
  Serial.print(" Pwm_c: "); Serial.println(pwm_c);
  
}

void stopMotorA(){
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, LOW); 
  analogWrite(ENA,0);}
  
void motorA_forward(int PWM){
  digitalWrite(IN1,HIGH); 
  digitalWrite(IN2,LOW); 
  analogWrite(ENA,PWM);}
  
void motorA_backward(int PWM){
  digitalWrite(IN1,LOW); 
  digitalWrite(IN2,HIGH); 
  analogWrite(ENA,PWM);}
  
void stopMotorB(){
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, LOW); 
  analogWrite(ENB,0);}
  
void motorB_forward(int PWM){
  digitalWrite(IN3,HIGH); 
  digitalWrite(IN4,LOW); 
  analogWrite(ENB,PWM);}
  
void motorB_backward(int PWM){
  digitalWrite(IN3,LOW); 
  digitalWrite(IN4,HIGH); 
  analogWrite(ENB,PWM);}
  
void stopMotorC(){
  digitalWrite(IN5, LOW); 
  digitalWrite(IN6, LOW); 
  analogWrite(ENC,0);}
  
void motorC_forward(int PWM){
  digitalWrite(IN5,HIGH); 
  digitalWrite(IN6,LOW); 
  analogWrite(ENC,PWM);}
  
void motorC_backward(int PWM){
  digitalWrite(IN5,LOW); 
  digitalWrite(IN6,HIGH); 
  analogWrite(ENC,PWM);}

void PateadorON(int pwm){
  pwm = constrain(pwm, 0, 255);
  digitalWrite(IN9, LOW);
  digitalWrite(IN10, HIGH);
  analogWrite(ENE, pwm);}

void PateadorOff(){ 
  digitalWrite(IN9, LOW);
  digitalWrite(IN10, LOW);
  analogWrite(ENE, 0);}

void CilindroOFF(){   
  digitalWrite(IN7, LOW);
  digitalWrite(IN8, LOW);
  analogWrite(END, 0);}

void CilindroON(int pwm){
  pwm = constrain(pwm, 0, 255);   
  digitalWrite(IN7, LOW);
  digitalWrite(IN8, HIGH);
  analogWrite(END, pwm);}
  
int mapPWM(float velocidad_angular, float max_rads){ //Definición de la función, requierimos la velocidad que queremos para la rueda y la máxima
  float porcentaje = velocidad_angular / max_rads; //Calculamos qué porcentaje de la velocidad máxima estamos usando. porcentaje=ωmax/​ωactual​​
  porcentaje = constrain(porcentaje, -1.0, 1.0); //Esto evita que el PWM sea mayor que 255.
  int pwm = abs(porcentaje) * 255; //Convertir porcentaje a PWM
  /*
  if (pwm > 0 && pwm < 70)
     pwm = 70;
    */
  return pwm;}
