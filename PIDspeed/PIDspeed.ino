/*
 * -----------------------------
 * IRB2001 - 2018-2
 * Modulo: Control de Sistemas Roboticos
 * -----------------------------
 * Programa Base Talleres del Curso
 * -----------------------------
 */

//-----------------------------------
// IMPORTAR librearias
#include "DualVNH5019MotorShield.h"
DualVNH5019MotorShield md;

// Definición de PINs
#define encoder0PinA  19
#define encoder0PinB  18
#define encoder1PinA  20
#define encoder1PinB  21

// Variables Tiempo
unsigned long time_ant = 0;
const int Period = 10000;   // 10 ms = 100Hz
float escalon = 0;
const float dt = Period *0.000001f;
float motorout    = 0.0;

// Variables de los Encoders y posicion
volatile long encoder0Pos = 0;
volatile long encoder1Pos = 0;
long newposition0;
long oldposition0 = 0;
long newposition1;
long oldposition1 = 0;
unsigned long newtime;
float vel0;
float vel1;

int error = 0;
int last_error = 0;
float pidTerm = 0;
int vel_ref = 60;
float Kp = 2;
float Kd = 0.3;
float Ki = 0.2;
float M1Speed = 0;
float err_ac = 0;


//-----------------------------------
// CONFIGURANDO INTERRUPCIONES
void doEncoder0A()
{
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }
}

void doEncoder0B()
{
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos--;
  } else {
    encoder0Pos++;
  }
}

void doEncoder1A()
{
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) {
    encoder1Pos++;
  } else {
    encoder1Pos--;
  }
}

void doEncoder1B()
{
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) {
    encoder1Pos--;
  } else {
    encoder1Pos++;
  }
}


//-----------------------------------
// CONFIGURACION
void setup()
{
  // Configurar MotorShield
  md.init();

  // Configurar Encoders
  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH);       // Incluir una resistencia de pullup en le entrada
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB, HIGH);       // Incluir una resistencia de pullup en le entrada
  pinMode(encoder1PinA, INPUT);
  digitalWrite(encoder1PinA, HIGH);       // Incluir una resistencia de pullup en le entrada
  pinMode(encoder1PinB, INPUT);
  digitalWrite(encoder1PinB, HIGH);       // Incluir una resistencia de pullup en le entrada
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder0A, CHANGE);  // encoder 0 PIN A
  attachInterrupt(digitalPinToInterrupt(encoder0PinB), doEncoder0B, CHANGE);  // encoder 0 PIN B
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoder1A, CHANGE);  // encoder 1 PIN A
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), doEncoder1B, CHANGE);  // encoder 1 PIN B


  // Configurar Serial port
  Serial.begin(115200);           //Inicializar el puerto serial (Monitor Serial)
  Serial.println("start");
}

//-----------------------------------
// LOOP PRINCIPAL
void loop() {
  if ((micros() - time_ant) >= Period)
  {
    newtime = micros();

    //-----------------------------------
    // Ejemplo variable alterando cada 5 segundos
    if ((newtime / 5000000) % 2)
      escalon = 100.0;
    else
      escalon = 0.0;

    //-----------------------------------
    // Actualizando Informacion de los encoders
    newposition0 = encoder0Pos;
    newposition1 = encoder1Pos;

    //-----------------------------------
    // Calculando Velocidad del motor
    float rpm = 31250;
    vel0 = (float)(newposition0 - oldposition0) * rpm / (newtime - time_ant); //RPM
    vel1 = (float)(newposition1 - oldposition1) * rpm / (newtime - time_ant); //RPM
    oldposition0 = newposition0;
    oldposition1 = newposition1;

    //-----------------------------------
    // Salida al Motor
    
    error = vel_ref - vel0; //Define the error as the offset from the reference value for the P-control
    err_ac = err_ac + error; //Define the accumulated error for the I-control
    pidTerm = (Kp * error) + (Kd * (error - last_error)) + Ki*err_ac; //The PID-term which is a sum of all its parts
    last_error = error; //shifts the current error to be the previous error, for the next loop
    M1Speed = constrain(pidTerm, 0, 400); //Constraint the input to the motor

    // Motor Voltage
    md.setM1Speed(M1Speed); //Sets the input for the first motor
    md.setM2Speed(0);

    // Reportar datos
    /*
    Serial.print("$,");
    Serial.print(newtime);
    Serial.print(",");
    Serial.print(newposition0);
    Serial.print(",");
    Serial.print(newposition1);
    Serial.print(",");
    Serial.print(escalon);
    Serial.print(",");
    */
    Serial.print(vel0);
    /*
    Serial.print(",");
    Serial.print(vel1);
    Serial.print(",");
    Serial.print(M1Speed);
    Serial.print(",");
    Serial.print(md.getM1CurrentMilliamps());
    Serial.print(",");
    Serial.print(md.getM2CurrentMilliamps());
    */
    Serial.println("");

    time_ant = newtime;
  }

}

