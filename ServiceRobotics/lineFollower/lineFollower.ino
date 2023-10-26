//https://www.pololu.com/docs/0J19/all

#include <QTRSensors.h>
#include "DualVNH5019MotorShield.h"


//Servo 
#include <Servo.h>
Servo servoLW;
Servo servoRW;

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2


// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {2,3, 4, 5, 6, 7, 8, 9},
 NUM_SENSORS, TIMEOUT, QTR_NO_EMITTER_PIN); //EMITTER_PIN
unsigned int sensorValues[NUM_SENSORS];

//pid
int error = 0;
int last_error = 0;
float pidTerm = 0;
float Kp = 0.15;
float Kd = 0.126;
float Ki = 0;
float err_ac = 0;
float RWBaseSpeed = -50; //revisar signos
float LWBaseSpeed = 50;
float RWSpeed ;
float LWSpeed;
int e2 = 0;
int e3 = 0;
int e4 = 0;
int e5 = 0;
int e6 = 0;
int errorDiff = 0;

void setup()
{
  servoLW.attach(10);
  servoRW.attach(11);
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}


void loop()
{
  unsigned int position = qtrrc.readLine(sensorValues);
  error = (position - 3500) ;
  err_ac = err_ac + error; //Integral term
  errorDiff = (0.5*error+0.3*e2+0.2*e3)-(0.5*e4+0.3*e5+0.2*e6)/3; // 
  pidTerm = (Kp * error) + (Kd * (errorDiff)) + Ki*err_ac;
  last_error = error;
  RWSpeed = constrain(RWBaseSpeed - pidTerm, -90, 90); //aqui hay que ver que sea + o -
  LWSpeed = constrain(LWBaseSpeed + pidTerm, -90, 90);
  RWSpeed += 90;
  LWSpeed += 90;
  
  servoLW.write(LWSpeed);
  servoRW.write(RWSpeed);

  e2 = error;
  e3 = e2;
  e4 = e3;
  e5 = e4;
  e6 = e5;
  Serial.print(position);
 /* Serial.print(error); // comment this line out if you are using raw values
 Serial.print("  ");
 Serial.print(errorDiff);
Serial.print("  ");
 Serial.print(pidTerm); 
 Serial.print("  ");
 Serial.print(RWSpeed);
 Serial.print("  ");
Serial.print(LWSpeed);*/
Serial.println(" ");

}
