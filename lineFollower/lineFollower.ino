 #include <QTRSensors.h>
#include "DualVNH5019MotorShield.h"

DualVNH5019MotorShield md;

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   A7     // emitter is controlled by digital pin 2

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {A8, A9, A10, A11, A12, A13, A14, A15},
 NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

//pid
int error = 0;
int last_error = 0;
float pidTerm = 0;
float Kp = 0.1235*1.2;
float Kd = 0.15*1.2;
float Ki = 0;
float err_ac = 0;
float M1BaseSpeed = -60; //speed of motor when all PID-terms == 0
float M2BaseSpeed = 60;
float M1Speed ;
float M2Speed;
float errorD = 0;
int e2 = 0;
int e3 = 0;
int e4 = 0;

void setup()
{
  md.init();
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
  unsigned int position = qtrrc.readLine(sensorValues); // Reads the position of the line from the sensor
  error = (position - 3500) ; //Sets the error as the difference from the middle value (3500) for the P-term
  err_ac = err_ac + error; //Calculates the accumulated error for the I-control
  errorD = (error-e2)/2+(e3-e4)/2; //Calculates the derivate with a mean for the D-control 
  pidTerm = (Kp * error) + (Kd * (errorD)) + Ki*err_ac; //the sum of the PID-control
  M1Speed = constrain((M1BaseSpeed + pidTerm), -250, 250); //Sets a constraint for how big the Motor input can be, between -250 & 250
  M2Speed = constrain((M2BaseSpeed + pidTerm), -250, 250); //same thing
  e2 = error; //shifts the current error to be the previous error, for the next loop
  e3 = e2; //same thing
  e4 = e3; //same thing
  md.setM1Speed(M1Speed); //Sets the speed of the motors
  md.setM2Speed(M2Speed);

  
  Serial.print(error); // comment this line out if you are using raw values
 Serial.print(",");
 Serial.print(pidTerm); 
 Serial.print(",");
 Serial.print(M1Speed);
 Serial.print(",");
Serial.print(M2Speed);
Serial.println(" ");
}
