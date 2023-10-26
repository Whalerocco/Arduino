// https://github.com/adafruit/DHT-sensor-library/blob/master/examples/DHTtester/DHTtester.ino

#include "DHT.h"
//DHT_11 temp sensor - type definition
#define DHTTYPE DHT11

//Temp sensors pins
//Pin 6 on Arduino - Connect to Inside temp sensor's output 
#define TEMPIN_PIN 6

//Pin 7 on Arduino - Connect to Outside temp sensor's output 
#define TEMPOUT_PIN 7

//Arduino pins for L293D motor driver - where to connect what on the Arduino

//Pin 3 on Arduino - Connect to pin 2 (IN1) of L293D
#define IN1_PIN 3 

//Pin 4 on Arduino - Connect to pin 7 (IN1) of L293D
#define IN2_PIN 4

//Pin 5 on Arduino - Connect to pin 1 (ENABLE 1) of L293D 
#define ENABLE_PIN 5

int fanOutput; //Used to store fan output value [0-255] where 0 == off & 255 == full on  
int t_in_old; //Stores the temp reading from the loop before - used if the current reading is bad
int t_out_old; //Stores the temp reading from the loop before - used if the current reading is bad
int t_sleep; //The sample delay time in seconds

DHT termOutside( TEMPIN_PIN, DHTTYPE ); //Termometer sensor 1 declaration
DHT termInside( TEMPOUT_PIN, DHTTYPE ); //Termometer sensor 2 declaration

void setup() { //Run once on start-up
  //When an enable input is high, the associated drivers are enabled, 
  //and their outputs are active and in phase with their inputs.
  pinMode(ENABLE_PIN,OUTPUT); //Enables driver 1 and 2 - works like a PWM. Output anywhere between 0-255 on this pin to control speed
  pinMode(IN1_PIN,OUTPUT); //Activates the pin on the Arduino, corresponding to the IN1 on the L293D, for output
  pinMode(IN2_PIN,OUTPUT); //Activates the pin on the Arduino, corresponding to the IN2 on the L293D, for output 
  /* The following commands control the motor direction
  IN1: LOW, IN2: LOW -> Fast motor stop
  IN1: LOW, IN2: HIGH -> Turn right
  IN1: HIGH, IN2: LOW -> Turn left
  IN1: HIGH, IN2: HIGH -> Fast motor stop
  */
 
  //serial.begin(9600); //Start serial communication - used in debugging
  termOutside.begin(); //Start outside sensor 
  termInside.begin(); // Start inside sensor

  //Init variable values
  fanOutput = 0;
  t_in_old = 0;
  t_out_old = 0;
  t_sleep = 60;
}

//Functions

//static bool measure_inside( float *temperature, float *humidity )
//{
//  static unsigned long measurement_timestamp = millis( );
//
//  /* Measure once every four seconds. */
//  if( millis( ) - measurement_timestamp > 3000ul )
//  {
//    if( tempInside.measure( temperature, humidity ) == true )
//    {
//      measurement_timestamp = millis( );
//      return( true );
//    }
//  }
//
//  return( false );
//}
//
//static bool measure_environment( float *temperature, float *humidity )
//{
//  static unsigned long measurement_timestamp = millis( );
//
//  /* Measure once every four seconds. */
//  if( millis( ) - measurement_timestamp > 3000ul )
//  {
//    if( tempOutside.measure( temperature, humidity ) == true )
//    {
//      measurement_timestamp = millis( );
//      return( true );
//    }
//  }
//
//  return( false );
//}


void loop(){
  //makes motor go one way - if wrong way then reverse the high and low
  digitalWrite(IN1_PIN, HIGH); 
  digitalWrite(IN2_PIN, LOW);
  
  //Read humidity from sensors
  float h_out = termOutside.readHumidity(); //Read the outside humidity
  float h_in = termOutside.readHumidity();  //Read the inside humidity
  
  // Read temperature as Celsius (the default)
  float t_out = termOutside.readTemperature();
  float t_in = termInside.readTemperature();

  //A check if the temp sensor readings are good, if not -> use old values
  if(isnan(t_out)){
    t_out = t_out_old;
  }else{
    t_out_old = t_out;
  }

  if(isnan(t_in)){
    t_in = t_in_old;
  }else{
    t_in_old = t_in;
  }
  
  //Adjust fan speed according to temp readings
  if(t_in < 7){ //The temp inside is low enough -> Turn off the fan
    fanOutput = 0;
  }else if(t_in - t_out < 3){ //The difference in temp is not big enough -> Turn off fan
    fanOutput = 0;
  }else{ //Else -> Max output from the fan
    fanOutput = 255;  
  }
  analogWrite(ENABLE_PIN, fanOutput); //Write the desired output level [0-255] to the motor, this controls the speed with a PWM
  
  delay(1000*t_sleep); //Sleep 1 min until next loop.
}
