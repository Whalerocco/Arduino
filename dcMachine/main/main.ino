//Arduino pins for L293D motor driver - where to connect what on the Arduino

//Pin 3 on Arduino - Connect to pin 2 (IN1) of L293D
#define IN1_PIN 3 

//Pin 4 on Arduino - Connect to pin 7 (IN1) of L293D
#define IN2_PIN 4

//Pin 5 on Arduino - Connect to pin 1 (ENABLE 1) of L293D 
#define ENABLE_PIN 5

int fanOutput; //Used to store fan output value [0-255] where 0 == off & 255 == full on  

void setup() {
  // put your setup code here, to run once:
  //When an enable input is high, the associated drivers are enabled, 
  //and their outputs are active and in phase with their inputs.
  pinMode(ENABLE_PIN,OUTPUT); //Enables driver 1 and 2 - works like a PWM. Output anywhere between 0-255 on this pin to control speed
  pinMode(IN1_PIN,OUTPUT); //Activates the pin on the Arduino, corresponding to the IN1 on the L293D, for output
  pinMode(IN2_PIN,OUTPUT); //Activates the pin on the Arduino, corresponding to the IN2 on the L293D, for output
  fanOutput = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(IN1_PIN, LOW); 
  digitalWrite(IN2_PIN, HIGH);
  fanOutput = 0;
  analogWrite(ENABLE_PIN, fanOutput); //Write the desired output level [0-255] to the motor, this controls the speed with a PWM
  delay(100);
  
  
}
