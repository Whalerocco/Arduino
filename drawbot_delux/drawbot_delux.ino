//All distances in mm

#include <stdio.h>
#include <math.h>
#include <Servo.h>
Servo servoL;
Servo servoR;

#define PWM_Left_Servo 2
#define PWM_Right_Servo 3

int dL;
int xMin, xMax, yMin, yMax; // Constraints, where it is legal to draw
int global_x, global_y;                   // Keeps track of the pen's position
float realAlpha, realBeta;          // Left and right servo angles, where 0 deg is when servo is pointing to the right.
int turnLimLow, turnLimHigh; // Min and max angles for servos
int d,L1,L2;
bool penActive;
int offsetDeg_L, offsetDeg_R;

void setup() {
  // put your setup code here, to run once:
  
  dL = 3; //[mm] increment of arc section, a smaller length should result in a smoother arc

  //Coordinates. ORIGO is at left server motor axis. Positive dir x == right, y == forward.
  xMin = -10; xMax = 60;
  yMin = 60; yMax = 140;
  d = 50; L1 = 80; L2 = 100; // d = distance between servos, L1 = length of lower arm part, L2 = length of upper arm part


  // Drawing/servos
  penActive = true;   // Active pen (down)
  
  // Init servo, offset angles
  servoL.attach(PWM_Left_Servo);
  servoR.attach(PWM_Right_Servo);
  offsetDeg_L = 10; 
  offsetDeg_R = 60;
  servoL.write(offsetDeg_L);
  servoR.write(offsetDeg_R);
  
  Serial.begin(9600); //Start serial communication - used in debugging
}

// Draws arc with center in x0,y0 from angle phi1 to phi2, with a radius of r
// Angles in rad
void drawArc(int x0, int y0, float phi1, float phi2, int r){
  float phi = phi1;     // Set the current angle (phi) to the init value (phi1)
  int xOut = 0; int yOut = 0; // The pen's (possibly) limited position
  int x_temp, y_temp;
  
  //calculate angle increment
  float dPhi = dL/r; // Circle arc and line (L) becomes more similar the smaller dL is.
  
  while(phi < phi2){
    //calculate new pen x,y
    x_temp = x0 + cos(phi+dPhi)*r;
    y_temp = y0 + sin(phi+dPhi)*r;
  
    //check that x & y is within constraint
    xOut = limitX(x_temp);
    yOut = limitY(y_temp);

    global_x = xOut;
    global_y = yOut;
    
    // draw line to new x,y
    movePen(xOut,yOut,true);
  
    //update phi
    phi = phi+dPhi;
    delay(50);
  }
 
}

// Moves pen to x,y. usePen decides if pen should be drawing or not during the movement.
void movePen(int x, int y, bool usePen){
  float alpha, beta;
  float a1, a2, gamma, delta, b1, b2, eps, psi;

  a1 = pow(x,2)+pow(y,2)+pow(L1,2)-pow(L2,2); // Term 1 == x^2+y^2+L1^2-L2^2
  a2 = 2*sqrt(pow(x,2)+pow(y,2))*L1;          // Term 2 == 2*L1*sqrt(x^2+y^2)
  gamma = acos(a1/a2);
  delta = atan2(y,x);
  alpha = delta + gamma;                      // alpha = delta + gamma 

  b1 = pow(d-x,2)+pow(y,2)+pow(L1,2)-pow(L2,2); //Term 1
  b2 = 2*sqrt(pow(d-x,2)+pow(y,2))*L1;          //Term 2
  eps = acos(b1/b2);
  psi = atan2(y,(d-x));
  beta = M_PI-eps-psi;                          // beta = 180-eps-psi
  
  setPenActive(usePen); // Sets pen to the commanded state
  global_x = x;
  global_y = y;
  turnServo_L(rad2deg(alpha));    //Turn left servo to alpha angle (convert to deg)
  turnServo_R(rad2deg(beta));     //Turn right servo to beta angle (convert to deg)
}

void setPenActive(bool setActive){
  if(setActive){  // Set pen to active (down)
    if(penActive == false){
      //move pen tilt servo down
      //wait x ms
    }
  }else{          // Set pen to inactive (up)
    if(penActive == true){
      //move pen tilt servo up
      //wait x ms
    }
  }
  penActive = setActive;
}

bool getPenActive(void){
  return penActive;
}

int limitX(int x){
    if(x > xMax){
      return xMax;
    }else if(x < xMin){
      return xMin;
    }
    return x;
}

int limitY(int y){
    if(y > yMax){
      return yMax;
    }else if(y < yMin){
      return yMin;
    }
    return y;
}

void turnServo_L(int deg){
  //Limit turn here
  int tempDeg = deg;
  if(tempDeg > 180){ // Fix for atan problem (?), maybe better to fix more locally
    tempDeg = tempDeg - 180;
  }
  int angOut = tempDeg + offsetDeg_L;
  realAlpha = angOut;
  servoL.write(angOut);
}

void turnServo_R(int deg){
  //Limit turn here
  int tempDeg = deg;
  if(tempDeg > 180){ // Fix for atan problem (?), maybe better to fix more locally
    tempDeg = tempDeg - 180;
  }
  int angOut = tempDeg + offsetDeg_R;
  realBeta = angOut;
  servoR.write(angOut);
}

int rad2deg(float radAng){
  return (radAng/(2*M_PI))*360;
}
void printSerial(){
  Serial.print("X: ");
  Serial.print(global_x);
  Serial.print(" Y: ");
  Serial.print(global_y);
  Serial.print(" Left (Alpha): ");
  Serial.print(realAlpha);
  Serial.print(" Right (Beta): ");
  Serial.println(realBeta);
}

void loop() {
  // put your main code here, to run repeatedly:
  /*movePen(25, 100, true);
  printSerial();
  delay(200);
  
  movePen(0,120, true);
  printSerial();
  delay(200);
  
  movePen(50, 100, true);
  printSerial();
  delay(200);
  
  movePen(30,90, true);
  printSerial();
  delay(200);*/

  drawArc(25, 110, 0, 2*M_PI, 10);
  printSerial();
  delay(2000);

  /*turnServo_L(90); //identify 0 deg
  turnServo_R(90); //identify 0 deg
  delay(3000);
  turnServo_L(100);
  turnServo_R(80);
  delay(3000);*/
}
