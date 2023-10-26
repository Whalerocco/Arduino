//https://www.pololu.com/docs/0J19/all
//TODO

#include <QTRSensors.h>
#include "DualVNH5019MotorShield.h"
#include <SR04.h>

//------PINS--------

//Servos
#define SLW_PIN 3
#define SRW_PIN 5
#define GRIP_PIN 6
#define LIFT_PIN 9

//Line sensor
#define EMITTER_PIN 8     // emitter is controlled by digital pin x
#define LS1 A0//Sensor furthest to the left
#define LS2 A1
#define LS3 A2
#define LS4 A3
#define LS5 A4
#define LS6 A5 //Sensor furthest to the right

//Ultrasonic distance sensors
#define TRIG_PIN_F 10
#define ECHO_PIN_F 11
#define TRIG_PIN_L 12
#define ECHO_PIN_L 13

//Snap switch
#define SNAP_PIN 7

//Servo 
#include <Servo.h>
Servo servoLW;
Servo servoRW;
Servo servoGrip;
Servo servoLift;

//Used to run setup code once
int setup1 = 1;

//Line sensor
#define NUM_SENSORS   6     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define readMode QTR_EMITTERS_ON //Used for the read function

//Snap switch
int snapState = LOW;

//Line sensor constructor
QTRSensorsRC qtrrc((unsigned char[]) {LS1, LS2, LS3, LS4, LS5, LS6},
 NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS]; //Where the readings are stored
unsigned int sensorValues2[NUM_SENSORS]; //Stores last raw reading
unsigned int sensorValues3[NUM_SENSORS]; //Stores last raw reading

unsigned int sensorValuesOld[NUM_SENSORS]; //Where the readings are stored
unsigned int sensorValuesOld2[NUM_SENSORS]; //Stores last raw reading
unsigned int sensorValuesOld3[NUM_SENSORS]; //Stores last raw reading

//Use several arrays to smoothen signal
int sensorH[NUM_SENSORS]; //To store the checkIfHigh() values
int sensorH2[NUM_SENSORS]; //Unused
int sensorH3[NUM_SENSORS]; //Unused
int sensorHMean[NUM_SENSORS];

//Ultrasonic Sensor
SR04 us_front = SR04(ECHO_PIN_F,TRIG_PIN_F);
SR04 us_left = SR04(ECHO_PIN_L,TRIG_PIN_L);
long us_d = 25; //Distance from front US sensor
long us_d_old = us_d;
long us_dl = 25; //Distance from right US sensor
long us_dl_old = us_dl;

/* CUSTOMIZABLE VALUES */
int autoCal = 0; //Overrides manual calibration

//Manual calibration - unused
// sLn = low calibration limit for pin n on sensor
int sL1 = 56; //
int sL3 = 8; //
int sL4 = 52; //
int sL5 = 56; // 
int sL6 = 52; // 
int sL8 = 52; //
int sL[NUM_SENSORS] = {sL1, sL3, sL4, sL5, sL6, sL8};
 
// sHn = high calibration limit for pin n on sensor
int sH1 = 2300; //2500; //2368; //2500 //2236 //2500
int sH3 = 1150; //1520; //1076; //1256; //1064 //1216
int sH4 = 1700; //1792; //1648; //1792; //1640 //1656
int sH5 = 2000; //2128; //1936; //2500; //1960 //1940
int sH6 = 1550; //1488; //1692; //1648; //1580 //1488
int sH8 = 1700; //1652; //1652; //1796; //1792 //1660
int sH[NUM_SENSORS] = {sH1, sH3, sH4, sH5, sH6, sH8};

//PID
int error = 0;
float pidTerm = 0;
float Kp = 0.024; //0.017//0.013;
float Kd = 0.0;
float err_sum = 0;
float err_dif = 0;
float RWBaseSpeed = 25; // 14//+ 90 to get actual speed, 0 -> 180
float LWBaseSpeed = -25; //-14
float RWSpeed;
float LWSpeed;
int e2 = 0;
int e3 = 0;
int e4 = 0;
int e5 = 0;
int e6 = 0;
int e7 = 0;
int e8 = 0;
int e9 = 0;
int e10 = 0;

//Blind turn
int t_blind_straight = 1000; //ms
int t_blind_turn = 1700;
int t_blind_straight2 = 0; //Time delay for straight after turn

//Junction detection
int jType; //Junction type
int j1 = 0, j2 = 0;
int readGood = true;
int turnTimer = 0;

//Maze variables
int maze[15][15] = {
    {-1, 0,-1, 0,-1, 0,-1, 0,-1, 0,-1, 0,-1, 0,-1},
    { 0, 9, 1,10, 0,11, 1,10, 1,11, 0,12, 1,11, 0},
    {-1, 1,-1, 1,-1, 1,-1, 1,-1, 0,-1, 0,-1, 1,-1},
    { 0,-1, 0,11, 1,12, 0,-1, 0, 8, 1,-1, 1,10, 0},
    {-1, 1,-1, 0,-1, 0,-1, 1,-1, 1,-1, 0,-1, 0,-1},
    { 0,-1, 0, 6, 1, 7, 0, 8, 1, 7, 0,16, 1,17, 0},
    {-1, 1,-1, 1,-1, 1,-1, 0,-1, 1,-1, 1,-1, 0,-1},
    { 0,-1, 0, 5, 0, 6, 1, 5, 1, 6, 0,15, 1,14, 0},
    {-1, 1,-1, 1,-1, 0,-1, 1,-1, 0,-1, 0,-1, 1,-1},
    { 0,-1, 0,-1, 0, 5, 1, 4, 1,-1, 1, 6, 0,13, 0},
    {-1, 1,-1, 1,-1, 0,-1, 1,-1, 0,-1, 1,-1, 1,-1},
    { 0, 4, 1, 3, 1, 2, 1, 3, 0, 8, 1, 7, 0,12, 0},
    {-1, 0,-1, 1,-1, 1,-1, 0,-1, 1,-1, 0,-1, 1,-1},
    { 0, 5, 1, 4, 0, 1, 1, 0, 0, 9, 1,-1, 1,11, 0},
    {-1, 0,-1, 0,-1, 0,-1, 0,-1, 0,-1, 0,-1, 0,-1},
};

int saveFirst = 1; //Decides if first cylinder should be saved first.
int findHome = 0; // Decides if the robot should automatically go home.
//save one cylinder first and then the other two
int actionList[25] = {3,3,1,1,4,3,0,0,3,3,0,0,3,0,3,0,3,3,5}; //Goes into long corridor last
  //int actionList[25] = {3,3,1,1,4,3,0,0,3,3,0,3,1,0,3,0,1,1,0,3,5}; //Goes into island loop last
  //int actionList[25] = {3,3,3,0,3,3,0,3,1,0,3,0,1,1,0,3,5}; // go for all three in a row

int actionNbr = 0;
int nbrSaved = 0;
bool finnished = false;

//Starting values
int startX = 2, startY = 6; //Starting position of robot
int rDirStart = 3; //The starting direction of the robot

//Current values
int thisX = startX, thisY = startY; //The current pos of the robot
int rDir = rDirStart; // robot's direction0 = north, 1 = east etc.
int matX = 2*thisX+1; //The corresponding position in the matrix
int matY = 2*thisY+1; //The corresponding position in the matrix
int pathDir; // direction of the path the robot should move in
int turnAction; // the resulting action from the command calculation, 
                // 0 = goStraight(), 1 = turnRight(), 2 = turn180(), 3 = turnLeft()
int pd_xN, pd_yN, pd_xE, pd_yE, pd_xS, pd_yS, pd_xW, pd_yW; // The path direction number of the cell in the      
int pd_min; //The minimum path distance value of a connected cell
int pd_xDir, pd_yDir;
bool goHome = false;


int printxy = 0;
int printSensorValues = 1;
int printdt = 0;
int printHigh = 0;
int printjType = 0;
int printGoHome = 0;

void setup()
{
  //attach servo motors
  servoLW.attach(SLW_PIN);
  servoRW.attach(SRW_PIN);
  servoGrip.attach(GRIP_PIN);
  servoLift.attach(LIFT_PIN);
  pinMode(SNAP_PIN, INPUT);
  Serial.begin(9600);
}

int checkIfHigh(int val){
  
    //Checks if the number is higher than ~half of the sensor range
    //
    if(val > 400){ 
      return 1;
    }else{
      return 0;
    }
}

void printCalibration(){
  //Prints some calibration values
    
    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc.calibratedMinimumOn[i]); //calibration min values emitter on
      Serial.print(' ');
      
    }
    Serial.println();
    
    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(sL[i]); 
      Serial.print(' ');  
    }
    Serial.println();
    
    // print the calibration maximum values measured when emitters were on
    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(qtrrc.calibratedMaximumOn[i]); //calibration max values emitter on
      Serial.print(' ');
    }
    Serial.println(' ');
    
    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.print(sH[i]); 
      Serial.print(' ');
    }
   
    Serial.println("Calibration done");
    Serial.println(' ');
}

//Function to run once in the beginning
void initSetup(){
    //Turn off motors initially
    turnOffW();
    servoGrip.write(100); //Claw open
    servoLift.write(50); //Arm up
    servoLift.detach();
    servoGrip.detach();
    
    Serial.println("Check 1");
    //calibration
    //if(autoCal){
      for (int i = 0; i < 250; i++)  // make the calibration take about 10 seconds
      {
        qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
      }
    //}

    /*
    if(autoCal){
      for(int i = 0; i < NUM_SENSORS; i++){
        sL[i] = qtrrc.calibratedMinimumOn[i];
        sH[i] = qtrrc.calibratedMaximumOn[i]
      }
    }*/
    delay(100);
    if(!autoCal){ //Set the cal. values of the sensor
      for(int i = 0; i< NUM_SENSORS; i++){
          qtrrc.calibratedMinimumOn[i] = sL[i];
          qtrrc.calibratedMaximumOn[i] = sH[i];
      }
    }
    qtrrc.readCalibrated(sensorValues, readMode); //updates line sensor values
    delay(100);
    for(int i = 0; i < NUM_SENSORS; i++){
          sensorH[i] = checkIfHigh(sensorValues[i]);
    }
    
    setup1 = 0;
}

//Turn type 0
void goStraight(){
    //turnOffW();
    //delay(100);
    servoRW.write(160); //180
    servoLW.write(20); //0
    delay(500);
    jType = 0; 
}

//Turn type 1
void turnRight(){
    turnOffW();
    servoRW.write(80);
    servoLW.write(100);
    delay(75);
    for(int g = 0; g<NUM_SENSORS;g++){
      sensorValuesOld[g] = 0;
    }
    delay(200);
    //servoRW.write(88); // RW speed 0
    servoRW.detach();
    servoLW.write(60);
    delay(1250);
    while(!sensorHMean[2]||!sensorHMean[3]){
      updateSmoothHigh(); 
      delay(25);
    }
    servoRW.attach(SRW_PIN);
    turnTimer = millis(); // Makes it so another type of turn can't be detected for a while
    Serial.print("Exit turn: ");
    for(int k = 0; k<NUM_SENSORS;k++){
        Serial.print(sensorValues[k]);
        Serial.print("\t");
    }
    jType = 0; 
}

//Turn type 3
void turnLeft(){
    turnOffW();
    servoRW.write(80);
    servoLW.write(100);
    delay(75);
    for(int g = 0; g<NUM_SENSORS;g++){
      sensorValuesOld[g] = 0;
    }
    delay(200);
    //servoLW.write(88); // LW speed 0
    servoLW.detach();
    servoRW.write(120);
    delay(1250);
    while(!sensorHMean[2]||!sensorHMean[3]){
      updateSmoothHigh(); 
      delay(25);
    }
    servoLW.attach(SLW_PIN);
    turnTimer = millis(); // Makes it so another type of turn can't be detected for a while
    Serial.print("Exit turn: ");
    for(int k = 0; k<NUM_SENSORS;k++){
        Serial.print(sensorValues[k]);
        Serial.print("\t");
    }
    jType = 0;
}

//Turn type 2
void turn180(){
    servoRW.write(150);
    servoLW.write(150);
    delay(1000);
    while(!sensorHMean[1]||!sensorHMean[2]){
      updateSmoothHigh();
      delay(25);  
    }
    jType = 0; //unused?
}

void blindTurn(){
  servoRW.write(115);
  servoLW.write(65);
  delay(t_blind_straight); //change this for perfect turn
  if(us_dl < 25){ //if the left distance sensor is low -> turn right
    servoRW.write(90);
    servoLW.write(40);
    updateLocation(1);
  }else{ //turn left
    servoRW.write(140);
    servoLW.write(90);
    updateLocation(3);
  }
  delay(t_blind_turn); //time to turn
  servoRW.write(110); //120
  servoLW.write(70); //80
  delay(t_blind_straight2); //time to reach line again
  jType = 0;
}

//Reads new values from sensor and also updates the smoothed sensorH vector (sensorHMean)
//The majority (2 at least in this case) of the sensorH1,2,3 vectors decides the value of sensorHMean[i]
void updateSmoothHigh(){
    for(int g = 0; g<NUM_SENSORS;g++){
      sensorValuesOld[g] = sensorValues[g];
    }
    qtrrc.readCalibrated(sensorValues, readMode); //updates line sensor values
    
    if(sensorValues[0] == 1000 && sensorValues[1] == 1000 && sensorValues[2] == 1000 && sensorValues[3] == 1000 && sensorValues[4] == 1000 && sensorValues[5] == 1000){
        for(int i = 0; i < NUM_SENSORS; i++){
        sensorValues[i] = sensorValuesOld[i];
      }  
    }
    
    
    for(int i = 0; i < NUM_SENSORS; i++){
        sensorHMean[i] = checkIfHigh(sensorValues[i]);
    }
      
}

void updateSmoothHigh2(){
    for(int g = 0; g<NUM_SENSORS;g++){
      sensorValuesOld[g] = sensorValues[g];
      sensorValuesOld2[g] = sensorValues2[g];
      sensorValuesOld3[g] = sensorValues3[g];
    }
    qtrrc.readCalibrated(sensorValues, readMode); //updates line sensor values
    delay(6); //10
    qtrrc.readCalibrated(sensorValues2, readMode); //updates line sensor values
    delay(6); //10
    qtrrc.readCalibrated(sensorValues3, readMode); //updates line sensor values
    /*if(sensorValues[0] == 1000 && sensorValues[1] == 1000 && sensorValues[2] == 1000 && sensorValues[3] == 1000 && sensorValues[4] == 1000 && sensorValues[5] == 1000){
        for(int i = 0; i < NUM_SENSORS; i++){
        sensorValues[i] = sensorValuesOld[i];
      }  
    }
    if(sensorValues2[0] == 1000 && sensorValues2[1] == 1000 && sensorValues2[2] == 1000 && sensorValues2[3] == 1000 && sensorValues2[4] == 1000 && sensorValues2[5] == 1000){
        for(int i = 0; i < NUM_SENSORS; i++){
        sensorValues2[i] = sensorValuesOld2[i];
      }  
    }
    if(sensorValues3[0] == 1000 && sensorValues3[1] == 1000 && sensorValues3[2] == 1000 && sensorValues3[3] == 1000 && sensorValues3[4] == 1000 && sensorValues3[5] == 1000){
        for(int i = 0; i < NUM_SENSORS; i++){
        sensorValues3[i] = sensorValuesOld3[i];
      }  
    }*/

    //Set sensorHMean to largest of three readings
    for(int i = 0; i<NUM_SENSORS; i++){
      int maxVal = 0;
      if(sensorValues[i] > maxVal){
        sensorHMean[i] = checkIfHigh(sensorValues[i]);
        maxVal = sensorHMean[i];
      }
      if(sensorValues2[i] > maxVal){
        sensorHMean[i] = checkIfHigh(sensorValues2[i]);
        maxVal = sensorHMean[i];
      }
      if(sensorValues3[i] > maxVal){
        sensorHMean[i] = checkIfHigh(sensorValues3[i]);
        maxVal = sensorHMean[i];
      }
      
    }
}

//Identifies the type of junction
int getJuncType(){
  
  if(sensorHMean[0] && (sensorHMean[2] || sensorHMean[3]) && !sensorHMean[5] && us_d < 25){
     //Left turn
     return 1;
  }else if(!sensorHMean[0] && (sensorHMean[2] || sensorHMean[3]) && sensorHMean[5] && us_d < 25){
    //Right turn
    return 2;
  }else if(sensorHMean[1] && sensorHMean[2] && sensorHMean[3] && sensorHMean[4] && us_d > 25){
    //X-junction
    return 3;
  }else if(!sensorHMean[0] && !sensorHMean[1] && !sensorHMean[2] && !sensorHMean[3] && !sensorHMean[4] && !sensorHMean[5] && us_d < 17 && us_d > 5){
    //Dead end
    return 4;
  }else if(sensorHMean[1] && sensorHMean[2] && sensorHMean[3] && sensorHMean[4] && us_d < 25){
    //T-junction coming from the bottom of the T
    return 5;
  }else if(sensorHMean[0] && (sensorHMean[2] || sensorHMean[3]) && !sensorHMean[5] && us_d > 25){
    //T-junction coming from the right
    return 6;
  }else if(!sensorHMean[0] && (sensorHMean[2] || sensorHMean[3]) && sensorHMean[5] && us_d > 25){
    //T-junction coming from the left
    return 7;
  }else if(!sensorHMean[0] && !sensorHMean[1] && !sensorHMean[2] && !sensorHMean[3] && !sensorHMean[4] && !sensorHMean[5] && us_d > 25 && us_d < 35 && us_dl < 13){
    //Blind turn
    return 8;
  }
  else{ //it is a line
    return 0;
  }
}

//Gets the next path distance number (that isn't a -1) in a direction 
int getNextPD(int matX, int matY, int dir){ 
//The path distance in the cell to the respective direction
    int pd_north, pd_east, pd_south, pd_west; //If these values are left unchanged below (because there is a wall in that direction) //we should still be able to compare them, so they need a big value (cause the smallest values is searched for). 
                                                        
    switch(dir){
      case 0: //Search to north (negative y-dir)
        if(maze[matY-2][matX] != -1){
          pd_north = maze[matY-2][matX];
          pd_xN = matX;
          pd_yN = matY-2;
          return pd_north;
        }else{
          return getNextPD(matX,matY-2,0);
        }
        break;
      case 1: //Search to east (positive x-dir)
        if(maze[matY][matX+2] != -1){
          pd_east = maze[matY][matX+2];
          pd_xE = matX+2;
          pd_yE = matY;
          return pd_east;
        }else{
          return getNextPD(matX+2,matY,1);
        }
          break;
      case 2: //Search to south (positive y-dir)
        if(maze[matY+2][matX] != -1){
          pd_south = maze[matY+2][matX];
          pd_xS = matX;
          pd_yS = matY+2;
          return pd_south;
        }else{
          return getNextPD(matX,matY+2,2);
        }
          break;
      case 3: //Search to west (negative x-dir)
        if(maze[matY][matX-2] != -1){
          pd_west = maze[matY][matX-2];
          pd_xW = matX-2;
          pd_yW = matY;
          return pd_west;
        }else{
          return getNextPD(matX-2,matY,3);
        }
          break;
    }
}

//Compares the different path distance numbers and returns the direction of the one with the lowest one
int comparePD(int pd_north, int pd_east, int pd_south, int pd_west){ 
    pd_min = 99;
    int dir;
    
    if(pd_north < pd_min){
      pd_min = pd_north;
      dir = 0;
      pd_xDir = pd_xN;
      pd_yDir = pd_yN;
    }
    if(pd_east < pd_min){
      pd_min = pd_east;
      dir = 1;
      pd_xDir = pd_xE;
      pd_yDir = pd_yE;
    }
    if(pd_south < pd_min){
      pd_min = pd_south;
      dir = 2;
      pd_xDir = pd_xS;
      pd_yDir = pd_yS;
    }
    if(pd_west < pd_min){
      pd_min = pd_west;
      dir = 3;
      pd_xDir = pd_xW;
      pd_yDir = pd_yW;
    }
    return dir;
}

int inv(int c){ //From matrix coord system to x,y
    return (c-1)/2;
}

//Takes a turn action and updates the robot's location to the next junction it will arrive at.
void updateLocation(int turnAction){
    int foo;
    int pDir = (turnAction + rDir)%4; //N,E,S,W direction of the path the robot is turning to
    foo = getNextPD(matX,matY,pDir); //updates pd_x&y for that direction

    switch(pDir){ //Updates pd_x/yDir depending on path direction
      case 0:
      pd_xDir = pd_xN;
      pd_yDir = pd_yN;
        break;
      case 1:
      pd_xDir = pd_xE;
      pd_yDir = pd_yE;
        break;
      case 2:
      pd_xDir = pd_xS;
      pd_yDir = pd_yS;
        break;
      case 3:
      pd_xDir = pd_xW;
      pd_yDir = pd_yW;
        break;
      
    }
    matX = pd_xDir; matY = pd_yDir; //updates the current (or soon to be current) location of the robot
    rDir = pDir; //And direction of the robot
}

//used for navigating in X- and T-junctions when searching for robots
int getNextAction(){
  int action;
  action = actionList[actionNbr];
  actionNbr++;
  return action;
}

//Used when searching for robot and deciding which action to take in a junction
void doNextAction(){
    int nextAction = getNextAction(); //gets next action and updates list index
    
    if(nextAction == 0){
      goStraight();
      updateLocation(0);
    }else if(nextAction == 1){
      turnRight();
      updateLocation(1);
    }else if(nextAction == 3){
      turnLeft();
      updateLocation(3);
    }else if(nextAction == 4){
      /* Drop first cylinder */
      dropCylinder();
    }else if(nextAction == 5){
      /* DONE WITH MAZE */
      //turnOffW();
      //VICTORY DONUT
      servoRW.write(180);
      servoLW.write(180);
      finnished = true;
    }
}

void turnOffW(){
  servoRW.write(88);
  servoLW.write(88);
}

void turnOnW(int lwSpeed, int rwSpeed){
  servoLW.write(lwSpeed);
  servoRW.write(rwSpeed);
}

void saveCylinder(){
    servoRW.detach();
    servoLW.detach();
    servoLift.attach(LIFT_PIN);
    servoGrip.attach(GRIP_PIN);
    turnOffW();
    servoLift.write(55);
    delay(500);
    servoGrip.write(115);
    delay(500);
    servoLift.write(120);
    delay(500);
    servoGrip.write(60);   //greppar om cylindern
    delay(500);      //måste  testa hur lång tid den behöver för att lyfta
    servoLift.write(20);   //lyfter hela armen
    delay(1000);      //ev mer delay
    servoGrip.write(110);  //öppnar klon
    delay(500);
    servoLift.write(70);    //sänker tillbaka armen
    delay(100);
    if(nbrSaved >= 2 && !saveFirst){
      servoGrip.write(60); //60
    }else{
      servoGrip.write(90); 
    }
    delay(300);
    servoLift.write(20); //puttar till cylindern
    delay(500);
    servoLift.write(50);
    servoGrip.write(100); //återställd
    delay(500);
    servoLift.detach();
    servoGrip.detach();
    servoRW.attach(SRW_PIN);
    servoLW.attach(SLW_PIN);
}

//used to lift up first cylinder
void saveFirstCylinder(){
    servoRW.detach();
    servoLW.detach();
    servoLift.attach(LIFT_PIN);
    servoGrip.attach(GRIP_PIN);
    turnOffW();
    delay(500);
    servoLift.write(55);
    delay(500);
    servoGrip.write(115);  //öppnar klon
    delay(500);
    servoLift.write(120);
    delay(500);
    servoGrip.write(60);   //greppar om cylindern
    delay(500);      //måste  testa hur lång tid den behöver för att lyfta
    servoLift.write(30);   //lyfter hela armen
    delay(1000);      //ev mer delay
    servoGrip.write(80);
    delay(500);
    servoRW.attach(SRW_PIN);
    servoLW.attach(SLW_PIN);
}

//used to drop the first cylinder
void dropCylinder(){
    delay(200);
    servoRW.detach();
    servoLW.detach();
    servoLift.write(120); // Lowers gripper
    delay(500);
    servoGrip.write(100);  //öppnar klon
    delay(500);
    servoRW.attach(SRW_PIN);
    servoLW.attach(SLW_PIN);
    turnOffW();
    delay(500);
    servoLW.write(120);
    servoRW.write(60);
    delay(400);
    servoLift.write(50); //återställd
    delay(500);
    turnOffW();
    servoLift.detach();
    servoGrip.detach();
    
}

void loop()
{
  int t1 = millis();
  //Runs setup once
  if(setup1){
    initSetup(); 
    printCalibration(); 
  }

  if(!finnished){
    //Read new sensor values
    updateSmoothHigh();
    jType = getJuncType(); //Decides what type of junction it seems to be, or a line
    if(millis()-turnTimer > 700){
      if(jType != 0){
        updateSmoothHigh2(); //Reads three more times to see what type of junction it is.
        jType = getJuncType();
      }
    }else{
      jType = 0;
    }
    if(printjType){
      Serial.print("jType: ");
      Serial.println(jType);
    }
    us_d = us_front.Distance(); //Reads ultrasonic sensor in front
    if(us_d == 0){
      us_d = us_d_old;
    }
    us_d_old = us_d;
    Serial.println(us_d);
    us_dl = us_left.Distance(); //Read ultrasonic sensor to the right
    if(us_dl == 0){
      us_dl = us_dl_old;
    }
     us_dl_old = us_dl;
    
    snapState = digitalRead(SNAP_PIN);
    //------------------------------------
    /*for(int k = 0; k<NUM_SENSORS;k++){
      Serial.print(sensorValues[k]);
      Serial.print(" ");
    }
    Serial.print(" ");
  */
    
    //If all three cylinders have been found --> Go back fastest way
    if(snapState == HIGH){ 
       if(saveFirst && nbrSaved == 0){
          saveFirstCylinder();
       }else{
          saveCylinder();
       } 
       nbrSaved++;
       if(nbrSaved == 3 && findHome){
        goHome = 1; //Go home if all saved
       }
    }
    else if(jType != 0 && goHome){ //Robot is at a junction and is commanded home
      //Turns off motors while calculating next action
      turnOffW();
      int pd_north = 99, pd_east = 99, pd_south = 99, pd_west = 99; //The path distance in the cell to the respective direction
      if(maze[matY-1][matX]){ //Wall or path to North (negative y-dir)
          pd_north = getNextPD(matX,matY,0); //Finds the next PD number and its location
      }
      if(maze[matY][matX+1]){ //Wall or path to East (positive x-dir)
          pd_east = getNextPD(matX,matY,1); //Finds the next PD number and its location
      }
      if(maze[matY+1][matX]){ //Wall or path to South (positive y-dir)
          pd_south = getNextPD(matX,matY,2); //Finds the next PD number and its location
      }
      if(maze[matY][matX-1]){ //Wall or path to East (negative x-dir)
          pd_west = getNextPD(matX,matY,3); //Finds the next PD number and its location
      }
  
      pathDir = comparePD(pd_north,pd_east,pd_south,pd_west); // FUNCTION THAT COMPARES WHICH DIRECTION HAS SMALLEST PD NUMBER
      matX = pd_xDir; matY = pd_yDir; //updates the current (or soon to be current) location of the robot
      if(printGoHome){
        Serial.print("PathDir ");
        Serial.print(pathDir);
        Serial.print(" Next x: ");
        Serial.print(inv(matX));
        Serial.print(" Next y: ");
        Serial.println(inv(matY));
       }
      
      turnAction = (4+pathDir-rDir)%4; //How the robot must turn in order to get the new direction
  
      //The turnAction decides the robot's next move.
      switch(turnAction){
        case 0: //Continue straight
          goStraight();
          break;
        case 1: //
          turnRight();
          break;
        case 2:
          turn180();
          break;
        case 3:
          turnLeft();
          break;
      }
      rDir = pathDir; //Updates robot direction
    }else{ //Normal operation when searching for cylinders
      
      //LEFT TURN
      if(jType == 1){ 
          turnLeft();
          updateLocation(3);
      }   
      //RIGHT TURN
      else if(jType == 2){
          turnRight();
          updateLocation(1);
      }
      //X-junction
      else if(jType == 3){
        doNextAction(); //also updates location
      }
      //Dead end 
      else if(jType == 4){ //
        //Checks if next move has something to do with dead-ends 
          if(actionList[actionNbr] == 4){ //If the next move is to lower the cylinder
            doNextAction();
          }else if(actionList[actionNbr] == 5){ //if the next move is to exit the maze
            doNextAction();
          }else{
            turn180();
            updateLocation(2);
          }
      }
      //T-junction coming from the bottom
      else if(jType == 5){
          doNextAction(); //also updates location
      }
      //T-junction coming from the right (in the T)
      else if(jType == 6){
          doNextAction(); //also updates location
      }
      //T-junction coming from the left (in the T)
      else if(jType == 7){
          doNextAction(); //also updates location
      }
      // Blind right or left turn
      else if(jType == 8){ 
          blindTurn(); //also updates location
      }
    }
    
  
    if(printxy){
      Serial.print("X: ");
      Serial.print(inv(matX));
      Serial.print(" Y: ");
      Serial.print(inv(matY));
      Serial.print(" rDir: ");
      Serial.println(rDir);
    }
    //LINE FOLLOWING
    //When not at a junction -> line following
    unsigned int position = qtrrc.readLine(sensorValues); //Gets position of line
    error = (position - (NUM_SENSORS-1)*1000/2) ; //CHECK MIDDLE POS VAL FOR 4 SENSORS ACTIVE
    pidTerm = (Kp * (0.5*error+0.3*e2+0.2*e3) );// + (Kd * (err_dif));// + (Kd * (err_dif)); // + Ki*err_ac;
  
    RWSpeed = constrain(RWBaseSpeed - pidTerm, -90, 90); //Det är samma tecken framför pidTerm för att den gör så den svänger
    LWSpeed = constrain(LWBaseSpeed - pidTerm, -90, 90);
    RWSpeed += 90;
    LWSpeed += 90;
    RWSpeed = constrain(RWSpeed,90, 180);
    LWSpeed = constrain(LWSpeed, 0, 90);
    
    servoRW.write(RWSpeed);
    servoLW.write(LWSpeed);
    
    //Set new 'old values'
    e3 = e2;
    e2 = error;
    
    //Serial.print(position);
    /* Serial.print(error); 
     */
  
     //Prints out what sensors are high
    //Serial.print("High sensors: ");
    if(printHigh){
      for(int k = 0; k<NUM_SENSORS;k++){
        Serial.print(sensorHMean[k]);
        Serial.print(" ");
      }
      Serial.println(" ");
    }
    if(printSensorValues){
      for(int k = 0; k<NUM_SENSORS;k++){
        Serial.print(sensorValues[k]);
        Serial.print("\t");
      }
    }

    
    int dt = millis()-t1;
    if(printdt){
      Serial.println(dt);
    }
  }
}
