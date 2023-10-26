int width = 8;;
int height = 8;
int r = 0; //row index
int c = 0; //col index

int initGrid[8][8] = {
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0}
  };

int xBuf[4] = {0,0,0,0}; //stores last points
int yBuf[4] = {0,0,0,0};
int curX = 2;
int curY = 4;
int nextX = 0;
int nextY = 0;
//x and y are defined like rows and colums

// LED matrix

#include "LedControl.h"
int DIN = 12;   // DIN pin of MAX7219 module
int CLK = 10;   // CLK pin of MAX7219 module
int CS = 11;    // CS pin of MAX7219 module

LedControl lc = LedControl(DIN,CLK,CS,1);

//GY-521 Accel/Gyro
#include<Wire.h>
const int MPU=0x68; 
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

void setup() {
  // put your setup code here, to run once:
  //LED matrix
  lc.shutdown(0,false);
  /* Set the brightness to a medium values */
  lc.setIntensity(0,4);
  /* and clear the display */
  lc.clearDisplay(0);
  
  //GY-521
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);
  Serial.begin(9600);
}

void loop() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,12,true); 

  GyX=Wire.read()<<8|Wire.read(); // row-direction
  GyY=Wire.read()<<8|Wire.read(); // col-direction
  
  Serial.print(" | GyX = "); Serial.println(GyX);
  Serial.println(" | GyY = "); Serial.println(GyY);

  //lc.setLed(0,curY,curX,false); //turn off current LED
  lc.setLed(0,yBuf[3],xBuf[3],false); //turns off last LED in line
  
  if(GyX > 3000){
    if(curX < 7){
    nextX = curX+1;
    curX = nextX;
    }
  }

  if(GyX < -3000){
    if(curX > 0){
    nextX = curX-1;
    curX = nextX;
    }
  }

  if(GyY < -3000){
    if(curY < 7){
    nextY = curY+1;
    curY = nextY;
    }
  }

  if(GyY > 3000){ //If tilt forward, the dot should move in negative row direction
    if(curY > 0){
    nextY = curY-1;
    curY = nextY;
    }
  }

  //update xy-points buffer
  for(int i = 2; i >= 0; i--){
    xBuf[i+1] = xBuf[i];
    yBuf[i+1] = yBuf[i];   
  }
  xBuf[0] = curX;
  yBuf[0] = curY;

  //lc.setLed(0,r,c,true);
  
  lc.setLed(0,yBuf[0],xBuf[0],true);
  delay(100);

}
