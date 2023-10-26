#include <stdio.h>
#include <stdlib.h>

int width = 8;;
int height = 8;
int r = 0; //row index
int c = 0; //col index
int dir = 0; // 0 = up, 1 = right etc.

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

int xBuf[8] = {0,0,0,0,0,0,0,0}; //stores last points
int yBuf[8] = {0,0,0,0,0,0,0,0};
int curX = 4; //cols
int curY = 6; //rows
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

  // Seed for a random number generator
  srand(time(0));
}

int getNewDir(int dir){ 
  int newDir; // 0 = up/Y-, 1 = right/X+, 2 = down/Y+, 3 = left/X-
  
  //check possible directions (everyone except for backwards)
  //choose direction where Gy is the highest
  if(dir == 0){ // Up, Can go in 0,1,3
    if(abs(Gy)>abs(Gx)){
      if(Gy<0){
        return 0;
      }else{ // If the backwards direction is the largest, special case
        if(Gx>0){
          return 1;
        }else{
          return 3;
        }
      }
    }else{
      if(Gx>0){
          return 1;
        }else{
          return 3;
        }
    }
  }else if(dir == 1){ //right, Can go in 0,1,2
    if(abs(Gx)>abs(Gy)){
      if(Gx>0){
        return 0;
      }else{ // If the backwards direction is the largest, special case
        if(Gy>0){
          return 2;
        }else{
          return 0;
        }
      }
    }else{
      if(Gy>0){
          return 2;
        }else{
          return 0;
        }
    }
  }else if(dir == 2){ // Down, Can go in 1,2,3
    if(abs(Gy)>abs(Gx)){
      if(Gy>0){
        return 2;
      }else{ // If the backwards direction is the largest, special case
        if(Gx>0){
          return 1;
        }else{
          return 3;
        }
      }
    }else{
      if(Gx>0){
          return 1;
        }else{
          return 3;
        }
    }
  }else if(dir == 3){ // Left, Can go in 0,2,3
    if(abs(Gx)>abs(Gy)){
      if(Gx<0){
        return 3;
      }else{ // If the backwards direction is the largest, special case
        if(Gy>0){
          return 2;
        }else{
          return 0;
        }
      }
    }else{
      if(Gy>0){
          return 2;
        }else{
          return 0;
        }
    }
  }
  
  return -1;
}

int updateCoordinates(int dir){
  int alive = 1; // Returns 0 if the snake crashes
  if(dir == 0){
    
  }else if(dir == 1){
    
  }else if(dir == 2){
    
  }else if(dir == 3){
    
  }
  nextY = curY-1;
  curY = nextY;
  /*
   * 
   * 
   */
  
  return alive;
}
void updateBuffer(){
//update xy-points buffer
  for(int i = 2; i >= 0; i--){ // CHANGE 2 to n
    xBuf[i+1] = xBuf[i];
    yBuf[i+1] = yBuf[i];   
  }
  xBuf[0] = curX;
  yBuf[0] = curY;
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

  lc.setLed(0,yBuf[3],xBuf[3],false); //turns off last LED in line
  dir = getNewDir(dir);
  updateCoordinates(dir);
  updateBuffer(); // Updates the snake 
  lc.setLed(0,yBuf[0],xBuf[0],true);
  delay(100);

  /*
   *  start with 2 length
   *  direction for next move can only be right or left (or continue straight) 
   *  relative to the current direction
   *  Every 3 seconds, when there is no food on the screen, a new appears   
   *  If a wall is hit or the snake hits itself the game restarts
   * 
   * 
   * 
   */


}
