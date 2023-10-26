int width = 8;;
int height = 8;
int r = 0; //row index
int c = 0; //col index

int initGrid[8][8] = {
  {0, 0, 0, 0, 0, 0, 0, 0}
  {0, 0, 0, 0, 0, 0, 0, 0}
  {0, 0, 0, 0, 0, 0, 0, 0}
  {0, 0, 0, 0, 0, 0, 0, 0}
  {0, 0, 0, 0, 0, 0, 0, 0}
  {0, 0, 0, 0, 0, 0, 0, 0}
  {0, 0, 0, 0, 0, 0, 0, 0}
  {0, 0, 0, 0, 0, 0, 0, 0}
  };


// LED matrix

#include <MaxMatrix.h>
int DIN = 7;   // DIN pin of MAX7219 module
int CLK = 6;   // CLK pin of MAX7219 module
int CS = 5;    // CS pin of MAX7219 module
int maxInUse = 1;
MaxMatrix m(DIN, CS, CLK, maxInUse); 

void setup() {
  // put your setup code here, to run once:
  m.init(); // MAX7219 initialization
  m.setIntensity(8); // initial led matrix intensity, 0-15
}

void loop() {
  // put your main code here, to run repeatedly:

}
