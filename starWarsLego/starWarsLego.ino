#define yLED 9
#define rLED 10
#define gLED1 11
#define gLED2 12
#define gHIGH 70
#define gMED 50

void setup() {
  // put your setup code here, to run once:
pinMode(yLED, OUTPUT); //Yellow LED
pinMode(rLED, OUTPUT); //Red LED
pinMode(gLED1, OUTPUT); //Green LED left side
pinMode(gLED2, OUTPUT); //Green LED right side
analogWrite(yLED, 100);
}

void flickerLED(int led, int maxVal){
    analogWrite(led, maxVal);
    delay(200);
    for(int n = maxVal-50; n < maxVal; n++){
      analogWrite(led, n);
      delay(5);
    }
    analogWrite(led, maxVal-20);
    delay(250);
    analogWrite(led, maxVal+20);
    delay(200);
    analogWrite(led, maxVal);
    delay(50);
}
void loop() {
  // put your main code here, to run repeatedly:

  for(int i = 0; i < 2; i++){
    switch(i){
      case 0: // blink green lights
        analogWrite(rLED, 0);
        for(int j = 0; j < 5; j++){
          analogWrite(gLED1, gHIGH);
          delay(1500);
          analogWrite(gLED2, gHIGH);
          delay(1000);
          analogWrite(gLED1, 0);
          delay(700);
          analogWrite(gLED2, 0);
          delay(1000);
        }
        break;
      case 1:
          analogWrite(rLED, 255);
          delay(100);
          analogWrite(gLED1, gHIGH);
          delay(150);
          analogWrite(rLED, 0);
          delay(150);
          analogWrite(rLED, 150);
          delay(150);
          analogWrite(rLED, 0);
          delay(150);
          analogWrite(rLED, 50);
          delay(150);
          analogWrite(rLED, 0);
          delay(200);
          analogWrite(rLED, 30);
          delay(250);
          analogWrite(rLED, 0);
          delay(100);
          analogWrite(rLED, 20);
          delay(100);
          analogWrite(rLED, 0);
          delay(100);
          analogWrite(gLED2, gHIGH);
          delay(300);
          flickerLED(gLED2, 50); // 700ms
          analogWrite(gLED1, 0);
          delay(700);
          analogWrite(gLED2, 0);
          delay(1000);
          analogWrite(gLED1, gHIGH);
          delay(1500);
          analogWrite(gLED2, gHIGH);
          delay(1000);
          analogWrite(gLED1, 0);
          delay(700);
          analogWrite(gLED2, 0);
          delay(1000);
        break;
    }
  }
  
  
}
