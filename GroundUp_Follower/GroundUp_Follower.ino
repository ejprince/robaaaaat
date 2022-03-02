#include <Arduino.h>
 
byte L;
byte LIMIT_SWITCH_ARM_DOWN;
#define Ts 20000
volatile boolean timerFlag;
IntervalTimer myTimer;
 
 
//IR PINS
#define LIMIT_SWITCH_ARM_UP_PIN 7
#define LIMIT_SWITCH_ARM_DOWN_PIN 8
 
void timerISR(void) {
 timerFlag = true;
}
 
char buffer[5];
 
void setup() {
 Serial1.begin(38400);
 pinMode(LIMIT_SWITCH_ARM_UP_PIN,INPUT_PULLUP);
 pinMode(LIMIT_SWITCH_ARM_DOWN_PIN,INPUT_PULLUP);
 myTimer.begin(timerISR,Ts);
 while(!Serial1);
}
 
void loop() {
 LIMIT_SWITCH_ARM_UP = digitalRead(LIMIT_SWITCH_ARM_UP_PIN);
 LIMIT_SWITCH_ARM_DOWN = digitalRead(LIMIT_SWITCH_ARM_UP_PIN);
  if(timerFlag) {
   timerFlag = false;
   sprintf(buffer,"%d,%d,%d,%d", LIMIT_SWITCH_ARM_UP,LIMIT_SWITCH_ARM_DOWN,1);
   Serial1.write(buffer);
   Serial1.flush();
 }
 
}
