#include <Arduino.h>
#include <Wire.h>
 
byte LIMIT_SWITCH_ARM_UP;
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
 
 
void setup() {
 Wire.begin();
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
   Serial1.write(5);
   Serial1.write(4);
   Serial1.write(3);
   Serial1.write(2);
   Serial1.write(1);
   Serial1.write(0);
   Serial1.flush();
 }
 
}
