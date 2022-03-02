#include <Arduino.h>
 
byte L;
byte LIMIT_SWITCH_ARM_DOWN;
#define Ts 20000
volatile boolean timerFlag;
IntervalTimer myTimer;
IntervalTimer UltrasonicTimer;
#include <NewPing.h>

NewPing sonar[2] = {   // Sensor object array.
  NewPing(20, 20, 200), // Each sensor's trigger pin, echo pin, and max distance to ping. 
  NewPing(19, 19, 200), 
};

//ULTRASONIC PINS
#define ULTRASONIC_L_PIN 19
#define ULTRASONIC_R_PIN 20
#define ULTRASONIC_F_PIN

//CURRENT_PIN
int PIN = 0;

int LEFT = 0;
int RIGHT = 1;
int LEFT_ULTRASONIC = 0;
int RIGHT_ULTRASONIC = 0;


void ultrasonicTimerPin(void) {
 if (PIN == LEFT) {
  LEFT_ULTRASONIC = sonar[0].ping_cm();
  PIN = RIGHT;
 }
 else {
  RIGHT_ULTRASONIC = sonar[1].ping_cm();
  PIN = LEFT;
 }
}

void timerISR(void) {
 timerFlag = true;
}
 
char buffer[5];
 
void setup() {
 Serial1.begin(38400);

 myTimer.begin(timerISR,Ts);
 UltrasonicTimer.begin(ultrasonicTimerPin,20000);

 while(Serial1);
}
 
void loop() {
  if(timerFlag) {
   timerFlag = false;
   sprintf(buffer,"%d,%d,", LEFT_ULTRASONIC,RIGHT_ULTRASONIC);
   Serial1.write(buffer);
   Serial1.flush();
 }
 
}
