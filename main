#include <Arduino.h>
#include <PID_v1.h>
#include <Wire.h>
#include <Encoder.h>
 
 
/*MOTOR PINS*/
 
// CONVEYOR MOTOR
#define PWM_INT 11
#define INT_D1 12
#define INT_D2 13
 
// ARM MOTOR
#define PWM_ROT 8
#define ROT_D1 9
#define ROT_D2 10
 
// ARM MOTOR ENCODER
//CHECK
#define ROTEA 15
#define ROTEB 16
 
// LEFT WHEEL MOTOR
#define PWM_L 7
#define L_D1 6
#define L_D2 5
 
// LEFT WHEEL ENCODER
#define LEA 17
#define LEB 21
 
// RIGHT WHEEL MOTOR
#define PWM_R 2
#define R_D1 3
#define R_D2 4
 
// RIGHT WHEEL ENCODER
#define REA  23
#define REB  22
 
// ULTRASONIC PINS
#define TRIGGER 20 // attach pin D3 Arduino to pin Trig of HC-SR04
#define ECHO_L 19  // attach pin D2 Arduino to pin Echo of HC-SR04
#define ECHO_R 18 // attach pin D2 Arduino to pin Echo of HC-SR04
//#define ECHO_F 17 // attach pin D2 Arduino to pin Echo of HC-SR04
 
int LIMIT_SWITCH_TOP;
int LIMIT_SWITCH_BOTTOM;
int CURRENT_ULTRASONIC;
 
//ENCODER CODERS
volatile unsigned long COUNTER_LEFT = 0;  // use volatile for shared variables
volatile unsigned long COUNTER_RIGHT = 0; // use volatile for shared variables
volatile unsigned long COUNTER_ARM = 0;   // use volatile for shared variables
volatile unsigned long COUNTER_ARM_TOTAL = 0;
Encoder COUNTER_RIGHT_ENCODER(REA,REB);
Encoder COUNTER_LEFT_ENCODER(LEA,LEB);
Encoder COUNTER_ARM_TOTAL_ENCODER(ROTEA,ROTEB);
 
double MOTOR_LEFT_DUTY_CYCLE = 0;
double MOTOR_RIGHT_DUTY_CYCLE = 0;
double MOTOR_ARM_DUTY_CYCLE = 0;
double MOTOR_INTAKE_DUTY_CYCLE = 0;
 
double AVERAGE_VELOCITY_LEFT = 0;
double AVERAGE_VELOCITY_RIGHT = 0;
 
bool POSITION_CONTROL = true;
 
int BACKWARD = 0;
int FORWARD = 1;
 
int DIRECTION_ARM;
 
double r_Kp = 6;
double r_Ki = 3; // 6.5;
double r_Kd = 0; // 5.5;
 
double l_Kp = 12;
double l_Ki = 6; // 6.5;
double l_Kd = 0; // 5.5;
 
double p_Kp = 0.5;
double p_Ki = 0.00; // 6.5;
double p_Kd = 0.1; // 5.5;
 
double MOTOR_LEFT_CURRENT_STATE = 0;
double MOTOR_RIGHT_CURRENT_STATE = 0;
 
double MOTOR_LEFT_VELOCITY = 0;
double MOTOR_LEFT_POSITION = 0;
 
double MOTOR_RIGHT_VELOCITY = 0;
double MOTOR_RIGHT_POSITION = 0;
 
double MOTOR_ARM_POSITION = 0;
 
double MOTOR_LEFT_SETPOINT = 20;
double MOTOR_RIGHT_SETPOINT = 20;
double MOTOR_ARM_SETPOINT = 100;
 
long duration_front;
double distance_front;
long duration_left;
double distance_left;
long duration_right;
double distance_right;
 
PID PID_LEFT(&MOTOR_LEFT_CURRENT_STATE, &MOTOR_LEFT_DUTY_CYCLE, &MOTOR_LEFT_SETPOINT, l_Kp, l_Ki, l_Kd, DIRECT);
PID PID_RIGHT(&MOTOR_RIGHT_CURRENT_STATE, &MOTOR_RIGHT_DUTY_CYCLE, &MOTOR_RIGHT_SETPOINT, r_Kp, r_Ki, r_Kd, DIRECT);
PID PID_ARM(&MOTOR_ARM_POSITION, &MOTOR_ARM_DUTY_CYCLE, &MOTOR_ARM_SETPOINT, p_Kp, p_Ki, p_Kd, DIRECT);
 
#define PULSES_PER_REVOLUTION 64
#define GEAR_RATIO 70
#define NINETY_DEGREES 90
 
#define MOTOR_SAMPLING_TIME 10000
#define VELOCITY_SAMPLING_TIME 10000
#define ULTRASONIC_SAMPLING_TIME 10000
#define REVERSE_ON_WALL_TIME 500000
#define FORWARD_TOWARD_WALL_TIME 5000000
 
IntervalTimer motorTimer;
IntervalTimer calculateVelocityTimer;
IntervalTimer calculateUltrasonicTimer;
IntervalTimer reverseWallTimer;
 
float calculateVelocity(int MOTOR)
{
 if (MOTOR == PWM_L)
 {
   float RPM = ((COUNTER_LEFT * 60) / (PULSES_PER_REVOLUTION)) / (0.010 * GEAR_RATIO);
   // float ANGULAR_VELOCITY = RPM * 2 * 3.14159 / 60;
   // float VELOCITY = ANGULAR_VELOCITY * .035;
   COUNTER_LEFT = 0;
   return RPM;
 }
 else
 {
   float RPM = ((COUNTER_RIGHT * 60) / (PULSES_PER_REVOLUTION)) / (0.010 * GEAR_RATIO);
   // float ANGULAR_VELOCITY = RPM * 2 * 3.14159 / 60;
   // float VELOCITY = ANGULAR_VELOCITY * .035;
   COUNTER_RIGHT = 0;
   return RPM;
 }
}
 
void calculateVelocities()
{
 MOTOR_LEFT_VELOCITY = calculateVelocity(PWM_L);
 MOTOR_RIGHT_VELOCITY = calculateVelocity(PWM_R);
}
 
void updateWheelMotors() {
 
 calculateVelocities();
 
 if (POSITION_CONTROL == true) {
   MOTOR_LEFT_CURRENT_STATE = COUNTER_LEFT_ENCODER.read();
   MOTOR_RIGHT_CURRENT_STATE = COUNTER_RIGHT_ENCODER.read();
 }
 else {
   MOTOR_LEFT_CURRENT_STATE = MOTOR_LEFT_VELOCITY;
   MOTOR_RIGHT_CURRENT_STATE = MOTOR_RIGHT_VELOCITY;
 }
 MOTOR_ARM_POSITION = COUNTER_ARM_TOTAL_ENCODER.read();
 Serial.println("LEFT");
 Serial.println(MOTOR_LEFT_VELOCITY);
 Serial.println(MOTOR_LEFT_DUTY_CYCLE);
 Serial.println(MOTOR_LEFT_SETPOINT);
 PID_RIGHT.Compute();
 PID_LEFT.Compute();
 PID_ARM.Compute();
 Serial.println("RIGHT");
 Serial.println(MOTOR_RIGHT_VELOCITY);
 Serial.println(MOTOR_RIGHT_DUTY_CYCLE);
 Serial.println(MOTOR_LEFT_SETPOINT);
 analogWrite(PWM_L, MOTOR_LEFT_DUTY_CYCLE);
 analogWrite(PWM_R, MOTOR_RIGHT_DUTY_CYCLE);
 
 if (MOTOR_ARM_DUTY_CYCLE < 0) {
   analogWrite(PWM_ROT, -MOTOR_ARM_DUTY_CYCLE);
   digitalWrite(ROT_D1,HIGH);
   digitalWrite(ROT_D2,LOW);
 }
 else {
   analogWrite(PWM_ROT, MOTOR_ARM_DUTY_CYCLE);
   digitalWrite(ROT_D1,LOW);
   digitalWrite(ROT_D2,HIGH);
 }
 
 Serial.println("COUNTER_LEFT_TOTAL");
 Serial.println(COUNTER_LEFT_ENCODER.read());
 Serial.println("COUNTER_RIGHT_TOTAL");
 Serial.println(COUNTER_RIGHT_ENCODER.read());
}
 
void calculateDistance()
{
 // digitalWrite(TRIGGER, LOW);
 // delayMicroseconds(2);
 // duration_front = pulseIn(ECHO_F, HIGH);
 // digitalWrite(TRIGGER, HIGH);
 // delayMicroseconds(10);
 // digitalWrite(TRIGGER, LOW);
 // duration_left = pulseIn(ECHO_L, HIGH);
 // digitalWrite(TRIGGER,HIGH);
 // if (duration_front == 0.0) {
 //   duration_front = 1000000;
 // }
 // if (duration_left == 0.0) {
 //   duration_left = 1000000;
 // }
 
 // distance_front = duration_front * 0.034 / 2;
 // distance_left = duration_left * 0.034 / 2;
 
 //Serial.println(distance_front);
 //Serial.println(distance_left);
}
 
/*ROTATIONS PER MINUTE*/
 
void countFallingEdgesLeft()
{
 COUNTER_LEFT = 1 + COUNTER_LEFT;
 if (millis() > 3000) {
 }
}
 
void countFallingEdgesRight()
{
 COUNTER_RIGHT = 1 + COUNTER_RIGHT;
 if (millis() > 3000) {
 }
}
 
void countFallingEdgesArm()
{
 COUNTER_ARM = 1 + COUNTER_ARM;
 COUNTER_ARM_TOTAL = COUNTER_ARM_TOTAL + 1;
 MOTOR_ARM_POSITION = COUNTER_ARM_TOTAL;
}
 
 
void SetMotor(int PIN, int DIRECTION, double SETPOINT)
{
 //ARM MOTOR
 if (PIN == PWM_ROT)
 {
   COUNTER_ARM_TOTAL_ENCODER.update(0);
   MOTOR_ARM_POSITION = SETPOINT;
   if (DIRECTION == BACKWARD)
   {
     digitalWrite(ROT_D1, HIGH);
     digitalWrite(ROT_D2, LOW);
 
   }
   else if (DIRECTION == FORWARD)
   {
     digitalWrite(ROT_D1, LOW);
     digitalWrite(ROT_D2, HIGH);
 
   }
 }
 //INTAKE MOTOR
 else if (PIN == PWM_INT)
 {
   if (DIRECTION == BACKWARD)
   {
   }
 
   else if (DIRECTION == FORWARD)
   {
   }
 }
 //LEFT MOTOR
 else if (PIN == PWM_L)
 {
   COUNTER_LEFT_ENCODER.update(0);
   MOTOR_LEFT_SETPOINT = SETPOINT;
   LEFT_M
   if (DIRECTION == BACKWARD)
   {
     digitalWrite(L_D1, HIGH);
     digitalWrite(L_D2, LOW);
   }
 
   else if (DIRECTION == FORWARD)
   {
     digitalWrite(L_D1, LOW);
     digitalWrite(L_D2, HIGH);
   }
 }
 
 //RIGHT MOTOR
 else if (PIN == PWM_R)
 {
   COUNTER_LEFT_ENCODER.update(0);
   MOTOR_RIGHT_SETPOINT = SETPOINT;
   if (DIRECTION == BACKWARD)
   {
     digitalWrite(R_D1, HIGH);
     digitalWrite(R_D2, LOW);
   }
 
   else if (DIRECTION == FORWARD)
   {
     digitalWrite(R_D1, LOW);
     digitalWrite(R_D2, HIGH);
   }
 }
}
 
void receiveEvent() {
 LIMIT_SWITCH_TOP = Wire.read();
 LIMIT_SWITCH_BOTTOM = Wire.read();
 Serial.println(LIMIT_SWITCH_TOP);
 Serial.println(LIMIT_SWITCH_BOTTOM);
}
 
void setup()
{
 
 Serial.begin(9600);
 Serial1.begin(38400);
 
 //SET PINS
 pinMode(TRIGGER, OUTPUT);
 pinMode(ECHO_L, INPUT);
 
 pinMode(PWM_L, OUTPUT);
 pinMode(L_D1, OUTPUT);
 pinMode(L_D2, OUTPUT);
 
 pinMode(PWM_R, OUTPUT);
 pinMode(R_D1, OUTPUT);
 pinMode(R_D2, OUTPUT);
 
 pinMode(PWM_ROT, OUTPUT);
 pinMode(ROT_D1, OUTPUT);
 pinMode(ROT_D2, OUTPUT);
 //ATTACH INTERRUPTS
 
 // RIGHT
 attachInterrupt(digitalPinToInterrupt(REA), countFallingEdgesRight, FALLING);
 attachInterrupt(digitalPinToInterrupt(REB), countFallingEdgesRight, FALLING);
 attachInterrupt(digitalPinToInterrupt(REA), countFallingEdgesRight, RISING);
 attachInterrupt(digitalPinToInterrupt(REB), countFallingEdgesRight, RISING);
 
 //LEFT
 attachInterrupt(digitalPinToInterrupt(LEA), countFallingEdgesLeft, FALLING);
 attachInterrupt(digitalPinToInterrupt(LEB), countFallingEdgesLeft, FALLING);
 attachInterrupt(digitalPinToInterrupt(LEA), countFallingEdgesLeft, RISING);
 attachInterrupt(digitalPinToInterrupt(LEB), countFallingEdgesLeft, RISING);
 
 //ARM
 attachInterrupt(digitalPinToInterrupt(ROTEA), countFallingEdgesArm, FALLING);
 attachInterrupt(digitalPinToInterrupt(ROTEB), countFallingEdgesArm, FALLING);
 attachInterrupt(digitalPinToInterrupt(ROTEA), countFallingEdgesArm, RISING);
 attachInterrupt(digitalPinToInterrupt(ROTEB), countFallingEdgesArm, RISING);
  PID_RIGHT.SetMode(AUTOMATIC);
 PID_RIGHT.SetOutputLimits(0, 255);
 PID_LEFT.SetMode(AUTOMATIC);
 PID_LEFT.SetOutputLimits(0, 255);
 PID_ARM.SetMode(AUTOMATIC);
 PID_ARM.SetOutputLimits(-255, 255);
 
 //SET DIRECTION OF ARM
 SetMotor(PWM_L, FORWARD, 20);
 SetMotor(PWM_R, FORWARD, 20); 
 //SetMotor(PWM_ROT, DIRECTION_ARM, -100);
 
 calculateUltrasonicTimer.begin(calculateDistance, ULTRASONIC_SAMPLING_TIME);
 motorTimer.begin(updateWheelMotors, MOTOR_SAMPLING_TIME);
}
 
void loop()
{
 if (Serial1.available()) //wait for teensy 3.6
 {
   String TEENSY = Serial1.readString();
   LIMIT_SWITCH_TOP = (TEENSY.substring(0,1)).toInt();
   LIMIT_SWITCH_BOTTOM = TEENSY.substring(2,3).toInt();
   }
}
