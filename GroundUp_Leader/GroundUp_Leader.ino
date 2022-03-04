#include <PID_v1.h>
#include <Encoder.h>
#define LEFT_SIDE -1
#define RIGHT_SIDE 1
#define BACK_SIDE 0

#define DRIVE_MOTOR_L 1
#define DRIVE_MOTOR_R 2
#define ARM_MOTOR 3
#define INTAKE_MOTOR 4

IntervalTimer UltrasonicTimer;
/*MOTOR PINS*/

// CONVEYOR MOTOR
#define PWM_INT 11
#define INT_D1 18
#define INT_D2 12

// ARM MOTOR
#define PWM_ROT 8
#define ROT_D1 10
#define ROT_D2 9

// ARM MOTOR ENCODER
//CHECK
#define ROTEA 16
#define ROTEB 15

// LEFT WHEEL MOTOR
#define PWM_L 2
#define L_D1 4
#define L_D2 3

// LEFT WHEEL ENCODER
#define LEA 22
#define LEB 23

// RIGHT WHEEL MOTOR
#define PWM_R 7
#define R_D1 5
#define R_D2 6

// RIGHT WHEEL ENCODER
#define REA  17
#define REB  21

// ULTRASONIC PINS
#define TRIGGER 19 // attach pin D3 Arduino to pin Trig of HC-SR04
#define ECHO_L 20  // attach pin D2 Arduino to pin Echo of HC-SR04

// LIMIT PINS
// #define U_LIM_1
#define L_LIM 0 

// LINE SENSOR PINS
#define L_LINE 1
#define M_LINE 13
#define R_LINE 14


#define WALLFOLLOW_MAXTURN 20

Encoder ENCODER_R(REA, REB);
Encoder ENCODER_L(LEA, LEB);
Encoder ENCODER_ARM(ROTEA, ROTEB);

int COUNTER_L_OLD = 0;
int COUNTER_R_OLD = 0;
int COUNTER_TIMER_L_OLD = 0;
int COUNTER_TIMER_R_OLD = 0;

#define PULSES_PER_REVOLUTION 64
#define DRIVE_GEAR_RATIO 70

#define INTAKE_SPEED 255

#define MAX_DRIVE_SPEED_RPM 100

#define NINETY_DEG_TURN_COUNTS 4500

double LMotorV, RMotorV, LMotorDC, RMotorDC, LMotorVSet, RMotorVSet;

double LVKp = 2;
double LVKi = 6;
double LVKd = 0;

double RVKp = 2;
double RVKi = 6;
double RVKd = 0;

double LMotorPos, RMotorPos, LMotorDesV, RMotorDesV, LMotorPosSet, RMotorPosSet;
double ArmMotorPos, ArmMotorDC, ArmMotorPosSet;

double LPKp = 1;
double LPKi = 0;
double LPKd = 0;

double RPKp = 1;
double RPKi = 0;
double RPKd = 0;

double ArmPKp = 0.8;
double ArmPKi = 0.2;
double ArmPKd = 0.1;

double WallDist, TurnAmount, WallDistSet;
double WallKp = 0.15;
double WallKi = 0.000;
double WallKd = 0;

PID L_V_PID(&LMotorV, &LMotorDC, &LMotorVSet, LVKp, LVKi, LVKd, DIRECT);
PID R_V_PID(&RMotorV, &RMotorDC, &RMotorVSet, RVKp, RVKi, RVKd, DIRECT);

PID L_P_PID(&LMotorPos, &LMotorDesV, &LMotorPosSet, LPKp, LPKi, LPKd, DIRECT);
PID R_P_PID(&RMotorPos, &RMotorDesV, &RMotorPosSet, RPKp, RPKi, RPKd, DIRECT);
PID Arm_P_PID(&ArmMotorPos, &ArmMotorDC, &ArmMotorPosSet, ArmPKp, ArmPKi, ArmPKd, DIRECT);

PID Wall_PID(&WallDist, &TurnAmount, &WallDistSet, WallKp, WallKi, WallKd, DIRECT);

int RIGHT = 0;
int LEFT = 1;
int FRONT = 2;

int upperLim;
int lowerLim;
int Ultra_L;
int Ultra_R;
int Ultra_F;

double LMotorVel;
double RMotorVel;

IntervalTimer EncVelTimer;
#define ENC_SAMPLE_DUR 5000
#define ULTRASONIC_TIMER 10000

/*--------------Level 1 Functions--------------*/
// 1a Get Velocity
void calcVelocity() {
  int CounterL = ENCODER_L.read() - COUNTER_L_OLD;
  COUNTER_L_OLD = ENCODER_L.read();
  LMotorVel = ((CounterL * 60) / (PULSES_PER_REVOLUTION)) / (ENC_SAMPLE_DUR * (0.000001) * DRIVE_GEAR_RATIO);
  int CounterR = ENCODER_R.read() - COUNTER_R_OLD;
  COUNTER_R_OLD = ENCODER_R.read();
  RMotorVel = ((CounterR * 60) / (PULSES_PER_REVOLUTION)) / (ENC_SAMPLE_DUR * (0.000001) * DRIVE_GEAR_RATIO);
}


double getVelocity(int motor) {
  if (motor == DRIVE_MOTOR_L) {
    return LMotorVel;
  } else if (motor == DRIVE_MOTOR_R) {
    return RMotorVel;
  }
}

// 1b Get Encoder Counts
int getEncoderCounts(int motor) {
  int counts = 0;
  if (motor == DRIVE_MOTOR_L) {
    counts = ENCODER_L.read();
  } else if (motor == DRIVE_MOTOR_R) {
    counts = ENCODER_R.read();
  } else if (motor == ARM_MOTOR) {
    counts = ENCODER_ARM.read();
  }
  return counts;
}

// 1c Run Motor
void runMotor(int motor, int dutyCycle) {
  int pin1;
  int pin2;
  int pwmPin;
  int dir = 1;
  if (dutyCycle < 0) {
    dir = -1;
  }
  dutyCycle = abs(dutyCycle);

  if (motor == DRIVE_MOTOR_L) {
    pin1 = L_D1;
    pin2 = L_D2;
    pwmPin = PWM_L;
  } else if (motor == DRIVE_MOTOR_R) {
    pin1 = R_D1;
    pin2 = R_D2;
    pwmPin = PWM_R;
  } else if (motor == ARM_MOTOR) {
    pin1 = ROT_D1;
    pin2 = ROT_D2;
    pwmPin = PWM_ROT;
  } else if (motor == INTAKE_MOTOR) {
    pin1 = INT_D1;
    pin2 = INT_D2;
    pwmPin = PWM_INT;
  }

  if (dir == 1) {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
  } else if (dir == -1) {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
  }

  analogWrite(pwmPin, dutyCycle);
}

// 1d Read Ultra
// TBD
int SenseUltra(int side) {
  if (side == RIGHT) {
    return Ultra_R;
  }
  if (side == LEFT) {
    return Ultra_L;

  }
  if (side == FRONT) {
    return Ultra_F;
  }
}

// 1e read lower limit
bool LowerLimitPressed() {
  if (digitalRead(L_LIM) == HIGH){
    return true;
  } else {
    return false;
  }
}

bool UpperLimitPressed() {
  return false;
}

bool ZeroDriveEncoders() {
  ENCODER_L.write(0);
  ENCODER_R.write(0);
}

/*--------------Level 2 Functions--------------*/
// 2a Take
void Take(int dir) {
  runMotor(INTAKE_MOTOR, dir * INTAKE_SPEED);
}

// 2b Velocity PID
void VelPID(int motor, int velocity) {
  int dir = velocity / abs(velocity);
  if (motor == DRIVE_MOTOR_L) {
    LMotorV = abs(getVelocity(motor));
    LMotorVSet = abs(velocity);
    L_V_PID.Compute();
    runMotor(motor, dir * LMotorDC);
  } else if (motor == DRIVE_MOTOR_R) {
    RMotorV = abs(getVelocity(motor));
    RMotorVSet = abs(velocity);
    R_V_PID.Compute();
    runMotor(motor, dir * RMotorDC);
    //Serial.print(RMotorV); Serial.print(", "); Serial.println(RMotorDC);
  }
}

/*--------------Level 3 Functions--------------*/
// 3a Drive straight
/*
  void PosPID(int motor, int pos){
  if (motor == DRIVE_MOTOR_L){
    LMotorPos = getEncoderCounts(motor);
    L_P_PID.Compute();
    VelPID(motor, LMotorDesV);
  } else if (motor == DRIVE_MOTOR_R){
    RMotorPos = getEncoderCounts(motor);
    R_P_PID.Compute();
    VelPID(motor, RMotorDesV);
  } else if (motor == ARM_MOTOR){
    ArmMotorPos = getEncoderCounts(motor);
    Arm_P_PID.Compute();
    runMotor(motor, ArmMotorDC);
  }
  }
*/

bool DriveToPos(int LeftDist, int RightDist, int LeftVelocity, int RightVelocity) { // direction in the sign of velocity; sign of dist does nothing
  bool LeftDone = false;
  bool RightDone = false;
  if (abs(getEncoderCounts(DRIVE_MOTOR_L)) < abs(LeftDist)) {
    VelPID(DRIVE_MOTOR_L, LeftVelocity);
  } else {
    runMotor(DRIVE_MOTOR_L, 0);
    LeftDone = true;
  }
  if (abs(getEncoderCounts(DRIVE_MOTOR_R)) < abs(RightDist)) {
    VelPID(DRIVE_MOTOR_R, RightVelocity);
  } else {
    runMotor(DRIVE_MOTOR_R, 0);
    RightDone = true;
  }
  if (LeftDone && RightDone) {
    return true;
  } else {
    return false;
  }
}

void ArmPID(int pos) {
  ArmMotorPos = getEncoderCounts(ARM_MOTOR);
  ArmMotorPosSet = pos;
  Arm_P_PID.Compute();
  if ((ArmMotorDC < 0 && !(LowerLimitPressed())) || (ArmMotorDC > 0 && !(UpperLimitPressed()))) {
    runMotor(ARM_MOTOR, ArmMotorDC);
  }
  Serial.println(ArmMotorDC);
}

// 3b Follow Wall
void FollowWall(int side, int AvgVel, int WallDistSetPoint) {
  Serial.println("Ultra_L");
  Serial.println(Ultra_L);
  Serial.println("WallDistSetPoint");
  Serial.println(WallDistSetPoint);
  //Serial.println("WallDist");
  //  Serial.println(WallDist);
  //  Serial.println(Ultra_L);
  //  Serial.println(Ultra_R);
  WallDistSet = WallDistSetPoint;
  Wall_PID.Compute();
  Serial.println("TurnAmount");
  Serial.println(TurnAmount);
  int TurnAmountGeneral = TurnAmount * side;
  int LVel = AvgVel + TurnAmount;
  int RVel = AvgVel;
  VelPID(DRIVE_MOTOR_L, LVel);
  VelPID(DRIVE_MOTOR_R, RVel);
}

/*--------------Level 4 Functions--------------*/
// 4a Drive Straight
bool DriveStraight(int dist, int velocity) {
  if (DriveToPos(dist, dist, velocity, velocity)) {
    return true;
  } else {
    return false;
  }
}
// 4b Turn Theta
bool RotateTheta(int theta, int velocity) {
  int dist = NINETY_DEG_TURN_COUNTS * theta / 90;
  if (theta < 0) {
    velocity = -1 * velocity;
  }
  if (DriveToPos(dist, dist, velocity, -velocity)) {
    return true;
  } else {
    return false;
  }
}


typedef enum {
  HOMING,INTAKE, STOP_LOADING,
  REVERSE_ON_LINE, LINE_FOLLOWING,
  ROTATE_TOWARD_WALL,
  MOVE_TOWARD_WALL, ROTATE_TOWARD_GOAL,
  MOVE_TOWARD_GOAL, REACHED_GOAL, 
  RAISE_ARM, OUTTAKE,FINAL_HOME,
  COMPLETE
} STATE_MACHINE;

STATE_MACHINE state;

void homeArm(){
  if (!LowerLimitPressed()){
    runMotor(ARM_MOTOR, -200);
  } else {
    runMotor(ARM_MOTOR, 0);
    ENCODER_ARM.write(0);
    state = INTAKE;
  }
}

void intake(){
  Take(1);
  if (DriveToPos(3700, 3700, -20, -20)){
    ZeroDriveEncoders();
    state = STOP_LOADING;
  }
}

void stopLoading(){
  Take(0);
  if (DriveToPos(3300, 3300, 20, 20)){
    ZeroDriveEncoders();
    state = REVERSE_ON_LINE;
  }
}

void reverseOnLine() {
  if (DriveToPos(26500, 26500, 30, 30)) {
    ZeroDriveEncoders();
    state = ROTATE_TOWARD_WALL;
  }
}
void rotateTowardWall() {
  if (DriveToPos(4000, 4000, -20, 20)) {
    ZeroDriveEncoders();
    state = MOVE_TOWARD_WALL;
  }
}
void moveTowardWall() {
  if (DriveToPos(15000, 15000, 20, 20)) {
    ZeroDriveEncoders();
    state = ROTATE_TOWARD_GOAL;
  }
}
void rotateTowardGoal() {
  if (DriveToPos(4000, 4000, 20, -20)) {
    ZeroDriveEncoders();
    state = MOVE_TOWARD_GOAL;
  }
}
void moveTowardGoal() {
  if (DriveToPos(12000, 12000, 30, 30)) {
    ZeroDriveEncoders();
    state = REACHED_GOAL;
  }
}

void reachedGoal() {
    if (DriveToPos(100, 100, -20, -20)) {
    ZeroDriveEncoders();
    state = RAISE_ARM;
  }
}

void raiseArm(){
  int targetPos = 3800;
  int error = 50;
  if (!(abs(getEncoderCounts(ARM_MOTOR) - targetPos) <= error)){
    ArmPID(targetPos);
  } else {
    runMotor(ARM_MOTOR, 0);
    state = OUTTAKE;
  }
}

void outtake(){
  Take(-1);
  delay(5000);
  Take(0);
  state = FINAL_HOME;
}

void finalHome(){
    if (!LowerLimitPressed()){
    runMotor(ARM_MOTOR, -200);
  } else {
    runMotor(ARM_MOTOR, 0);
    ENCODER_ARM.write(0);
    state = COMPLETE;
  }
}

void ultrasonicTimerPin(void) {
//  digitalWrite(TRIGGER, LOW);
//  // Reads the echoPin, returns the sound wave travel time in microseconds
//  int duration = pulseIn(ECHO_L, HIGH);
//  digitalWrite(TRIGGER, HIGH);
//  // Calculating the distance
//  Ultra_L = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
}

bool followLine(int lineVelocity, int turnVelocity){
  bool middleLine = digitalRead(M_LINE);
  bool leftLine = digitalRead(L_LINE);
  bool rightLine = digitalRead(R_LINE);
  int leftVelocity;
  int rightVelocity;
  if (middleLine && !(leftLine) && !(rightLine)){
    leftVelocity = lineVelocity;
    rightVelocity = lineVelocity;
  }else if(leftLine && !(middleLine) && !(rightLine)){
    leftVelocity = lineVelocity - turnVelocity;
    rightVelocity = lineVelocity + turnVelocity;
  }else if(rightLine && !(middleLine) && !(leftLine)){
    leftVelocity = lineVelocity + turnVelocity;
    rightVelocity = lineVelocity - turnVelocity;
  }else if(leftLine && (middleLine) && !(rightLine)){
    leftVelocity = lineVelocity - turnVelocity/2;
    rightVelocity = lineVelocity + turnVelocity/2;
  }else if(rightLine && (middleLine) && !(leftLine)){
    leftVelocity = lineVelocity + turnVelocity/2;
    rightVelocity = lineVelocity - turnVelocity/2;
  }else if(middleLine && (leftLine || rightLine)){
    leftVelocity = 0;
    rightVelocity = 0;
    runMotor(DRIVE_MOTOR_L, 0);
    runMotor(DRIVE_MOTOR_R, 0);
    //return true;
  }else{
    leftVelocity = -turnVelocity;
    rightVelocity = turnVelocity;
  }
  VelPID(DRIVE_MOTOR_L, leftVelocity);
  VelPID(DRIVE_MOTOR_R, rightVelocity);
  //Serial.print(leftLine); Serial.print(middleLine); Serial.print(rightLine); Serial.println();
  //delay(10);
  if(millis() % 50 == 0){
    Serial.print(leftVelocity); Serial.print(" "); Serial.println(rightVelocity);
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);

  EncVelTimer.begin(calcVelocity, ENC_SAMPLE_DUR);
  UltrasonicTimer.begin(ultrasonicTimerPin, 10000);
  pinMode(PWM_L, OUTPUT);
  pinMode(L_D1, OUTPUT);
  pinMode(L_D2, OUTPUT);

  pinMode(PWM_R, OUTPUT);
  pinMode(R_D1, OUTPUT);
  pinMode(R_D2, OUTPUT);

  pinMode(PWM_ROT, OUTPUT);
  pinMode(ROT_D1, OUTPUT);
  pinMode(ROT_D2, OUTPUT);

  pinMode(L_LIM, INPUT_PULLUP);

  pinMode(TRIGGER, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(ECHO_L, INPUT); // Sets the echoPin as an INPUT

  pinMode(L_LINE, INPUT);

  L_V_PID.SetMode(AUTOMATIC);
  L_V_PID.SetOutputLimits(0, 255);
  L_V_PID.SetSampleTime(50);
  R_V_PID.SetMode(AUTOMATIC);
  R_V_PID.SetOutputLimits(0, 255);
  R_V_PID.SetSampleTime(50);

  L_P_PID.SetMode(AUTOMATIC);
  L_P_PID.SetOutputLimits(-MAX_DRIVE_SPEED_RPM, MAX_DRIVE_SPEED_RPM);
  R_P_PID.SetMode(AUTOMATIC);
  R_P_PID.SetOutputLimits(-MAX_DRIVE_SPEED_RPM, MAX_DRIVE_SPEED_RPM);
  Arm_P_PID.SetMode(AUTOMATIC);
  Arm_P_PID.SetOutputLimits(-255, 255);
  Arm_P_PID.SetSampleTime(50);

  Wall_PID.SetMode(AUTOMATIC);
  Wall_PID.SetOutputLimits(-WALLFOLLOW_MAXTURN, WALLFOLLOW_MAXTURN);
  ZeroDriveEncoders();
  state = HOMING;
  state = LINE_FOLLOWING;
}



void loop() {
    switch (state) {
      case HOMING:
        homeArm();
        break;
      case INTAKE:
        intake();
        break;
      case STOP_LOADING:
        stopLoading();
        break;
      case REVERSE_ON_LINE:
        reverseOnLine();
        break;
      case ROTATE_TOWARD_WALL:
        rotateTowardWall();
        break;
      case MOVE_TOWARD_WALL:
        moveTowardWall();
        break;
      case ROTATE_TOWARD_GOAL:
        rotateTowardGoal();
        break;
      case MOVE_TOWARD_GOAL:
        moveTowardGoal();
        break;
      case REACHED_GOAL:
        reachedGoal();
        break;
      case RAISE_ARM:
        raiseArm();
        break;
      case OUTTAKE:
        outtake();
        break;
      case FINAL_HOME:
        finalHome();
        break;
      case COMPLETE:
        break;
      case LINE_FOLLOWING:
        followLine(-30, -10);
        break;
      default:    // Should never get into an unhandled state
        Serial.println("What is this I do not even...");
        }

}

//    char buff[50];
//    String TEENSY = Serial1.read();
//    TEENSY.toCharArray(buff, 50);
//    Serial.println(TEENSY);
//    char* token = strtok(buff, ",");
//    int Ultra_L = token[0];
//    Serial.println(TEENSY);
//    int Ultra_R = token[0];
