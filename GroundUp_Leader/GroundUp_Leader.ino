#include <PID_v1.h>

#include <Encoder.h>

/*MOTOR PINS*/

#define DRIVE_MOTOR_L 1
#define DRIVE_MOTOR_R 2
#define ARM_MOTOR 3
#define INTAKE_MOTOR 4
 
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
#define LEA 21
#define LEB 17
 
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


Encoder ENCODER_R(REA,REB);
Encoder ENCODER_L(LEA,LEB);
Encoder ENCODER_ARM(ROTEA,ROTEB);

int COUNTER_L_OLD = 0;
int COUNTER_R_OLD = 0;
int COUNTER_TIMER_L_OLD = 0;
int COUNTER_TIMER_R_OLD = 0;

#define PULSES_PER_REVOLUTION 64
#define DRIVE_GEAR_RATIO 70

#define INTAKE_SPEED 255

#define MAX_DRIVE_SPEED_RPM 100

double LMotorV, RMotorV, LMotorDC, RMotorDC, LMotorVSet, RMotorVSet;

double LVKp = 10;
double LVKi = 5;
double LVKd = 0;

double RVKp = 10;
double RVKi = 5;
double RVKd = 0;

double LMotorPos, RMotorPos, LMotorDesV, RMotorDesV, LMotorPosSet, RMotorPosSet;
double ArmMotorPos, ArmMotorDC, ArmMotorPosSet;

double LPKp = 1;
double LPKi = 0;
double LPKd = 0;

double RPKp = 1;
double RPKi = 0;
double RPKd = 0;

double ArmPKp = 0.5;
double ArmPKi = 0;
double ArmPKd = 0.1;

PID L_V_PID(&LMotorV, &LMotorDC, &LMotorVSet, LVKp, LVKi, LVKd, DIRECT);
PID R_V_PID(&RMotorV, &RMotorDC, &RMotorVSet, RVKp, RVKi, RVKd, DIRECT);

PID L_P_PID(&LMotorPos, &LMotorDesV, &LMotorPosSet, LPKp, LPKi, LPKd, DIRECT);
PID R_P_PID(&RMotorPos, &RMotorDesV, &RMotorPosSet, RPKp, RPKi, RPKd, DIRECT);
PID Arm_P_PID(&ArmMotorPos, &ArmMotorDC, &ArmMotorPosSet, ArmPKp, ArmPKi, ArmPKd, DIRECT);

/*--------------Level 1 Functions--------------*/
// 1a Get Velocity
float getVelocity(int motor){
  float RPM = 0;
  if (motor == DRIVE_MOTOR_L){
    int CounterL = ENCODER_L.read()-COUNTER_L_OLD;
    int CounterTimerL = millis()-COUNTER_TIMER_L_OLD;
    COUNTER_TIMER_L_OLD = millis();
    COUNTER_L_OLD = ENCODER_L.read();
    RPM = ((CounterL * 60) / (PULSES_PER_REVOLUTION)) / (CounterTimerL * (1/1000) * DRIVE_GEAR_RATIO);
  } else if (motor == DRIVE_MOTOR_R){
    int CounterR = ENCODER_R.read()-COUNTER_R_OLD;
    int CounterTimerR = millis()-COUNTER_TIMER_R_OLD;
    COUNTER_TIMER_R_OLD = millis();
    COUNTER_R_OLD = ENCODER_R.read();
    RPM = ((CounterR * 60) / (PULSES_PER_REVOLUTION)) / (CounterTimerR * (1/1000) * DRIVE_GEAR_RATIO);
  }
  return RPM;
}

// 1b Get Encoder Counts
int getEncoderCounts(int motor){
  int counts = 0;
  if (motor == DRIVE_MOTOR_L){
    counts = ENCODER_L.read();
  } else if (motor == DRIVE_MOTOR_R){
    counts = ENCODER_R.read();
  } else if (motor == ARM_MOTOR){
    counts = ENCODER_ARM.read();
  }
  return counts;
}

// 1c Run Motor
void runMotor(int motor, int dutyCycle){
  int pin1;
  int pin2;
  int pwmPin;
  int dir = 1;
  if (dutyCycle < 0){
    dir = -1;
  }
  dutyCycle = abs(dutyCycle);
  
  if (motor == DRIVE_MOTOR_L){
    pin1 = L_D1;
    pin2 = L_D2;
    pwmPin = PWM_L;    
  } else if (motor == DRIVE_MOTOR_R){
    pin1 = R_D1;
    pin2 = R_D2;
    pwmPin = PWM_R;    
  } else if (motor == ARM_MOTOR){
    pin1 = ROT_D1;
    pin2 = ROT_D2;
    pwmPin = PWM_ROT;    
  } else if (motor == INTAKE_MOTOR){
    pin1 = INT_D1;
    pin2 = INT_D2;
    pwmPin = PWM_INT;    
  }

  if (dir == 1){
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
  } else if (dir == -1){
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
  }

  analogWrite(pwmPin, dutyCycle);
}

// 1d Read Ultra
// TBD


/*--------------Level 2 Functions--------------*/
// 2a Take
void Take(int dir){
  runMotor(INTAKE_MOTOR, dir * INTAKE_SPEED);
}

// 2b Velocity PID
void VelPID(int motor, int velocity){
  if (motor == DRIVE_MOTOR_L){
    LMotorV = getVelocity(motor);
    L_V_PID.Compute();
    runMotor(motor, LMotorDC);
  } else if (motor == DRIVE_MOTOR_R){
    RMotorV = getVelocity(motor);
    R_V_PID.Compute();
    runMotor(motor, RMotorDC);
  }
}

/*--------------Level 3 Functions--------------*/
// 3a Position PID
void PosPID(int motor, int velocity, int pos){
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

void setup() {
  // put your setup code here, to run once:
  L_V_PID.SetMode(AUTOMATIC);
  L_V_PID.SetOutputLimits(-255,255);
  R_V_PID.SetMode(AUTOMATIC);
  R_V_PID.SetOutputLimits(-255,255);

  L_P_PID.SetMode(AUTOMATIC);
  L_P_PID.SetOutputLimits(-MAX_DRIVE_SPEED_RPM, MAX_DRIVE_SPEED_RPM);
  R_P_PID.SetMode(AUTOMATIC);
  R_P_PID.SetOutputLimits(-MAX_DRIVE_SPEED_RPM, MAX_DRIVE_SPEED_RPM);
  Arm_P_PID.SetMode(AUTOMATIC);
  Arm_P_PID.SetOutputLimits(-MAX_DRIVE_SPEED_RPM, MAX_DRIVE_SPEED_RPM);
  
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
