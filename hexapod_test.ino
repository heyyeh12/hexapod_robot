#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // default address (0x40)

/* Pulse width min and max */
#define SERVOMIN  128 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  551 // this is the 'maximum' pulse length count (out of 4096)
#define FREQ      60  // runtime frequency

/*        FRONT
 *  6   /--000--\    1
 *  5 /--0000000--\  2
 *  4   /--000--\    3
 */
 
/* Servo pin numbers - see above diagram for what HIP/KNEE #1-6 refers to */
#define HIP1       5  
#define KNEE1      7  //
#define HIP2       4
#define KNEE2      13 //
#define HIP3       3
#define KNEE3      15 //
#define HIP4       1
#define KNEE4      4  //
#define HIP5       2
#define KNEE5      11 //
#define HIP6       0
#define KNEE6      12 //

/* Positions - in degrees */
#define DOWN1      100
#define UP1        0
#define FORWARD1   70
#define BACKWARD1  110
#define DOWN2      100
#define UP2        0
#define FORWARD2   70
#define BACKWARD2  110
#define DOWN3      100
#define UP3        0
#define FORWARD3   70
#define BACKWARD3  110
#define DOWN4      100
#define UP4        0
#define FORWARD4   110
#define BACKWARD4  70
#define DOWN5      100
#define UP5        0
#define FORWARD5   110
#define BACKWARD5  70
#define DOWN6      100
#define UP6        0
#define FORWARD6   110
#define BACKWARD6  70

/* FUNCTION PROTOTYPES (so order of function definitions doesn't matter) */
void sweepServo(uint8_t n, uint16_t start_ang, uint16_t end_ang, double t, int wait = 0);
void setServo(uint8_t n, uint16_t deg, int wait = 0);
void setServos(uint8_t servos[], uint16_t poses[], int n, int wait = 0);
void initializeServos(int servos, uint16_t angle);

void circleLegWalk();
void Twist();
void oneLegWalk(uint8_t hip, uint8_t knee, uint16_t forward, uint16_t backward, uint16_t up, uint16_t down, uint8_t wait = 1000);
void legWalk();

/* SETUP LOOP */
void setup() {
  Serial.begin(9600);
  Serial.println("Hexipod Servo test!");
  pwm.begin();
  pwm.setPWMFreq(FREQ);  // Analog servos run at ~60 Hz updates
  /* Initial position */
  // setAllServos(90);
}

/*************
 * MAIN LOOP *
 *************/
void loop() {

/*Set to 90*/
setAllServos(90);
/********************/

/* CIRCLE LEG WALK */
//circleLegWalk();
/********************/  

/* TWIST */
//Twist();
/********************/
  
/* LEG WALK */
// legWalk();
// oneLegWalk(HIP1, KNEE1, FORWARD1, BACKWARD1, UP1, DOWN1, 1000);
/*******************/

}

/*****************
 * "DANCE" MOVES *
 *****************/
void circleLegWalk() {
  oneLegWalk(HIP1, KNEE1, FORWARD1, BACKWARD1, UP1, DOWN1);
  oneLegWalk(HIP2, KNEE2, FORWARD2, BACKWARD2, UP2, DOWN2);
  oneLegWalk(HIP3, KNEE3, FORWARD3, BACKWARD3, UP3, DOWN3);
  oneLegWalk(HIP4, KNEE4, BACKWARD4, FORWARD4, UP4, DOWN4);
  oneLegWalk(HIP5, KNEE5, BACKWARD5, FORWARD5, UP5, DOWN5);
  oneLegWalk(HIP6, KNEE6, BACKWARD6, FORWARD6, UP6, DOWN6);
}
 
/* Twist middle with stationary legs */
void Twist() {
  sweepServo(HIP1, FORWARD1, BACKWARD1, 2);
  sweepServo(HIP2, FORWARD2, BACKWARD2, 2);
  sweepServo(HIP3, FORWARD3, BACKWARD3, 2);
  
  sweepServo(HIP4, BACKWARD4, FORWARD4, 2);
  sweepServo(HIP5, BACKWARD5, FORWARD5, 2);
  sweepServo(HIP6, BACKWARD6, FORWARD6, 2);
  
  delay(3000);
  
  sweepServo(HIP1, BACKWARD1, FORWARD1, 2);
  sweepServo(HIP2, BACKWARD2, FORWARD2, 2);
  sweepServo(HIP3, BACKWARD3, FORWARD3, 2);

  sweepServo(HIP4, FORWARD4, BACKWARD4, 2);
  sweepServo(HIP5, FORWARD5, BACKWARD5, 2);
  sweepServo(HIP6, FORWARD6, BACKWARD6, 2);
  

  
  delay(3000);
}
 
/* Untested: Walk one leg */
void oneLegWalk(uint8_t hip, uint8_t knee, uint16_t forward, uint16_t backward, uint16_t up, uint16_t down, uint8_t wait)
{
  /* 1) sweep back */
  setServo(hip, backward);
  delay(wait);
  /* 2) leg up */
  setServo(knee, up);
  delay(wait);
  /* 3) sweep forward */
  setServo(hip, forward);
  delay(wait);
  /* 4) leg down */
  setServo(knee, down);
  delay(wait); 
}

/* Uncomment/modify below to specify which leg */
void legWalk()
{
  /* 1) sweep back */
  setServo(HIP1, BACKWARD1);
//  setServo(HIP3, BACKWARD3);
//  setServo(HIP5, BACKWARD5);
//  delay(1000);
/* 2) leg up */
  setServo(KNEE1, UP1);
//  setServo(KNEE3, UP3);
//  setServo(KNEE5, UP5);
//  delay(1000);
/* 3) sweep forward */
  setServo(HIP1, FORWARD1);
//  setServo(HIP3, FORWARD3);
//  setServo(HIP5, FORWARD5);
//  delay(1000);
/* 4) leg down */
  setServo(KNEE1, DOWN1);
//  setServo(KNEE3, DOWN3);
//  setServo(KNEE5, DOWN5);
//  delay(1000);
}

/**************************
 * USEFUL SERVO FUNCTIONS *
 **************************/
 
/* Set servo n to specified angle (in degrees), with optional wait (in milliseconds) after setting */
void setServo(uint8_t n, uint16_t deg, int wait)
{
  Serial.print(n); Serial.print(" ");Serial.print(deg); Serial.println(" deg");
  uint16_t pulse = map(deg, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(n, 0, pulse);
  delay(wait);
}

/* Set all servos to angle (in degrees) */
void setAllServos(uint16_t angle)
{
  for (int i = 0; i < 15; i++) {
    setServo(i, angle);
  }
}

/* Sweep from start_ang to end_ang in t seconds, with optional wait (in milliseconds) after the sweep */
void sweepServo(uint8_t n, uint16_t start_ang, uint16_t end_ang, double t, int wait)
{
   uint16_t start_pulse = map(start_ang, 0, 180, SERVOMIN, SERVOMAX);
   uint16_t end_pulse = map(end_ang, 0, 180, SERVOMIN, SERVOMAX);
   Serial.print("start:"); Serial.print(start_ang); Serial.print(" "); Serial.println(start_pulse);
   Serial.print("end:"); Serial.print(end_ang); Serial.print(" "); Serial.println(end_pulse);
   uint16_t p;
   double dt; 
   if (start_pulse < end_pulse) {
    for (p = start_pulse; p < end_pulse; p++) {
      dt = t*1000/(end_pulse-start_pulse); // msec per change in pulse
      Serial.print(p); Serial.print(" ");
      pwm.setPWM(n, 0, p);
      delay(dt);
    }
   } else {
    for (p = start_pulse; p > end_pulse; p--) {
      dt = t*1000/(start_pulse-end_pulse); // msec per change in pulse
      Serial.print(p); Serial.print(" ");
      pwm.setPWM(n, 0, p);
      delay(dt);
    }
  }
  Serial.print("dt:"); Serial.println(dt);
  Serial.println(" Finished sweep!");
  delay(wait);
}

/* Untested: Sweep array of "number" servos all at once - probably will have power issues */
void sweepServos(uint8_t n[], uint16_t start_ang, uint16_t end_ang, double t, double number, int wait)
{
  uint16_t start_pulse = map(start_ang, 0, 180, SERVOMIN, SERVOMAX);
  uint16_t end_pulse= map(end_ang, 0, 180, SERVOMIN, SERVOMAX);

   uint16_t p;
   double dt = t*1000/abs(start_pulse-end_pulse); // msec per change in pulse
   Serial.print("dt:"); Serial.println(dt);
   if (start_pulse < end_pulse) {
    for (p = start_pulse; p < end_pulse; p++) {
      Serial.print(p); Serial.print(" ");
      for (int j = 0; j < number; j++) {
        pwm.setPWM(n[j], 0, p);
      }
      delay(dt);
    }
   } else {
    for (p = start_pulse; p > end_pulse; p--) {
      Serial.print(p); Serial.print(" ");
      for (int j = 0; j < number; j++) {
        pwm.setPWM(n[j], 0, p);
      }
      delay(dt);
    }
  }
  Serial.println(" Finished sweep!");
  delay(wait);
}

/* Untested: Set n servos using setServo() function */
void setServos(uint8_t servos[], uint16_t poses[], int n, int wait)
{
  for(int i = 0; i < n; i++) {
    setServo(servos[i], poses[i]);
  }
  delay(wait);
}

