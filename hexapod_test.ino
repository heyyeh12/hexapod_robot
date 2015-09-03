#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // default address (0x40)

/* Pulse width min and max */
#define SERVOMIN  128 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  551 // this is the 'maximum' pulse length count (out of 4096)
#define SERVODIF  427 // difference between two (for reference)
#define FREQ      60  // runtime frequency

/*        FRONT
 *  6   /--000--\    1
 *  5 /--0000000--\  2
 *  4   /--000--\    3
 */
 
/* Servo specifications */
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

/* Positions */
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

double sweepTime;

/* FUNCTION PROTOTYPES (so order doesn't matter) */
void sweepServo(uint8_t n, uint16_t start_ang, uint16_t end_ang, double t, int wait = 0);
void setServo(uint8_t n, uint16_t deg, int wait = 0);
void setServos(uint8_t servos[], uint16_t poses[], int n, int wait = 0);
void oneLegWalk(uint8_t hip, uint8_t knee, uint16_t forward, uint16_t backward, uint16_t up, uint16_t down, uint8_t wait = 1000);
void initializeServos(int servos, uint16_t angle);

/* SETUP LOOP */
void setup() {
  Serial.begin(9600);
  Serial.println("Hexipod Servo test!");
  pwm.begin();
  pwm.setPWMFreq(FREQ);  // Analog servos run at ~60 Hz updates
  /* Initial position */
//  setServo(0, 90);
//  setServo(1, 90);
//  setServo(2, 90);
//  setServo(3, 90);
//  setServo(4, 90);
//  setServo(5, 90);
//  setServo(6, 90);
//  setServo(7, 90);
//  setServo(8, 90);
//  setServo(9, 90);
//  setServo(10, 90);
//  setServo(11, 90);
//  setServo(12, 90);
//  setServo(13, 90);
//  setServo(14, 90);
//  setServo(15, 90);

 sweepTime = 1;
}

void loop() {

//uint8_t hips[] = {HIP1, HIP2, HIP3, HIP4, HIP5, HIP6};
//sweepServos(hips, FORWARD1, BACKWARD1, 1, 6, 2000);
//sweepServos(hips, BACKWARD1,FORWARD1, 2, 6, 2000);


/* TWIST */
//sweepServo(HIP1, FORWARD1, BACKWARD1, 2);
//sweepServo(HIP2, FORWARD2, BACKWARD2, 2);
//sweepServo(HIP3, FORWARD3, BACKWARD3, 2);
//
//sweepServo(HIP4, FORWARD4, BACKWARD4, 2);
//sweepServo(HIP5, FORWARD5, BACKWARD5, 2);
//sweepServo(HIP6, FORWARD6, BACKWARD6, 2);
//
//delay(3000);
//
//sweepServo(HIP1, BACKWARD1, FORWARD1, 2);
//sweepServo(HIP2, BACKWARD2, FORWARD2, 2);
//sweepServo(HIP3, BACKWARD3, FORWARD3, 2);
//
//sweepServo(HIP4, BACKWARD4, FORWARD4, 2);
//sweepServo(HIP5, BACKWARD5, FORWARD5, 2);
//sweepServo(HIP6, BACKWARD6, FORWARD6, 2);
//
//delay(3000);
/********************/

/* CIRCLE LEG WALK */
//oneLegWalk(HIP1, KNEE1, FORWARD1, BACKWARD1, UP1, DOWN1);
//oneLegWalk(HIP2, KNEE2, FORWARD2, BACKWARD2, UP2, DOWN2);
//oneLegWalk(HIP3, KNEE3, FORWARD3, BACKWARD3, UP3, DOWN3);
//oneLegWalk(HIP4, KNEE4, BACKWARD4, FORWARD4, UP4, DOWN4);
//oneLegWalk(HIP5, KNEE5, BACKWARD5, FORWARD5, UP5, DOWN5);
//oneLegWalk(HIP6, KNEE6, BACKWARD6, FORWARD6, UP6, DOWN6);
/********************/  

//  setServo(0, 0);
//  setServo(1, 0);
//  setServo(2, 0);
//  setServo(3, 0);
//  setServo(6, 0);
//  setServo(7, 0);
//  setServo(4, 0);
//  setServo(5, 0);
//  setServo(8, 0);
//  setServo(9, 0);
//  setServo(10, 0);
//  setServo(11, 0);
//  setServo(12, 0);
//  setServo(13, 0);
//  setServo(14, 0);
//  setServo(15, 0, 1000);

/*Set to 90*/
//  setServo(0, 90);
//  setServo(1, 90);
//  setServo(2, 90);
//  setServo(3, 90);  
//  setServo(6, 90);
//  setServo(7, 90);
//  setServo(4, 90);
//  setServo(5, 90);
//  setServo(8, 90);
//  setServo(9, 90);
//  setServo(10, 90);
//  setServo(11, 90);
//  setServo(12, 90);
//  setServo(13, 90);
//  setServo(14, 90);
//  setServo(15, 90, 3000);
/************/ 
//  setServo(0, 110);
//  setServo(1, 110);
//  setServo(2, 110);
//  setServo(3, 110);  
//  setServo(6, 110);
//  setServo(7, 110);
//  setServo(4, 110);
//  setServo(5, 110);
//  setServo(8, 110);
//  setServo(9, 110);
//  setServo(10, 110);
//  setServo(11, 110);
//  setServo(12, 110);
//  setServo(13, 110);
//  setServo(14, 110);
//  setServo(15, 110, 3000);
  
/* LEG WALK */
/* 1) sweep back */
//  setServo(HIP1, BACKWARD1);
//  setServo(HIP3, BACKWARD3);
//  setServo(HIP5, BACKWARD5);
//  delay(1000);
/* 2) leg up */
//  setServo(KNEE1, UP1);
//  setServo(KNEE3, UP3);
//  setServo(KNEE5, UP5);
//  delay(1000);
/* 3) sweep forward */
//  setServo(HIP1, FORWARD1);
//  setServo(HIP3, FORWARD3);
//  setServo(HIP5, FORWARD5);
//  delay(1000);
/* 4) leg down */
//  setServo(KNEE1, DOWN1);
//  setServo(KNEE3, DOWN3);
//  setServo(KNEE5, DOWN5);
//  delay(1000);
/************/
}
/*****************
 * "DANCE" MOVES *
 *****************/
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

/********************
 * USEFUL FUNCTIONS *
 ********************/
 
/* Set servo n to specified angle (in degrees), with optional wait (in milliseconds) */
void setServo(uint8_t n, uint16_t deg, int wait)
{
  Serial.print(n); Serial.print(" ");Serial.print(deg); Serial.println(" deg");
  uint16_t pulse = map(deg, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(n, 0, pulse);
  delay(wait);
}

/* Useful to initialize X number of servos to angle */
void setAllServos(uint16_t angle)
{
  for (int i = 0; i < 15; i++) {
    setServo(i, angle);
  }
}

/* Sweep from start_ang to end_ang in t seconds, with optional wait (in milliseconds) */
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

