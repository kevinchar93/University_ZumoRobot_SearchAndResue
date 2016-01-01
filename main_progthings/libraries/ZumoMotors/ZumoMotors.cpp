#include <Energia.h>

#include "ZumoMotors.h"

#define REDBEAR 1

#ifdef REDBEAR
// redbear cc3200 (PWM only available on pins 5 and 6)
#define PWM_L 6
#define PWM_R 5 
#else
#define PWM_L 10
#define PWM_R 9
#endif

#define DIR_L 8
#define DIR_R 7

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined (__AVR_ATmega32U4__)
  #define USE_20KHZ_PWM
#endif
#undef USE_20KHZ_PWM // redbear cc3200 (need portable, non-AVR, code)

static boolean flipLeft = false;
static boolean flipRight = false;

// constructor (doesn't do anything)
ZumoMotors::ZumoMotors()
{
}

// initialize timer1 to generate the proper PWM outputs to the motor drivers
void ZumoMotors::init2()
{
  pinMode(PWM_L,  OUTPUT);
  pinMode(PWM_R,  OUTPUT);
  pinMode(DIR_L, OUTPUT);
  pinMode(DIR_R, OUTPUT);
#ifdef REDBEAR
  /* redbear pins 5 & 6 are connected to pins 9 & 10, so we these to be input
   * so they don't inadvertently drive
   */
  pinMode(10, INPUT);    
  pinMode(9, INPUT);
#endif

#ifdef USE_20KHZ_PWM
  // Timer 1 configuration
  // prescaler: clockI/O / 1
  // outputs enabled
  // phase-correct PWM
  // top of 400
  //
  // PWM frequency calculation
  // 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20kHz
  TCCR1A = 0b10100000;
  TCCR1B = 0b00010001;
  ICR1 = 400;
#endif
}

// enable/disable flipping of left motor
void ZumoMotors::flipLeftMotor(boolean flip)
{
  flipLeft = flip;
}

// enable/disable flipping of right motor
void ZumoMotors::flipRightMotor(boolean flip)
{
  flipRight = flip;
}

// set speed for left motor; speed is a number between -400 and 400
void ZumoMotors::setLeftSpeed(int speed)
{
  init(); // initialize if necessary
  boolean reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed; // make speed a positive quantity
    reverse = 1;    // preserve the direction
  }
  if (speed > 400)  // Max 
    speed = 400;
    
#ifdef USE_20KHZ_PWM
  OCR1B = speed;
#else
  analogWrite(PWM_L, speed * 51 / 80); // default to using analogWrite, mapping 400 to 255
#endif 

  if (reverse ^ flipLeft) // flip if speed was negative or flipLeft setting is active, but not both
    digitalWrite(DIR_L, HIGH);
  else
    digitalWrite(DIR_L, LOW);
}

// set speed for right motor; speed is a number between -400 and 400
void ZumoMotors::setRightSpeed(int speed)
{
  init(); // initialize if necessary
    
  boolean reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > 400)  // Max PWM dutycycle
    speed = 400;
    
#ifdef USE_20KHZ_PWM
  OCR1A = speed;
#else
  analogWrite(PWM_R, speed * 51 / 80); // default to using analogWrite, mapping 400 to 255
#endif

  if (reverse ^ flipRight) // flip if speed was negative or flipRight setting is active, but not both
    digitalWrite(DIR_R, HIGH);
  else
    digitalWrite(DIR_R, LOW);
}

// set speed for both motors
void ZumoMotors::setSpeeds(int leftSpeed, int rightSpeed)
{
  setLeftSpeed(leftSpeed);
  setRightSpeed(rightSpeed);
}

void ZumoMotors::setNormalizedSpeeds(float leftSpeed, float rightSpeed)
{
  setLeftSpeed((int)leftSpeed * 400);
  setRightSpeed((int)rightSpeed * 400);
}
