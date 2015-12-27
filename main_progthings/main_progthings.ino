#include <QTRSensors.h>
#include <ZumoBuzzer.h>
#include <Pushbutton.h>
#include <ZumoReflectanceSensorArray.h>
#include <ZumoMotors.h>

ZumoBuzzer buzzer;
ZumoReflectanceSensorArray sensorArray;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);

#define CALIBRATE_TIME 5000
#define TOP_SPEED 200
#define NUM_SENSORS 6
#define TOP_POINT 5000
#define MID_POINT 2500
#define MID_POINT_UPPER 2700
#define MID_POINT_BOTTOM 2200

#define PROPORTION_CONST 4
#define INTEGRAL_CONST 2000
#define DERIVATVE_CONST 6

boolean keepLooping = false;
unsigned long beginTime = 0;
int spinSpeed = 220;
unsigned int position = 0;
unsigned int sensorVals[NUM_SENSORS];
int leftSpeed = 0;
int rightSpeed = 0;

void setup()
{

    // innit the sensor array
    sensorArray.init();

    // wait for the user press before beginning, play sound, wait before starting
    button.waitForButton();
    buzzer.play(">g32>>c32");
    delay(3000);

    // begin looping and performing calibration
    keepLooping = true;

    beginTime = millis();

    while (keepLooping)
    {
        if((millis() - beginTime) < CALIBRATE_TIME)
        {
            sensorArray.calibrate();
        }
        else
        {
            position = sensorArray.readLine(sensorVals);
            if ((position > MID_POINT_BOTTOM) && (position  < MID_POINT_UPPER))
            {
                spinSpeed = 0;
                keepLooping = false;
            }
        }
        move(motors, spinSpeed, -spinSpeed, 1, 0);
    }
    buzzer.play(">g32>>c32");
    button.waitForButton();
    buzzer.play(">g32>>c32");
    delay(1000);
}

int proportional = 0;
int last_proportional = 0;
int derivative = 0;
int pow_diff = 0;
int integral = 0;

void loop()
{
    position = sensorArray.readLine(sensorVals);

    proportional = ((int)position) - 2500;
    derivative = proportional - last_proportional;
    integral += proportional;

    last_proportional = proportional;

    pow_diff = proportional/PROPORTION_CONST + integral/INTEGRAL_CONST + derivative*DERIVATVE_CONST;

    if (pow_diff > TOP_SPEED)
    {
        pow_diff = TOP_SPEED;
    }

    if (pow_diff < -TOP_SPEED)
    {
        pow_diff = -TOP_SPEED;
    }

    if (pow_diff < 0)
    {
        move(motors, TOP_SPEED+pow_diff, TOP_SPEED, 1 ,0);
    }
    else
    {
        move(motors, TOP_SPEED, TOP_SPEED - pow_diff, 1, 0);
    }
}

int calcLeftSpeed (int pos, int max)
{
    if (pos >= MID_POINT)
    {
        return max;
    }
    else
    {
        float mul = (float) pos / (float) MID_POINT;
        return (int) (mul * max);
    }
}

int calcRightSpeed (int pos, int max)
{
    if (pos <= MID_POINT)
    {
        return max;
    }
    else
    {
        float mul = (float) (pos - MID_POINT) / (float) MID_POINT;
        return (int) (mul * max);
    }
}

void move(ZumoMotors zm, int leftSpeed, int rightSpeed,
                unsigned int times, unsigned int wait) {
    for (int i = 0; i < times; i++){
        zm.setSpeeds(leftSpeed, rightSpeed);
        delay(wait);
    }
}

void moveForward(ZumoMotors zm, unsigned int speed, unsigned int times, unsigned int wait) {
    for (int i = 0; i < times; i++){
        zm.setSpeeds(speed, speed);
        delay(wait);
    }
}

void moveBackward(ZumoMotors zm, unsigned int speed, unsigned int times, unsigned int wait) {
    for (int i = 0; i < times; i++){
        zm.setSpeeds(-speed, -speed);
        delay(wait);
    }
}
