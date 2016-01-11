#include <QTRSensors.h>
#include <ZumoBuzzer.h>
#include <Pushbutton.h>
#include <ZumoReflectanceSensorArray.h>
#include <ZumoMotors.h>
#include <Wire.h>
#include <IMUManager.h>
#include <Utilities.h>
#include <TurnSensor.h>

#define NUM_SENSORS 6

//#define SOUND_IMU_INNIT "! V10 cdefgab>cbagfedc"
#define READY_TO_MOVE "L16 cdegreg4"
#define SOUND_IMU_INNIT "L16 ccdeced4"
#define SOUND_BUTTON_CLICK ">g32>>c32>e32>>d32"

ZumoReflectanceSensorArray sensorArray;
ZumoMotors motors;

TurnSensor turner(Serial);
bool zeroGyro = false;

unsigned int sensorVals[NUM_SENSORS];

void setup()
{
    Serial.begin(9600);

    waitForButtonPushBuzz(2000, SOUND_IMU_INNIT);
    turner.init();
}

void loop()
{
    waitForButtonPushBuzz(3000, READY_TO_MOVE);

    while (true)
    {
        /*
        rotateToAngle(turner, -90.0, motors);
        driveStraightFor(turner, 150);
        delay(100);
        rotateToAngle(turner, 90.0, motors);
        delay(100);
        rotateToAngle(turner, 90.0, motors);
        driveStraightFor(turner, 150);
        delay(100);
        rotateToAngle(turner, -90.0, motors);
        driveStraightFor(turner, 500);
        delay(100);
        */
        driveStraightFor(turner, 10000);
    }
}

void waitForButtonPushBuzz(int delayTime, const char* sound)
{
    Pushbutton button(ZUMO_BUTTON);
    ZumoBuzzer buzzer;

    //buzzer.play("! V10 cdefgab>cbagfedc");
    buzzer.play(sound);
    button.waitForButton();

    //buzzer.play(">g32>>c32>e32>>d32");
    buzzer.play(SOUND_BUTTON_CLICK);
    delay(delayTime);
}

/*
void calibrateSensorArray ()
{
    boolean keepLooping = false;
    unsigned long beginTime = 0;
    int spinSpeed = 220;
    unsigned int position = 0;

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

/*
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
*/
