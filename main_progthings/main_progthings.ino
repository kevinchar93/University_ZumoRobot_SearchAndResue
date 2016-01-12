#include <QTRSensors.h>
#include <ZumoBuzzer.h>
#include <Pushbutton.h>
#include <ZumoReflectanceSensorArray.h>
#include <ZumoMotors.h>
#include <Wire.h>
#include <Utilities.h>
#include <TurnSensor.h>
#include "types.h"

#define NUM_SENSORS 6

//#define SOUND_IMU_INNIT "! V10 cdefgab>cbagfedc"
#define READY_TO_MOVE "L16 cdegreg4"
#define SOUND_IMU_INNIT "L16 ccdeced4"
#define SOUND_LINE_SENSOR_INNIT "L16 gcccdecb"
#define SOUND_BUTTON_CLICK ">g32>>c32>e32>>d32"

ZumoReflectanceSensorArray sensorArray;
ZumoMotors motors;

TurnSensor turner(Serial);

// Vars used to calculate average time to perform a correction
uint32_t averageCorrectionTime = 0;
uint32_t totalCorrectionTime = 0;
uint16_t numCorrections = 0;

// Vars used to calculate average time to perform a drive to an opposite wall
uint32_t averageWallDriveTime = 0;
uint32_t totalWallDriveTime = 0;
uint16_t numWallDrives = 0;

unsigned int sensorValues[NUM_SENSORS];

void setup()
{
    Serial.begin(9600);

    waitForButtonPushBuzz(2000, SOUND_IMU_INNIT);
    turner.init();

    waitForButtonPushBuzz(2000, SOUND_LINE_SENSOR_INNIT);
    calibrateSensorArray();
    waitForButtonPushBuzz(3000, READY_TO_MOVE);
}

void loop()
{
    WALL_SENSE ws;

    while (true)
    {
        rotateToAngle(turner, -90.0);
        ws = driveStraightUntilLine(turner, ws);
        delay(100);
        rotateToAngle(turner, 90.0);
        delay(100);
        rotateToAngle(turner, 90.0);
        ws = driveStraightUntilLine(turner, ws);
        delay(100);
        rotateToAngle(turner, -90.0);
        driveStraightFor(turner, 300);
        delay(100);
    }

    // driveStraightUntilLine(turner);
    // waitForButtonPushBuzz(3000, READY_TO_MOVE);


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
