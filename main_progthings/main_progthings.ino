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

#define DRIVE_FORWARD_TIME 400
#define RIGHT_TURN_90_DEG -90.0
#define LEFT_TURN_90_DEG 90.0
#define DELAY_TIME 100

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
WALL_INFO ws;

void setup()
{
    Serial.begin(9600);
    Serial.println("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");

    waitForButtonPushBuzz(2000, SOUND_IMU_INNIT);
    turner.init();


    waitForButtonPushBuzz(2000, SOUND_LINE_SENSOR_INNIT);
    Serial.println("Begin calibrateSensorArray");
    calibrateSensorArray();
    Serial.println("Complete");
    waitForButtonPushBuzz(3000, READY_TO_MOVE);
}

void loop()
{

    while (true)
    {
        rotateToAngle(RIGHT_TURN_90_DEG);
        ws = driveStraightUntilLine(ws, RIGHT);
        Serial.println("--------------------");
        Serial.println("Response: ");
        printWallInfo(ws);
        Serial.println("--------------------");
        delay(DELAY_TIME);
        rotateToAngle(LEFT_TURN_90_DEG);
        delay(DELAY_TIME);
        rotateToAngle(LEFT_TURN_90_DEG);
        ws = driveStraightUntilLine(ws, LEFT);
        delay(DELAY_TIME);
        rotateToAngle(RIGHT_TURN_90_DEG);
        driveForwardFor(DRIVE_FORWARD_TIME);
        delay(DELAY_TIME);
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
