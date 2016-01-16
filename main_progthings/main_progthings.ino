#include <QTRSensors.h>
#include <ZumoBuzzer.h>
#include <Pushbutton.h>
#include <ZumoReflectanceSensorArray.h>
#include <ZumoMotors.h>
#include <Wire.h>
#include <Utilities.h>
#include <TurnSensor.h>
#include <NewPing.h>
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

#define TOP_SPEED 150

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

WALL_INFO currWallInfo;
WALL_INFO prevWallInfo;

POSITION_ESTIMATE currPosEstimate;

bool roomSearched = false;

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

    // Init wall info structs
    currWallInfo.lastLeftWall       = WS_NIL;
    currWallInfo.lastRightWall      = WS_NIL;
    currWallInfo.lastForwardWall    = WS_NIL;

    prevWallInfo.lastLeftWall       = WS_NIL;
    prevWallInfo.lastRightWall      = WS_NIL;
    prevWallInfo.lastForwardWall    = WS_NIL;

    // Init the position estimates
    currPosEstimate = PE_AT_START;
    currPosEstimate = PE_AT_START;

}

void loop()
{
    // We are about to update the current wall information, save the previous info
    prevWallInfo = currWallInfo;

    switch (currPosEstimate)
    {
        case PE_CORRIDOOR:
            Serial.println("In Corridoor\n\n");
            moveAndSearch(DRIVE_FORWARD_TIME);
            roomSearched = false;
            break;

        case PE_PARTIAL_LEFT:
            Serial.println("Partial wall to the left\n\n");
            moveAndSearch(DRIVE_FORWARD_TIME);
            break;
        case PE_PARTIAL_RIGHT:
            Serial.println("Partial wall to the right\n\n");
            moveAndSearch(DRIVE_FORWARD_TIME);
            break;

        case PE_UNCERTAIN:
            Serial.println("Uncertain of position\n\n");
            moveAndSearch(DRIVE_FORWARD_TIME+50);
            roomSearched = false;
            break;

        case PE_AT_START:
            Serial.println("Beginning search and rescue\n\n");
            moveAndSearch(DRIVE_FORWARD_TIME);
            roomSearched = false;
            break;

        case PE_RIGHT_ROOM:
            Serial.println("In Right Room\n\n");
            moveAndSearchRightRoom();
            roomSearched = true;
            break;

        case PE_LEFT_ROOM:
            Serial.println("In Left Room\n\n");
            moveAndSearchLeftRoom();
            roomSearched = true;
            break;

        case PE_RIGHT_CORNER:
            Serial.println("Taking Right Corner\n\n");
            rotateToAngle(RIGHT_TURN_90_DEG, false, TOP_SPEED);
            moveAndSearch(DRIVE_FORWARD_TIME + 100);
            roomSearched = false;
            break;

        case PE_LEFT_CORNER:
            Serial.println("Taking Left Corner\n\n");
            rotateToAngle(LEFT_TURN_90_DEG, false, TOP_SPEED);
            moveAndSearch(DRIVE_FORWARD_TIME + 100);
            roomSearched = false;
            break;

        case PE_END_OF_MAZE:
            Serial.println("Reach end of Maze Turning Around\n\n");

            // Reset Averages
            averageCorrectionTime = 0;
            totalCorrectionTime = 0;
            numCorrections = 0;

            averageWallDriveTime = 0;
            totalWallDriveTime = 0;
            numWallDrives = 0;

            rotateToAngle(RIGHT_TURN_90_DEG, false, TOP_SPEED);
            rotateToAngle(RIGHT_TURN_90_DEG, false, TOP_SPEED);
            moveAndSearch(DRIVE_FORWARD_TIME);
            roomSearched = false;
            break;
    }

    currPosEstimate = estimatePosition(currWallInfo, prevWallInfo);
    delay(DELAY_TIME);
}

void moveAndSearch(int driveTime)
{
    // Drive forward, until time out or line is hit
    currWallInfo = driveForwardFor(driveTime, FORWARD, currWallInfo);

    // Turn to the right and go up towards the line
    rotateToAngle(RIGHT_TURN_90_DEG, false, TOP_SPEED);
    currWallInfo = driveStraightUntilLine(currWallInfo, RIGHT);

    // Turn back to the center
    rotateToAngle(LEFT_TURN_90_DEG, false, TOP_SPEED);

    // Turn to the left and go uptowards the line
    rotateToAngle(LEFT_TURN_90_DEG, false, TOP_SPEED);
    currWallInfo = driveStraightUntilLine(currWallInfo, LEFT);

    // Turn back to center
    rotateToAngle(RIGHT_TURN_90_DEG, false, TOP_SPEED);
}


void moveAndSearchRightRoom ()
{
    bool sweep1 = false;
    bool sweep2 = false;
    bool sweep3 = false;

    currWallInfo = driveForwardFor(DRIVE_FORWARD_TIME, FORWARD, currWallInfo);
    rotateToAngle(RIGHT_TURN_90_DEG, false, TOP_SPEED);
    currWallInfo = driveStraightUntilLine(currWallInfo, RIGHT);

    if(false == roomSearched)
    {
        // search the room
        sweep1 = rotateToAngle(RIGHT_TURN_90_DEG, true, 100);
        sweep2 = rotateToAngle(LEFT_TURN_90_DEG, true, 100);
        sweep3 = rotateToAngle(LEFT_TURN_90_DEG, true, 100);

        rotateToAngle(LEFT_TURN_90_DEG, false, TOP_SPEED);
        currWallInfo = driveStraightUntilLine(currWallInfo, LEFT);
        rotateToAngle(RIGHT_TURN_90_DEG, false, TOP_SPEED);
    }
    else
    {
        rotateToAngle(LEFT_TURN_90_DEG, false, TOP_SPEED);
        rotateToAngle(LEFT_TURN_90_DEG, false, TOP_SPEED);
        currWallInfo = driveStraightUntilLine(currWallInfo, LEFT);
        rotateToAngle(RIGHT_TURN_90_DEG, false, TOP_SPEED);
    }

    if (sweep1 || sweep2 || sweep3)
    {
        Serial.println("There is a person in the Room!");
        makeSirenNoise();
    }

}


void moveAndSearchLeftRoom ()
{
    bool sweep1 = false;
    bool sweep2 = false;
    bool sweep3 = false;

    currWallInfo = driveForwardFor(DRIVE_FORWARD_TIME, FORWARD, currWallInfo);
    rotateToAngle(RIGHT_TURN_90_DEG, false, TOP_SPEED);
    currWallInfo = driveStraightUntilLine(currWallInfo, RIGHT);

    if(false == roomSearched)
    {
        // search the room
        rotateToAngle(LEFT_TURN_90_DEG, false, TOP_SPEED);
        rotateToAngle(LEFT_TURN_90_DEG, false, TOP_SPEED);
        currWallInfo = driveStraightUntilLine(currWallInfo, LEFT);

        sweep1 = rotateToAngle(LEFT_TURN_90_DEG, true, 100);
        sweep2 = rotateToAngle(RIGHT_TURN_90_DEG, true, 100);
        sweep3 = rotateToAngle(RIGHT_TURN_90_DEG, true, 100);
    }
    else
    {
        rotateToAngle(LEFT_TURN_90_DEG, false, TOP_SPEED);
        rotateToAngle(LEFT_TURN_90_DEG, false, TOP_SPEED);
        currWallInfo = driveStraightUntilLine(currWallInfo, LEFT);
        rotateToAngle(RIGHT_TURN_90_DEG, false, TOP_SPEED);
    }

    if (sweep1 || sweep2 || sweep3)
    {
        Serial.println("There is a person in the Room!");
        makeSirenNoise();
    }
}

void makeSirenNoise()
{
    ZumoBuzzer buzzer;

    for (int j = 0; j <= 7; j++)
    {
        for (int i = 635; i < 1000; i+=10)
        {
            buzzer.playFrequency(i, 5, 15);
            delay(5);
        }

        for (int i = 1000; i >= 635; i-=10)
        {
            buzzer.playFrequency(i, 5, 15);
            delay(5);
        }
    }
}

POSITION_ESTIMATE estimatePosition (WALL_INFO currInfo, WALL_INFO prevInfo)
{
    // Try to estimate what situation the robot is in based on the sensed wall info
    if ((currInfo.lastLeftWall == WS_NIL) &&
        (currInfo.lastRightWall == WS_NIL) &&
        (currInfo.lastForwardWall == WS_NIL))
    {
        return PE_AT_START;
    }

    if ((currInfo.lastLeftWall == WS_FULL_WALL) &&
        (currInfo.lastRightWall == WS_FULL_WALL) &&
        (currInfo.lastForwardWall == WS_NO_WALL_AHEAD))
    {
        return PE_CORRIDOOR;
    }

    if ((currInfo.lastLeftWall == WS_PARTIAL_WALL) &&
        (currInfo.lastRightWall == WS_FULL_WALL) &&
        (currInfo.lastForwardWall == WS_NO_WALL_AHEAD))
    {
        return PE_PARTIAL_LEFT;
    }

    if ((currInfo.lastLeftWall == WS_FULL_WALL) &&
        (currInfo.lastRightWall == WS_PARTIAL_WALL) &&
        (currInfo.lastForwardWall == WS_NO_WALL_AHEAD))
    {
        return PE_PARTIAL_RIGHT;
    }

    if ((currInfo.lastLeftWall == WS_FULL_WALL || currInfo.lastLeftWall == WS_PARTIAL_WALL) &&
        (currInfo.lastRightWall == WS_NO_WALL) &&
        (currInfo.lastForwardWall == WS_WALL_AHEAD))
    {
        return PE_RIGHT_CORNER;
    }

    if ((currInfo.lastLeftWall == WS_NO_WALL) &&
        (currInfo.lastRightWall == WS_FULL_WALL || currInfo.lastRightWall == WS_PARTIAL_WALL) &&
        (currInfo.lastForwardWall == WS_WALL_AHEAD))
    {
        return PE_LEFT_CORNER;
    }

    if ((currInfo.lastLeftWall == WS_FULL_WALL || currInfo.lastLeftWall == WS_PARTIAL_WALL) &&
        (currInfo.lastRightWall == WS_NO_WALL) &&
        (currInfo.lastForwardWall == WS_NO_WALL_AHEAD) &&
        (prevInfo.lastLeftWall == WS_FULL_WALL || currInfo.lastLeftWall == WS_PARTIAL_WALL) &&
        (prevInfo.lastRightWall == WS_NO_WALL) &&
        (prevInfo.lastForwardWall == WS_NO_WALL_AHEAD))
    {
        return PE_RIGHT_ROOM;
    }

    if ((currInfo.lastRightWall == WS_FULL_WALL || currInfo.lastRightWall == WS_PARTIAL_WALL) &&
        (currInfo.lastLeftWall == WS_NO_WALL) &&
        (currInfo.lastForwardWall == WS_NO_WALL_AHEAD) &&
        (prevInfo.lastRightWall == WS_FULL_WALL || currInfo.lastRightWall == WS_PARTIAL_WALL) &&
        (prevInfo.lastLeftWall == WS_NO_WALL) &&
        (prevInfo.lastForwardWall == WS_NO_WALL_AHEAD))
    {
        return PE_LEFT_ROOM;
    }


    if ((currInfo.lastLeftWall == WS_FULL_WALL || currInfo.lastLeftWall == WS_PARTIAL_WALL) &&
        (currInfo.lastRightWall == WS_FULL_WALL || currInfo.lastRightWall == WS_PARTIAL_WALL) &&
        (currInfo.lastForwardWall == WS_WALL_AHEAD))
    {
        return PE_END_OF_MAZE;
    }


    return PE_UNCERTAIN;
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
