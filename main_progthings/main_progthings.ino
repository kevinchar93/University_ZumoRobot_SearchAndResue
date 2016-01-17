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

#define READY_TO_MOVE "L16 cdegreg4"
#define SOUND_IMU_INNIT "L16 ccdeced4"
#define SOUND_LINE_SENSOR_INNIT "L16 gcccdecb"
#define SOUND_BUTTON_CLICK ">g32>>c32>e32>>d32"

#define TOP_SPEED 150
#define DRIVE_STRAIGHT_SPEED 100

/* Vars used to access robot hardware */
ZumoReflectanceSensorArray sensorArray;
ZumoMotors motors;
TurnSensor turner(Serial);

unsigned int sensorValues[NUM_SENSORS];

/* Vars used to calculate average time to perform a correction */
uint32_t averageCorrectionTime = 0;
uint32_t totalCorrectionTime = 0;
uint16_t numCorrections = 0;

/* Vars used to calculate average time to perform a drive to an opposite wall */
uint32_t averageWallDriveTime = 0;
uint32_t totalWallDriveTime = 0;
uint16_t numWallDrives = 0;

/* Variables used to store sensory information about the robots surroundings and the robots esimated position */
WALL_INFO currWallInfo;
POSITION_ESTIMATE currPosEstimate;

/* Used as a flag to indicate if the current room has been searched for objects yet */
bool roomSearched = false;

void setup()
{
    Serial.begin(9600);

    /* Clear the terminal that we are printing this output to */
    Serial.println("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");

    /* Wait for user button press to begin init of the TurnSensor module */
    waitForButtonPushBuzz(2000, SOUND_IMU_INNIT);
    turner.init();

    /* Wait for user button press to begin calibration of line sensors */
    waitForButtonPushBuzz(2000, SOUND_LINE_SENSOR_INNIT);
    Serial.println("Begin calibrateSensorArray");
    calibrateSensorArray();
    Serial.println("Complete");

    /* Init wall info structs */
    currWallInfo.lastLeftWall       = WS_NIL;
    currWallInfo.lastRightWall      = WS_NIL;
    currWallInfo.lastForwardWall    = WS_NIL;

    /* Init the position estimates */
    currPosEstimate = PE_AT_START;
    currPosEstimate = PE_AT_START;

    /* Wait for user to press button to begin operation */
    waitForButtonPushBuzz(3000, READY_TO_MOVE);
}

void loop()
{
    /* Switch statement to decide how the robot should act based on what its current
       surroundings in the map are */
    switch (currPosEstimate)
    {
        /* If we are in the corridoor perform the standard rountine of moving
           forward and search around the robot */
        case PE_CORRIDOOR:
            Serial.println("In Corridoor\n\n");
            moveAndSearch(DRIVE_FORWARD_TIME);
            roomSearched = false;
            break;

        /* If we have detected a partial wall to the left perform the move forward
           and search rountine but drive forward for 400 ms longer to get past the partial
           wall as quickly as possible */
        case PE_PARTIAL_LEFT:
            Serial.println("Partial wall to the left\n\n");
            moveAndSearch(DRIVE_FORWARD_TIME+400);
            roomSearched = false;
            break;

        /* If we have detected a partial wall to the right perform the move forward
           and search rountine but drive forward for 400 ms longer to get past the partial
           wall as quickly as possible */
        case PE_PARTIAL_RIGHT:
            Serial.println("Partial wall to the right\n\n");
            moveAndSearch(DRIVE_FORWARD_TIME+400 );
            roomSearched = false;
            break;

        /* In this situation we cannot give a good estimate of where the robot is
           so just perform the move forward an search roution - print to the console
           the uncertain state of the robot */
        case PE_UNCERTAIN:
            Serial.println("Uncertain of position\n\n");
            moveAndSearch(DRIVE_FORWARD_TIME);
            roomSearched = false;
            break;

        /* The robot has just started out at the beginning of the track, move forward around
            search to begin getting information about where it is */
        case PE_AT_START:
            Serial.println("Beginning search and rescue\n\n");
            moveAndSearch(DRIVE_FORWARD_TIME);
            roomSearched = false;
            break;

        /* The robot is in a room on the right, begin the procedure which searches the right
           room for objects. It will determine if the room has been searched yet using the roomSearched
           flag and sound an alarm if an object is detected inside a room.*/
        case PE_RIGHT_ROOM:
            moveAndSearchRightRoom();
            roomSearched = true;
            break;

        /* The robot is in a room on the left, begin the procedure which searches the left
           room for objects. It will determine if the room has been searched yet using the roomSearched
           flag and sound an alarm if an object is detected inside a room.*/
        case PE_LEFT_ROOM:
            moveAndSearchLeftRoom();
            roomSearched = true;
            break;

        /* The robot is in a corner that turns to the right, perform a right hand turn of 90 degress and
           continues the normal move forward and search rountine, instead move forward for an extra 450 ms
           so that we clear the edge of the corner completley */
        case PE_RIGHT_CORNER:
            Serial.println("Taking Right Corner\n\n");
            rotateToAngle(RIGHT_TURN_90_DEG, false, TOP_SPEED);
            moveAndSearch(DRIVE_FORWARD_TIME + 450);
            roomSearched = false;
            break;

        /* The robot is in a corner that turns to the left, perform a left hand turn of 90 degress and
           continues the normal move forward and search rountine, instead move forward for an extra 450 ms
           so that we clear the edge of the corner completley */
        case PE_LEFT_CORNER:
            Serial.println("Taking Left Corner\n\n");
            rotateToAngle(LEFT_TURN_90_DEG, false, TOP_SPEED);
            moveAndSearch(DRIVE_FORWARD_TIME + 450);
            roomSearched = false;
            break;

        /* The robot has reached the end of the maze, so perform a 180 degree turn to face the other
           direction, then continue as normal with the move forward and search routine */
        case PE_END_OF_MAZE:
            Serial.println("Reach end of Maze Turning Around\n\n");
            rotateToAngle(RIGHT_TURN_90_DEG, false, TOP_SPEED);
            rotateToAngle(RIGHT_TURN_90_DEG, false, TOP_SPEED);
            moveAndSearch(DRIVE_FORWARD_TIME);
            roomSearched = false;
            break;
    }

    /* Using the sensory data that the robot has just collected, try to estimate where in the map
       the robot is */
    currPosEstimate = estimatePosition(currWallInfo, currPosEstimate);
    delay(DELAY_TIME);
}

/* Standard procedure the the robot performs to build up an estimate of its surroundings */
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

    // Turn back to face forwards
    rotateToAngle(RIGHT_TURN_90_DEG, false, TOP_SPEED);
}

/* Called to instruct the robot how to act when in a right room */
void moveAndSearchRightRoom ()
{
    // Vars used to store results of object sweep
    bool sweep1 = false;
    bool sweep2 = false;
    bool sweep3 = false;

    // Drive forward for the standard ammount of time or until bot hit a wall, turn to face right drive into the right room
    currWallInfo = driveForwardFor(DRIVE_FORWARD_TIME, FORWARD, currWallInfo);
    rotateToAngle(RIGHT_TURN_90_DEG, false, TOP_SPEED);
    currWallInfo = driveStraightUntilLine(currWallInfo, RIGHT);

    // only perform the sweep if the room hasn't been searched yet and there's no wall in the corridor in front the bot
    if(false == roomSearched && currWallInfo.lastForwardWall == WS_NO_WALL_AHEAD)
    {
        // search the room with sensor, scan by sweeping right, then back to the center then left
        sweep1 = rotateToAngle(RIGHT_TURN_90_DEG, true, 100);
        sweep2 = rotateToAngle(LEFT_TURN_90_DEG, true, 100);
        sweep3 = rotateToAngle(LEFT_TURN_90_DEG, true, 100);

        // rotate back to face the wall on the left the go up towards it
        rotateToAngle(LEFT_TURN_90_DEG, false, TOP_SPEED);
        currWallInfo = driveStraightUntilLine(currWallInfo, LEFT);

        // turn back to face forwards
        rotateToAngle(RIGHT_TURN_90_DEG, false, TOP_SPEED);
    }
    else
    {
        // turn to face the left, drive  upto the left wall then turn back to face forwards
        rotateToAngle(LEFT_TURN_90_DEG, false, TOP_SPEED);
        rotateToAngle(LEFT_TURN_90_DEG, false, TOP_SPEED);
        currWallInfo = driveStraightUntilLine(currWallInfo, LEFT);
        rotateToAngle(RIGHT_TURN_90_DEG, false, TOP_SPEED);
    }

    if (currWallInfo.lastForwardWall == WS_NO_WALL_AHEAD)
    {
        Serial.println("In Right Room\n\n");
    }

    if (sweep1 || sweep2 || sweep3)
    {
        // An object was detected on one of the sweeps, print a console message, sound alarm
        Serial.println("*** There is a person in the Room! ***");
        makeSirenNoise();
    }
}

/* Called to instruct the robot how to act when in a left room */
void moveAndSearchLeftRoom ()
{
    // Vars used to store results of object sweep
    bool sweep1 = false;
    bool sweep2 = false;
    bool sweep3 = false;

    // Drive forward for the standard ammount of time or until bot hit a wall, turn to face right
    currWallInfo = driveForwardFor(DRIVE_FORWARD_TIME, FORWARD, currWallInfo);
    rotateToAngle(RIGHT_TURN_90_DEG, false, TOP_SPEED);
    currWallInfo = driveStraightUntilLine(currWallInfo, RIGHT);

    // only perform the sweep if the room hasn't been searched yet and there's no wall in the corridor infront the bot
    if(false == roomSearched && currWallInfo.lastForwardWall == WS_NO_WALL_AHEAD)
    {
        // turn towards the left room drive into it
        rotateToAngle(LEFT_TURN_90_DEG, false, TOP_SPEED);
        rotateToAngle(LEFT_TURN_90_DEG, false, TOP_SPEED);
        currWallInfo = driveStraightUntilLine(currWallInfo, LEFT);

        // search the room with sensor, scan by sweeping left, then back to the center then right
        sweep1 = rotateToAngle(LEFT_TURN_90_DEG, true, 100);
        sweep2 = rotateToAngle(RIGHT_TURN_90_DEG, true, 100);
        sweep3 = rotateToAngle(RIGHT_TURN_90_DEG, true, 100);
    }
    else
    {
        // turn to face the left, drive upto the left wall then turn back to face forwards
        rotateToAngle(LEFT_TURN_90_DEG, false, TOP_SPEED);
        rotateToAngle(LEFT_TURN_90_DEG, false, TOP_SPEED);
        currWallInfo = driveStraightUntilLine(currWallInfo, LEFT);
        rotateToAngle(RIGHT_TURN_90_DEG, false, TOP_SPEED);
    }

    if (currWallInfo.lastForwardWall == WS_NO_WALL_AHEAD)
    {
        Serial.println("In Left Room\n\n");
    }

    if (sweep1 || sweep2 || sweep3)
    {
        // An object was detected on one of the sweeps, print a console message, sound alarm
        Serial.println("*** There is a person in the Room! ***");
        makeSirenNoise();
    }
}

/* Used to make a siren like noise when a object is detected in a room */
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

/* Used to estimate where in the map the robot currently is */
POSITION_ESTIMATE estimatePosition (WALL_INFO currInfo, POSITION_ESTIMATE prevEstimate)
{
    // Try to estimate what situation the robot is in based on the sensed wall info
    if ((currInfo.lastLeftWall == WS_NIL) &&
        (currInfo.lastRightWall == WS_NIL) &&
        (currInfo.lastForwardWall == WS_NIL))
    {
        // we have no information to work with so the robot is at the beginningo f the map
        return PE_AT_START;
    }

    if ((currInfo.lastLeftWall == WS_FULL_WALL) &&
        (currInfo.lastRightWall == WS_FULL_WALL) &&
        (currInfo.lastForwardWall == WS_NO_WALL_AHEAD))
    {
        // there are two walls on each side of the robot and no wall in front, we are in a corridor
        return PE_CORRIDOOR;
    }

    if ((currInfo.lastLeftWall == WS_PARTIAL_WALL) &&
        (currInfo.lastRightWall == WS_FULL_WALL) &&
        (currInfo.lastForwardWall == WS_NO_WALL_AHEAD))
    {
        // there is a partial wall on the left, full wall on the right and no wall in front
        // there is a partial wall on the left, a potential lfet room or left turn
        return PE_PARTIAL_LEFT;
    }

    if ((currInfo.lastLeftWall == WS_FULL_WALL) &&
        (currInfo.lastRightWall == WS_PARTIAL_WALL) &&
        (currInfo.lastForwardWall == WS_NO_WALL_AHEAD))
    {
        // there is a partial wall on the right, full wall on the left and no wall in front
        // there is a partial wall on the right, a potential lfet room or right turn
        return PE_PARTIAL_RIGHT;
    }

    if ((currInfo.lastLeftWall == WS_FULL_WALL || currInfo.lastLeftWall == WS_PARTIAL_WALL) &&
        (currInfo.lastRightWall == WS_NO_WALL) &&
        (currInfo.lastForwardWall == WS_WALL_AHEAD))
    {
        // there is a partial or full wall on the left, no wall on the right and a wall in front
        // we are in a corner with a right turn
        return PE_RIGHT_CORNER;
    }

    if ((currInfo.lastLeftWall == WS_NO_WALL) &&
        (currInfo.lastRightWall == WS_FULL_WALL || currInfo.lastRightWall == WS_PARTIAL_WALL) &&
        (currInfo.lastForwardWall == WS_WALL_AHEAD))
    {
        // there is a partial or full wall on the right, no wall on the left and a wall in front
        // we are in a corner with a left turn
        return PE_LEFT_CORNER;
    }

    if ((currInfo.lastLeftWall == WS_FULL_WALL || currInfo.lastLeftWall == WS_PARTIAL_WALL) &&
        (currInfo.lastRightWall == WS_NO_WALL) &&
        (currInfo.lastForwardWall == WS_NO_WALL_AHEAD) &&
        (prevEstimate == PE_PARTIAL_RIGHT || prevEstimate == PE_RIGHT_ROOM))
    {
        // there is a partial or full wall on the left, no wall on the right, no wall in front
        // and the previous estimate was either a partial wall on the right or a room on the right
        // we are in a room on the right
        return PE_RIGHT_ROOM;
    }

    if ((currInfo.lastRightWall == WS_FULL_WALL || currInfo.lastRightWall == WS_PARTIAL_WALL) &&
        (currInfo.lastLeftWall == WS_NO_WALL) &&
        (currInfo.lastForwardWall == WS_NO_WALL_AHEAD) &&
        (prevEstimate == PE_PARTIAL_LEFT || prevEstimate == PE_LEFT_ROOM))
    {
        // there is a partial or full wall on the right, no wall on the left, no wall in front
        // and the previous estimate was either a partial wall on the left or a room on the left
        // we are in a room on the left
        return PE_LEFT_ROOM;
    }


    if ((currInfo.lastLeftWall == WS_FULL_WALL || currInfo.lastLeftWall == WS_PARTIAL_WALL) &&
        (currInfo.lastRightWall == WS_FULL_WALL || currInfo.lastRightWall == WS_PARTIAL_WALL) &&
        (currInfo.lastForwardWall == WS_WALL_AHEAD))
    {
        // the is a full or partial wall to the left and right of us, and a wall in front
        // we have reached the end of the maze
        return PE_END_OF_MAZE;
    }

    // we aare unsure of where the robot is with the current information
    return PE_UNCERTAIN;
}

/* Play a custom sound to let user know we are waiting on a button press,
  will halt the program till button press also makes a sound to let user
    know button has been pressed */
void waitForButtonPushBuzz(int delayTime, const char* sound)
{
    Pushbutton button(ZUMO_BUTTON);
    ZumoBuzzer buzzer;

    buzzer.play(sound);
    button.waitForButton();

    buzzer.play(SOUND_BUTTON_CLICK);
    delay(delayTime);
}
