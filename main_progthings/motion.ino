#include <PIDController.h>

#define ROT_PROPOTION 6.5
#define ROT_INTEGRAL 0.001
#define ROT_DIRIVATIVE 0.0

#define PID_DIRECT 0
#define PID_REVERSE 1
#define SAMPLE_TIME 100

#define UP_BOUND_ROT 1.0
#define LOW_BOUND_ROT -1.0

#define TOP_SPEED 150
#define MIN_SPEED 0


void rotateToAngle(TurnSensor trn, float angle)
{
    PIDController rotationPID = PIDController((float) 6.5, (float) 0.001, (float) 0.0);
    float error, motorSpeed, targetAngle, currentHeading;
    bool keepLooping = true;

    trn.reset();
    trn.update();
    currentHeading = trn.getCurrentHeading();
    targetAngle = Utilities::wrapAngle(currentHeading + angle);

    while (keepLooping)
    {
        trn.update();
        error = Utilities::wrapAngle( trn.getCurrentHeading() - targetAngle);
        motorSpeed = rotationPID.calculate(fabs(error));
        keepLooping = !(Utilities::inRange(fabs(error), LOW_BOUND_ROT, UP_BOUND_ROT));

        if (false == keepLooping)
        {
            motorSpeed = 0;
        }

        if (motorSpeed > TOP_SPEED)
        {
            motorSpeed = TOP_SPEED;
        }

        if (error > 0)
        {
                            // left speed, right speed
            ZumoMotors::setSpeeds(motorSpeed, -motorSpeed);
        }
        else
        {
                            // left speed, right speed
            ZumoMotors::setSpeeds(-motorSpeed, motorSpeed);
        }
    }
}

void driveStraightFor (TurnSensor trn, unsigned long durationMs)
{
    int16_t targetAngle, power, error, speedDifference;
    int16_t rightSpeed, leftSpeed;
    unsigned long initTime;
    bool timedOut = false;
    const int16_t straightSpeed = 130;
    const uint16_t proportion = 12;
    const float integral = 0.5;

    trn.reset();
    trn.update();
    targetAngle = trn.getCurrentHeading();
    initTime = millis();

    while (!timedOut)
    {
        trn.update();
        error = Utilities::wrapAngle( trn.getCurrentHeading() - targetAngle);
        speedDifference = (error * proportion) + ((int)(error * integral));

        leftSpeed = straightSpeed + speedDifference;
        rightSpeed = straightSpeed - speedDifference;

        leftSpeed = constrain(leftSpeed, 0, (int16_t)straightSpeed);
        rightSpeed = constrain(rightSpeed, 0, (int16_t)straightSpeed);

        timedOut = ((millis() - initTime) > durationMs);

        if (timedOut)
        {
            leftSpeed = 0;
            rightSpeed = 0;
        }
        ZumoMotors::setSpeeds(leftSpeed,rightSpeed);
    }
}

WALL_INFO driveStraightUntilLine (TurnSensor trn, WALL_INFO wallSense, DRIVE_DIRECTION driveDir)
{
    Serial.println("----------------");
    switch (driveDir)
    {
        case LEFT:
            Serial.println("Facing Left");
            break;

        case RIGHT:
            Serial.println("Facing Right");
    }
    Serial.println("Begin driveStraightUntilLine");
    printWallInfo(wallSense);

    bool wallDriveFinihsed = false;
    bool correctingFinished = false;
    uint16_t lSpeed = 50;
    uint16_t rSpeed = 50;
    uint32_t initCorrectionTime = 0;
    uint32_t initWallDriveTime = 0;
    uint16_t diffTime = 0;
    WALL_INFO response = wallSense;


    /* Simply we keep going forward here, until we hit a wall or timeout compared
       to how long it would take to drive to the average wall, which would me we
       are driving into free space, either a turn or a room */

    initWallDriveTime = millis();
    while (!wallDriveFinihsed)
    {
        sensorArray.readCalibrated(sensorValues);
        // for (byte i = 0; i < 6; i++)
        // {
        //   Serial.print(sensorValues[i]);
        //   Serial.print(' ');
        // }
        // Serial.println();
        wallDriveFinihsed = ((sensorValues[LEFT_IR_SNSR] >= THRESHOLD_ON_LINE) ||
                       (sensorValues[RIGHT_IR_SNSR] >= THRESHOLD_ON_LINE));



       if(wallDriveFinihsed)
        {
            lSpeed = 0;
            rSpeed = 0;

            // If there was no opposite wall on the other side of the corridor we do not want to
            // add the time it took to get back to the other wall, this would mess up our averages!
            if ((wallSense.lastRightWall != WS_NO_WALL) && (wallSense.lastLeftWall != WS_NO_WALL))
            {
                Serial.println("calculate avarage");
                diffTime = millis() - initWallDriveTime;
                totalWallDriveTime += diffTime;
                numWallDrives++;
                averageWallDriveTime = totalWallDriveTime / numWallDrives;
            }

            // set the response to say we've hit a wall - we may or may not have hit it correctly
            setWallInfo(driveDir, &response, WS_HIT_WALL);
            Serial.println("Hit a wall");
        }
        // if the last time we were at this side of the corridor we sensed a partial wall
        // or no wall at all, initiate a time out so we don't drive off into free space forever
        else if (((driveDir == LEFT && (wallSense.lastLeftWall == WS_PARTIAL_WALL || wallSense.lastLeftWall == WS_NO_WALL)) ||
                 (driveDir == RIGHT && (wallSense.lastRightWall == WS_PARTIAL_WALL || wallSense.lastRightWall == WS_NO_WALL))) &&
                 (numWallDrives > 5))
        {
            if ((millis() - initWallDriveTime) > (averageWallDriveTime * 2.5))
            {
                ZumoMotors::setSpeeds(0, 0);
                Serial.println("No wall reached");
                // set the response to say that we haven't reached a wall and free space has been
                // detected, potential room or corridoor
                setWallInfo(driveDir, &response, WS_NO_WALL);
                return response;
            }
        }
        ZumoMotors::setSpeeds(lSpeed, rSpeed);
    }

    Serial.println("Reached line");

   // This checks to see if both sensors on the edge of the sensor array have reached a wall
    sensorArray.readCalibrated(sensorValues);
    correctingFinished = ((sensorValues[LEFT_IR_SNSR] >= THRESHOLD_ON_LINE) &&
               (sensorValues[RIGHT_IR_SNSR] >= THRESHOLD_ON_LINE));
    delay(100);

    if (!correctingFinished)
    {
        Serial.println("Performing corrections line");
        response = performCorrections(wallSense, driveDir);
        Serial.println("Corrections complete");
    }

    Serial.println("----------------\n\n\n");
    return response;
}

/* Even though we have reached the target wall it is probable that the robots turn was off
   by a few degrees, this needs to be corrected so that it does not drift on future
   turns. We can use the sensors on opposite ends of the sensor array to determine if
   the robot is lined up with the target wall. If we drive straight at the target wall
   both sensors should say they have reached it, if not we can move the corresponding
   track to straighten the robot up

   It make use of prior information about the robots last movement to evaluate what to do
   */
WALL_INFO performCorrections (WALL_INFO wallSense, DRIVE_DIRECTION driveDir)
{
    Serial.println("Begin performCorrections");

    bool correctingFinished = false;
    uint16_t lSpeed = 50;
    uint16_t rSpeed = 50;
    uint32_t initCorrectionTime = 0;
    uint16_t diffTime = 0;
    WALL_INFO response = wallSense;

    initCorrectionTime = millis();
    while (!correctingFinished)
    {
        sensorArray.readCalibrated(sensorValues);
        // for (byte i = 0; i < 6; i++)
        // {
        //   Serial.print(sensorValues[i]);
        //   Serial.print(' ');
        // }
        // Serial.println();

        // Check if both sensors on the array have reached the wall- making the correction complete
        if ((sensorValues[LEFT_IR_SNSR] >= THRESHOLD_ON_LINE) &&
        (sensorValues[RIGHT_IR_SNSR] >= THRESHOLD_ON_LINE))
        {
            diffTime = millis() - initCorrectionTime;

            // Tally the correction times and calculate an average correction time to use as a timeout value
            totalCorrectionTime += diffTime;
            numCorrections++;
            averageCorrectionTime = totalCorrectionTime / numCorrections;
            lSpeed = 0;
            rSpeed = 0;
            correctingFinished = true;
            // set the response to say that we've hit a full wall
            setWallInfo(driveDir, &response, WS_FULL_WALL);
            // Serial.print("dt ");
            // Serial.println(diffTime);
            // Serial.print("tct ");
            // Serial.println(totalCorrectionTime);
            // Serial.print("nc ");
            // Serial.println(numCorrections);
            // Serial.print("act ");
            // Serial.println(averageCorrectionTime);

        }
        else if ((sensorValues[LEFT_IR_SNSR] >= THRESHOLD_ON_LINE) &&
        (sensorValues[RIGHT_IR_SNSR] < THRESHOLD_ON_LINE))
        {
            // The right sensor has not reached the wall but the left has, adjust right track
            lSpeed = 0;
            rSpeed = 100;
        }
        else if ((sensorValues[RIGHT_IR_SNSR] >= THRESHOLD_ON_LINE) &&
        (sensorValues[LEFT_IR_SNSR] < THRESHOLD_ON_LINE))
        {
            // The right sensor has not reached the wall but the left has, adjust right track
            lSpeed = 100;
            rSpeed = 0;
        }
        else
        {
            // None of the scenarios above apply (but we are close), move the robot forward slowly until one fits
            lSpeed = 70;
            rSpeed = 70;
        }

        /* The average time it takes to perform a correction is tracked and this is unsed
           to time corrections that do not have full wall to correct up to (near corners & room entrances)
           if the current correction lasts longer that 1.5 times a normal correction, check to see
           if we might be near a partial wall */

        bool timeOutCheck = true;

        /* if we are coming from a side of the corridor with no wall or a partial wall
           we need as long as it takes to perfrom any corrective movements to eliminate any movement
           errors, so don't check to see if the correction has timeed out compared to the average
           correction time */
        if ((driveDir == LEFT) && (wallSense.lastRightWall == WS_NO_WALL))
            timeOutCheck = false;

        if ((driveDir == RIGHT) && (wallSense.lastLeftWall == WS_NO_WALL))
            timeOutCheck = false;

        if ((driveDir == LEFT) && (wallSense.lastRightWall == WS_PARTIAL_WALL))
            timeOutCheck = false;

        if ((driveDir == RIGHT) && (wallSense.lastLeftWall == WS_PARTIAL_WALL))
            timeOutCheck = false;

        if (timeOutCheck)
        {
            if ((millis() - initCorrectionTime > (averageCorrectionTime * 1.5)) && (numCorrections >= 8))
            {
                /* It is possible for a correction to take longer than usual and the robot not be near a
                   partial wall, we need to check this by looking at the sensors on the edge of the sensor
                   array (depending on which side has reached a wall) and see if it is near a wall from its
                   reflective value - if it is not, it is almost certainly a partial wall - so stop correcting */
                if ((sensorValues[LEFT_IR_SNSR] >= THRESHOLD_ON_LINE) &&
                    (sensorValues[RIGHT_IR_SNSR] <= THRESHOLD_NEAR_LINE))
                {
                    lSpeed = 0;
                    rSpeed = 0;
                    correctingFinished = true;
                    // set the wall info to say that we've hit a partial wall, precursor to
                    // either a room or corridor
                    setWallInfo(driveDir, &response, WS_PARTIAL_WALL);
                    Serial.println("Correction time out");
                }
                else if ((sensorValues[RIGHT_IR_SNSR] >= THRESHOLD_ON_LINE) &&
                        (sensorValues[LEFT_IR_SNSR] <= THRESHOLD_NEAR_LINE))
                {
                    lSpeed = 0;
                    rSpeed = 0;
                    correctingFinished = true;
                    setWallInfo(driveDir, &response, WS_PARTIAL_WALL);
                    Serial.println("Correction time out");
                }
            }
        }
        ZumoMotors::setSpeeds(lSpeed,rSpeed);
    }

    return response;
}

/* Function used to set the wall info that has been sensed by the zumo's sesnor
   array, the info is set based on the direction the zumo is facing, this function
   is used to shorten code in functions that use it */
void setWallInfo (DRIVE_DIRECTION dir, WALL_INFO* info, WALL_SENSE set)
{
    switch (dir)
    {
        case LEFT:
            info->lastLeftWall = set;
            break;

        case RIGHT:
            info->lastRightWall = set;
            break;

        case FORWARD:
            info->lastForwardWall = set;
            break;

    }
}

// For debug purposes
void printWallInfo (WALL_INFO info)
{
    Serial.print("lastLeftWall: ");
    Serial.println(info.lastLeftWall);

    Serial.print("lastRightWall: ");
    Serial.println(info.lastRightWall);

    Serial.print("lastForwardWall: ");
    Serial.println(info.lastForwardWall);

}
