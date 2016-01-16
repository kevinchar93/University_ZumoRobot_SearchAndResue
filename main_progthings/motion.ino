#include <PIDController.h>

#define ROT_PROPOTION 6.5
#define ROT_INTEGRAL 0.001
#define ROT_DIRIVATIVE 0.0

#define PID_DIRECT 0
#define PID_REVERSE 1
#define SAMPLE_TIME 100

#define UP_BOUND_ROT 1.0
#define LOW_BOUND_ROT -1.0


#define MIN_SPEED 0

#define PING_SPEED 100

#define TRIG 2
#define ECHO 6
#define MAX_DISTANCE 30

bool rotateToAngle(float angle, bool sweepForObject, int topSpeed)
{
    PIDController rotationPID = PIDController((float) 6.5, (float) 0.001, (float) 0.0);
    float error, motorSpeed, targetAngle, currentHeading;
    bool keepLooping = true;
    bool objectFound = false;
    NewPing ping(TRIG, ECHO, MAX_DISTANCE);
    ZumoBuzzer buzzer;

    turner.reset();
    turner.update();
    currentHeading = turner.getCurrentHeading();
    targetAngle = Utilities::wrapAngle(currentHeading + angle);

    while (keepLooping)
    {
        turner.update();
        error = Utilities::wrapAngle( turner.getCurrentHeading() - targetAngle);
        motorSpeed = rotationPID.calculate(fabs(error));
        keepLooping = !(Utilities::inRange(fabs(error), LOW_BOUND_ROT, UP_BOUND_ROT));

        if (sweepForObject && !objectFound)
        {
            if(ping.ping_cm() > 0)
            {
                objectFound = true;
                buzzer.playFrequency(500, 10, 15);
                buzzer.playFrequency(600, 10, 15);
            }
        }

        if (false == keepLooping)
        {
            motorSpeed = 0;
        }

        if (motorSpeed > topSpeed)
        {
            motorSpeed = topSpeed;
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

    //delay(100);
    return objectFound;
}

WALL_INFO driveForwardFor (unsigned long durationMs, DRIVE_DIRECTION driveDir, WALL_INFO wallInfo)
{
    int16_t targetAngle, power, error, speedDifference;
    int16_t rightSpeed, leftSpeed;
    unsigned long initTime;

    bool timedOut = false;
    bool frontLineDetected = false;
    bool reverseCorrectionNeeded = false;

    const int16_t straightSpeed = 100;
    const uint16_t proportion = 12;
    const float integral = 0.5;

    WALL_INFO response = wallInfo;

    turner.reset();
    turner.update();
    targetAngle = turner.getCurrentHeading();
    initTime = millis();

    while (!timedOut)
    {
        turner.update();
        error = Utilities::wrapAngle( turner.getCurrentHeading() - targetAngle);
        speedDifference = (error * proportion) + ((int)(error * integral));

        leftSpeed = straightSpeed + speedDifference;
        rightSpeed = straightSpeed - speedDifference;

        leftSpeed = constrain(leftSpeed, 0, (int16_t)straightSpeed);
        rightSpeed = constrain(rightSpeed, 0, (int16_t)straightSpeed);

        // Check if we've driven for the set ammount of time
        timedOut = ((millis() - initTime) > durationMs);

        if (timedOut)
        {
            leftSpeed = 0;
            rightSpeed = 0;

            // No wall detected ahead set wall info
            setWallInfo(driveDir, &response, WS_NO_WALL_AHEAD);
        }

        // Check if we've hit a line in front , if so stop the bot
        sensorArray.readCalibrated(sensorValues);
        frontLineDetected = ((sensorValues[LEFT_IR_SNSR] >= THRESHOLD_ON_LINE) &&
                       (sensorValues[RIGHT_IR_SNSR] >= THRESHOLD_ON_LINE));


        if (frontLineDetected)
        {
            leftSpeed = 0;
            rightSpeed = 0;
            timedOut = true;
            reverseCorrectionNeeded = true;

            // Wall detected in front of the bot
            setWallInfo(driveDir, &response, WS_WALL_AHEAD);
        }
        ZumoMotors::setSpeeds(leftSpeed,rightSpeed);
    }

    if(reverseCorrectionNeeded)
    {
        // perfrom reverse corrections
        perfromReverseCorrections();
    }

    return response;
}

void perfromReverseCorrections()
{
    bool correctingFinished = false;
    uint16_t lSpeed = -50;
    uint16_t rSpeed = -50;

    while (!correctingFinished)
    {
        sensorArray.readCalibrated(sensorValues);

        // Check if both sensors on the array have come off the wall
        if ((sensorValues[LEFT_IR_SNSR] < THRESHOLD_NEAR_LINE) &&
        (sensorValues[RIGHT_IR_SNSR] < THRESHOLD_NEAR_LINE))
        {
            rSpeed = 0;
            lSpeed = 0;
            correctingFinished = true;
        }
        else if ((sensorValues[LEFT_IR_SNSR] >= THRESHOLD_NEAR_LINE) &&
        (sensorValues[RIGHT_IR_SNSR] < THRESHOLD_NEAR_LINE))
        {
            // The right sensor has not reached the wall but the left has, adjust right track
            lSpeed = -80;
            rSpeed = 0;
        }
        else if ((sensorValues[RIGHT_IR_SNSR] >= THRESHOLD_NEAR_LINE) &&
        (sensorValues[LEFT_IR_SNSR] < THRESHOLD_NEAR_LINE))
        {
            // The right sensor has not reached the wall but the left has, adjust right track
            lSpeed = 0;
            rSpeed = -80;
        }
        else
        {
            // None of the scenarios above apply (but we are close), move the robot forward slowly until one fits
            lSpeed = -50;
            rSpeed = -50;
        }

        ZumoMotors::setSpeeds(lSpeed, rSpeed);
    }

    delay(150);
    ZumoMotors::setSpeeds(-50, -50);
    delay(100);
    ZumoMotors::setSpeeds(0, 0);
}

WALL_INFO driveStraightUntilLine (WALL_INFO wallInfo, DRIVE_DIRECTION driveDir)
{
    // Serial.println("Begin driveStraightUntilLine");
    // printWallInfo(wallInfo);

    bool wallDriveFinihsed = false;
    bool correctingFinished = false;
    uint16_t lSpeed = 50;
    uint16_t rSpeed = 50;
    uint32_t initCorrectionTime = 0;
    uint32_t initWallDriveTime = 0;
    uint16_t diffTime = 0;
    WALL_INFO response = wallInfo;


    /* Simply we keep going forward here, until we hit a wall or timeout compared
       to how long it would take to drive to the average wall, which would me we
       are driving into free space, either a turn or a room */

    initWallDriveTime = millis();
    while (!wallDriveFinihsed)
    {
        sensorArray.readCalibrated(sensorValues);
        wallDriveFinihsed = ((sensorValues[LEFT_IR_SNSR] >= THRESHOLD_ON_LINE) ||
                       (sensorValues[RIGHT_IR_SNSR] >= THRESHOLD_ON_LINE));

       if(wallDriveFinihsed)
        {
            lSpeed = 0;
            rSpeed = 0;

            // If there was no opposite wall on the other side of the corridor we do not want to
            // add the time it took to get back to the other wall, this would mess up our averages!
            if ((wallInfo.lastRightWall != WS_NO_WALL) && (wallInfo.lastLeftWall != WS_NO_WALL))
            {
                // Serial.println("calculate avarage");
                diffTime = millis() - initWallDriveTime;
                totalWallDriveTime += diffTime;
                numWallDrives++;
                averageWallDriveTime = totalWallDriveTime / numWallDrives;
            }

            // set the response to say we've hit a wall - we may or may not have hit it correctly
            setWallInfo(driveDir, &response, WS_HIT_WALL);
            // Serial.println("Hit a wall");
        }
        // if the last time we were at this side of the corridor we sensed a partial wall
        // or no wall at all, initiate a time out so we don't drive off into free space forever
        else if (((driveDir == LEFT && (wallInfo.lastLeftWall == WS_PARTIAL_WALL || wallInfo.lastLeftWall == WS_NO_WALL)) ||
                 (driveDir == RIGHT && (wallInfo.lastRightWall == WS_PARTIAL_WALL || wallInfo.lastRightWall == WS_NO_WALL))) &&
                 (numWallDrives > 5))
        {
            if ((millis() - initWallDriveTime) > (averageWallDriveTime * 2.5))
            {
                ZumoMotors::setSpeeds(0, 0);
                // Serial.println("No wall reached");
                // set the response to say that we haven't reached a wall and free space has been
                // detected, potential room or corridoor
                setWallInfo(driveDir, &response, WS_NO_WALL);
                return response;
            }
        }
        ZumoMotors::setSpeeds(lSpeed, rSpeed);
    }

    // Serial.println("Reached line");

   // This checks to see if both sensors on the edge of the sensor array have reached a wall
    sensorArray.readCalibrated(sensorValues);
    correctingFinished = ((sensorValues[LEFT_IR_SNSR] >= THRESHOLD_ON_LINE) &&
               (sensorValues[RIGHT_IR_SNSR] >= THRESHOLD_ON_LINE));
    delay(100);

    if (!correctingFinished)
    {
        // Serial.println("Performing corrections line");
        response = performCorrections(wallInfo, driveDir);
        // Serial.println("Corrections complete");
    }

    delay(50);

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
WALL_INFO performCorrections (WALL_INFO wallInfo, DRIVE_DIRECTION driveDir)
{
    // Serial.println("Begin performCorrections");

    bool correctingFinished = false;
    uint16_t lSpeed = 50;
    uint16_t rSpeed = 50;
    uint32_t initCorrectionTime = 0;
    uint16_t diffTime = 0;
    WALL_INFO response = wallInfo;

    initCorrectionTime = millis();
    while (!correctingFinished)
    {
        sensorArray.readCalibrated(sensorValues);

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
        if ((driveDir == LEFT) && (wallInfo.lastRightWall == WS_NO_WALL))
            timeOutCheck = false;

        if ((driveDir == RIGHT) && (wallInfo.lastLeftWall == WS_NO_WALL))
            timeOutCheck = false;

        if ((driveDir == LEFT) && (wallInfo.lastRightWall == WS_PARTIAL_WALL))
            timeOutCheck = false;

        if ((driveDir == RIGHT) && (wallInfo.lastLeftWall == WS_PARTIAL_WALL))
            timeOutCheck = false;

        if (timeOutCheck)
        {
            if ((millis() - initCorrectionTime > (averageCorrectionTime * 2)) && (numCorrections >= 8))
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
                    // Serial.println("Correction time out");
                }
                else if ((sensorValues[RIGHT_IR_SNSR] >= THRESHOLD_ON_LINE) &&
                        (sensorValues[LEFT_IR_SNSR] <= THRESHOLD_NEAR_LINE))
                {
                    lSpeed = 0;
                    rSpeed = 0;
                    correctingFinished = true;
                    setWallInfo(driveDir, &response, WS_PARTIAL_WALL);
                    // Serial.println("Correction time out");
                }
            }
        }
        ZumoMotors::setSpeeds(lSpeed,rSpeed);
    }

    // Delay to make motion less abrupt
    delay(50);
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
    Serial.print("lastLeftWall:    ");
    Serial.println(wallSenseToStr(info.lastLeftWall));

    Serial.print("lastRightWall:    ");
    Serial.println(wallSenseToStr(info.lastRightWall));

    Serial.print("lastForwardWall:    ");
    Serial.println(wallSenseToStr(info.lastForwardWall));

}

const char* wallSenseToStr (WALL_SENSE sense)
{
    switch (sense)
    {
        case WS_NIL:
        return "NIL";
        break;

        case WS_FULL_WALL:
        return "FULL WALL";
        break;

        case WS_PARTIAL_WALL:
        return "PARTIAL WALL";
        break;

        case WS_NO_WALL:
        return "NO WALL";
        break;

        case WS_NO_WALL_AHEAD:
        return "NO WALL AHEAD";
        break;

        case WS_HIT_WALL:
        return "HIT WALL";
        break;

        case WS_WALL_AHEAD:
        return "WALL AHEAD";
        break;
    }
}
