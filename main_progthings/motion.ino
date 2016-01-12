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
    const int16_t straightSpeed = 150;
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

WALL_SENSE driveStraightUntilLine (TurnSensor trn, WALL_SENSE wallSense)
{
    bool wallDriveFinihsed = false;
    bool correctingFinished = false;
    uint16_t lSpeed = 50;
    uint16_t rSpeed = 50;
    uint32_t initCorrectionTime = 0;
    uint32_t initWallDriveTime = 0;
    uint16_t diffTime = 0;

    // Serial.println("Go to line");

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

        diffTime = millis() - initWallDriveTime;
        Serial.println(averageWallDriveTime);
       if ((diffTime > (averageWallDriveTime * 1.2)) && (numWallDrives > 5))
       {
           ZumoMotors::setSpeeds(0, 0);
           break;
       }

       if(wallDriveFinihsed)
        {
            lSpeed = 0;
            rSpeed = 0;
            totalWallDriveTime += diffTime;
            numWallDrives++;
            averageWallDriveTime = totalWallDriveTime / numWallDrives;
        }
        ZumoMotors::setSpeeds(lSpeed, rSpeed);
    }

    // Serial.println("Reached line");

    /* Even though we have reached the target wall it is probable that the robots turn was off
       by a few degrees, this needs to be corrected so that it does not drift on future
       turns. We can use the sensors on opposite ends of the sensor array to determine if
       the robot is lined up with the target wall. If we drive straight at the target wall
       both sensors should say they have reached it, if not we can move the corresponding
       track to straighten the robot up */

   // This checks to see if both sensors on the edge of the sensor array have reached a wall
   if (wallDriveFinihsed)
    {
        correctingFinished = ((sensorValues[LEFT_IR_SNSR] >= THRESHOLD_ON_LINE) &&
                   (sensorValues[RIGHT_IR_SNSR] >= THRESHOLD_ON_LINE));
        delay(100);
    }


    // if (!correctingFinished)
    //     Serial.println("Perform corrections");

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
            lSpeed = 0;
            rSpeed = 0;
            correctingFinished = true;
            diffTime = millis() - initCorrectionTime;

            // Tally the correction times and calculate an average correction time to use as a timeout value
            totalCorrectionTime += diffTime;
            numCorrections++;
            averageCorrectionTime = totalCorrectionTime / numCorrections;
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
            rSpeed = 80;
        }
        else if ((sensorValues[RIGHT_IR_SNSR] >= THRESHOLD_ON_LINE) &&
        (sensorValues[LEFT_IR_SNSR] < THRESHOLD_ON_LINE))
        {
            // The right sensor has not reached the wall but the left has, adjust right track
            lSpeed = 80;
            rSpeed = 0;
        }
        else
        {
            // None of the scenarios above apply (but we are close), move the robot forward slowly until one fits
            lSpeed = 50;
            rSpeed = 50;
        }

        /* The average time it takes to perform a correction is tracked and this is unsed
           to time corrections that do not have full wall to correct up to (near corners & room entrances)
           if the current correction lasts longer that 1.5 times a normal correction, check to see
           if we might be near a partial wall */
        if ((millis() - initCorrectionTime > (averageCorrectionTime * 1.5)) && (numCorrections > 5))
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
            }
            else if ((sensorValues[RIGHT_IR_SNSR] >= THRESHOLD_ON_LINE) &&
                    (sensorValues[LEFT_IR_SNSR] <= THRESHOLD_NEAR_LINE))
            {
                lSpeed = 0;
                rSpeed = 0;
                correctingFinished = true;
            }
        }
        // Serial.print("spd ");
        // Serial.print(lSpeed);
        // Serial.print(", ");
        // Serial.print(rSpeed);
        // Serial.println("-----");
        ZumoMotors::setSpeeds(lSpeed,rSpeed);
    }
}
