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


void rotateToAngle(TurnSensor trn, float angle, ZumoMotors motors)
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
            motors.setSpeeds(motorSpeed, -motorSpeed);
        }
        else
        {
                            // left speed, right speed
            motors.setSpeeds(-motorSpeed, motorSpeed);
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

        ZumoMotors::setLeftSpeed(leftSpeed);
        ZumoMotors::setRightSpeed(rightSpeed);
    }
}
