#include <PIDController.h>

#define ROT_PROPOTION 6.5
#define ROT_INTEGRAL 0.001
#define ROT_DIRIVATIVE 0.0

#define PID_DIRECT 0
#define PID_REVERSE 1
#define SAMPLE_TIME 100

#define UP_BOUND_ROT 1.0
#define LOW_BOUND_ROT -1.0

#define TOP_SPEED 180
#define MIN_SPEED 10


void rotateToAngle(IMUManager _imu, float _angle, ZumoMotors _motors)
{
    PIDController rotationPID = PIDController((float) 6.5, (float) 0.001, (float) 0.0);
    float error, motorSpeed, targetAngle, currentHeading;
    bool keepLooping = true;

    _imu.readGyro();
    currentHeading = IMUManager::getGyroYaw();
    targetAngle = Utilities::wrapAngle(currentHeading + _angle);

    while (keepLooping)
    {
        _imu.readGyro();
        error = Utilities::wrapAngle(IMUManager::getGyroYaw() - targetAngle);
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
        else if (motorSpeed < MIN_SPEED)
        {
            motorSpeed = 0;
        }

        if (error > 0)
        {
                            // left speed, right speed
            _motors.setSpeeds(motorSpeed, -motorSpeed);
        }
        else
        {
                            // left speed, right speed
            _motors.setSpeeds(-motorSpeed, motorSpeed);
        }
    }
}
