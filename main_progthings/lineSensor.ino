#define LINE_SENSOR_CALIBRATE_TIME 10000

#define THRESHOLD_NEAR_LINE 200
#define THRESHOLD_ON_LINE 399

#define LEFT_IR_SNSR 0
#define RIGHT_IR_SNSR 5

void calibrateSensorArray ()
{
    const int spinSpeed = 200;
    sensorArray.init();

    for(int i = 0; i < 160; i++)
    {
      if ((i > 10 && i <= 30) || (i > 50 && i <= 70) || (i > 90 && i <= 110) ||  (i > 130 && i <= 150) )
      {
        ZumoMotors::setSpeeds(-spinSpeed, spinSpeed);
      }
      else
      {
        ZumoMotors::setSpeeds(spinSpeed, -spinSpeed);
      }

      sensorArray.calibrate();
      delay(20);
    }

    ZumoMotors::setSpeeds(0, 0);
}
