#define THRESHOLD_NEAR_LINE 200
#define THRESHOLD_ON_LINE 399

#define LEFT_IR_SNSR 0
#define RIGHT_IR_SNSR 5

/* This function calibrates the reflectance sensor array by sweeping the reflectance
   sensors over the lightest and darkest things that it will encounter and calling the
   calibrate function */
void calibrateSensorArray ()
{
    const int spinSpeed = 150;
    sensorArray.init();

    for(int i = 0; i < 160; i++)
    {
      if ((i > 10 && i <= 30) || (i > 50 && i <= 70) || (i > 90 && i <= 110) ||  (i > 130 && i <= 150) )
      {
          // when counter is on specified numbers above spin right
        ZumoMotors::setSpeeds(-spinSpeed, spinSpeed);
      }
      else
      {
          // on all other numbers spin left
        ZumoMotors::setSpeeds(spinSpeed, -spinSpeed);
      }

      // call this repeatedly to calibrate the sensorArray
      sensorArray.calibrate();
      delay(20);
    }

    // stop robot moving
    ZumoMotors::setSpeeds(0, 0);
}
