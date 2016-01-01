#include <QTRSensors.h>
#include <ZumoBuzzer.h>
#include <Pushbutton.h>
#include <ZumoReflectanceSensorArray.h>
#include <ZumoMotors.h>
#include <Wire.h>
#include <IMUManager.h>

#define CALIBRATE_TIME 5000
#define TOP_SPEED 200
#define NUM_SENSORS 6

ZumoBuzzer buzzer;
ZumoReflectanceSensorArray sensorArray;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);

IMUManager imu;
bool zeroGyro = false;
bool gyroInZeroMode = true;


unsigned int sensorVals[NUM_SENSORS];

void setup()
{
    Serial.begin(9600);
    Serial.println("sensor init phase");

    Wire.begin();
    imu = IMUManager();

    /* initialize Zumo accelerometer and magnetometer */
    imu.initAccel();
    imu.enableAccelDefault();

    /* initialize Zumo gyro */
    if (!imu.initGyro()) {
        Serial.print("Failed to autodetect gyro type!");
        delay(1000);
    }
    imu.enableGyroDefault();

    Serial.println("Calibrating gyro for 2 seconds: keep zumo still during calibration period");
    gyroInZeroMode = true;
    imu.calibrateGyro(2);
    imu.zeroGyroXAxis();
    imu.zeroGyroYAxis();
    imu.zeroGyroZAxis();
    gyroInZeroMode = false;

    Serial.println("sensorSetup done.");

}

void loop()
{
    int imuCount = 0;

    /* update IMU data every 5 ms (200 Hz) */

    if (zeroGyro) {
        imu.calibrateGyro(1);
        imu.zeroGyroXAxis();
        imu.zeroGyroYAxis();
        imu.zeroGyroZAxis();
        zeroGyro = false;
    }

    /* read data from all IMU sensors */
    imu.readGyro();

    float currHeading = IMUManager::getGyroYaw();

    Serial.println(currHeading);
}

/*
void calibrateSensorArray ()
{
    boolean keepLooping = false;
    unsigned long beginTime = 0;
    int spinSpeed = 220;
    unsigned int position = 0;

    // innit the sensor array
    sensorArray.init();

    // wait for the user press before beginning, play sound, wait before starting
    button.waitForButton();
    buzzer.play(">g32>>c32");
    delay(3000);

    // begin looping and performing calibration
    keepLooping = true;

    beginTime = millis();

    while (keepLooping)
    {
        if((millis() - beginTime) < CALIBRATE_TIME)
        {
            sensorArray.calibrate();
        }
        else
        {
            position = sensorArray.readLine(sensorVals);
            if ((position > MID_POINT_BOTTOM) && (position  < MID_POINT_UPPER))
            {
                spinSpeed = 0;
                keepLooping = false;
            }
        }
        move(motors, spinSpeed, -spinSpeed, 1, 0);
    }
    buzzer.play(">g32>>c32");
    button.waitForButton();
    buzzer.play(">g32>>c32");
    delay(1000);
}

/*
int calcLeftSpeed (int pos, int max)
{
    if (pos >= MID_POINT)
    {
        return max;
    }
    else
    {
        float mul = (float) pos / (float) MID_POINT;
        return (int) (mul * max);
    }
}

int calcRightSpeed (int pos, int max)
{
    if (pos <= MID_POINT)
    {
        return max;
    }
    else
    {
        float mul = (float) (pos - MID_POINT) / (float) MID_POINT;
        return (int) (mul * max);
    }
}
*/
void move(ZumoMotors zm, int leftSpeed, int rightSpeed,
                unsigned int times, unsigned int wait) {
    for (int i = 0; i < times; i++){
        zm.setSpeeds(leftSpeed, rightSpeed);
        delay(wait);
    }
}

void moveForward(ZumoMotors zm, unsigned int speed, unsigned int times, unsigned int wait) {
    for (int i = 0; i < times; i++){
        zm.setSpeeds(speed, speed);
        delay(wait);
    }
}

void moveBackward(ZumoMotors zm, unsigned int speed, unsigned int times, unsigned int wait) {
    for (int i = 0; i < times; i++){
        zm.setSpeeds(-speed, -speed);
        delay(wait);
    }
}
