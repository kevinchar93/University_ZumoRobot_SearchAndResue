#define FIRST_CALIBRATE_TIME 5

void initImuGyro(IMUManager _imu)
{
    /* initialise  gyro */
    if (!_imu.initGyro())
    {
        Serial.print("Failed to autodetect gyro type!");
        delay(10000);
    }
    _imu.enableGyroDefault();

    // Run first calibration, calibrate for 5 seconds
    calibrateImuGyro(_imu, FIRST_CALIBRATE_TIME);
}

void calibrateImuGyro(IMUManager _imu, int _calibrationTimeSec)
{
    _imu.calibrateGyro(_calibrationTimeSec);
    _imu.zeroGyroXAxis();
    _imu.zeroGyroYAxis();
    _imu.zeroGyroZAxis();
}
