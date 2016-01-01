/*
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "IMUManager.h"

#ifndef M_PI
#define M_PI    3.14159265358979323846
#endif

LSM303 IMUManager::accel;
L3G IMUManager::gyro;
Utilities IMUManager::util;

float IMUManager::zAngle_gyro = 0;
float IMUManager::yAngle_gyro = 0;
float IMUManager::xAngle_gyro = 0;

float IMUManager::zReferenceAngle_gyro = 0;
float IMUManager::yReferenceAngle_gyro = 0;
float IMUManager::xReferenceAngle_gyro = 0;

float IMUManager::tuningFactor_gyro = 0;
float IMUManager::filteredAngle_gyro = 0;

float IMUManager::accel_x_offset = 0;
float IMUManager::accel_y_offset = 0;
float IMUManager::accel_z_offset = 0;

int IMUManager::gyro_x_offset = 0;
int IMUManager::gyro_y_offset = 0;
int IMUManager::gyro_z_offset = 0;

bool IMUManager::finishedCalibration_accel = 0;
long IMUManager::initCalibrationTime_accel = 0;

int IMUManager::accel_x = 0;
int IMUManager::accel_y = 0;
int IMUManager::accel_z = 0;

int IMUManager::gyro_x = 0;
int IMUManager::gyro_y = 0;
int IMUManager::gyro_z = 0;

int IMUManager::mag_x = 0;
int IMUManager::mag_y = 0;
int IMUManager::mag_z = 0;

/* initialize the minimum values to the highest possible values
 * and the maximum values to the lowest possible values so that 
 * their initial values will form a upper and lower bound for calibration
 */
LSM303::vector<int16_t> IMUManager::mag_min = {32767, 32767, 32767};
LSM303::vector<int16_t> IMUManager::mag_max = {-32767, -32767, -32767};

IMUManager::IMUManager()
{
    accel = LSM303();
    gyro = L3G();
}

bool IMUManager::initGyro()
{
    return gyro.init();
}

bool IMUManager::initAccel()
{
    return accel.init();
}

bool IMUManager::initMag()
{
    return accel.init();
}

void IMUManager::enableGyroDefault()
{
    gyro.enableDefault();
}

void IMUManager::enableAccelDefault()
{
    accel.enableDefault();
}

void IMUManager::enableMagDefault()
{
    accel.enableDefault();
}

/*
 * ======== readGyro ========
 * Read gyroscope data and calculate integrated angles
 */
void IMUManager::readGyro()
{
    static long lastTime_gyro = 0;

    gyro.read();

    gyro_x = (int) (gyro.g.x - gyro_x_offset);
    gyro_y = (int) (gyro.g.y - gyro_y_offset);
    gyro_z = (int) (gyro.g.z - gyro_z_offset);

    if (lastTime_gyro != 0) {
        long currTime_gyro = millis();
        float zDps = gyro_z * GYRO_CONVERSION_FACTOR;
        float yDps = gyro_y * GYRO_CONVERSION_FACTOR;
        float xDps = gyro_x * GYRO_CONVERSION_FACTOR;
        float time = ((currTime_gyro - lastTime_gyro) / 1000.0f);
        zAngle_gyro += zDps * time;
        yAngle_gyro += yDps * time;
        xAngle_gyro += xDps * time;
        lastTime_gyro = currTime_gyro;
    }
    else {
        lastTime_gyro = millis();
    }
}

void IMUManager::readAccel()
{
    accel.readAcc();

    accel_x = accel.a.x;
    accel_y = accel.a.y;
    accel_z = accel.a.z;
}

void IMUManager::readMag()
{
    accel.readMag();

    mag_x = accel.m.x;
    mag_y = accel.m.y;
    mag_z = accel.m.z;
}

/*
 * ======== calibrateGyro ========
 * Calculate and offset error for each of the gyro's axes
 *
 * Sums the gyro readings over a period of time and averages 
 * them to identify a stationary offset to be subtracted 
 * from the raw gyro values. Assumes that there is no angular
 * rotation during the calibration period.
 */
void IMUManager::calibrateGyro(int numSeconds)
{
    int sumX = 0;
    int sumY = 0;
    int sumZ = 0;
    int count = 0;
    unsigned long startTime = millis();

    do {
        gyro.read();
        sumX += ((int) gyro.g.x);
        sumY += ((int) gyro.g.y);
        sumZ += ((int) gyro.g.z);
        count++;
        delay(2);
    } while (((millis() - startTime) / 1000.0) < numSeconds);

    Serial.print("Over a period of ");
    Serial.print(numSeconds);
    Serial.print(" seconds, gyro drifted ");
    Serial.print(sumX);
    Serial.print(" around the x-axis and ");
    Serial.print(sumY);
    Serial.print(" around the y-axis and ");
    Serial.print(sumZ);
    Serial.println(" around the z-axis");

    gyro_x_offset = (int)(sumX / (float)(count) + 0.5f);
    gyro_y_offset = (int)(sumY / (float)(count) + 0.5f);
    gyro_z_offset = (int)(sumZ / (float)(count) + 0.5f);

    Serial.print("X Zero offset: ");
    Serial.println(gyro_x_offset);
    Serial.print("Y Zero offset: ");
    Serial.println(gyro_y_offset);
    Serial.print("Z Zero offset: ");
    Serial.println(gyro_z_offset);
}

/*
 * ======== calibrateAccelerometer ========
 * Calculate and offset error for each of the accelerometer's axes
 *
 * Sums the accelerometer readings over a period of time and averages 
 * them to identify a stationary offset to be subtracted from the 
 * raw accelerometer values. Assumes that the X and Y accelerations 
 * are zero and the Z acceleration is 1 g during the calibration
 * period.
 */
void IMUManager::calibrateAccelerometer(int numSeconds)
{
    finishedCalibration_accel = false;
    float sumZReadings = 0;
    float sumYReadings = 0;
    float sumXReadings = 0;
    float newZVal = 0;
    float newYVal = 0;
    float newXVal = 0;
    int count = 0;

    if (initCalibrationTime_accel == 0) {
        initCalibrationTime_accel = millis();
    }
    else {
        while (!finishedCalibration_accel) {
            accel.read();
            finishedCalibration_accel = (((millis() - initCalibrationTime_accel)
                    / 1000.0) >= numSeconds);
            newZVal = ((int) accel.a.z * ACCEL_CONVERSION_FACTOR);
            newYVal = ((int) accel.a.y * ACCEL_CONVERSION_FACTOR);
            newXVal = ((int) accel.a.x * ACCEL_CONVERSION_FACTOR);
            sumZReadings += newZVal;
            sumYReadings += newYVal;
            sumXReadings += newXVal;
            count++;
            if (finishedCalibration_accel) {
                accel_y_offset = -((float) (sumYReadings / (float) (count)));
                accel_x_offset = -((float) (sumXReadings / (float) (count)));
                accel_z_offset = -(((float) (sumZReadings) / (float) (count))
                        - 1.0f);
                Serial.print("Accelerometer X offset: ");
                Serial.print(accel_x_offset);
                Serial.print(" g's, Y offset: ");
                Serial.print(accel_y_offset);
                Serial.print(" g's, Z offset: ");
                Serial.print(accel_z_offset);
            }
        }
        initCalibrationTime_accel = 0;
    }
}

/*
 * ======== calibrateMagnetometer ========
 * Track magnetometer data to adjust for inherent bias/error
 *
 * Finds the max and min values for each the magnetometer's axis
 * readings in order to normalize the raw measurements from -1.0 
 * to 1.0. Run this method while rotating the zumo in all three
 * rotational degrees of freedom (roll, pitch yaw).
 *
 * Runs until a minimum of numSamples are obtained and a minimum
 * difference between min and max values is detected
 */
void IMUManager::calibrateMagnetometer(int numSamples)
{
    unsigned int i = 0;
    int currentSumDifferences = 0; /* once this values exceeds a certain amount, calibration is done */

    while ((i < numSamples) || (currentSumDifferences < MAG_CALIBRATION_THRESHOLD))
    {
        accel.readMag();

        /* if new value is lower than min, update the min */
        mag_min.x = util.min(mag_min.x, accel.m.x);
        mag_min.y = util.min(mag_min.y, accel.m.y);
        mag_min.z = util.min(mag_min.z, accel.m.z);

        /* if new value is higher than max, update the max */
        mag_max.x = util.max(mag_max.x, accel.m.x);
        mag_max.y = util.max(mag_max.y, accel.m.y);
        mag_max.z = util.max(mag_max.z, accel.m.z);

        currentSumDifferences = (mag_max.x-mag_min.x)+(mag_max.y-mag_min.y)+(mag_max.z-mag_min.z);

        i++;
        delay(50);
    }

    if (mag_max.x == mag_min.x) mag_max.x = mag_min.x + 1;
    Serial.print("min.x   ");
    Serial.print(mag_min.x);
    Serial.println();
    Serial.print("max.x   ");
    Serial.print(mag_max.x);
    Serial.println();
    
    if (mag_max.y == mag_min.y) mag_max.y = mag_min.y + 1;
    Serial.print("min.y   ");
    Serial.print(mag_min.y);
    Serial.println();
    Serial.print("max.y   ");
    Serial.print(mag_max.y);
    Serial.println();

    if (mag_max.z == mag_min.z) mag_max.z = mag_min.z + 1;
    Serial.print("min.z   ");
    Serial.print(mag_min.z);
    Serial.println();
    Serial.print("max.z   ");
    Serial.print(mag_max.z);
    Serial.println();

    /* Set calibrated values to accel.m_max and accel.m_min */
    accel.m_max.x = mag_max.x;
    accel.m_max.y = mag_max.y;
    accel.m_max.z = mag_max.z;

    accel.m_min.x = mag_min.x;
    accel.m_min.y = mag_min.y;
    accel.m_min.z = mag_min.z;
}

/* return the angular velocity measured along the X, Y, and Z axis in deg/s */
float IMUManager::getGyroX()
{
    return gyro_x * GYRO_CONVERSION_FACTOR;
}

float IMUManager::getGyroY()
{
    return gyro_y * GYRO_CONVERSION_FACTOR;
}

float IMUManager::getGyroZ()
{
    return gyro_z * GYRO_CONVERSION_FACTOR;
}

/* return the roll pitch and yaw angles in degrees */
float IMUManager::getGyroRoll()
{
    return util.wrapAngle(xAngle_gyro - xReferenceAngle_gyro);
}

float IMUManager::getGyroPitch()
{
    return util.wrapAngle(yAngle_gyro - yReferenceAngle_gyro);
}

float IMUManager::getGyroYaw()
{
    return util.wrapAngle(zAngle_gyro - zReferenceAngle_gyro);
}

/* return the gyro's abolute heading angle at the current angle 
   along the X, Y and Z axis */
void IMUManager::zeroGyroXAxis()
{
    xReferenceAngle_gyro = xAngle_gyro;
}

void IMUManager::zeroGyroYAxis()
{
    yReferenceAngle_gyro = yAngle_gyro;
}

void IMUManager::zeroGyroZAxis()
{
    zReferenceAngle_gyro = zAngle_gyro;
}

/* return the acceleration measured along the X, Y, and Z axis */
float IMUManager::getAccelX()
{
    return ((accel.a.x - accel_x_offset) * ACCEL_CONVERSION_FACTOR);
}

float IMUManager::getAccelY()
{
    return ((accel.a.y - accel_y_offset) * ACCEL_CONVERSION_FACTOR);
}

float IMUManager::getAccelZ()
{
    return ((accel.a.z - accel_z_offset) * ACCEL_CONVERSION_FACTOR);
}

/* return the normalized magnetic field reading measured along the X, Y, and Z axis */
float IMUManager::getMagX()
{
    return 2.0*(float)(accel.m.x - accel.m_min.x) / ( accel.m_max.x - accel.m_min.x) - 1.0;
}

float IMUManager::getMagY()
{
    return 2.0*(float)(accel.m.y - accel.m_min.y) / ( accel.m_max.y - accel.m_min.y) - 1.0;
}

float IMUManager::getMagZ()
{
    return 2.0*(float)(accel.m.z - accel.m_min.z) / ( accel.m_max.z - accel.m_min.z) - 1.0;
}

/*
 * ======== getTiltAngle ========
 * Returns the angle tilted with respect to vertical
 *
 * Calculated by taking the inverse tangent of the z-axis
 * acceleration (gravity) over the x-axis acceleration
 */
float IMUManager::getTiltAngle()
{
    float xAcc = getAccelX();
    float zAcc = getAccelZ();
    float angInRads = (float) atan2(zAcc, xAcc);

    return util.wrapAngle(((angInRads * 180.0) / M_PI)-180);
}

/*
 * ======== getFilteredTiltAngle ========
 * Returns the angle tilted with respect to vertical
 *
 * Calculated by combining the gyro angular velocity data
 * and the accelerometer's tilt angle
 */
float IMUManager::getFilteredTiltAngle()
{
    filteredAngle_gyro = (COMPLEMENTARY_FILTER_WEIGHT * (filteredAngle_gyro + getGyroY()*0.0050)) 
                         + ((1-COMPLEMENTARY_FILTER_WEIGHT) * getTiltAngle());
    return filteredAngle_gyro;
}






