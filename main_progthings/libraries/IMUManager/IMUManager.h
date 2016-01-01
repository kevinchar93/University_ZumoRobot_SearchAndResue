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

#ifndef IMUManager_h
#define IMUManager_h

#include "LSM303.h"
#include "L3G.h"
#include "Utilities.h"

class IMUManager {
public:
    IMUManager();

    static bool initGyro();
    static bool initAccel();
    static bool initMag();

    static void enableGyroDefault();
    static void enableAccelDefault();
    static void enableMagDefault();

    static void readGyro();
    static void readAccel();
    static void readMag();

    static void calibrateGyro(int numSeconds);
    static void calibrateAccelerometer(int numSeconds);
    static void calibrateMagnetometer(int numSamples);

    static float getGyroX();
    static float getGyroY();
    static float getGyroZ();

    static float getGyroYaw();
    static float getGyroPitch();
    static float getGyroRoll();

    static float getAccelX();
    static float getAccelY();
    static float getAccelZ();

    static float getMagX();
    static float getMagY();
    static float getMagZ();

    static void zeroGyroXAxis();
    static void zeroGyroYAxis();
    static void zeroGyroZAxis();

    static float getTiltAngle();
    static float getFilteredTiltAngle();

    /* raw acceleration values as measured by the accelerometer */
    static int accel_x;
    static int accel_y;
    static int accel_z;

    /* raw angular velocity values as measured by the gyro */
    static int gyro_x;
    static int gyro_y;
    static int gyro_z;

    /* raw magnetic field values as measured by the magnetometer */
    static int mag_x;
    static int mag_y;
    static int mag_z;
private:
    static LSM303 accel;
    static L3G gyro;

    static Utilities util;

    /* gyro variables */
    static float zAngle_gyro;
    static float yAngle_gyro;
    static float xAngle_gyro;

    static float zReferenceAngle_gyro;
    static float yReferenceAngle_gyro;
    static float xReferenceAngle_gyro;

    static int gyro_x_offset;
    static int gyro_y_offset;
    static int gyro_z_offset;

    static float avg_gyro_z_zero_offset;
    static float avg_gyro_y_zero_offset;
    static float avg_gyro_x_zero_offset;

    static float tuningFactor_gyro;
    static float filteredAngle_gyro;

    /* accelerometer variables */
    static float accel_x_offset;
    static float accel_y_offset;
    static float accel_z_offset;

    static bool finishedCalibration_accel;
    static long initCalibrationTime_accel;

    /* magnetometer variables */
    static LSM303::vector<int16_t> mag_min;
    static LSM303::vector<int16_t> mag_max;

    /* constants: based on the default setup */
    static const float GYRO_CONVERSION_FACTOR;   /* deg/s */
    static const float ACCEL_CONVERSION_FACTOR; /* g */
    static const float MAG_CONVERSION_FACTOR;   /* gauss */

    // const static float GYRO_SAMPLING_RATE
    // const static float ACCEL_SAMPLING_RATE
    // const static float MAG_SAMPLING_RATE

    static const float COMPLEMENTARY_FILTER_WEIGHT;
    static const int MAG_CALIBRATION_THRESHOLD;

};
#endif
