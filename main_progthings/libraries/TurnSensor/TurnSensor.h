/* Turnsensor.h and TurnSensor.cpp provide functions for
configuring the L3GD20H gyro, calibrating it, and using it to
measure how much the robot has turned about its Z axis. */

#ifndef TURN_SENSOR_H
#define TURN_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <L3G.h>

class TurnSensor
{
    public:
        TurnSensor();
        TurnSensor(HardwareSerial &print);
        void init();
        void reset();
        void update();
        int16_t getTurnRate();
        int32_t getCurrentHeading();

        // This constant represents a turn of 45 degrees.
        const int32_t turnAngle45 = 0x20000000;

        // This constant represents a turn of 90 degrees.
        const int32_t turnAngle90 = turnAngle45 * 2;

        // This constant represents a turn of approximately 1 degree.
        const int32_t turnAngle1 = (turnAngle45 + 22) / 45;

    private:
        void calibrate(int numTimes);

        /* turnAngle is a 32-bit unsigned integer representing the amount
        the robot has turned since the last time turnSensorReset was
        called.  This is computed solely using the Z axis of the gyro, so
        it could be inaccurate if the robot is rotated about the X or Y
        axes.

        Our convention is that a value of 0x20000000 represents a 45
        degree counter-clockwise rotation.  This means that a uint32_t
        can represent any angle between 0 degrees and 360 degrees.  If
        you cast it to a signed 32-bit integer by writing
        (int32_t)turnAngle, that integer can represent any angle between
        -180 degrees and 180 degrees. */
        uint32_t _turnAngle;

        // turnRate is the current angular rate of the gyro, in units of
        // 0.07 degrees per second.
        int16_t _turnRate;

        // This is the average reading obtained from the gyro's Z axis
        // during calibration.
        int16_t _gyroOffset;

        // This variable helps us keep track of how much time has passed
        // between readings of the gyro.
        uint32_t _gyroLastUpdate;

        L3G _gyro;
        bool _debugOn;
        HardwareSerial* _print;
};

#endif
