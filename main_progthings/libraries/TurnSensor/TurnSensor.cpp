/* Turnsensor.h and TurnSensor.cpp provide functions for
configuring the L3GD20H gyro, calibrating it, and using it to
measure how much the robot has turned about its Z axis.

 Original implementation at:
 https://github.com/pololu/zumo-32u4-arduino-library/tree/master/examples/RotationResist

 Modified by Kevin Charles, 07/01/16 for easier usage in Assignment

*/

#include "TurnSensor.h"

// Create Trun sensro with debugging off
TurnSensor::TurnSensor()
{
    _debugOn = false;
    _turnRate = 0;
    _turnAngle = 0;
    _gyroOffset = 0;
    _gyroLastUpdate = 0;
}

// Create turnSensor with debugging on
TurnSensor::TurnSensor(HardwareSerial &print)
{
    _debugOn = true;
    _print = &print;
    _turnRate = 0;
    _turnAngle = 0;
    _gyroOffset = 0;
    _gyroLastUpdate = 0;
}

void TurnSensor::init()
{
    Wire.begin();
    _gyro.init();

    // 800 Hz output data rate,
    // low-pass filter cutoff 100 Hz
    _gyro.writeReg(L3G::CTRL1, 0b11111111);

    // 2000 dps full scale
    _gyro.writeReg(L3G::CTRL4, 0b00100000);

    // High-pass filter disabled
    _gyro.writeReg(L3G::CTRL5, 0b00000000);

    if(_debugOn) {_print->println("Beginning Gyro Calibration");}

    // Calibrate the gyro
    calibrate(5120);

    if(_debugOn) {_print->println("Gyro Calibration Complete");}

    // Reset the measurmenst before returning
    this->reset();
}

// This should be called to set the starting point for measuring
// a turn.  After calling this, turnAngle will be 0.
void TurnSensor::reset()
{
    _gyroLastUpdate = micros();
    _turnAngle = 0;
}

// Read the gyro and update the angle.  This should be called as
// frequently as possible while using the gyro to do turns.
void TurnSensor::update()
{
    // Read the measurements from the gyro.
    _gyro.read();
    _turnRate = _gyro.g.z - _gyroOffset;

    // Figure out how much time has passed since the last update dt(delta time)
    uint16_t m = micros();
    uint16_t dt = m - _gyroLastUpdate;
    _gyroLastUpdate = m;

    // Multiply dt by turnRate in order to get an estimation of how
    // much the robot has turned since the last update.
    // (angular change = angular velocity * time)
    int32_t da = (int32_t)_turnRate * dt;

    // The units of da are gyro digits times microseconds.  We need
    // to convert those to the units of turnAngle, where 2^29 units
    // represents 45 degrees.  The conversion from gyro digits to
    // degrees per second (dps) is determined by the sensitivity of
    // the gyro: 0.07 degrees per second per digit.
    //
    // (0.07 dps/digit) * (1/1000000 s/us) * (2^29/45 unit/degree)
    // = 14680064/17578125 unit/(digit*us)
    _turnAngle += (int64_t)da * 14680064 / 17578125;
}

int16_t TurnSensor::getTurnRate()
{
    return _turnRate;
}

// Return _turnAngle as angle between -180 degrees and 180 degrees
int32_t TurnSensor::getCurrentHeading()
{
    return (((int32_t)_turnAngle >> 16) * 360) >> 16;
}

// Calibrate the gyro by calculating an offset for any drift
void TurnSensor::calibrate(int numTimes)
{
    int32_t total = 0;
    for (uint16_t i = 0; i < numTimes; i++)
    {
      // Wait for new data to be available, then read it.
      while(!_gyro.readReg(L3G::STATUS_REG) & 0x08);
      _gyro.read();

      // Add the Z axis reading to the total.
      total += _gyro.g.z;
    }
    _gyroOffset = total / numTimes;
}
