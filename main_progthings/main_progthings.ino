#include <QTRSensors.h>
#include <ZumoBuzzer.h>
#include <Pushbutton.h>
#include <ZumoReflectanceSensorArray.h>
#include <ZumoMotors.h>

ZumoMotors motors;

void setup() {
  // put your setup code here, to run once:

}

void loop() {

  move(motors, 200, 80, 400, 0);

}

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
