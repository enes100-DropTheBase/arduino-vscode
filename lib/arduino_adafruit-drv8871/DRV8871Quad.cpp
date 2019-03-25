/*
  DRV8871Quad.cpp - Library for interacting with DRV8871Quad
*/

#include "DRV8871Quad.h"
#include "Arduino.h"
#include "DRV8871.h"

DRV8871Quad::DRV8871Quad(DRV8871* motor1, DRV8871* motor2, DRV8871* motor3,
                         DRV8871* motor4) {
  _motor1 = motor1;
  _motor2 = motor2;
  _motor3 = motor3;
  _motor4 = motor4;
}

void DRV8871Quad::drive(byte speed, byte direction) {
  _motor1->drive(speed, direction, 0);
  _motor2->drive(speed, direction, 0);
  _motor3->drive(speed, direction, 0);
  _motor4->drive(speed, direction, 0);
}

void DRV8871Quad::turn(byte speed, byte direction) {
  if (direction == TURN_LEFT) {
    _motor1->drive(speed, _motor1->DIRECTION_FORWARD, 0);
    _motor2->drive(speed, _motor1->DIRECTION_BACKWARD, 0);
    _motor3->drive(speed, _motor1->DIRECTION_FORWARD, 0);
    _motor4->drive(speed, _motor1->DIRECTION_BACKWARD, 0);
  } else if (direction == TURN_RIGHT) {
    _motor1->drive(speed, _motor1->DIRECTION_BACKWARD, 0);
    _motor2->drive(speed, _motor1->DIRECTION_FORWARD, 0);
    _motor3->drive(speed, _motor1->DIRECTION_BACKWARD, 0);
    _motor4->drive(speed, _motor1->DIRECTION_FORWARD, 0);
  }
}

void DRV8871Quad::breakdown(byte targetSpeed) {
  _motor1->breakdown(targetSpeed, 0);
  _motor2->breakdown(targetSpeed, 0);
  _motor3->breakdown(targetSpeed, 0);
  _motor4->breakdown(targetSpeed, 0);
}