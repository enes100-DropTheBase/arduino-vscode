/*
  DRV8871Quad.h - Library for interacting with DRV8871Quad
*/

#ifndef DRV8871Quad_h
#define DRV8871Quad_h

#include "Arduino.h"
#include "DRV8871.h"

class DRV8871Quad {
 public:
  DRV8871Quad(DRV8871* motor1, DRV8871* motor2, DRV8871* motor3,
              DRV8871* motor4);
  const byte DIRECTION_FORWARD = 1;
  const byte DIRECTION_BACKWARD = 2;
  const byte TURN_LEFT = 2;
  const byte TURN_RIGHT = 3;
  void drive(byte speed, byte direction);
  void turn(byte speed, byte direction);
  void breakdown(byte targetSpeed = 0);

 private:
  DRV8871* _motor1;
  DRV8871* _motor2;
  DRV8871* _motor3;
  DRV8871* _motor4;
};
#endif
