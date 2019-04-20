#ifndef BASIC_MOVEMENT_H
#define BASIC_MOVEMENT_H

#include <Arduino.h>

#include "pin_configuration.h"

void moveForward(int speed);
void moveBackward(int speed);
void turnRight(int speed);
void turnLeft(int speed);
void stop();

#endif