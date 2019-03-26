#ifndef SensorThread_h
#define SensorThread_h

#include "Arduino.h"

class SensorThread : public Thread {
 public:
  int value;
  int pin;
  void run();
};

#endif