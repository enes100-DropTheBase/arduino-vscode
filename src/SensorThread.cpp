#include <Arduino.h>
#include <Thread.h>

#include "SensorThread.h"

void SensorThread::run() {
  Serial.print("Thread ID: ");
  Serial.print(this->ThreadID);
  Serial.print(": ");
  Serial.println("Reading value");
  runned();
}