#include <Arduino.h>
#include <DRV8871.h>
#include <DRV8871Quad.h>
#include <Enes100.h>
#include <Thread.h>
#include <ThreadController.h>
#include <TimerOne.h>

#include "SensorThread.h"

// pin configuration (for Arduino Mega)
// Motors need PWM pins

// Front Right
#define FR_MOTOR_IN1 6
#define FR_MOTOR_IN2 7

// Front Left
#define FL_MOTOR_IN1 8
#define FL_MOTOR_IN2 9

// Back Right
#define BR_MOTOR_IN1 10
#define BR_MOTOR_IN2 11

// Back Left
#define BL_MOTOR_IN1 12
#define BL_MOTOR_IN2 13

#define SPEED1 255
#define MAX_SPEED 255

#define MARKER_ID 1
#define APC_RX 19
#define APC_TX 18

void moveForward(int speed);
void moveBackward(int speed);
void turnRight(int speed);
void turnLeft(int speed);

/*
SensorThread analog1 = SensorThread();
SensorThread analog2 = SensorThread();

// Instantiate a new ThreadController
ThreadController controller = ThreadController();

// This is the callback for the Timer
void timerCallback() { controller.run(); } */

void setup() {
  Enes100.begin("Drop the Base", CHEMICAL, MARKER_ID, APC_RX, APC_TX);

  Enes100.print("Destination is at (");
  Enes100.print(Enes100.destination.x);
  Enes100.print(", ");
  Enes100.print(Enes100.destination.y);
  Enes100.println(")");
  Serial.begin(9600);
  Serial.println("Serial Output");

  /*

  analog1.pin = A1;
  analog1.setInterval(1000);

  analog2.pin = A2;
  analog2.setInterval(1000);

  controller.add(&analog1);
  controller.add(&analog2);

  Timer1.initialize(20000);
  Timer1.attachInterrupt(timerCallback);
  Timer1.start();

  */
}

void loop() {
  // Update the OSV's current location
  if (Enes100.updateLocation()) {
    Enes100.print("OSV is at (");
    Enes100.print(Enes100.location.x);
    Enes100.print(", ");
    Enes100.print(Enes100.location.y);
    Enes100.print(", ");
    Enes100.print(Enes100.location.theta);
    Enes100.println(")");
  } else {
    // OSV's location was not found
    Enes100.println("404 Not Found");
  }

  moveForward(SPEED1);

  delay(10000);

  /*

    Serial.print("Sensor Value: ");
    Serial.println(analog1.value);

    */
}

void moveForward(int speed) {
  digitalWrite(FR_MOTOR_IN1, LOW);
  analogWrite(FR_MOTOR_IN2, speed);

  digitalWrite(FL_MOTOR_IN1, LOW);
  analogWrite(FL_MOTOR_IN2, speed);

  digitalWrite(BR_MOTOR_IN1, LOW);
  analogWrite(BR_MOTOR_IN2, speed);

  digitalWrite(BL_MOTOR_IN1, LOW);
  analogWrite(BL_MOTOR_IN2, speed);
}

void moveBackward(int speed) {
  digitalWrite(FR_MOTOR_IN2, LOW);
  analogWrite(FR_MOTOR_IN1, speed);

  digitalWrite(FL_MOTOR_IN2, LOW);
  analogWrite(FL_MOTOR_IN1, speed);

  digitalWrite(BR_MOTOR_IN2, LOW);
  analogWrite(BR_MOTOR_IN1, speed);

  digitalWrite(BL_MOTOR_IN2, LOW);
  analogWrite(BL_MOTOR_IN1, speed);
}

void turnRight(int speed) {
  digitalWrite(FR_MOTOR_IN2, LOW);
  analogWrite(FR_MOTOR_IN1, speed);

  digitalWrite(FL_MOTOR_IN1, LOW);
  analogWrite(FL_MOTOR_IN2, speed);

  digitalWrite(BR_MOTOR_IN2, LOW);
  analogWrite(BR_MOTOR_IN1, speed);

  digitalWrite(BL_MOTOR_IN1, LOW);
  analogWrite(BL_MOTOR_IN2, speed);
}

void turnLeft(int speed) {
  digitalWrite(FR_MOTOR_IN1, LOW);
  analogWrite(FR_MOTOR_IN2, speed);

  digitalWrite(FL_MOTOR_IN2, LOW);
  analogWrite(FL_MOTOR_IN1, speed);

  digitalWrite(BR_MOTOR_IN1, LOW);
  analogWrite(BR_MOTOR_IN2, speed);

  digitalWrite(BL_MOTOR_IN2, LOW);
  analogWrite(BL_MOTOR_IN1, speed);
}