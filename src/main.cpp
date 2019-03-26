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
#define MOTOR1_IN1 6
#define MOTOR1_IN2 7
#define MOTOR2_IN1 8
#define MOTOR2_IN2 9
#define MOTOR3_IN1 10
#define MOTOR3_IN2 11
#define MOTOR4_IN1 12
#define MOTOR4_IN2 13

#define SPEED1 50
// Is this right?
#define MAX_SPEED 255

#define MARKER_ID 1
#define APC_RX 19
#define APC_TX 18

DRV8871 frontRightMotor(MOTOR1_IN1, MOTOR1_IN2);
DRV8871 frontLeftMotor(MOTOR2_IN1, MOTOR2_IN2);
DRV8871 backRightMotor(MOTOR3_IN1, MOTOR3_IN2);
DRV8871 backLeftMotor(MOTOR4_IN1, MOTOR4_IN2);

DRV8871Quad quadMotorController(&frontRightMotor, &frontLeftMotor,
                                &backRightMotor, &backLeftMotor);

SensorThread analog1 = SensorThread();
SensorThread analog2 = SensorThread();

// Instantiate a new ThreadController
ThreadController controller = ThreadController();

// This is the callback for the Timer
void timerCallback() { controller.run(); }

void setup() {
  Enes100.begin("Drop the Base", CHEMICAL, MARKER_ID, APC_RX, APC_TX);

  Enes100.print("Destination is at (");
  Enes100.print(Enes100.destination.x);
  Enes100.print(", ");
  Enes100.print(Enes100.destination.y);
  Enes100.println(")");
  Serial.begin(9600);
  Serial.println("Serial Output");

  analog1.pin = A1;
  analog1.setInterval(1000);

  analog2.pin = A2;
  analog2.setInterval(1000);

  controller.add(&analog1);
  controller.add(&analog2);

  Timer1.initialize(20000);
  Timer1.attachInterrupt(timerCallback);
  Timer1.start();
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

  Serial.print("Sensor Value: ");
  Serial.println(analog1.value);

  Serial.println("Moving Forward");
  quadMotorController.drive(SPEED1, quadMotorController.DIRECTION_FORWARD);
  delay(2000);
  Serial.println("Turning Left");
  quadMotorController.turn(10, quadMotorController.TURN_LEFT);
  delay(2000);
  Serial.println("Moving Backward");
  quadMotorController.drive(SPEED1, quadMotorController.DIRECTION_BACKWARD);
  delay(2000);
  Serial.println("Turning Right");
  quadMotorController.turn(10, quadMotorController.TURN_RIGHT);
  delay(2000);
  Serial.println("Moving Forward");
  quadMotorController.drive(SPEED1, quadMotorController.DIRECTION_FORWARD);
  delay(1000);
  Serial.println("Stopping");
  quadMotorController.breakdown();
  delay(2000);
}