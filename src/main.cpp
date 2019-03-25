#include <Arduino.h>
#include <DRV8871.h>
#include <DRV8871quad.h>
#include <Enes100.h>

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

DRV8871 motor1(MOTOR1_IN1, MOTOR1_IN2);
DRV8871 motor2(MOTOR2_IN1, MOTOR2_IN2);
DRV8871 motor3(MOTOR3_IN1, MOTOR3_IN2);
DRV8871 motor4(MOTOR4_IN1, MOTOR4_IN2);

DRV8871Quad quadMotorController(&motor1, &motor2, &motor3, &motor4);

void setup() {}

void loop() {

  quadMotorController.drive(SPEED1, quadMotorController.DIRECTION_FORWARD);
  delay(1000);
  quadMotorController.turn(10, quadMotorController.TURN_LEFT);
  delay(1000);
  quadMotorController.drive(SPEED1, quadMotorController.DIRECTION_BACKWARD);
  delay(1000);
  quadMotorController.turn(10, quadMotorController.TURN_RIGHT);
  delay(1000);
  quadMotorController.drive(SPEED1, quadMotorController.DIRECTION_FORWARD);
  delay(1000);
  quadMotorController.breakdown();
  delay(1000);
}