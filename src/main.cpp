#include <Arduino.h>
#include <DRV8871.h>
#include <DRV8871Quad.h>
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

#define MARKER_ID 1
#define APC_RX 19
#define APC_TX 18

DRV8871 frontRightMotor(MOTOR1_IN1, MOTOR1_IN2);
DRV8871 frontLeftMotor(MOTOR2_IN1, MOTOR2_IN2);
DRV8871 backRightMotor(MOTOR3_IN1, MOTOR3_IN2);
DRV8871 backLeftMotor(MOTOR4_IN1, MOTOR4_IN2);

DRV8871Quad quadMotorController(&frontRightMotor, &frontLeftMotor, &backRightMotor, &backLeftMotor);

void setup() {
  Enes100.begin("Drop the Base", CHEMICAL, MARKER_ID, APC_RX, APC_TX);

  Enes100.print("Destination is at (");
  Enes100.print(Enes100.destination.x);
  Enes100.print(", ");
  Enes100.print(Enes100.destination.y);
  Enes100.println(")");
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