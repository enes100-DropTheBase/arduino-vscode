#include <Arduino.h>
#include <DRV8871.h>
#include <DRV8871Quad.h>
#include <Enes100.h>
#include <NewPing.h>
#include <Thread.h>
#include <ThreadController.h>
#include <TimerOne.h>

// pin configuration (for Arduino Mega)
// Motors need PWM pins

// Front Right
#define FR_MOTOR_IN1 7
#define FR_MOTOR_IN2 6

// Front Left
#define FL_MOTOR_IN1 9
#define FL_MOTOR_IN2 8

// Back Right
#define BR_MOTOR_IN1 11
#define BR_MOTOR_IN2 10

// Back Left
#define BL_MOTOR_IN1 13
#define BL_MOTOR_IN2 12

#define SPEED1 255
#define MAX_SPEED 255

// APC220
#define MARKER_ID 1
#define APC_RX 19
#define APC_TX 18

// Ultrasonic sensors
#define RIGHT_TRIGGER_PIN 50
#define RIGHT_ECHO_PIN 51
#define LEFT_TRIGGER_PIN 52
#define LEFT_ECHO_PIN 53
#define MAX_DISTANCE 200

void moveForward(int speed);
void moveBackward(int speed);
void turnRight(int speed);
void turnLeft(int speed);

NewPing rightSonar(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE);
NewPing leftSonar(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN, MAX_DISTANCE);

void setup() {
  Enes100.begin("Drop the Base", CHEMICAL, MARKER_ID, APC_RX, APC_TX);

  Enes100.print("Destination is at (");
  Enes100.print(Enes100.destination.x);
  Enes100.print(", ");
  Enes100.print(Enes100.destination.y);
  Enes100.println(")");
  Serial.begin(9600);
  Serial.println("Serial Output");
}

void loop() {
  Enes100.begin("Drop the Base", CHEMICAL, MARKER_ID, APC_RX, APC_TX);
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

  Serial.print("Right Ping: ");
  Serial.print(rightSonar.ping_cm());
  Serial.println("cm");

  Serial.print("Left Ping: ");
  Serial.print(leftSonar.ping_cm());
  Serial.println("cm");

  moveForward(SPEED1);

  delay(1000);
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