#include "basic_movement.h"

void moveForward(int speed) {
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  analogWrite(RIGHT_MOTOR_IN2, speed);

  digitalWrite(LEFT_MOTOR_IN1, LOW);
  analogWrite(LEFT_MOTOR_IN2, speed);
}

void moveBackward(int speed) {
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
  analogWrite(RIGHT_MOTOR_IN1, speed);

  digitalWrite(LEFT_MOTOR_IN2, LOW);
  analogWrite(LEFT_MOTOR_IN1, speed);
}

void turnRight(int speed) {
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
  analogWrite(RIGHT_MOTOR_IN1, speed);

  digitalWrite(LEFT_MOTOR_IN1, LOW);
  analogWrite(LEFT_MOTOR_IN2, speed);
}

void turnLeft(int speed) {
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  analogWrite(RIGHT_MOTOR_IN2, speed);

  digitalWrite(LEFT_MOTOR_IN2, LOW);
  analogWrite(LEFT_MOTOR_IN1, speed);
}

void stop() {
  // TODO: this might be coasting instead of actually stopping
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  analogWrite(RIGHT_MOTOR_IN2, LOW);

  digitalWrite(LEFT_MOTOR_IN1, LOW);
  analogWrite(LEFT_MOTOR_IN2, LOW);
}