#include <Arduino.h>
#include <Enes100.h>
#include <NewPing.h>

// pin configuration (for Arduino Mega)
// Motors need PWM pins

// Front Right
#define FR_MOTOR_IN1 3
#define FR_MOTOR_IN2 2

// Front Left
#define FL_MOTOR_IN1 5
#define FL_MOTOR_IN2 4

// Back Right
#define BR_MOTOR_IN1 7
#define BR_MOTOR_IN2 6

// Back Left
#define BL_MOTOR_IN1 11
#define BL_MOTOR_IN2 10

#define SPEED1 255
#define MAX_SPEED 255

// APC220
#define MARKER_ID 5
#define APC_RX 50
#define APC_TX 51

// Ultrasonic sensors
#define RIGHT_TRIGGER_PIN 50
#define RIGHT_ECHO_PIN 51
#define LEFT_TRIGGER_PIN 52
#define LEFT_ECHO_PIN 53
#define MAX_DISTANCE 200

#define ARENA_HEIGHT 2
#define ARENA_WIDTH 4

double getAngleToDest();
double getDistToDest();
void goAroundObstacle();
void updateLocation();
void stop();
void turn(double targetAngle);
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
  // Serial.begin(9600);
  // Serial.println("Serial Output");
}

void loop() {
  updateLocation();

  Enes100.print("Current X: ");
  Enes100.println(Enes100.location.x);
  Enes100.print("Current Y: ");
  Enes100.println(Enes100.location.y);

  if (Enes100.location.x < 1 && Enes100.location.y > 0.45) {
    // Go to the bottom corner
    turn(-PI / 2);
    moveForward(255);
    while (Enes100.location.y > 0.45) {
      // TODO: make it slow down when getting close
      stop();
      updateLocation();
      moveForward(255);
      delay(100);
    }
    stop();
  } else if (Enes100.location.x < 3) {
    // Go across the bottom
    turn(0);
    moveForward(255);
    while (  // rightSonar.ping_cm() > 0.25 && leftSonar.ping_cm() > 0.25 &&
        Enes100.location.x < 3) {
      updateLocation();
      // TODO: periodically recheck angle and adjust if off course
    }

    stop();
    // if (rightSonar.ping_cm() <= 0.25 || leftSonar.ping_cm() <= 0.25) {
    //   goAroundObstacle();
    // }
  } /* else {
    double targetAngle = getAngleToDest();
    if (getDistToDest() > 0.1) {
      // Go to the destination
      if (Enes100.location.x > Enes100.destination.x) {
        if (targetAngle < 0) {
          targetAngle += PI;
        } else {
          targetAngle -= PI;
        }
      }

      Enes100.print("Distance to destination: ");
      Enes100.println(getDistToDest());

      Enes100.print("Target Angle: ");
      Enes100.println(targetAngle * 180 / PI);

      // turn to face destination
      turn(targetAngle);

      // move forward
      moveForward(255);

      if (getDistToDest() < 0.5) {
        delay(100);
      } else {
        delay(1000);
      }

      // stop motors
      stop();
    }
  } */

  stop();
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

double getAngleToDest() {
  updateLocation();
  double deltaX = Enes100.location.x - Enes100.destination.x;
  double deltaY = Enes100.location.y - Enes100.destination.y;

  // Enes100.println(deltaX);
  // Enes100.println(deltaY);

  double angle = atan(deltaY / deltaX);

  // Enes100.println(angle);

  return angle;
}

double getDistToDest() {
  updateLocation();
  double deltaX = Enes100.location.x - Enes100.destination.x;
  double deltaY = Enes100.location.y - Enes100.destination.y;

  return sqrt(pow(deltaX, 2) + pow(deltaY, 2));
}

void goAroundObstacle() {
  Enes100.println("Avoiding Obstacle");
  updateLocation();
  turn(PI / 4);
  moveForward(255);
  updateLocation();

  double currentX = Enes100.location.x;
  double targetX = currentX + 0.55;

  // int offset = 0;

  while (currentX < targetX) {
    updateLocation();
    currentX = Enes100.location.x;
    // if (Enes100.location.y < ARENA_WIDTH / 3) {
    //   if (rightSonar.ping_cm() > leftSonar.ping_cm() &&
    //       leftSonar.ping_cm() < 0.2) {
    //     offset += PI / 20;
    //     Enes100.println("Compensating left");
    //     turn(PI / 4 + offset);
    //   } else if (rightSonar.ping_cm() < leftSonar.ping_cm() &&
    //              rightSonar.ping_cm() < 0.2) {
    //     Enes100.println("Compensating right");
    //     offset -= PI / 20;
    //     turn(PI / 4 + offset);
    //   }
    // }

    delay(100);
  }
  stop();
  updateLocation();

  Enes100.print("Current X: ");
  Enes100.println(Enes100.location.x);
  Enes100.print("Current Y: ");
  Enes100.println(Enes100.location.y);

  if (Enes100.location.x < 2.5) {
    turn(-PI / 2.7);
    moveForward(255);
    double currentY = Enes100.location.y;
    while (currentY > 0.4) {
      updateLocation();
      currentY = Enes100.location.y;
    }
  }

  stop();
}

void stop() {
  digitalWrite(FR_MOTOR_IN1, LOW);
  analogWrite(FR_MOTOR_IN2, LOW);

  digitalWrite(FL_MOTOR_IN1, LOW);
  analogWrite(FL_MOTOR_IN2, LOW);

  digitalWrite(BR_MOTOR_IN1, LOW);
  analogWrite(BR_MOTOR_IN2, LOW);

  digitalWrite(BL_MOTOR_IN1, LOW);
  analogWrite(BL_MOTOR_IN2, LOW);
}

void turn(double targetAngle) {
  stop();
  Enes100.println("Turning");
  updateLocation();
  // Enes100.print("Difference");
  // Enes100.println(fabs(Enes100.location.theta - targetAngle));
  // TODO: this is too reliant on the vision system
  double angleDifference = fabs(Enes100.location.theta - targetAngle);
  while (angleDifference > 0.2) {
    Enes100.println(angleDifference);
    int speed = 125;
    if (angleDifference < 0.3) {
      speed = 100;
    }
    if (Enes100.location.theta - targetAngle > 0) {
      turnRight(speed);
    } else {
      turnLeft(speed);
    }
    delay(100);
    stop();

    updateLocation();

    angleDifference = fabs(Enes100.location.theta - targetAngle);
  }
  Enes100.println("Done Turning");
}

void updateLocation() {
  while (!Enes100.updateLocation()) {
    Enes100.println("Unable to update location");
    delay(100);
  }
  // TODO: Add checking for validity of location data (sometimes gets overflow
  // or garbage values)

  // Enes100.print("OSV is at (");
  // Enes100.print(Enes100.location.x);
  // Enes100.print(", ");
  // Enes100.print(Enes100.location.y);
  // Enes100.print(", ");
  // Enes100.print(Enes100.location.theta);
  // Enes100.println(")");
}