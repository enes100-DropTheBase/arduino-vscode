#include <Arduino.h>
#include <Enes100.h>
#include <NewPing.h>

// pin configuration (for Arduino Uno/Nano)
// Motors need PWM pins

// Right motors
#define RIGHT_MOTOR_IN1 5
#define RIGHT_MOTOR_IN2 3

// Left motors
#define LEFT_MOTOR_IN1 9
#define LEFT_MOTOR_IN2 6

#define SPEED1 255
#define MAX_SPEED 255

// APC220
#define MARKER_ID 6
#define APC_RX 7
#define APC_TX 8

// Ultrasonic sensors
#define RIGHT_TRIGGER_PIN 12
#define RIGHT_ECHO_PIN 12
#define LEFT_TRIGGER_PIN 13
#define LEFT_ECHO_PIN 13
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
float pingLeft();
float pingRight();

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
  stop();
#ifdef SERIAL_DEBUG
  Serial.print("Right: ");
  Serial.println(pingRight());
  Serial.print("Left: ");
  Serial.println(pingLeft());
#endif

  updateLocation();

  Enes100.print("Current X: ");
  Enes100.println(Enes100.location.x);
  Enes100.print("Current Y: ");
  Enes100.println(Enes100.location.y);

  if (Enes100.location.x < 1 && Enes100.location.y > 0.45) {
    Enes100.println("Going to bottom corner");
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
    Enes100.println("Going across bottom");
    turn(0);
    moveForward(255);
    Enes100.print("Right: ");
    Enes100.println(pingRight());
    Enes100.print("Left: ");
    Enes100.println(pingLeft());
    while (Enes100.location.x < 1 ||
           ((pingRight() > 50 || pingRight() == 0) &&
            (pingLeft() > 50 || pingLeft() == 0) && Enes100.location.x < 3)) {
      stop();

      Enes100.print("Right: ");
      Enes100.println(pingRight());
      Enes100.print("Left: ");
      Enes100.println(pingLeft());

      updateLocation();

      moveForward(255);

      delay(200);
      // TODO: periodically recheck angle and adjust if off course
    }

    stop();
    if (pingRight() <= 50 || pingLeft() <= 50) {
      Enes100.println("Going around obstacle");
      goAroundObstacle();
    }
  } /* else if (Enes100.updateLocation() && Enes100.location.x < 4 &&
  Enes100.location.x >= 3) { Enes100.println("Going to destination"); double
  targetAngle = getAngleToDest(); if (getDistToDest() > 0.1) {
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
  stop();
  updateLocation();

  turn(PI / 4);
  updateLocation();

  double currentX = Enes100.location.x;
  double currentY = Enes100.location.y;

  double targetX = currentX + 0.75;
  double targetY = 0.7;

  // int offset = 0;

  while (currentY < targetY) {
    stop();
    updateLocation();
    currentY = Enes100.location.y;
    moveForward(255);
    delay(100);
  }

  turn(0);

  updateLocation();

  currentX = Enes100.location.x;

  while (currentX < targetX) {
    stop();
    updateLocation();
    currentX = Enes100.location.x;
    moveForward(255);
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
  // TODO: this might be coasting instead of actually stopping
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  analogWrite(RIGHT_MOTOR_IN2, LOW);

  digitalWrite(LEFT_MOTOR_IN1, LOW);
  analogWrite(LEFT_MOTOR_IN2, LOW);
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
  while (bool loc = !Enes100.updateLocation() || Enes100.location.theta > 10 ||
                    Enes100.location.x > 10 || Enes100.location.y > 10 ||
                    Enes100.location.x < -1 || Enes100.location.y < -1 ||
                    Enes100.location.theta < -10) {
    if (!loc) {
      Enes100.println("Invalid location");
    } else {
      Enes100.println("Unable to update location");
    }
    delay(100);
  }

  Enes100.print("OSV is at (");
  Enes100.print(Enes100.location.x);
  Enes100.print(", ");
  Enes100.print(Enes100.location.y);
  Enes100.print(", ");
  Enes100.print(Enes100.location.theta);
  Enes100.println(")");
}

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

float pingLeft() {
  unsigned long duration = leftSonar.ping_median(3);

  float cm = (duration / 2) * 0.0343;

  if (cm > 100) {
    return 0;
  } else {
    return cm;
  }
}

float pingRight() {
  unsigned long duration = rightSonar.ping_median(3);

  float cm = (duration / 2) * 0.0343;

  if (cm > 100) {
    return 0;
  } else {
    return cm;
  }
}