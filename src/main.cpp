#include <Arduino.h>
#include <Enes100.h>
#include <NewPing.h>

// pin configuration (for Arduino Uno)
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
#define MARKER_ID 12
#define APC_RX 7
#define APC_TX 8

// Ultrasonic sensors
// TODO: Set up in 3 wire mode to save 2 pins
#define RIGHT_TRIGGER_PIN 12
#define RIGHT_ECHO_PIN 13
#define LEFT_TRIGGER_PIN 2
#define LEFT_ECHO_PIN 4
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
    turn(-PI / 2);
    moveForward(255);
    while (Enes100.location.y > 0.45) {
      updateLocation();
    }
    stop();
  } else if (Enes100.location.x < 3) {
    turn(0);
    moveForward(255);
    while (  // rightSonar.ping_cm() > 0.25 && leftSonar.ping_cm() > 0.25 &&
        Enes100.location.x < 3) {
      updateLocation();
    }

    stop();
    // if (rightSonar.ping_cm() <= 0.25 || leftSonar.ping_cm() <= 0.25) {
    //   goAroundObstacle();
    // }
  } else {
    double targetAngle = getAngleToDest();
    if (getDistToDest() > 0.1) {
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
  }

  stop();
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
  // TODO: add checks to make sure location can be updated
  Enes100.updateLocation();
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

  int offset = 0;

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
  // TODO: this might be coasting instead of actually stopping
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  analogWrite(RIGHT_MOTOR_IN2, LOW);

  digitalWrite(LEFT_MOTOR_IN1, LOW);
  analogWrite(LEFT_MOTOR_IN2, LOW);
}

void turn(double targetAngle) {
  updateLocation();
  Enes100.print("Difference");
  Enes100.println(fabs(Enes100.location.theta - targetAngle));
  // TODO: this is too reliant on the vision system
  while (fabs(Enes100.location.theta - targetAngle) > 0.09) {
    if (Enes100.location.theta - targetAngle > 0) {
      turnRight(150);
    } else {
      turnLeft(150);
    }
    delay(100);
    stop();

    updateLocation();
  }
}

void updateLocation() {
  while (!Enes100.updateLocation()) {
    Enes100.println("Unable to update location");
  }
  // Enes100.print("OSV is at (");
  // Enes100.print(Enes100.location.x);
  // Enes100.print(", ");
  // Enes100.print(Enes100.location.y);
  //   Enes100.print(", ");
  // Enes100.print(Enes100.location.theta);
  // Enes100.println(")");
}