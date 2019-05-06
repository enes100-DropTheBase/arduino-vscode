#include <Arduino.h>
#include <Enes100.h>
#include <NewPing.h>
#include <Servo.h>

#include "basic_movement.h"
#include "pin_configuration.h"

#define ENES100_DEBUG
//#define DEBUG_UPDATE_LOCATION

#define MAX_SPEED 255

// APC 220
#define MARKER_ID 13

#define ARENA_HEIGHT 2
#define ARENA_WIDTH 4

// Max ultrasonic distance in cm
#define MAX_DISTANCE 200

// pH
#define Offset 0.00  // deviation compensate
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth 40     // times of collection
int pHArray[ArrayLenth];  // Store the average value of the sensor feedback
int pHArrayIndex = 0;

double getAngleToDest();
double getDistToDest();
void goAroundObstacle();
void updateLocation();
void turn(double targetAngle);
float pingLeft();
float pingRight();
double avergearray(int* arr, int number);

NewPing rightSonar(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE);
NewPing leftSonar(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN, MAX_DISTANCE);

String status = "";

Servo servo;

int servoAngle = 0;  // servo position in degrees

void setup() {
  delay(500);
  Enes100.begin("Drop the Base", CHEMICAL, MARKER_ID, APC_RX, APC_TX);

#ifdef ENES100_DEBUG
  Enes100.print("Destination is at (");
  Enes100.print(Enes100.destination.x);
  Enes100.print(", ");
  Enes100.print(Enes100.destination.y);
  Enes100.println(")");
#endif
#ifdef SERIAL_DEBUG
  Serial.begin(9600);
  Serial.println("Serial Output");
#endif

  digitalWrite(LEFT_PUMP, LOW);
  digitalWrite(RIGHT_PUMP, LOW);

  servo.attach(SERVO_PIN);
  servo.write(servoAngle);
  delay(700);
  servo.detach();
}

void loop() {
  /*
  analogWrite(LEFT_PUMP, 255);
  analogWrite(RIGHT_PUMP, 255);

  delay(1000);

  digitalWrite(LEFT_PUMP, LOW);
  digitalWrite(RIGHT_PUMP, LOW); */

  if (Enes100.destination.x < 0.5 || Enes100.destination.y < 0.01) {
    // This means init failed
#ifdef ENES100_DEBUG
    Enes100.println("Warning, destination is not valid: retrying connection");
#endif
    Enes100.begin("Drop the Base", CHEMICAL, MARKER_ID, APC_RX, APC_TX);
  }

  stop();
#ifdef SERIAL_DEBUG
  Serial.print("Right: ");
  Serial.println(pingRight());
  Serial.print("Left: ");
  Serial.println(pingLeft());
#endif

  updateLocation();
  status = "";

#ifdef ENES100_DEBUG
  Enes100.print("Current X: ");
  Enes100.println(Enes100.location.x);
  Enes100.print("Current Y: ");
  Enes100.println(Enes100.location.y);
#endif

  if (Enes100.location.x < 1 && Enes100.location.y > 0.45) {
    // Go to the bottom corner
    status = "Going to bottom corner";
#ifdef ENES100_DEBUG
    Enes100.println(status);
#endif

    turn(-PI / 2);
    moveForward(255);
    while (Enes100.location.y > 0.45) {
      // TODO: make it slow down when getting close
      stop();
      updateLocation();

      // This tries to adjust if off course. Needs testing
      if (fabs(Enes100.location.theta - PI / 2) > 0.12) {
        turn(-PI / 2);
      }

      moveForward(255);
      delay(100);
    }
    stop();
  } else if (Enes100.location.x < 3) {
    status = "Going across bottom";
#ifdef ENES100_DEBUG
    Enes100.println(status);
#endif
    turn(0);
    moveForward(255);
#ifdef ENES100_DEBUG
    Enes100.print("Right: ");
    Enes100.println(pingRight());
    Enes100.print("Left: ");
    Enes100.println(pingLeft());
#endif
    while (Enes100.location.x < 1 ||
           ((pingRight() > 40 || pingRight() == 0) &&
            (pingLeft() > 40 || pingLeft() == 0) && Enes100.location.x < 3)) {
      stop();

#ifdef ENES100_DEBUG
      Enes100.print("Right: ");
      Enes100.println(pingRight());
      Enes100.print("Left: ");
      Enes100.println(pingLeft());
#endif

      updateLocation();

      // This tries to adjust if off course. Needs testing
      if (Enes100.location.x > 1.2 && Enes100.location.y > 0.5) {
        turn(-PI / 8);
      } else if (fabs(Enes100.location.theta) > 0.15) {
        turn(0);
      }

      moveForward(255);

      delay(200);

      // TODO: periodically recheck angle and adjust if off course
    }

    stop();
    // TODO: Make it so only a decrease in distance from multiple reading when
    // the OSV is moving towards the obstacle will trigger the avoidance
    // (detects 60 cm away, then 55, 40, etc.)
    if ((pingRight() <= 40 && pingRight() != 0) ||
        (pingLeft() <= 40 && pingLeft() != 0)) {
      status = "Going around obstacle";
#ifdef ENES100_DEBUG
      Enes100.println(status);
#endif
      goAroundObstacle();
    }
  } else if (Enes100.location.x < 4 && Enes100.location.x >= 3 &&
             getDistToDest() > 0.2) {
    status = "Going to destination";
#ifdef ENES100_DEBUG
    Enes100.println(status);
#endif
    double targetAngle = getAngleToDest();
    if (getDistToDest() > 0.2) {
      // Go to the destination
#ifdef ENES100_DEBUG
      Enes100.print("Destination is at (");
      Enes100.print(Enes100.destination.x);
      Enes100.print(", ");
      Enes100.print(Enes100.destination.y);
      Enes100.println(")");
      Enes100.print("Distance to destination: ");
      Enes100.println(getDistToDest());
      Enes100.print("Target Angle: ");
      Enes100.println(targetAngle * 180 / PI);
#endif
      // turn to face destination
      turn(targetAngle);
      // move forward
      moveForward(255);
      if (getDistToDest() < 0.5) {
        delay(100);
      } else {
        delay(300);
      }
      // stop motors
      stop();
    }
  } else if (Enes100.location.x < 4 && Enes100.location.x >= 3 &&
             getDistToDest() <= 0.2) {
    status = "At destination";

    double targetAngle = getAngleToDest();
    turn(targetAngle);

#ifdef ENES100_DEBUG
    Enes100.println(status);
#endif
    stop();
    servo.attach(SERVO_PIN);
    // move the micro servo from 0 degrees to 90 degrees
    for (servoAngle = 0; servoAngle < 90; servoAngle++) {
      servo.write(servoAngle);
      delay(10);
    }
    servo.detach();

    // Start sample collection
    analogWrite(LEFT_PUMP, 255);

    delay(10000);
    unsigned long targetTime = millis() + 5000;
    unsigned long samplingTime = millis();
    float pHValue = 6;
    while (millis() < targetTime) {
      float voltage;
      if (millis() - samplingTime > samplingInterval) {
        pHArray[pHArrayIndex++] = analogRead(PH_SENSOR_PIN);
        if (pHArrayIndex == ArrayLenth) pHArrayIndex = 0;
        voltage = avergearray(pHArray, ArrayLenth) * 5.0 / 1024;
        pHValue = 3.5 * voltage + Offset;
        samplingTime = millis();
      }
    }
    Enes100.mission(pHValue);

    delay(10000);

    // Stop sample collection
    digitalWrite(LEFT_PUMP, LOW);
  }

  stop();
}

double getAngleToDest() {
  updateLocation();
  double deltaX = Enes100.destination.x - Enes100.location.x;
  double deltaY = Enes100.destination.y - Enes100.location.y;

  // Enes100.println(deltaX);
  // Enes100.println(deltaY);

  double angle = atan2(deltaY, deltaX);

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
  status = "Avoiding Obstacle";
#ifdef ENES100_DEBUG
  Enes100.println("Avoiding Obstacle");
#endif
  stop();

  updateLocation();

  double currentX = Enes100.location.x;
  double currentY = Enes100.location.y;

  // TODO: Take into account exact position from obstacle using ultrasonic

  float offset = 0.45;

  float rightPingCm = pingRight();
  float leftPingCm = pingLeft();

  if ((rightPingCm > 10 && rightPingCm < 100) ||
      (leftPingCm > 10 && leftPingCm < 100)) {
    if (rightPingCm > leftPingCm) {
      offset = rightPingCm / 100;
    } else {
      offset = leftPingCm / 100;
    }
  } else {
#ifdef ENES100_DEBUG
    Enes100.println("Not using ultrasonic sensors for calculation");
#endif
  }

  double targetX = currentX + offset + 0.05;
  double targetY = 0.7;

  double angle = atan2(targetY - currentY, targetX - currentX);

#ifdef ENES100_DEBUG
  Enes100.print("Angle obstacle: ");
  Enes100.println(angle);
#endif

  turn(angle);

  updateLocation();

  targetX = currentX + 0.75;  // This isn't used right now
  targetY = 0.7;

  while (currentY < targetY) {
    // TODO: continuously recheck angle and adjust accordingly
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

#ifdef ENES100_DEBUG
  Enes100.print("Current X: ");
  Enes100.println(Enes100.location.x);
  Enes100.print("Current Y: ");
  Enes100.println(Enes100.location.y);
#endif

  if (Enes100.location.x < 2.5) {
    turn(-PI / 2.7);
    double currentY = Enes100.location.y;
    while (currentY > 0.4) {
      updateLocation();
      currentY = Enes100.location.y;
      moveForward(255);
      delay(100);
      stop();
    }
  }

  stop();
}

void turn(double targetAngle) {
  stop();
#ifdef ENES100_DEBUG
  Enes100.println("Turning");
#endif
  updateLocation();
  // Enes100.print("Difference");
  // Enes100.println(fabs(Enes100.location.theta - targetAngle));
  // TODO: this is too reliant on the vision system
  double angleDifference = fabs(Enes100.location.theta - targetAngle);
  while (angleDifference > 0.1) {
    // Enes100.println(angleDifference);
    int speed = 220;
    if (angleDifference < 0.3) {
      speed = 200;
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
#ifdef ENES100_DEBUG
  Enes100.println("Done Turning");
#endif
}

void updateLocation() {
  while (bool loc = !Enes100.updateLocation() || Enes100.location.theta > 10 ||
                    Enes100.location.x > 10 || Enes100.location.y > 10 ||
                    Enes100.location.x < 0.02 || Enes100.location.y < -0.02 ||
                    Enes100.location.theta < -10) {
#ifdef ENES100_DEBUG
    if (!loc) {
      Enes100.println("Invalid location");
    } else {
#ifdef DEBUG_UPDATE_LOCATION
      Enes100.println("Unable to update location");
#endif
    }
#endif
    delay(100);
  }
#ifdef ENES100_DEBUG
  Enes100.print(status);
  Enes100.print(": ");
  Enes100.print("OSV is at (");
  Enes100.print(Enes100.location.x);
  Enes100.print(", ");
  Enes100.print(Enes100.location.y);
  Enes100.print(", ");
  Enes100.print(Enes100.location.theta);
  Enes100.println(")");
#endif
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

double avergearray(int* arr, int number) {
  int i;
  int max, min;
  double avg;
  long amount = 0;
  if (number <= 0) {
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if (number < 5) {  // less than 5, calculated directly statistics
    for (i = 0; i < number; i++) {
      amount += arr[i];
    }
    avg = amount / number;
    return avg;
  } else {
    if (arr[0] < arr[1]) {
      min = arr[0];
      max = arr[1];
    } else {
      min = arr[1];
      max = arr[0];
    }
    for (i = 2; i < number; i++) {
      if (arr[i] < min) {
        amount += min;  // arr<min
        min = arr[i];
      } else {
        if (arr[i] > max) {
          amount += max;  // arr>max
          max = arr[i];
        } else {
          amount += arr[i];  // min<=arr<=max
        }
      }  // if
    }    // for
    avg = (double)amount / (number - 2);
  }  // if
  return avg;
}