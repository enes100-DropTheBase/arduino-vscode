// pin configuration (for Arduino Uno/Nano)

#ifndef PIN_CONFIGURATION_H
#define PIN_CONFIGURATION_H

// APC 220
#define APC_RX 7
#define APC_TX 8

// Motors need PWM pins
// Right motors
#define RIGHT_MOTOR_IN1 5
#define RIGHT_MOTOR_IN2 3
// Left motors
#define LEFT_MOTOR_IN1 9
#define LEFT_MOTOR_IN2 6

// Ultrasonic sensors
#define RIGHT_TRIGGER_PIN 12
#define RIGHT_ECHO_PIN 12
#define LEFT_TRIGGER_PIN 13
#define LEFT_ECHO_PIN 13

// pH meter Analog output to Arduino Analog Input 7
#define PH_SENSOR_PIN A7

// Servo for arm
#define SERVO_PIN 10

#endif