#include "Arduino.h"
#pragma once
// Pin definitions for TB6612FNG Motor Driver
#define MOTOR_L_PWM        5
#define MOTOR_L_IN1       12
#define MOTOR_L_IN2       11

#define MOTOR_R_PWM        6
#define MOTOR_R_IN1        9
#define MOTOR_R_IN2       10

#define MOTOR_STBY        13

// Motor control function
void setMotorSpeed(int left, int right) {
    digitalWrite(MOTOR_STBY, HIGH);
    if (left >= 0) {
        digitalWrite(MOTOR_L_IN1, HIGH);
        digitalWrite(MOTOR_L_IN2, LOW);
        analogWrite(MOTOR_L_PWM, left);
    } else {
        digitalWrite(MOTOR_L_IN1, LOW);
        digitalWrite(MOTOR_L_IN2, HIGH);
        analogWrite(MOTOR_L_PWM, -left);
    }

    if (right >= 0) {
        digitalWrite(MOTOR_R_IN1, HIGH);
        digitalWrite(MOTOR_R_IN2, LOW);
        analogWrite(MOTOR_R_PWM, right);
    } else {
        digitalWrite(MOTOR_R_IN1, LOW);
        digitalWrite(MOTOR_R_IN2, HIGH);
        analogWrite(MOTOR_R_PWM, -right);
    }
}
void turnLeft(int speed) {
    setMotorSpeed(0, speed);
}
void pivotLeft(int speed) {
    setMotorSpeed(-speed, speed);
}
void turnRight(int speed) {
    setMotorSpeed(speed, 0);
}
void pivotRight(int speed) {
    setMotorSpeed(speed, -speed);
}
void stopMotors() {         // Smooth Stop
    digitalWrite(MOTOR_STBY, LOW);
    setMotorSpeed(0, 0);
}
void shortBrake() {         // Instant Stop
  setMotorSpeed(0, 0);
}
void shortBrake(int del) {
  shortBrake();
  delay(del);
}