#include "motors.h"

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
void shortBrake(int del) {
  shortBrake();
  delay(del);
}
void shortBrake() {
  setMotorSpeed(0, 0);
}