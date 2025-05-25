#pragma once

#include "Arduino.h"

#ifndef MOTORS_H
#define MOTORS_H

// Pin definitions for TB6612FNG Motor Driver
#define MOTOR_L_PWM        5
#define MOTOR_L_IN1       12
#define MOTOR_L_IN2       11

#define MOTOR_R_PWM        6
#define MOTOR_R_IN1        9
#define MOTOR_R_IN2       10

#define MOTOR_STBY        13

void setMotorSpeed(int left, int right);
void shortBrake(int del);
void shortBrake();

#endif