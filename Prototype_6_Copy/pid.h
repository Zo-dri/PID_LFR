#pragma once

#include "Arduino.h"
#include "config.h"
#include "declarations.h"
#include "sensors.h"
#include "motors.h"
#include "utils.h"

#ifndef PID_H
#define PID_H

float getLinePosition();
float getPositionError();
float pidControl();

#endif