#pragma once

#include "Arduino.h"
#include "config.h"
#include "declarations.h"
#include "motors.h"
#include "memory.h"
#include "utils.h"

#ifndef SENSORS_H
#define SENSORS_H



// Pin definitions for MUX
#define MUX_SIG  A5  // Analog signal pin from MUX
#define MUX_S0    7
#define MUX_S1    4
#define MUX_S2    3
#define MUX_S3    2

calibrationData sensor;


void selectSensor(uint8_t n);

#if defined (AnalogSENSOR)
void readSensors(bool print);
void readCalibratedSensors(bool invert);
#endif

#if defined (DigitalSENSOR)
void readSensors(bool print);
void readCalibratedSensors(bool invert);
#endif

void calibrateSensors();

#endif