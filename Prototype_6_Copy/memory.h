#pragma once

#include <EEPROM.h>
#include "config.h"
#include "declarations.h"
#include "sensors.h"

#ifndef MEMORY_H
#define MEMORY_H

struct calibrationData {
  int Min[NUM_SENSORS];
  int Max[NUM_SENSORS];
  bool calibrationDone = false;
};

void readEEPROM();



#endif
