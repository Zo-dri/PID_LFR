#pragma once

#include <ArduinoJson.h>
// #include "config.h"
// #include "declarations.h"


#ifndef LFRCONSOLE_H
#define LFRCONSOLE_H

#define JSON_MAX_LEN 196

void consoleAppTx(int sensorValues[], int pos, float error);
void consoleAppRx(char cmd, float pidValues[]);

#endif