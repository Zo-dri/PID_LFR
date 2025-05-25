#pragma once

#include <string.h>
#include "config.h"

#ifndef DECLARATIONS_H
#define DECLARATIONS_H

bool LFRConsole = true;
bool debug = false;

#if (NUM_SENSORS > 16)
  #error "Library does not support more than 16 channels"
#endif

int sensorValues[NUM_SENSORS];
char sensorValuesDig[NUM_SENSORS + 1];
struct calibrationData {
  int Min[NUM_SENSORS];
  int Max[NUM_SENSORS];
  bool calibrationDone = false;
};

bool prepareLeft = false, prepareRight = false;
bool BTDrive =   false;
bool errorDir =  false;
bool inverted =  false;
bool solidLine = false;
float error =      0.0;
float integral =   0.0;
int lastError =      0;
int lastPosition =      0;

unsigned long tBuzz, tNoLine, tSolidLine, t1, t2, t3;
bool noLine = false, buzzed = false, solidLineTStarted = false;
bool t1Started = false, t2Started = false, t3Started = false;

#endif