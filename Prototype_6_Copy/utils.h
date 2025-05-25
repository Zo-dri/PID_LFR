#pragma once

#include "Arduino.h"
#include "config.h"
#include "declarations.h"

#ifndef UTILS_H
#define UTILS_H


void printArr(int arr[], uint8_t size);
void printlnArr(int arr[], uint8_t size);
void printArr(char arr[], uint8_t size);
void printlnArr(char arr[], uint8_t size);
float readBattery();
void buzz();
#endif