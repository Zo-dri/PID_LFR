#pragma once

#ifndef CONFIG_H
#define CONFIG_H

// Select the sensor type. Only define one.
// #define AnalogSENSOR
#define DigitalSENSOR

#define BUZZER     8
#define LIGHT     A0
#define BUTTON    A2

#define BATTERY   A1
#define vRatio   3.0
#if defined (AnSENSOR)
  #define calibrationOffset 50
#endif

// PID Constants (adjustable via Bluetooth)
float Kp = 25.0;  
float Ki = 0.01; 
float Kd = 10;  

// Sensor Configuration
#define SENSORS   12
const int NUM_SENSORS = 12;

int noLineDelay = 50;
int backTrackDelay = 250;
int solidLineStopDel = 100;
int hardTurnDel = 40;
int miscSpeed = 120;
int baseSpeed =    150;
int maxSpeed =     180;
int minSpeed =     -120;

#endif