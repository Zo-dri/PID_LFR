#include <Arduino.h>

#include "config.h"
#include "declarations.h"
#include "sensors.h"
#include "motors.h"
#include "memory.h"
#include "pid.h"
#include "utils.h"

#include "LFRConsole.h"



// PID Controller

// Function to receive PID values via Bluetooth
void receiveBluetoothData() {
  if (Serial.available()) {
    char cmd = Serial.read();
    Serial.print(cmd); Serial.print(": ");
    switch(cmd) {
      case 'P': Kp = Serial.parseFloat(); EEPROM.put(0x00, Kp); Serial.print(Kp); break;    // Occupies address 0x00 to 0x03
      case 'I': Ki = Serial.parseFloat(); EEPROM.put(0x04, Ki); Serial.print(Ki); break;    // Occupies address 0x04 to 0x07
      case 'D': Kd = Serial.parseFloat(); EEPROM.put(0x08, Kd); Serial.print(Kd); break;    // Occupies address 0x08 to 0x0B
      case 'M': baseSpeed = Serial.parseInt(); EEPROM.put(0x0C, baseSpeed); Serial.print(baseSpeed); break;    // Occupies address 0x0C to 0x0D
      // case 'm': miscSpeed = Serial.parseInt(); EEPROM.put(0x31, miscSpeed); Serial.print("Misc Spd:"); Serial.print(miscSpeed); break;
      // case 'T': noLineDelay = Serial.parseInt(); EEPROM.put(0x33, noLineDelay); Serial.print("No Line Del:"); Serial.print(noLineDelay); break;
      // case 't': backTrackDelay = Serial.parseInt(); EEPROM.put(0x35, backTrackDelay); Serial.print("Back Track Del:"); Serial.print(backTrackDelay); break;
      case 'm': miscSpeed = Serial.parseInt(); EEPROM.put(0x42, miscSpeed); Serial.print("Misc Spd:"); Serial.print(miscSpeed); break;
      case 'T': noLineDelay = Serial.parseInt(); EEPROM.put(0x44, noLineDelay); Serial.print("No Line Del:"); Serial.print(noLineDelay); break;
      case 't': backTrackDelay = Serial.parseInt(); EEPROM.put(0x46, backTrackDelay); Serial.print("Back Track Del:"); Serial.print(backTrackDelay); break;


      case 'F': setMotorSpeed(baseSpeed, baseSpeed); break;
      case 'B': setMotorSpeed(-baseSpeed, -baseSpeed); break;
      case 'L': setMotorSpeed(-baseSpeed, baseSpeed); break;
      case 'R': setMotorSpeed(baseSpeed, -baseSpeed); break;

      case 'C': sensor.calibrationDone = false; Serial.print("Calibrate "); break;
      case 'Y': digitalWrite(BUZZER, !digitalRead(BUZZER)); Serial.print("Buzzer "); Serial.print(digitalRead(BUZZER) ? "ON" : "OFF"); break;
      case 'U': digitalWrite(LIGHT, !digitalRead(LIGHT)); Serial.print("Light "); Serial.print(digitalRead(LIGHT) ? "ON" : "OFF"); break;
      // case 'y': digitalWrite(BUZZER, LOW); break;

      // case 'l': LFRConsole = !LFRConsole; if(LFRConsole) debug = false; EEPROM.put(0x2F, LFRConsole); EEPROM.put(0x30, debug); break;
      // case 'd': debug = !debug; if(debug) LFRConsole = false; EEPROM.put(0x2F, LFRConsole); EEPROM.put(0x30, debug); break;
      case 'l': LFRConsole = !LFRConsole; if(LFRConsole) debug = false; EEPROM.put(0x40, LFRConsole); EEPROM.put(0x41, debug); break;
      case 'd': debug = !debug; if(debug) LFRConsole = false; EEPROM.put(0x40, LFRConsole); EEPROM.put(0x41, debug); break;
      case 'x': BTDrive = false; Serial.print(BTDrive); break;
      case 'X': BTDrive = true; debug = false;   //Fall-through to stop
      // case 'S': setMotorSpeed(0, 0); delay(10000); //Fall-through to stop
      default: setMotorSpeed(0, 0); 
      // digitalWrite(MOTOR_STBY, LOW); 
      //consoleAppRx(cmd);
    }
    
  }
}


// Setup function
void setup() {
    Serial.begin(115200);

    readEEPROM();
    Serial.println();
    Serial.println("*********************Prototype 6*********************");
    Serial.print("P:\t");
    Serial.println(Kp);
    Serial.print("I:\t");
    Serial.println(Ki);
    Serial.print("D:\t");
    Serial.println(Kd);
    Serial.print("Speed:\t");
    Serial.println(baseSpeed);
    Serial.println("Calibration: ");
    Serial.print("Min: ");
    printlnArr(sensor.Min, NUM_SENSORS);
    Serial.print("Max: ");
    printlnArr(sensor.Max, NUM_SENSORS);
    Serial.print("Debug:\t\t\t"); Serial.println(debug ? "True" : "False");
    Serial.print("Console:\t\t"); Serial.println(LFRConsole ? "True" : "False");
    Serial.print("Misc Speed:\t\t"); Serial.println(miscSpeed);
    Serial.print("No Line Delay:\t\t"); Serial.print(noLineDelay); Serial.println("ms");
    Serial.print("Back Track Delay:\t"); Serial.print(backTrackDelay); Serial.println("ms");

    Serial.println("*****************************************************");

    pinMode(BUTTON, INPUT_PULLUP);
    if(!digitalRead(BUTTON)) sensor.calibrationDone = false;
    delay(2000);

    pinMode(MUX_S0, OUTPUT);
    pinMode(MUX_S1, OUTPUT);
    pinMode(MUX_S2, OUTPUT);
    pinMode(MUX_S3, OUTPUT);
    pinMode(BUZZER, OUTPUT);
    pinMode(LIGHT, OUTPUT);
    pinMode(MUX_SIG, INPUT);
    pinMode(MOTOR_STBY, OUTPUT);
    pinMode(MOTOR_L_PWM, OUTPUT);
    pinMode(MOTOR_L_IN1, OUTPUT);
    pinMode(MOTOR_L_IN2, OUTPUT);
    pinMode(MOTOR_R_PWM, OUTPUT);
    pinMode(MOTOR_R_IN1, OUTPUT);
    pinMode(MOTOR_R_IN2, OUTPUT);
    digitalWrite(MOTOR_STBY, LOW);
}

// Main loop
void loop() {
  float pos;
  if(!sensor.calibrationDone){
    calibrateSensors();
  } else {
    if(!BTDrive) {
      pos = pidControl();
    }
  }

  if(millis() > tBuzz + 100 && buzzed == false) {
    buzzed = true;
    digitalWrite(BUZZER, LOW);
    if(!inverted) digitalWrite(LIGHT, LOW);
    else digitalWrite(LIGHT, HIGH);
    tBuzz = millis();
  }
  digitalWrite(LIGHT, inverted);

  if(LFRConsole) {
    consoleAppTx(sensorValues, pos, error);
  }
  if(debug) {
    Serial.print("Sen:\t");
    printArr(sensorValues, NUM_SENSORS);
    Serial.print(pos); Serial.print("\t");
    Serial.print(errorDir);
    Serial.print(" Batt: "); Serial.print(readBattery()); Serial.print("V");
    Serial.println();
  }

  receiveBluetoothData();
}