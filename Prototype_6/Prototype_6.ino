#include <EEPROM.h>
// #include <string.h>
#include "LFRConsole.h"
#include "motors.h"
#include "utils.h"

// #define JSON_MAX_LEN 196

// Pin definitions for MUX
#define MUX_SIG  A5  // Analog signal pin from MUX
#define MUX_S0    7
#define MUX_S1    4
#define MUX_S2    3
#define MUX_S3    2

// Pin definitions for TB6612FNG Motor Driver
#define MOTOR_L_PWM        5  
#define MOTOR_L_IN1       12  
#define MOTOR_L_IN2       11  
#define MOTOR_R_PWM        6 
#define MOTOR_R_IN1        9  
#define MOTOR_R_IN2       10  
#define MOTOR_STBY        13  // Standby pin

#define BUZZER     8
#define LIGHT     A0
#define BATTERY   A1
#define vRatio   3.0
#define BUTTON    A2

#define calibrationOffset 50

// #define AnSENSOR
#define DiSENSOR

bool LFRConsole = false;
bool debug = false;

// PID Constants (adjustable via Bluetooth)
float Kp = 25.0;  
float Ki = 0.01; 
float Kd = 10;  

// Sensor Configuration
#define NUM_SENSORS 12

int sensorValues[NUM_SENSORS];
// char sensorValuesDig[NUM_SENSORS + 1];
uint16_t sensorValuesDig = 0x0000;
struct calibrationData {
  int Min[NUM_SENSORS];
  int Max[NUM_SENSORS];
  bool calibrationDone = false;
};
calibrationData sensor;

bool BTDrive =   false;
bool inverted =  false;
bool solidLine = false, solidLineTStarted = false;
bool noLine =    false;
bool buzzed =    false;
float error =      0.0;
bool errorDir =  false;
float integral =   0.0;
bool sharpTurn = false, sharpTurnDir = false;
int noLineStopDelay = 50;
int backTrackDuration = 250;
int solidLineStopDel = 75;
int hardTurnDel = 40;
byte brakeDuration = 50;
int miscSpeed = 120;
int lastError =      0;
int lastPosition =      0;
// int maxSpeed =     255; 
int baseSpeed =    150;
int maxSpeed =     180;       // Working
// int maxSpeed =     200; 
int minSpeed =     -120;    // Working
// int minSpeed =     -150; 

unsigned long tBuzz, tNoLine, tSolidLine, t1, t2, t3;
bool t1Started = false, t2Started = false, t3Started = false;


float readBattery() {
  return (analogRead(BATTERY) * 5.0 * vRatio) / 1023;
}

void selectSensor(byte n) {
  digitalWrite(MUX_S0, (n & 0x01));
  digitalWrite(MUX_S1, (n & 0x02));
  digitalWrite(MUX_S2, (n & 0x04));
  digitalWrite(MUX_S3, (n & 0x08));
}
// Function to read sensors from MUX (charge then measure)
#if defined (AnSENSOR)
void readSensors(bool print) {
    for (uint8_t i = 0; i < NUM_SENSORS; i += 1) {
      sensorValues[i] = 0;
    }

    for (uint8_t j = 0; j < 4; j++) {
      for (uint8_t i = 1; i < NUM_SENSORS + 1; i += 1) {
        selectSensor(i);
        // add the conversion result
        sensorValues[i - 1] += analogRead(MUX_SIG);
      }
    }

    // get the rounded average of the readings for each sensor
    for (uint8_t i = 0; i < NUM_SENSORS; i += 1) {
      sensorValues[i] = (sensorValues[i] + (4 >> 1)) / 4;
    }

    if(debug && print) {
      printlnArr(sensorValues, NUM_SENSORS);
    }
}

void readCalibratedSensors(bool invert) {
  if(debug) Serial.print("\t\t");
  readSensors(true);
  if(invert) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      sensorValues[i] = map(sensorValues[i], sensor.Min[i] + calibrationOffset + 50, sensor.Max[i] - 20, 1000, 0);
      sensorValues[i] = constrain(sensorValues[i], 0, 1000);
      sensorValuesDig[i] = (sensorValues[i] > 700) ? '1' : '0';
    }
  } else {
    for (int i = 0; i < NUM_SENSORS; i++) {
      sensorValues[i] = map(sensorValues[i], sensor.Min[i] + calibrationOffset + 50, sensor.Max[i] - 20, 0, 1000);
      sensorValues[i] = constrain(sensorValues[i], 0, 1000);
      sensorValuesDig[i] = (sensorValues[i] > 700) ? '1' : '0';
    }
  }
  sensorValuesDig[NUM_SENSORS] = '\0';
}
#endif
#if defined (DiSENSOR)
void readSensors(bool print) {
    for (uint8_t i = 0; i < NUM_SENSORS; i += 1) {
      sensorValues[i] = 0;
    }

    // for (uint8_t j = 0; j < 4; j++) {
      for (uint8_t i = 1; i < NUM_SENSORS + 1; i += 1) {
        selectSensor(i);
        // add the conversion result
        sensorValues[i - 1] = digitalRead(MUX_SIG);
      }
    // }

    // get the rounded average of the readings for each sensor
    // for (uint8_t i = 0; i < NUM_SENSORS; i += 1) {
    //   sensorValues[i] = (sensorValues[i] + (4 >> 1)) / 4;
    // }

    if(debug && print) {
      printlnArr(sensorValues, NUM_SENSORS);
    }
}
void readCalibratedSensors(bool invert) {
  // if(debug) Serial.print("\t\t");
  readSensors(false);
  sensorValuesDig = 0x0000;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if(invert) sensorValues[i] = map(sensorValues[i], sensor.Min[i], sensor.Max[i], 1000, 0);
    else       sensorValues[i] = map(sensorValues[i], sensor.Min[i], sensor.Max[i], 0, 1000);
    sensorValues[i] = constrain(sensorValues[i], 0, 1000);
    bool currentDigiVal = (sensorValues[i] > 700 ? 1 : 0);
    sensorValuesDig |= (currentDigiVal << ( NUM_SENSORS - 1 - i));
    // sensorValuesDig[i] = sensorValues[i] > 700 ? '1' : '0';
  }
  // sensorValuesDig[NUM_SENSORS] = '\0';
  
  // String digVals = sensorValuesDig;
  int digVals = sensorValuesDig;
  if(digVals == 0b010000000000 ||
     digVals == 0b011000000000 ||
     digVals == 0b001000000000 ||
     digVals == 0b001100000000 ||
     digVals == 0b000100000000 ||
     digVals == 0b000110000000 ||
     digVals == 0b000010000000 ||
     digVals == 0b000011000000 ||
     digVals == 0b000001000000 ||
     digVals == 0b000001100000 ||
     digVals == 0b000000100000 ||
     digVals == 0b000000110000 ||
     digVals == 0b000000010000 ||
     digVals == 0b000000011000 ||
     digVals == 0b000000001000 ||
     digVals == 0b000000001100 ||
     digVals == 0b000000000100 ||
     digVals == 0b000000000110 ||
     digVals == 0b000000000010 ) inverted = false;
  else
  if(digVals == 0b100011111111 ||
     digVals == 0b100001111111 ||
     digVals == 0b110001111111 ||
     digVals == 0b110000111111 ||
     digVals == 0b111000111111 ||
     digVals == 0b111000011111 ||
     digVals == 0b111100011111 ||
     digVals == 0b111100001111 ||
     digVals == 0b111110001111 ||
     digVals == 0b111110000111 ||
     digVals == 0b111110000111 ||
     digVals == 0b111111000111 ||
     digVals == 0b111111000011 ||
     digVals == 0b111111100011 ||
     digVals == 0b111111100001 ||
     digVals == 0b111111110001 ) inverted = true;
  
  if(inverted) {
    for(int i = 0; i < NUM_SENSORS; i++) {
      sensorValues[i] = map(sensorValues[i], 0, 1000, 1000, 0);
      // sensorValuesDig[i] = map(sensorValues[i], 0, 1000, 1, 0);
      sensorValuesDig ^= 0b111111111111;
    }
  }
}
#endif

// Function to calculate line position
float getLinePosition() {
    long weightedSum = 0;
    long sum = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
        int value = sensorValues[i];
        weightedSum += (long)i * value;
        sum += value;
    }

    solidLine = (sum > (NUM_SENSORS * 1000) - 500) ? true : false;

    if (sum < 800) return -1;
    return (float)weightedSum / sum;
}

float getPositionError() {
  float error;
  int sum = 0;
  int positionWeight = 0;
  for(int i = -6; i <= 0; i++) {
    int temp = sensorValues[i + 6] * i;
    positionWeight += temp;
    sum += sensorValues[i + 6];
  }
  for(int i = 1; i <= 6; i++) {
    int temp = sensorValues[i + 5] * i;
    positionWeight += temp;
    sum += sensorValues[i + 5];
  }
  error = (float)positionWeight / (float)sum;
  if(positionWeight == 0) return 0;
  return error;
}

void calibrateSensors() {
  stopMotors();
  digitalWrite(LIGHT, HIGH);
  Serial.println("\nCalibrating... Move the robot over the line!");
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensor.Min[i] = 1023;
    sensor.Max[i] = 0;
  }
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) { 
    readSensors(false);
    for (int i = 0; i < NUM_SENSORS; i++) {
      sensor.Min[i] = min(sensor.Min[i], sensorValues[i]);
      sensor.Max[i] = max(sensor.Max[i], sensorValues[i]);
    }
  }
  sensor.calibrationDone = true;
  Serial.println("Calibration complete!");
  digitalWrite(LIGHT, LOW);

  EEPROM.put(0x0E ,sensor);

  if(debug) {
    Serial.print("Min:\t");
    printlnArr(sensor.Min, NUM_SENSORS);
    Serial.print("Max:\t");
    printlnArr(sensor.Max, NUM_SENSORS);
    Serial.println();
  }
}


void buzz() {
  tBuzz = millis();
  buzzed = false;
  digitalWrite(BUZZER, HIGH);
  if(!inverted) digitalWrite(LIGHT, HIGH);
  else digitalWrite(LIGHT, LOW);
}

// PID Controller
float pidControl() {
  readCalibratedSensors(true);          // Invert readings
  float position = getLinePosition();
  checkSharpTurn();

  error = position - (((float)NUM_SENSORS - 1.0) / 2);
  // error = getPositionError();
  if(debug){
    Serial.print("Err:"); Serial.print(error);
  }
  if(solidLine) {
    if(buzzed && millis() > tBuzz + 1000) buzz();
    if(solidLineTStarted == false) tSolidLine = millis();
    solidLineTStarted = true;
    
    if(millis() > tSolidLine + solidLineStopDel) {
      // Serial.println("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
      setMotorSpeed(0, 0);
      delay(1000);
    }
  } else {
      solidLineTStarted = false;
  }

  if(position != -1) {
    errorDir = error > 0 ? true : false;
  }
  
  integral = constrain(integral + error, -500, 500);
  int derivative = error - lastError;
  lastError = error;
  int output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  if (position == -1 ) {
    if(noLine == false) {
      tNoLine = millis();
      noLine = true;
    }

    if(millis() > tNoLine + noLineStopDelay) {
      // if(millis() < tNoLine + (noLineStopDelay + backTrackDuration)) setMotorSpeed(-miscSpeed, -miscSpeed);
      if(millis() < tNoLine + (noLineStopDelay + backTrackDuration + brakeDuration)) {
        shortBrake(brakeDuration);
        // if(!errorDir) setMotorSpeed(-miscSpeed + 100, -miscSpeed);
        // else setMotorSpeed(-miscSpeed, -miscSpeed + 100);
        if(!errorDir) setMotorSpeed(-miscSpeed + 100, -miscSpeed);
        else setMotorSpeed(-miscSpeed, -miscSpeed + 100);
      }
      else {
        if(sharpTurn) {
          if(sharpTurnDir) pivotLeft(miscSpeed);
          else pivotRight(miscSpeed);
          if(position != -1) sharpTurn = false;
          lastPosition = position;
          return position;
        }
        // if(!errorDir) setMotorSpeed(-miscSpeed, miscSpeed);
        // else setMotorSpeed(miscSpeed, -miscSpeed);
        if(!errorDir) pivotLeft(miscSpeed);
        else pivotRight(miscSpeed);
      }
    }
      // Rotate in place if no line is detected
    lastPosition = position;
    return position;
  } else {
    noLine = false;
  }

  if(position > -1 && lastPosition == -1) sharpTurn = false;


  int leftSpeed = constrain(baseSpeed + output, minSpeed, maxSpeed);
  int rightSpeed = constrain(baseSpeed - output, minSpeed, maxSpeed);

  
  // else 
  setMotorSpeed(leftSpeed, rightSpeed);

  lastPosition = position;
  return position;
}
void checkSharpTurn() {
  // String digVals = String(sensorValuesDig);
  // String str = sensorValuesDig;
  // for(int i = 0; i < NUM_SENSORS; i++) {
  //   str += (char)sensorValuesDig[i];
  // }
  if(debug) {
    Serial.print("Sharp");
    // Serial.print(digVals);
    Serial.print(sensorValuesDig, BIN);
  }
  // if(/*digVals == F("100000000000") ||
  //    digVals == F("110000000000") ||*/
  //    digVals == F("001000001000") ||
  //    digVals == F("001000011000") ||
  //    digVals == F("001100011000") ||
  //    digVals == F("001110011000") ||
  //    digVals == F("000110011000") ||
  //    digVals == F("000100011000") ||
  //    digVals == F("000010011000") ||
  //    digVals == F("000111011000") ||
  //    digVals == F("000011011000") ||
  //    digVals == F("000001011000") ||
  //    digVals == F("000011111000") ||
  //    digVals == F("000111111000") ||
  //    digVals == F("010000010000") ||
  //    digVals == F("010000110000") ||
  //    digVals == F("011000110000") ||
  //    digVals == F("011100110000") ||
  //    digVals == F("001100110000") ||
  //    digVals == F("001000110000") ||
  //    digVals == F("000100110000") ||
  //    digVals == F("001110110000") ||
  //    digVals == F("000110110000") ||
  //    digVals == F("000010110000") ||
  //    digVals == F("000111110000") ||
  //    digVals == F("001111110000") ||
  //    digVals == F("100000100000") ||
  //    digVals == F("100001100000") ||
  //    digVals == F("110001100000") ||
  //    digVals == F("111001100000") ||
  //    digVals == F("011001100000") ||
  //    digVals == F("010001100000") ||
  //    digVals == F("001001100000") ||
  //    digVals == F("011101100000") ||
  //    digVals == F("001101100000") ||
  //    digVals == F("000101100000") ||
  //    digVals == F("001111100000") ||
  //    digVals == F("011111100000") ||
  //    digVals == F("111111111000") ||
  //    digVals == F("111111110000") ||
  //    digVals == F("111111100000") ||
  //    digVals == F("111111000000") ||
  //    digVals == F("111110000000") ||
  //    digVals == F("111100000000") ||
  //    digVals == F("011111110000") && digVals != F("111111111111") ) {
  //   sharpTurn = true;
  //   sharpTurnDir = true;
  //   // setMotorSpeed(-0, abs(output));
  //   // delay(hardTurnDel);
  // } else 
  // if(/*digVals == F("000000000001") ||
  //    digVals == F("000000000011") ||*/
  //    digVals == F("000001000001") ||
  //    digVals == F("000001100001") ||
  //    digVals == F("000001100011") ||
  //    digVals == F("000001100111") ||
  //    digVals == F("000001100110") ||
  //    digVals == F("000001100010") ||
  //    digVals == F("000001100100") ||
  //    digVals == F("000001101110") ||
  //    digVals == F("000001111100") ||
  //    digVals == F("000001111110") ||
  //    digVals == F("000100000100") ||
  //    digVals == F("000110000100") ||
  //    digVals == F("000110001100") ||
  //    digVals == F("000110011100") ||
  //    digVals == F("000110011000") ||
  //    digVals == F("000110001000") ||
  //    digVals == F("000110010000") ||
  //    digVals == F("000110111000") ||
  //    digVals == F("000111110000") ||
  //    digVals == F("000111111000") ||
  //    digVals == F("000010000010") ||
  //    digVals == F("000011000010") ||
  //    digVals == F("000011000110") ||
  //    digVals == F("000011001110") ||
  //    digVals == F("000011001100") ||
  //    digVals == F("000011000100") ||
  //    digVals == F("000011001000") ||
  //    digVals == F("000011011100") ||
  //    digVals == F("000011111000") ||
  //    digVals == F("000011111100") ||
  //    digVals == F("000111111111") ||
  //    digVals == F("000011111111") ||
  //    digVals == F("000001111111") ||
  //    digVals == F("000000111111") ||
  //    digVals == F("000000011111") ||
  //    digVals == F("000000001111") ||
  //    digVals == F("000011111110") && digVals != F("111111111111") ) {
  //     sharpTurn = true;
  //     sharpTurnDir = false;
  //   // setMotorSpeed(abs(output), -0);
  //   // delay(hardTurnDel);
  // } 
  // else sharpTurn = false;
  int digvals = sensorValuesDig;
  if((
    /*sensorValuesDig == 0b001000001000 ||
     sensorValuesDig == 0b001000011000 ||
     sensorValuesDig == 0b001100011000 ||
     sensorValuesDig == 0b001110011000 ||
     sensorValuesDig == 0b000110011000 ||
     sensorValuesDig == 0b000100011000 ||
     sensorValuesDig == 0b000010011000 ||
     sensorValuesDig == 0b000111011000 ||
     sensorValuesDig == 0b000011011000 ||
     sensorValuesDig == 0b000001011000 ||
     sensorValuesDig == 0b000011111000 ||
     sensorValuesDig == 0b000111111000 ||
     sensorValuesDig == 0b010000010000 ||
     sensorValuesDig == 0b010000110000 ||
     sensorValuesDig == 0b011000110000 ||
     sensorValuesDig == 0b011100110000 ||
     sensorValuesDig == 0b001100110000 ||
     sensorValuesDig == 0b001000110000 ||
     sensorValuesDig == 0b000100110000 ||
     sensorValuesDig == 0b001110110000 ||
     sensorValuesDig == 0b000110110000 ||
     sensorValuesDig == 0b000010110000 ||
     sensorValuesDig == 0b000111110000 ||
     sensorValuesDig == 0b001111110000 ||
     sensorValuesDig == 0b100000000000 ||
     sensorValuesDig == 0b110000000000 ||
     sensorValuesDig == 0b100001100000 ||
     sensorValuesDig == 0b100001110000 ||
     sensorValuesDig == 0b110001110000 ||
     sensorValuesDig == 0b011100111000 ||
     sensorValuesDig == 0b011000111000 ||
     sensorValuesDig == 0b011100111000 ||
     sensorValuesDig == 0b111100111000 ||
     sensorValuesDig == 0b111100111100 ||
     sensorValuesDig == 0b110001100000 ||
     sensorValuesDig == 0b111001100000 ||
     sensorValuesDig == 0b011001100000 ||
     sensorValuesDig == 0b010001100000 ||
     sensorValuesDig == 0b001001100000 ||
     sensorValuesDig == 0b011101100000 ||
     sensorValuesDig == 0b001101100000 ||
     sensorValuesDig == 0b000101100000 ||
     sensorValuesDig == 0b001111100000 ||
     sensorValuesDig == 0b011111100000 ||
     sensorValuesDig == 0b111111111000 ||
     sensorValuesDig == 0b111111110000 ||
     sensorValuesDig == 0b111111100000 ||
     sensorValuesDig == 0b111111000000 ||
     sensorValuesDig == 0b111110000000 ||
     sensorValuesDig == 0b111100000000 ||
     sensorValuesDig == 0b011111110000*/
     sensorValuesDig == 0b100000000000 ||
     sensorValuesDig == 0b110000000000 ||
     sensorValuesDig == 0b100000100000 ||
     sensorValuesDig == 0b100001100000 ||
     sensorValuesDig == 0b110001100000 ||
     sensorValuesDig == 0b110001110000 ||
     sensorValuesDig == 0b011000111000 ||
     sensorValuesDig == 0b011100111000 ||
     sensorValuesDig == 0b111000111000 ||
     sensorValuesDig == 0b111001100000 ||
     sensorValuesDig == 0b011001100000 ||
     sensorValuesDig == 0b010001100000 ||
     sensorValuesDig == 0b001001100000 ||
     sensorValuesDig == 0b011101100000 ||
     sensorValuesDig == 0b001111100000 ||
     sensorValuesDig == 0b011111100000 ||
     sensorValuesDig == 0b001000010000 ||
     sensorValuesDig == 0b001000011000 ||
     sensorValuesDig == 0b001100011000 ||
     sensorValuesDig == 0b001110011000 ||
     sensorValuesDig == 0b000110011000 ||
     sensorValuesDig == 0b000100011000 ||
     sensorValuesDig == 0b000010011000 ||
     sensorValuesDig == 0b000111011000 ||
     sensorValuesDig == 0b000111111000 ||
     sensorValuesDig == 0b000111111000 ||
     sensorValuesDig == 0b010000010000 ||
     sensorValuesDig == 0b010000110000 ||
     sensorValuesDig == 0b011000110000 ||
     sensorValuesDig == 0b011100110000 ||
     sensorValuesDig == 0b001100110000 ||
     sensorValuesDig == 0b001000110000 ||
     sensorValuesDig == 0b000100110000 ||
     sensorValuesDig == 0b001110110000 ||
     sensorValuesDig == 0b000111110000 ||
     sensorValuesDig == 0b001111110000 ||
     sensorValuesDig == 0b111111110000 ||
     sensorValuesDig == 0b111111100000 ||
     sensorValuesDig == 0b111111000000 ||
     sensorValuesDig == 0b111110000000 ||
     sensorValuesDig == 0b111100000000 ||
     sensorValuesDig == 0b111000000000 ||
     sensorValuesDig == 0b011111100000) && !inverted) {
    sharpTurn = true;
    sharpTurnDir = true;
  }
  // else
  if((sensorValuesDig == 0b000000000001 ||
     sensorValuesDig == 0b000000000011 ||
     sensorValuesDig == 0b000001000001 ||
     sensorValuesDig == 0b000001100001 ||
     sensorValuesDig == 0b000001100011 ||
     sensorValuesDig == 0b000011100011 ||
     sensorValuesDig == 0b000111000110 ||
     sensorValuesDig == 0b000111001110 ||
     sensorValuesDig == 0b000111000111 ||
     sensorValuesDig == 0b000001100111 ||
     sensorValuesDig == 0b000001100110 ||
     sensorValuesDig == 0b000001100010 ||
     sensorValuesDig == 0b000001100100 ||
     sensorValuesDig == 0b000001101110 ||
     sensorValuesDig == 0b000001111100 ||
     sensorValuesDig == 0b000001111110 ||
     sensorValuesDig == 0b000100000100 ||
     sensorValuesDig == 0b000110000100 ||
     sensorValuesDig == 0b000110001100 ||
     sensorValuesDig == 0b000110011100 ||
     sensorValuesDig == 0b000110011000 ||
     sensorValuesDig == 0b000110001000 ||
     sensorValuesDig == 0b000110010000 ||
     sensorValuesDig == 0b000110111000 ||
     sensorValuesDig == 0b000111110000 ||
     sensorValuesDig == 0b000111111000 ||
     sensorValuesDig == 0b000010000010 ||
     sensorValuesDig == 0b000011000010 ||
     sensorValuesDig == 0b000011000110 ||
     sensorValuesDig == 0b000011001110 ||
     sensorValuesDig == 0b000011001100 ||
     sensorValuesDig == 0b000011000100 ||
     sensorValuesDig == 0b000011001000 ||
     sensorValuesDig == 0b000011011100 ||
     sensorValuesDig == 0b000011111000 ||
     sensorValuesDig == 0b000011111100 ||
     sensorValuesDig == 0b000111111111 ||
     sensorValuesDig == 0b000011111111 ||
     sensorValuesDig == 0b000001111111 ||
     sensorValuesDig == 0b000000111111 ||
     sensorValuesDig == 0b000000011111 ||
     sensorValuesDig == 0b000000001111 ||
     sensorValuesDig == 0b000011111110) && !inverted ) {
    sharpTurn = true;
    sharpTurnDir = false;
  }

}

void readEEPROM(){
  EEPROM.get(0x00, Kp);
  EEPROM.get(0x04, Ki);
  EEPROM.get(0x08, Kd);
  EEPROM.get(0x0C, baseSpeed);
  EEPROM.get(0x0E, sensor);
  // EEPROM.get(0x40, LFRConsole);
  EEPROM.get(0x41, debug);
  EEPROM.get(0x42, miscSpeed);
  EEPROM.get(0x44, noLineStopDelay);
  EEPROM.get(0x46, backTrackDuration);
  EEPROM.get(0x48, brakeDuration);
  EEPROM.get(0x49, solidLineStopDel);
}

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
      case 'm': miscSpeed = Serial.parseInt(); EEPROM.put(0x42, miscSpeed); Serial.print("Misc Spd:"); Serial.print(miscSpeed); break;
      case 'T': noLineStopDelay = Serial.parseInt(); EEPROM.put(0x44, noLineStopDelay); Serial.print("No Line Stop-Delay:"); Serial.print(noLineStopDelay); break;
      case 't': backTrackDuration = Serial.parseInt(); EEPROM.put(0x46, backTrackDuration); Serial.print("Back Track Duration:"); Serial.print(backTrackDuration); break;
      case 'b': brakeDuration = Serial.parseInt(); EEPROM.put(0x48, (byte)brakeDuration); Serial.print("Brake Duration:"); Serial.print(brakeDuration); break;
      case 's': solidLineStopDel = Serial.parseInt(); EEPROM.put(0x49, solidLineStopDel); Serial.print("Stop Delay:"); Serial.print(solidLineStopDel); break;


      case 'F': setMotorSpeed(baseSpeed, baseSpeed); break;
      case 'B': setMotorSpeed(-baseSpeed, -baseSpeed); break;
      case 'L': setMotorSpeed(-baseSpeed, baseSpeed); break;
      case 'R': setMotorSpeed(baseSpeed, -baseSpeed); break;

      case 'C': sensor.calibrationDone = false; Serial.print("Calibrate "); break;
      case 'Y': digitalWrite(BUZZER, !digitalRead(BUZZER)); Serial.print("Buzzer "); Serial.print(digitalRead(BUZZER) ? "ON" : "OFF"); break;
      case 'U': digitalWrite(LIGHT, !digitalRead(LIGHT)); Serial.print("Light "); Serial.print(digitalRead(LIGHT) ? "ON" : "OFF"); break;
      // case 'y': digitalWrite(BUZZER, LOW); break;

      case 'l': LFRConsole = !LFRConsole; if(LFRConsole) debug = false; EEPROM.put(0x40, LFRConsole); EEPROM.put(0x41, debug); break;
      case 'd': debug = !debug; if(debug) LFRConsole = false; EEPROM.put(0x40, LFRConsole); EEPROM.put(0x41, debug); break;
      // case '{': consoleAppRx(cmd); break;
      case 'x': BTDrive = false; Serial.print(BTDrive); break;
      case 'X': BTDrive = true; debug = false;   //Fall-through to stop
      default: setMotorSpeed(0, 0); 
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
  Serial.print("No Line Delay:\t\t"); Serial.print(noLineStopDelay); Serial.println("ms");
  Serial.print("Back Track Delay:\t"); Serial.print(backTrackDuration); Serial.println("ms");
  Serial.print("Breaking Duration:\t"); Serial.print(brakeDuration); Serial.println("ms");
  Serial.print("No Line Stop Delay:\t"); Serial.print(solidLineStopDel); Serial.println("ms");

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
    Serial.print(pos); Serial.print(" "); 
    Serial.print(lastPosition); Serial.print("\t");
    Serial.print(errorDir); Serial.print("\t");
    Serial.print(sharpTurn); Serial.print(" "); Serial.print(sharpTurnDir); 
    Serial.println();
  }

  receiveBluetoothData();
}