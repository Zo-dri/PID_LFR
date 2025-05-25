#include <EEPROM.h>
#include "LFRConsole.h"
#include "motors.h"
#include "utils.h"


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

/* *******CAUTION******* *
 * Select only one       *
 * ********************* */
// #define AnalogSENSOR
#define DigitalSENSOR

bool LFRConsole = false;
bool debug = false;

// PID Constants (adjustable via Bluetooth)
float Kp = 25.0;  
float Ki = 0.01; 
float Kd = 10;  

// Sensor Configuration
#define NUM_SENSORS 12

//Values Range from 0 - 1000
int sensorValues[NUM_SENSORS];

// {NUM_SENSOR}bit value 
uint16_t sensorValuesDig = 0x0000;

// Stores data about Sensor Calibration
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
float lastError =  0.0;
bool errorDir =  false;
float integral =   0.0;

int noLineStopDelay = 50;
int backTrackDuration = 250;
int solidLineStopDel = 75;
byte brakeDuration = 50;

int miscSpeed =  120;
int baseSpeed =  150;
int maxSpeed  =  180;
int minSpeed  = -120;

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

// Check for {n} consecutive {value} in {sensorValuesDig}
bool checkConsecutive(byte n, bool value) {
  bool val0 = sensorValuesDig & (1 << (NUM_SENSORS - 1));   // Value [0]
  bool val1 = sensorValuesDig & 1;                          // VAlue [11]
  if(val0 != val1) return false;
  else if(val0 == value) return false;

  bool changed = false;
  byte consecSize = 0;
  val0 = (sensorValuesDig & (1 << NUM_SENSORS - 1));

  for(int i = 0; i < NUM_SENSORS; i++) {
    val1 = sensorValuesDig & (1 << NUM_SENSORS - 1 - i);

    if(val0 != val1) {
      if(val1 == value) {
        changed = true;
        // consecSize = 0;            // Count all consecutive sequences
      }
      else changed = false;
    }

    if(changed) consecSize++;
    // Serial.print(consecSize);
    val0 = val1;
  }
  return (consecSize == n);
}

// Function to read sensors from MUX
void readSensors(bool print) {
  #if defined (AnalogSENSOR)
    for (uint8_t i = 0; i < NUM_SENSORS; i += 1) {
      sensorValues[i] = 0;
    }

    for (uint8_t j = 0; j < 4; j++) {
      for (uint8_t i = 1; i < NUM_SENSORS + 1; i += 1) {
        selectSensor(i);
        sensorValues[i - 1] += analogRead(MUX_SIG);
      }
    }
    // get the rounded average of the readings for each sensor
    for (uint8_t i = 0; i < NUM_SENSORS; i += 1) {
      sensorValues[i] = (sensorValues[i] + (4 >> 1)) / 4;
    }
  #endif
  #if defined (DigitalSENSOR)
    for (uint8_t i = 1; i < NUM_SENSORS + 1; i += 1) {
      selectSensor(i);
      sensorValues[i - 1] = digitalRead(MUX_SIG);
    }
  #endif
  
  if(debug && print) printlnArr(sensorValues, NUM_SENSORS);
}
void readCalibratedSensors(bool invert, bool print) {
  readSensors(print);
  sensorValuesDig = 0x0000;

  for (int i = 0; i < NUM_SENSORS; i++) {
    #if defined (AnalogSENSOR)
      if(invert) sensorValues[i] = map(sensorValues[i], sensor.Min[i] + calibrationOffset + 50, sensor.Max[i] - 20, 1000, 0);
      else       sensorValues[i] = map(sensorValues[i], sensor.Min[i] + calibrationOffset + 50, sensor.Max[i] - 20, 0, 1000);
    #endif
    #if defined (DigitalSENSOR)
      if(invert) sensorValues[i] = map(sensorValues[i], sensor.Min[i], sensor.Max[i], 1000, 0);
      else       sensorValues[i] = map(sensorValues[i], sensor.Min[i], sensor.Max[i], 0, 1000);
    #endif
    sensorValues[i] = constrain(sensorValues[i], 0, 1000);
    bool currentDigiVal = (sensorValues[i] > 700 ? 1 : 0);
    sensorValuesDig |= (currentDigiVal << (NUM_SENSORS - 1 - i));
  }
  
  if(checkConsecutive(1, 1) || checkConsecutive(2, 1)) inverted = false;
  else 
  if(checkConsecutive(3, 0) || checkConsecutive(4, 0)) inverted = true;

  if(inverted) {
    for(int i = 0; i < NUM_SENSORS; i++) {
      sensorValues[i] = map(sensorValues[i], 0, 1000, 1000, 0);
      sensorValuesDig ^= 0b111111111111;
    }
  }
}

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
  int sum = 0;
  int positionWeight = 0;
  for(int i = -6; i <= 0; i++) {
    positionWeight += sensorValues[i + 6] * i;
    sum += sensorValues[i + 6];
  }
  for(int i = 1; i <= 6; i++) {
    positionWeight += sensorValues[i + 5] * i;
    sum += sensorValues[i + 5];
  }
  if(positionWeight == 0) return 0;
  return (float)positionWeight / (float)sum;
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
  readCalibratedSensors(true, false);          // Invert readings
  float position = getLinePosition();

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
        if(!errorDir) pivotLeft(miscSpeed);
        else pivotRight(miscSpeed);
      }
    }
      // Rotate in place if no line is detected
    return position;
  } else {
    noLine = false;
  }

  int leftSpeed = constrain(baseSpeed + output, minSpeed, maxSpeed);
  int rightSpeed = constrain(baseSpeed - output, minSpeed, maxSpeed);

  setMotorSpeed(leftSpeed, rightSpeed);

  return position;
}

void readEEPROM(){
  EEPROM.get(0x00, Kp);
  EEPROM.get(0x04, Ki);
  EEPROM.get(0x08, Kd);
  EEPROM.get(0x0C, baseSpeed);
  EEPROM.get(0x0E, sensor);

  // EEPROM.get(0x4F, LFRConsole);
  EEPROM.get(0x50, debug);
  EEPROM.get(0x51, miscSpeed);
  EEPROM.get(0x53, noLineStopDelay);
  EEPROM.get(0x55, backTrackDuration);
  EEPROM.get(0x57, brakeDuration);
  EEPROM.get(0x58, solidLineStopDel);
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

      case 'M': baseSpeed =         Serial.parseInt();  EEPROM.put(0x0C, baseSpeed);            Serial.print(baseSpeed); break;    // Occupies address 0x0C to 0x0D
      case 'm': miscSpeed =         Serial.parseInt();  EEPROM.put(0x51, miscSpeed);            Serial.print("Misc Spd:");            Serial.print(miscSpeed); break;
      case 'T': noLineStopDelay =   Serial.parseInt();  EEPROM.put(0x53, noLineStopDelay);      Serial.print("No Line Stop-Delay:");  Serial.print(noLineStopDelay); break;
      case 't': backTrackDuration = Serial.parseInt();  EEPROM.put(0x55, backTrackDuration);    Serial.print("Back Track Duration:"); Serial.print(backTrackDuration); break;
      case 'b': brakeDuration =     Serial.parseInt();  EEPROM.put(0x57, (byte)brakeDuration);  Serial.print("Brake Duration:");      Serial.print(brakeDuration); break;
      case 's': solidLineStopDel =  Serial.parseInt();  EEPROM.put(0x58, solidLineStopDel);     Serial.print("Stop Delay:");          Serial.print(solidLineStopDel); break;


      case 'F': setMotorSpeed(baseSpeed, baseSpeed);    break;
      case 'B': setMotorSpeed(-baseSpeed, -baseSpeed);  break;
      case 'L': setMotorSpeed(-baseSpeed, baseSpeed);   break;
      case 'R': setMotorSpeed(baseSpeed, -baseSpeed);   break;

      case 'C': sensor.calibrationDone = false; Serial.print("Calibrate "); break;
      case 'Y': digitalWrite(BUZZER, !digitalRead(BUZZER)); Serial.print("Buzzer ");  Serial.print(digitalRead(BUZZER) ? "ON" : "OFF"); break;
      case 'U': digitalWrite(LIGHT, !digitalRead(LIGHT));   Serial.print("Light ");   Serial.print(digitalRead(LIGHT) ? "ON" : "OFF"); break;
      // case 'y': digitalWrite(BUZZER, LOW); break;

      case 'l': LFRConsole = !LFRConsole; if(LFRConsole) debug = false; EEPROM.put(0x4F, LFRConsole); EEPROM.put(0x50, debug); break;
      case 'd': debug = !debug;           if(debug) LFRConsole = false; EEPROM.put(0x4F, LFRConsole); EEPROM.put(0x50, debug); break;
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
  Serial.println("*********************Prototype 7*********************");
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
    Serial.print(errorDir); Serial.print("\t");
    Serial.println();
  }

  receiveBluetoothData();
}