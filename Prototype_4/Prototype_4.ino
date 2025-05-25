#include <EEPROM.h>
#include <ArduinoJson.h>
#include <string.h>

#define JSON_MAX_LEN 196

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

bool LFRConsole = true;
bool debug = false;

// PID Constants (adjustable via Bluetooth)
float Kp = 25.0;  
float Ki = 0.01; 
float Kd = 10;  

// Sensor Configuration
#define NUM_SENSORS 12
int sensorValues[NUM_SENSORS];
char sensorValuesDig[NUM_SENSORS + 1];
struct calibrationData {
  int Min[NUM_SENSORS];
  int Max[NUM_SENSORS];
  bool calibrationDone = false;
};
calibrationData sensor;

bool prepareLeft = false, prepareRight = false;
bool BTDrive =   false;
bool errorDir =  false;
bool inverted =  false;
bool solidLine = false;
bool solidLineTStarted = false;
bool noLine =    false;
bool buzzed =    false;
float error =      0.0;
float integral =   0.0;
int noLineDelay = 50;
int backTrackDelay = 250;
int solidLineStopDel = 100;
int hardTurnDel = 40;
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

void printArr(int arr[], byte size) {
  for(int i = 0; i < size; i++) {
    Serial.print(i + 1); Serial.print(":");
    Serial.print(arr[i]); Serial.print("\t");
  }
}
void printArr(char arr[], byte size) {
  for(int i = 0; i < size; i++) {
    Serial.print(i + 1); Serial.print(":");
    Serial.print(arr[i]); Serial.print("\t");
  }
}
void printlnArr(int arr[], byte size) {
  printArr(arr, size);
  Serial.println();
}

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
      // if(i == 3) sensorValues[i] = map(sensorValues[i], sensor.Min[i] + calibrationOffset + 50, sensor.Max[i] - 20, 0, 1000); // Invert value of Sensor 4 
      /*else */sensorValues[i] = map(sensorValues[i], sensor.Min[i] + calibrationOffset + 50, sensor.Max[i] - 20, 1000, 0);
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
  if(invert) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      // if(i == 3) sensorValues[i] = map(sensorValues[i], sensor.Min[i] + calibrationOffset + 50, sensor.Max[i] - 20, 0, 1000); // Invert value of Sensor 4 
      /*else */sensorValues[i] = map(sensorValues[i], sensor.Min[i], sensor.Max[i], 1000, 0);
      sensorValues[i] = constrain(sensorValues[i], 0, 1000);
      sensorValuesDig[i] = sensorValues[i] > 700 ? '1' : '0';
    }
  } else {
    for (int i = 0; i < NUM_SENSORS; i++) {
      sensorValues[i] = map(sensorValues[i], sensor.Min[i], sensor.Max[i], 0, 1000);
      sensorValues[i] = constrain(sensorValues[i], 0, 1000);
      sensorValuesDig[i] = sensorValues[i] > 700 ? '1' : '0';
    }
  }
  sensorValuesDig[NUM_SENSORS] = '\0';
  // String digVals = String(sensorValuesDig);
  String digVals = sensorValuesDig;
  if(digVals == "010000000000" ||
     digVals == "011000000000" ||
     digVals == "001000000000" ||
     digVals == "001100000000" ||
     digVals == "000100000000" ||
     digVals == "000110000000" ||
     digVals == "000010000000" ||
     digVals == "000011000000" ||
     digVals == "000001000000" ||
     digVals == "000001100000" ||
     digVals == "000000100000" ||
     digVals == "000000110000" ||
     digVals == "000000010000" ||
     digVals == "000000011000" ||
     digVals == "000000001000" ||
     digVals == "000000001100" ||
     digVals == "000000000100" ||
     digVals == "000000000110" ||
     digVals == "000000000010" ) inverted = false;
  else
  if(digVals == "101111111111" ||
     digVals == "100111111111" ||
     digVals == "110111111111" ||
     digVals == "110011111111" ||
     digVals == "111011111111" ||
     digVals == "111001111111" ||
     digVals == "111101111111" ||
     digVals == "111100111111" ||
     digVals == "111110111111" ||
     digVals == "111110011111" ||
     digVals == "111111011111" ||
     digVals == "111111001111" ||
     digVals == "111111101111" ||
     digVals == "111111100111" ||
     digVals == "111111110111" ||
     digVals == "111111110011" ||
     digVals == "111111111011" ||
     digVals == "111111111001" ||
     digVals == "111111111101") {
    inverted = true;
    for(int i = 0; i < NUM_SENSORS; i++) {
      sensorValues[i] = map(sensorValues[i], 0, 1000, 1000, 0);
      sensorValuesDig[i] = map(sensorValues[i], 0, 1000, 1, 0);
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
  // for(int i = 0; i < 12; i++) {
  //   sensorValues[i] = sensorValues[i] ? 0 : 1;
  // }
  // printArr(sensorValues, NUM_SENSORS);  Serial.print(" ");
  for(int i = -6; i <= -0; i++) {
    int temp = sensorValues[i + 6] * i;
    positionWeight += temp;
    sum += sensorValues[i + 6];
    // Serial.print(temp);
  }
  for(int i = 1; i <= 6; i++) {
    int temp = sensorValues[i + 5] * i;
    positionWeight += temp;
    sum += sensorValues[i + 5];
    // Serial.print(temp);
  }
  error = (float)positionWeight / (float)sum;
  // Serial.print(" ");
  // Serial.println(error);
  if(positionWeight == 0) return 0;
  return error;
}

void calibrateSensors() {
  setMotorSpeed(0,0);
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
      // Serial.println(lastPosition);
      noLine = true;
    }

    if(millis() > tNoLine + noLineDelay) {
      if(millis() < tNoLine + (noLineDelay + backTrackDelay)) setMotorSpeed(-miscSpeed, -miscSpeed);
      // if(millis() < tNoLine + (noLineDelay + backTrackDelay)) {
      //   if(!errorDir) setMotorSpeed(-miscSpeed + 100, -miscSpeed);
      //   else setMotorSpeed(-miscSpeed, -miscSpeed + 100);
      // }
      else {
        if(!errorDir) setMotorSpeed(-miscSpeed, miscSpeed);
        else setMotorSpeed(miscSpeed, -miscSpeed);
        // setMotorSpeed(miscSpeed, -miscSpeed);
      }
    } else {
      // baseSpeed -= 30;
    }
      // Rotate in place if no line is detected
    return position;
  } else {
    noLine = false;
    // EEPROM.get(0x0C, baseSpeed);
  }


  int leftSpeed = constrain(baseSpeed + output, minSpeed, maxSpeed);
  int rightSpeed = constrain(baseSpeed - output, minSpeed, maxSpeed);
  // int leftSpeed = baseSpeed + output;
  // int rightSpeed = baseSpeed - output;
  // if(leftSpeed > maxSpeed) leftSpeed = maxSpeed;
  // if(rightSpeed > maxSpeed) rightSpeed = maxSpeed;

  // if(sensorValuesDig[0] == '1' && sensorValuesDig[1] == '1' && sensorValuesDig != "111111111111" ) {
  //   setMotorSpeed(-180, 180);
  //   delay(25);
  // } else if(sensorValuesDig[NUM_SENSORS - 2] == '1' && sensorValuesDig[NUM_SENSORS - 1] == '1' && sensorValuesDig != "111111111111" ) {
  //   setMotorSpeed(180, -180);
  //   delay(25);
  // } else setMotorSpeed(leftSpeed, rightSpeed);
  // if(millis() > t1 + 300) {
  // // if(error == 0 || millis() > t1 + 100) {
  //   prepareLeft = false;
  //   prepareRight = false;
  //   t1Started = false;
  // }
  // if(sensorValuesDig[0] == '1') prepareLeft = true;
  // if(sensorValuesDig[NUM_SENSORS - 1] == '1') prepareRight = true;

  String digVals = String(sensorValuesDig);
  // Serial.println(digVals);
  if(digVals == "110000000000" ||
     digVals == "100000100000" ||
     digVals == "100001100000" ||
     digVals == "110001100000" ||
     digVals == "111001100000" ||
     digVals == "011001100000" ||
     digVals == "010001100000" ||
     digVals == "001001100000" ||
     digVals == "011101100000" ||
     digVals == "001101100000" ||
     digVals == "000101100000" ||
     digVals == "001111100000" ||
     digVals == "011111100000" ||
     digVals == "111111100000" ||
     digVals == "111111000000" ||
     digVals == "111110000000" ||
     digVals == "111100000000" ||
     digVals == "011111110000" && digVals != "111111111111" ) {
    // if(!t1Started) {
    //   t1 = millis();
    //   t1Started = true;
    // }
      // shortBrake(50);
      setMotorSpeed(-0, abs(output));
    delay(hardTurnDel);
  } else 
  if(digVals == "000000000011" ||
     digVals == "000001000001" ||
     digVals == "000001100001" ||
     digVals == "000001100011" ||
     digVals == "000001100111" ||
     digVals == "000001100110" ||
     digVals == "000001100010" ||
     digVals == "000001100100" ||
     digVals == "000001101110" ||
     digVals == "000001111100" ||
     digVals == "000001111110" ||
     digVals == "000001111111" ||
     digVals == "000000111111" ||
     digVals == "000000011111" ||
     digVals == "000000001111" ||
     digVals == "000011111110" && digVals != "111111111111" ) {
    // if(!t1Started) {
    //   t1 = millis();
    //   t1Started = true;
    // }
    // shortBrake(50);
    setMotorSpeed(abs(output), -0);
    delay(hardTurnDel);
  } 
  // if(prepareLeft) {
  //   if(!t1Started ) t1 = millis();
  //   t1Started = true;
  //   setMotorSpeed(-0, miscSpeed);
  //   delay(10);
  // } else if(prepareRight) {
  //   if(!t1Started) t1 = millis();
  //   t1Started = true;
  //   setMotorSpeed(miscSpeed, -0);
  //   delay(10);
  // }
  else 
  setMotorSpeed(leftSpeed, rightSpeed);

  lastPosition = position;

  return position;
}

// Motor control function
void setMotorSpeed(int left, int right) {
    digitalWrite(MOTOR_STBY, HIGH);
    if (left >= 0) {
        digitalWrite(MOTOR_L_IN1, HIGH);
        digitalWrite(MOTOR_L_IN2, LOW);
        analogWrite(MOTOR_L_PWM, left);
    } else {
        digitalWrite(MOTOR_L_IN1, LOW);
        digitalWrite(MOTOR_L_IN2, HIGH);
        analogWrite(MOTOR_L_PWM, -left);
    }

    if (right >= 0) {
        digitalWrite(MOTOR_R_IN1, HIGH);
        digitalWrite(MOTOR_R_IN2, LOW);
        analogWrite(MOTOR_R_PWM, right);
    } else {
        digitalWrite(MOTOR_R_IN1, LOW);
        digitalWrite(MOTOR_R_IN2, HIGH);
        analogWrite(MOTOR_R_PWM, -right);
    }
}
void shortBrake(int del) {
  shortBrake();
  delay(del);
}
void shortBrake() {
  setMotorSpeed(0, 0);
}

void readEEPROM(){
  EEPROM.get(0x00, Kp);
  EEPROM.get(0x04, Ki);
  EEPROM.get(0x08, Kd);
  EEPROM.get(0x0C, baseSpeed);
  EEPROM.get(0x0E, sensor);
  EEPROM.get(0x40, LFRConsole);
  EEPROM.get(0x41, debug);
  EEPROM.get(0x42, miscSpeed);
  EEPROM.get(0x44, noLineDelay);
  EEPROM.get(0x46, backTrackDelay);
  // EEPROM.get(0x2F, LFRConsole);
  // EEPROM.get(0x30, debug);
  // EEPROM.get(0x31, miscSpeed);
  // EEPROM.get(0x33, noLineDelay);
  // EEPROM.get(0x35, backTrackDelay);
  // sensor.calibrationDone = false;
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

void consoleAppTx(int pos) {
  JsonDocument consoleTX;
  char output[JSON_MAX_LEN];
  // printArr(sensorValuesDig, NUM_SENSORS + 1);
  // Serial.print("Digi");
  // String str = "";
  // for(int i = 0; i < NUM_SENSORS; i++) {
  //   Serial.print(sensorValuesDig[i]);
  //   if(sensorValuesDig[i] == '1') str = str + '1';
  //   else str += '0';
  // }
  // Serial.print(str);
  // consoleTX["di"] = String(sensorValuesDig);
  JsonArray data = consoleTX["an"].to<JsonArray>();
  for(int i = 0; i < NUM_SENSORS; i++) {
    data.add(map(sensorValues[i], 0, 1000, 0, 99));
  }
  consoleTX["l1"] = "Error";
  consoleTX["v1"] = error;
  consoleTX["l2"] = "Position";
  consoleTX["v2"] = pos;
  
  serializeJson(consoleTX, output);
  Serial.print(output);
  // Serial.print(sensorValuesDig);
  // printArr(sensorValuesDig, NUM_SENSORS);
  Serial.println();

}
void consoleAppRx(char cmd) {
  JsonDocument consoleRX;
  Serial.flush();
  String consoleData = Serial.readStringUntil('\n');
  consoleData = cmd + consoleData;
  DeserializationError deSerError = deserializeJson(consoleRX, consoleData);
  if(error){
    Serial.print("Deserialize JSON FAILED: ");
    // Serial.print(error.f_str());
    Serial.println();
    consoleRX.clear();
    return;
  }
  Kp = consoleRX["P"];
  Ki = consoleRX["I"];
  Kd = consoleRX["D"];
  baseSpeed = consoleRX["ms"];
  // noLineDelay = consoleRX["de"];

}


// Setup function
void setup() {
    Serial.begin(115200);

    readEEPROM();
    Serial.println();
    Serial.println("*********************Prototype 4*********************");
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
    consoleAppTx(pos);
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