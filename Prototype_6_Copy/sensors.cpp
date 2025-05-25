#include "sensors.h"

void selectSensor(byte n) {
    digitalWrite(MUX_S0, (n & 0x01));
    digitalWrite(MUX_S1, (n & 0x02));
    digitalWrite(MUX_S2, (n & 0x04));
    digitalWrite(MUX_S3, (n & 0x08));
  }
  
#if defined (AnalogSENSOR)
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

#if defined (DigitalSENSOR)
  void readSensors(bool print) {
    for (uint8_t i = 0; i < NUM_SENSORS; i += 1) {
      sensorValues[i] = 0;
    }

    for (uint8_t i = 1; i < NUM_SENSORS + 1; i += 1) {
      selectSensor(i);
      sensorValues[i - 1] = digitalRead(MUX_SIG);
    }
    if(debug && print) {
      printlnArr(sensorValues, NUM_SENSORS);
    }
  }
  void readCalibratedSensors(bool invert) {
    // if(debug) Serial.print("\t\t");
    readSensors(false);
    for (int i = 0; i < NUM_SENSORS; i++) {
      sensorValues[i] = map(sensorValues[i], sensor.Min[i], sensor.Max[i], invert ? 1000 : 0, invert ? 0 : 1000);
      sensorValuesDig[i] = sensorValues[i] > 700 ? '1' : '0';
      sensorValues[i] = constrain(sensorValues[i], 0, 1000);
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

