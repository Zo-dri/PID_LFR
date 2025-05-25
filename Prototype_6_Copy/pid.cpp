#include "pid.h"

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
  for(int i = -6; i <= -0; i++) {
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
    if((digVals == "110000000000" ||
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
       digVals == "011111110000") && (digVals != "111111111111") ) {
      // if(!t1Started) {
      //   t1 = millis();
      //   t1Started = true;
      // }
        // shortBrake(50);
        setMotorSpeed(-0, abs(output));
      delay(hardTurnDel);
    } else 
    if((digVals == "000000000011" ||
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
       digVals == "000011111110") && (digVals != "111111111111") ) {
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
  