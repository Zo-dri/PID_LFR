#include "utils.h"

void printArr(int arr[], uint8_t size) {
    for(int i = 0; i < size; i++) {
      Serial.print(i + 1); Serial.print(":");
      Serial.print(arr[i]); Serial.print("\t");
    }
  }
void printlnArr(int arr[], uint8_t size) {
printArr(arr, size);
Serial.println();
}
void printArr(char arr[], uint8_t size) {
for(int i = 0; i < size; i++) {
    Serial.print(i + 1); Serial.print(":");
    Serial.print(arr[i]); Serial.print("\t");
}
}
void printlnArr(char arr[], uint8_t size) {
    printArr(arr, size);
    Serial.println();
  }

float readBattery() {
  return (analogRead(BATTERY) * 5.0 * vRatio) / 1023.0;
}

void buzz() {
    tBuzz = millis();
    buzzed = false;
    digitalWrite(BUZZER, HIGH);
    if(!inverted) digitalWrite(LIGHT, HIGH);
    else digitalWrite(LIGHT, LOW);
  }
  