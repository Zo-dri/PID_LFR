#pragma once

// #include "config.h"
// #include "declarations.h"
void printArr(int arr[], byte size) {
  for(int i = 0; i < size; i++) {
    Serial.print(i + 1); Serial.print(":");
    Serial.print(arr[i]); Serial.print("\t");
  }
}
void printlnArr(int arr[], byte size) {
  printArr(arr, size);
  Serial.println();
}
void printArr(char arr[], byte size) {
  for(int i = 0; i < size; i++) {
    Serial.print(i + 1); Serial.print(":");
    Serial.print(arr[i]); Serial.print("\t");
  }
}
void printlnArr(char arr[], byte size) {
  printArr(arr, size);
  Serial.println();
}
