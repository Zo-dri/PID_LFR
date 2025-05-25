#include "memory.h"

void readEEPROM(float pidValues[]){
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
  }
  