# PID_LFR
Various iterations of Arduino sketch for a PID Line Follower

MCU Tested:
  - ESP32 DEVKIT (Fried)
  - Arduino NANO

Sensors Tested:
  - RLS08
  - QTR8A
  - Bharat Sensors 8CH board
  - HyperDrive 12CH Board

Motor Drivers Tested:
  - L298N
  - BTS7960
  - TB6612FNG

Motors Tested:
  - N20 Geared 600RPM
  - N30 Geared 600RPM at 3.7V (DISASTER!)

Currently supports:
  - PID Line Following algorithm
  - both Analog and Digital Sensors
  - live update of variables via Bluetooth or Serial
  - EEPROM Storage

To-Do:
  - Fix Battery Voltage Measurement
  - Fix Sharp Turns (Remove Hard Coding)
  - Add logic for Exotic Patterns
