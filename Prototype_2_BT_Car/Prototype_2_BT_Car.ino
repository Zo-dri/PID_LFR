#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// Motor A (Left)
#define PWMA 14
#define AIN1 13
#define AIN2 12

// Motor B (Right)
#define PWMB 15
#define BIN1 2
#define BIN2 4

#define STB  27

// #define SWLED 16
#define SWLED 5
// PWM settings
#define PWM_FREQ 1000
#define PWM_RES 8 // 8-bit resolution (0-255)

// Motor speed (adjustable)
#define FULL_SPEED 255
#define HALF_SPEED 200
#define STOP_SPEED 0

int speed = FULL_SPEED;

void setup() {
    Serial.begin(115200);
    Serial2.begin(9600);
    SerialBT.begin("ESP32_RC_CAR"); // Bluetooth device name

    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(STB, OUTPUT);
    pinMode(SWLED, OUTPUT);

    // Attach PWM channels
    ledcAttach(PWMA, PWM_FREQ, PWM_RES);
    ledcAttach(PWMB, PWM_FREQ, PWM_RES);

    stopMotors();

    digitalWrite(STB, HIGH);
}

void loop() {
  if(SerialBT.connected()) {
    digitalWrite(SWLED, LOW);
  } else {
    digitalWrite(SWLED, HIGH);
  }
  if (SerialBT.available()) {
      char command = SerialBT.read();
      Serial.println(command);
      processCommand(command);
  }
  if (Serial2.available()) {
      Serial.write(Serial2.read());
  }
}

void processCommand(char command) {
    switch (command) {
        case 'F': moveForward (speed);      break;
        case 'B': moveBackward(speed);      break;
        case 'L': turnLeft    (speed);      break;
        case 'R': turnRight   (speed);      break;
        case 'S': stopMotors  ();           break;
        case '0': speed = FULL_SPEED * 0.1; break;
        case '1': speed = FULL_SPEED * 0.2; break;
        case '2': speed = FULL_SPEED * 0.3; break;
        case '3': speed = FULL_SPEED * 0.4; break;
        case '4': speed = FULL_SPEED * 0.5; break;
        case '5': speed = FULL_SPEED * 0.6; break;
        case '6': speed = FULL_SPEED * 0.7; break;
        case '7': speed = FULL_SPEED * 0.8; break;
        case '8': speed = FULL_SPEED * 0.9; break;
        case '9': speed = FULL_SPEED * 1.0; break;

        default: stopMotors();
    }
}

void moveForward(int speed) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    ledcWrite(PWMA, speed);

    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    ledcWrite(PWMB, speed);
}

void moveBackward(int speed) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    ledcWrite(PWMA, speed);

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    ledcWrite(PWMB, speed);
}

void turnLeft(int speed) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    ledcWrite(PWMA, speed);

    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    ledcWrite(PWMB, speed);
}

void turnRight(int speed) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    ledcWrite(PWMA, speed);

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    ledcWrite(PWMB, speed);
}

void stopMotors() {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    ledcWrite(PWMA, STOP_SPEED);

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    ledcWrite(PWMB, STOP_SPEED);
}