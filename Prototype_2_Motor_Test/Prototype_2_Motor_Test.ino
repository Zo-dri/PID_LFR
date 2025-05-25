#define Lin1   13
#define Lin2   12
#define LPWM   14
#define Rin1   2
#define Rin2   4
#define RPWM   15
#define STB   27

void setup() {
  Serial.begin(115200);
    // ledcAttach(Lpwm, 1000, 8);
  ledcAttach(PWM, 2000, 8);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  // pinMode(PWM, OUTPUT);
  pinMode(STB, OUTPUT);
  // put your setup code here, to run once:

}

void loop() {
  int maxSpeed = 255;
  Serial.println("STANDBY HIGH");
  digitalWrite(STB, HIGH);
  Serial.println("LPWM 0-255");
  for(int i = 0; i < maxSpeed; i++){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);

    // digitalWrite(PWM, HIGH);
    ledcWrite(PWM, i);
    // analogWrite(PWM, i);
    delay(5);
  }
  delay(5000);
  Serial.println("LPWM 255-0");
  for(int i = maxSpeed; i >= 0; i--){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);

    // digitalWrite(PWM, HIGH);
    ledcWrite(PWM, i);
    // analogWrite(PWM, i);
    delay(5);
  }
  delay(1000);
  Serial.println("RPWM 0-255");
  for(int i = 0; i < maxSpeed; i++){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);

    // digitalWrite(PWM, HIGH);
    ledcWrite(PWM, i);
    // analogWrite(PWM, i);
    delay(5);
  }
  delay(5000);
  Serial.println("rPWM 255-0");
  for(int i = maxSpeed; i >= 0; i--){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);

    // digitalWrite(PWM, HIGH);
    ledcWrite(PWM, i);
    // analogWrite(PWM, i);
    delay(5);
  }
  delay(1000);
}
