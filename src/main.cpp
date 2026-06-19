#include <Arduino.h>

const int MOT1_DIR = 10;
const int MOT1_PWM = 9;

const int MOT2_DIR = 12;
const int MOT2_PWM = 11;

void setup() {
  Serial.begin(9600);

  pinMode(MOT1_DIR, OUTPUT);
  pinMode(MOT1_PWM, OUTPUT);
  pinMode(MOT2_DIR, OUTPUT);
  pinMode(MOT2_PWM, OUTPUT);

  Serial.println("Motor repeat test started");
}

void loop() {
  Serial.println("Motors forward");

  digitalWrite(MOT1_DIR, LOW);
  digitalWrite(MOT2_DIR, LOW);

  analogWrite(MOT1_PWM, 255);
  analogWrite(MOT2_PWM, 255);

  delay(3000);

  Serial.println("Motors off");

  analogWrite(MOT1_PWM, 0);
  analogWrite(MOT2_PWM, 0);

  delay(2000);
}
