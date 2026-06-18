#include <Arduino.h>
// #include <Wire.h>
// #include <VL6180X.h>

/*
    Key Responsibilities:       
        - Setup sensors / motors
        - call task functions
        - task functions use motor/sensor helper 


    // Week 4 Task 
        Drive forward 200 mm accurately (+- 75mm)
        Rotate in place - 4 90 degree turns counter clockwise 
        4 90 degree turns clockwise 
        After each of these iterations it should be generally facing original direction
        
*/

// Functions 
void driveForwardMM(int distanceMM);
void turnLeft90();
void turnRight90();
void stopMotors();


void setup() {
  // Set up pins, sensors, motors
  Serial.begin(9600);

  delay(1000);

  // Week 4 barebones movement:
  driveForwardMM(200);

  delay(1000);

  for (int i = 0; i < 4; i++) {
    turnLeft90();
    delay(300);
  }

  for (int i = 0; i < 4; i++) {
    turnRight90();
    delay(300);
  }

  stopMotors();
}

void loop() {
  // Empty for now
}
