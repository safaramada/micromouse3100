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

// ----- Constants -----
const int DRIVE_SPEED = 150;       
const int TURN_SPEED = 140;    




// ----- Function -----
void setupMotors();
void setMotorSpeeds(int leftSpeed, int rightSpeed);


void driveForwardMM(int distanceMM);
void turnLeft90();
void turnRight90();
void stopMotors();

void setup() {
  Serial.begin(9600);
  setupMotors();

  delay(1000); // Give time to place robot after powering on.

  // Task 1: drive 200 mm, then stop.
  driveForwardMM(200);
  stopMotors();

  delay(1000);

  // Task 2a: four 90 degree counter-clockwise turns.
  for (int i = 0; i < 4; i++) {
    turnLeft90();
    stopMotors();
    delay(300);
  }

  delay(1000);

  // Task 2b: four 90 degree clockwise turns.
  for (int i = 0; i < 4; i++) {
    turnRight90();
    stopMotors();
    delay(300);
  }

  stopMotors();
}















void loop() {
  // Week 4 task only runs once in setup().
}

void setupMotors() {
 
  stopMotors();
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {

 
}

void driveForwardMM(int distanceMM) {



  int driveTimeMS = distanceMM * MS_PER_MM;

  setMotorSpeeds(DRIVE_SPEED, DRIVE_SPEED);
  delay(driveTimeMS);
  stopMotors();
}

void turnLeft90() {



  setMotorSpeeds(-TURN_SPEED, TURN_SPEED);
  delay(100); // Replace with constant after testing.
  stopMotors();
}

void turnRight90() {



  setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
  delay(100); // Replace with constant after testing.
  stopMotors();
}

void stopMotors() {



  setMotorSpeeds(0, 0);
}
