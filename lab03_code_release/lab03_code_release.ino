#include <Wire.h>
#include <VL6180X.h>

#define FRONT_XSHUT A2 // TOF1GPIO
#define LEFT_XSHUT A1  // TOF2GPIO
#define RIGHT_XSHUT A0 // TOF3GPIO

#define FRONT_ADDRESS 0x30
#define LEFT_ADDRESS 0x31
#define RIGHT_ADDRESS 0x32

VL6180X frontSensor;
VL6180X leftSensor;
VL6180X rightSensor;

bool frontSensorReady = false;
bool leftSensorReady = false;
bool rightSensorReady = false;

void printI2CBusState(const char* label) {
  Serial.print(label);
  Serial.print(" SDA=");
  Serial.print(digitalRead(A4) == HIGH ? "HIGH" : "LOW");
  Serial.print(" SCL=");
  Serial.println(digitalRead(A5) == HIGH ? "HIGH" : "LOW");
}

byte i2cProbe(uint8_t address) {
  Serial.print("Probing address 0x");
  if (address < 16) {
    Serial.print("0");
  }
  Serial.println(address, HEX);

  Wire.beginTransmission(address);
  byte error = Wire.endTransmission();

  Serial.print("I2C response code: ");
  Serial.println(error);

  return error;
}

bool i2cDevicePresent(uint8_t address) {
  return i2cProbe(address) == 0;
}

bool setupSensor(VL6180X& sensor, uint8_t xshutPin, uint8_t newAddress, const char* name) {
  Serial.print("Turning on ");
  Serial.println(name);

  digitalWrite(xshutPin, HIGH);
  delay(100);
  printI2CBusState("After turning sensor on:");

  Serial.print("Checking for ");
  Serial.print(name);
  Serial.println(" at default address 0x29...");

  if (!i2cDevicePresent(0x29)) {
    Serial.print(name);
    Serial.println(" not found. Check wiring, power, SDA/SCL, and XSHUT pin.");
    return false;
  }

  Serial.print("Initialising ");
  Serial.println(name);

  sensor.init();
  sensor.configureDefault();
  sensor.setTimeout(500);
  sensor.setAddress(newAddress);

  Serial.print(name);
  Serial.print(" started at address 0x");
  Serial.println(newAddress, HEX);

  return true;
}

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
  delay(10);
  Serial.println("Three VL6180X LiDAR test starting");
  Serial.println("Nano I2C pins: SDA = A4, SCL = A5");
  printI2CBusState("Before Wire.begin:");

  pinMode(FRONT_XSHUT, OUTPUT);
  pinMode(LEFT_XSHUT, OUTPUT);
  pinMode(RIGHT_XSHUT, OUTPUT);

  digitalWrite(FRONT_XSHUT, LOW);
  digitalWrite(LEFT_XSHUT, LOW);
  digitalWrite(RIGHT_XSHUT, LOW);

  Wire.begin();
  Wire.setClock(100000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000, true);
#endif
  delay(100);

  Serial.println("XSHUT pins used: front=A2, left=A1, right=A0");
  printI2CBusState("After Wire.begin with sensors off:");

  frontSensorReady = setupSensor(frontSensor, FRONT_XSHUT, FRONT_ADDRESS, "Front sensor");
  leftSensorReady = setupSensor(leftSensor, LEFT_XSHUT, LEFT_ADDRESS, "Left sensor");
  rightSensorReady = setupSensor(rightSensor, RIGHT_XSHUT, RIGHT_ADDRESS, "Right sensor");

  if (!frontSensorReady || !leftSensorReady || !rightSensorReady) {
    Serial.println("One or more sensors did not start.");
    Serial.println("If it stops at the first sensor, A2 may not be connected to that sensor's XSHUT/SHDN pin.");
  }

  Serial.println("Reading distances in mm...");
}

void printRange(const char* name, VL6180X& sensor) {
  uint8_t range = sensor.readRangeSingleMillimeters();

  Serial.print(name);
  Serial.print(": ");

  if (sensor.timeoutOccurred()) {
    Serial.print("timeout");
  } else {
    Serial.print(range);
    Serial.print(" mm");
  }

  Serial.print("  ");
}

const uint8_t DETECTION_DISTANCE = 100; // millimetres

void blinkCount(uint8_t numberOfBlinks) {
  for (uint8_t i = 0; i < numberOfBlinks; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);

    digitalWrite(LED_BUILTIN, LOW);
    delay(150);
  }

  delay(500);
}

void loop() {
  // Rapid blinking means a sensor failed during startup.
  if (!frontSensorReady || !leftSensorReady || !rightSensorReady) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);

    digitalWrite(LED_BUILTIN, LOW);
    delay(100);

    return;
  }

  uint8_t frontDistance = frontSensor.readRangeSingleMillimeters();
  bool frontValid = !frontSensor.timeoutOccurred();

  uint8_t leftDistance = leftSensor.readRangeSingleMillimeters();
  bool leftValid = !leftSensor.timeoutOccurred();

  uint8_t rightDistance = rightSensor.readRangeSingleMillimeters();
  bool rightValid = !rightSensor.timeoutOccurred();

  // Put your hand close to only one sensor at a time.
  if (frontValid && frontDistance < DETECTION_DISTANCE) {
    blinkCount(1); // Front sensor: one blink
  } else if (leftValid && leftDistance < DETECTION_DISTANCE) {
    blinkCount(2); // Left sensor: two blinks
  } else if (rightValid && rightDistance < DETECTION_DISTANCE) {
    blinkCount(3); // Right sensor: three blinks
  } else {
    // No object closer than 100 mm.
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
}
