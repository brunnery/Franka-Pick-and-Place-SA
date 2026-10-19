// Main gripper sequence (loop): iris motors forward, rotation motor, iris motors backward.
// The iris motors run in opposite directions ("forward" = A FORWARD, B BACKWARD).
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Port mapping
const uint8_t PORT_IRIS_A = 1;
const uint8_t PORT_IRIS_B = 3;
const uint8_t PORT_ROT = 4;

// Speeds (0-255) and timings
const uint8_t SPEED_IRIS = 200;
const uint8_t SPEED_ROT = 255;
const unsigned long IRIS_MS = 3000;
const unsigned long ROT_MS = 5000;
const unsigned long PAUSE_MS = 1000;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *irisA;
Adafruit_DCMotor *irisB;
Adafruit_DCMotor *rot;

void stopAll() {
  irisA->run(RELEASE);
  irisB->run(RELEASE);
  rot->run(RELEASE);
}

void runIris(bool forward) {
  irisA->run(forward ? FORWARD : BACKWARD);
  irisB->run(forward ? BACKWARD : FORWARD);
  delay(IRIS_MS);
  stopAll();
  delay(PAUSE_MS);
}

void runRot() {
  rot->run(FORWARD);
  delay(ROT_MS);
  stopAll();
  delay(PAUSE_MS);
}

void setup() {
  Serial.begin(9600);
  if (!AFMS.begin()) {
    Serial.println("Motor shield not found");
    while (1);
  }
  irisA = AFMS.getMotor(PORT_IRIS_A);
  irisB = AFMS.getMotor(PORT_IRIS_B);
  rot = AFMS.getMotor(PORT_ROT);
  irisA->setSpeed(SPEED_IRIS);
  irisB->setSpeed(SPEED_IRIS);
  rot->setSpeed(SPEED_ROT);
}

void loop() {
  Serial.println("Iris forward");
  runIris(true);
  Serial.println("Rotation");
  runRot();
  Serial.println("Iris backward");
  runIris(false);
}
