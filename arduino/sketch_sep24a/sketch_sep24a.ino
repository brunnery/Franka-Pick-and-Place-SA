// Test sequence (loop): iris motors back and forth IRIS_CYCLES times, then rotation motor.
// The iris motors run in opposite directions ("forward" = A FORWARD, B BACKWARD).
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Port mapping (same as iris_gripper). M1 is blocked: never connect a motor to it.
const uint8_t PORT_BLOCKED = 1;
const uint8_t PORT_IRIS_A = 2;
const uint8_t PORT_IRIS_B = 3;
const uint8_t PORT_ROT = 4;
static_assert(PORT_IRIS_A != PORT_BLOCKED && PORT_IRIS_B != PORT_BLOCKED && PORT_ROT != PORT_BLOCKED,
              "Motor port M1 is blocked, do not use it");

// Speeds (0-255) and timings
const uint8_t SPEED_IRIS = 100;
const uint8_t SPEED_ROT = 255;
const unsigned long IRIS_MS = 2000;    // iris run time per direction
const unsigned long ROT_MS = 8000;
const unsigned long PAUSE_MS = 250;    // short stop between direction changes, spares the gears
const uint8_t IRIS_CYCLES = 2;         // iris back and forth this often before the rotation

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *irisA;
Adafruit_DCMotor *irisB;
Adafruit_DCMotor *rot;

void stopAll() {
  irisA->run(RELEASE);
  irisB->run(RELEASE);
  rot->run(RELEASE);
  delay(PAUSE_MS);
}

void runIris(bool forward) {
  irisA->run(forward ? FORWARD : BACKWARD);
  irisB->run(forward ? BACKWARD : FORWARD);
  delay(IRIS_MS);
  stopAll();
}

void runRot() {
  rot->run(FORWARD);
  delay(ROT_MS);
  stopAll();
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
  for (uint8_t i = 0; i < IRIS_CYCLES; i++) {
    Serial.println("Iris backward");
    runIris(false);
    Serial.println("Iris forward");
    runIris(true);
  }
  Serial.println("Rotation");
  runRot();
}
