// Test: iris motors M2 and M3 run in opposite directions for RUN_MS, pause, then reverse.
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Port mapping (same as iris_gripper). M1 is blocked: never connect a motor to it.
const uint8_t PORT_BLOCKED = 1;
const uint8_t PORT_IRIS_A = 2;
const uint8_t PORT_IRIS_B = 3;
static_assert(PORT_IRIS_A != PORT_BLOCKED && PORT_IRIS_B != PORT_BLOCKED,
              "Motor port M1 is blocked, do not use it");

// Speed (0-255) and timings
const uint8_t SPEED_IRIS = 200;
const unsigned long RUN_MS = 3000;     // run time per direction
const unsigned long PAUSE_MS = 1000;   // pause between runs

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *irisA;
Adafruit_DCMotor *irisB;

void runIris(bool forward) {
  irisA->run(forward ? FORWARD : BACKWARD);
  irisB->run(forward ? BACKWARD : FORWARD);
  delay(RUN_MS);
  irisA->run(RELEASE);
  irisB->run(RELEASE);
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
  irisA->setSpeed(SPEED_IRIS);
  irisB->setSpeed(SPEED_IRIS);
}

void loop() {
  Serial.println("Iris forward (M2 FORWARD, M3 BACKWARD)");
  runIris(true);
  Serial.println("Iris backward (M2 BACKWARD, M3 FORWARD)");
  runIris(false);
}
