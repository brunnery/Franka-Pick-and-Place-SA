// Test: iris motors M2 and M3 run once "forward" (M2 FORWARD, M3 BACKWARD) for RUN_MS, then
// stop. Result 19.10.: this OPENS the iris (iris_gripper: CLOSE_IS_A_FORWARD = false).
// Press reset on the board to run it again.
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Port mapping (same as iris_gripper). M1 is blocked: never connect a motor to it.
const uint8_t PORT_BLOCKED = 1;
const uint8_t PORT_IRIS_A = 2;
const uint8_t PORT_IRIS_B = 3;
static_assert(PORT_IRIS_A != PORT_BLOCKED && PORT_IRIS_B != PORT_BLOCKED,
              "Motor port M1 is blocked, do not use it");

// Speed (0-255) and timings
const uint8_t SPEED_IRIS = 100;
const unsigned long RUN_MS = 1500;     // run time

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *irisA;
Adafruit_DCMotor *irisB;

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

  Serial.println("Iris open (M2 FORWARD, M3 BACKWARD)");
  irisA->run(BACKWARD);
  irisB->run(FORWARD);
  delay(RUN_MS);
  irisA->run(RELEASE);
  irisB->run(RELEASE);
  Serial.println("Stopped");
}

void loop() {}