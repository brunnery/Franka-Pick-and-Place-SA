// Test: rotation motor M4 runs FORWARD at SPEED_ROT for ROT_MS, then stops (once, after
// reset). Count the turns of the gripper to get turns per second for the screwing time
// (place.screw.duration_ms in tasks.yaml); note the direction seen from above.
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Port mapping (same as iris_gripper). M1 is blocked: never connect a motor to it.
const uint8_t PORT_BLOCKED = 1;
const uint8_t PORT_ROT = 4;
static_assert(PORT_ROT != PORT_BLOCKED, "Motor port M1 is blocked, do not use it");

// Speed (0-255) and timing. SPEED_ROT = place.screw.speed in tasks.yaml.
const uint8_t SPEED_ROT = 150;
const unsigned long ROT_MS = 500;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *rot;

void setup() {
  Serial.begin(9600);
  if (!AFMS.begin()) {
    Serial.println("Motor shield not found");
    while (1);
  }
  rot = AFMS.getMotor(PORT_ROT);
  rot->setSpeed(SPEED_ROT);

  Serial.print("Rotation forward (M4), speed ");
  Serial.print(SPEED_ROT);
  Serial.print(", ");
  Serial.print(ROT_MS);
  Serial.println(" ms");
  rot->run(FORWARD);
  delay(ROT_MS);
  rot->run(RELEASE);
  Serial.println("Rotation stopped");
}

void loop() {
}
