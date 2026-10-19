// Test: rotation motor M4 runs forward at max speed for ROT_MS, then stops.
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Port mapping (same as iris_gripper). M1 is blocked: never connect a motor to it.
const uint8_t PORT_BLOCKED = 1;
const uint8_t PORT_ROT = 4;
static_assert(PORT_ROT != PORT_BLOCKED, "Motor port M1 is blocked, do not use it");

// Speed (0-255) and timing
const uint8_t SPEED_ROT = 255;
const unsigned long ROT_MS = 5000;

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

  Serial.println("Rotation forward (M4)");
  rot->run(FORWARD);
  delay(ROT_MS);
  rot->run(RELEASE);
  Serial.println("Rotation stopped");
}

void loop() {
}
