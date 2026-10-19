// Test: M4 runs forward at max speed for RUN_MS, then stops.
#include <Wire.h>
#include <Adafruit_MotorShield.h>

const uint8_t PORT = 4;                // motor port
const uint8_t SPEED = 255;             // 0-255 (max)
const unsigned long RUN_MS = 5000;     // run time

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor;

void setup() {
  Serial.begin(9600);
  if (!AFMS.begin()) {
    Serial.println("Motor shield not found");
    while (1);
  }
  motor = AFMS.getMotor(PORT);
  motor->setSpeed(SPEED);

  Serial.println("M4 FORWARD");
  motor->run(FORWARD);
  delay(RUN_MS);
  motor->run(RELEASE);
  Serial.println("M4 stopped");
}

void loop() {
}
