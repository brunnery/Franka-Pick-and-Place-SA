// Test: M1 and M3 run in opposite directions for RUN_MS, pause, then reverse.
#include <Wire.h>
#include <Adafruit_MotorShield.h>

const uint8_t PORT_A = 1;              // motor port A
const uint8_t PORT_B = 3;              // motor port B (opposite direction to A)
const uint8_t SPEED = 200;             // 0-255
const unsigned long RUN_MS = 3000;     // run time per direction
const unsigned long PAUSE_MS = 1000;   // pause between runs

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorA;
Adafruit_DCMotor *motorB;

void runPair(uint8_t dirA, uint8_t dirB) {
  motorA->run(dirA);
  motorB->run(dirB);
  delay(RUN_MS);
  motorA->run(RELEASE);
  motorB->run(RELEASE);
  delay(PAUSE_MS);
}

void setup() {
  Serial.begin(9600);
  if (!AFMS.begin()) {
    Serial.println("Motor shield not found");
    while (1);
  }
  motorA = AFMS.getMotor(PORT_A);
  motorB = AFMS.getMotor(PORT_B);
  motorA->setSpeed(SPEED);
  motorB->setSpeed(SPEED);
}

void loop() {
  Serial.println("M1 FORWARD, M3 BACKWARD");
  runPair(FORWARD, BACKWARD);
  Serial.println("M1 BACKWARD, M3 FORWARD");
  runPair(BACKWARD, FORWARD);
}
