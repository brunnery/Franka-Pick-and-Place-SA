#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Motorshield mit Standardadresse 0x60
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
Adafruit_DCMotor *motor3 = AFMS.getMotor(3);
Adafruit_DCMotor *motor4 = AFMS.getMotor(4);

const uint8_t SPEED    = 180;   // 0-255, Speed fuer M3 und M2
const uint8_t SPEED_M4 = 255;   // Speed fuer M4

const uint8_t ZYKLEN_VOR_M4 = 4;      // nach so vielen M3/M2-Zyklen dreht M4
const unsigned long M4_DAUER = 5000;  // M4 Laufzeit in ms

void setup() {
  Serial.begin(9600);

  if (!AFMS.begin()) {
    Serial.println("Motorshield nicht gefunden");
    while (1);
  }

  motor2->setSpeed(SPEED);
  motor3->setSpeed(SPEED);
  motor4->setSpeed(SPEED_M4);
}

// Ein Zyklus M3/M2: 2 s vorwaerts, Pause, 2 s rueckwaerts, Pause
void zyklusM3M2() {
  // 2 Sekunden vorwaerts
  motor3->run(BACKWARD);
  motor2->run(FORWARD);
  delay(2000);

  // kurz anhalten, schont Motoren und Getriebe
  motor3->run(RELEASE);
  motor2->run(RELEASE);
  delay(250);

  // 2 Sekunden rueckwaerts
  motor3->run(FORWARD);
  motor2->run(BACKWARD);
  delay(2000);

  motor3->run(RELEASE);
  motor2->run(RELEASE);
  delay(250);
}

void loop() {
  // 4 Zyklen M3/M2
  for (uint8_t i = 0; i < ZYKLEN_VOR_M4; i++) {
    zyklusM3M2();
  }

  // danach M4 fuer 5 Sekunden
  motor4->run(FORWARD);
  delay(M4_DAUER);
  motor4->run(RELEASE);
  delay(250);
}