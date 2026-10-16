#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Motorshield mit Standardadresse 0x60
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor3 = AFMS.getMotor(3);
Adafruit_DCMotor *motor4 = AFMS.getMotor(4);

const uint8_t SPEED = 80;               // 0-255, langsam fuer den ersten Test
const unsigned long DAUER = 3000;       // M1/M4 Laufzeit pro Richtung in ms
const unsigned long PAUSE = 250;        // Pause zwischen Richtungswechseln in ms
const uint8_t ZYKLEN_M1M4 = 2;          // so oft hin und her vor M3
const unsigned long DAUER_M3 = 8000;    // M3 Laufzeit in ms

void setup() {
  Serial.begin(9600);

  if (!AFMS.begin()) {
    Serial.println("Motorshield nicht gefunden");
    while (1);
  }

  motor1->setSpeed(SPEED);
  motor3->setSpeed(SPEED);
  motor4->setSpeed(SPEED);
}

void stopp() {
  // kurz anhalten, schont Motoren und Getriebe
  motor1->run(RELEASE);
  motor3->run(RELEASE);
  motor4->run(RELEASE);
  delay(PAUSE);
}

// Ein Zyklus M1/M4 gegenlaeufig: 3 s in eine Richtung, 3 s zurueck
void zyklusM1M4() {
  motor1->run(BACKWARD);
  motor4->run(FORWARD);
  delay(DAUER);
  stopp();

  motor1->run(FORWARD);
  motor4->run(BACKWARD);
  delay(DAUER);
  stopp();
}

void loop() {
  for (uint8_t i = 0; i < ZYKLEN_M1M4; i++) {
    zyklusM1M4();
  }

  // danach M3 fuer 8 Sekunden
  motor3->run(FORWARD);
  delay(DAUER_M3);
  stopp();
}
