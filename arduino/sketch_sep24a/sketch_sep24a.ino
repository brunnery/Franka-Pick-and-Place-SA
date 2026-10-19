#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Motorshield mit Standardadresse 0x60
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// M1 ist gesperrt: dort nie einen Motor anschliessen
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
Adafruit_DCMotor *motor3 = AFMS.getMotor(3);
Adafruit_DCMotor *motor4 = AFMS.getMotor(4);

const uint8_t SPEED = 180;               // 0-255, langsam fuer den ersten Test
const unsigned long DAUER = 3000;       // M2/M3 Laufzeit proRichtung in ms
const unsigned long PAUSE = 250;        // Pause zwischen Richtungswechseln in ms
const uint8_t ZYKLEN_M2M3 = 2;          // so oft hin und her vor M4
const unsigned long DAUER_M4 = 8000;    // M4 Laufzeit in ms

void setup() {
  Serial.begin(9600);

  if (!AFMS.begin()) {
    Serial.println("Motorshield nicht gefunden");
    while (1);
  }

  motor2->setSpeed(SPEED);
  motor4->setSpeed(SPEED);
  motor3->setSpeed(SPEED);
}

void stopp() {
  // kurz anhalten, schont Motoren und Getriebe
  motor2->run(RELEASE);
  motor4->run(RELEASE);
  motor3->run(RELEASE);
  delay(PAUSE);
}

// Ein Zyklus M2/M3 gegenlaeufig: 3 s in eine Richtung, 3 s zurueck
void zyklusM2M3() {
  motor2->run(BACKWARD);
  motor3->run(FORWARD);
  delay(DAUER);
  stopp();

  motor2->run(FORWARD);
  motor3->run(BACKWARD);
  delay(DAUER);
  stopp();
}

void loop() {
  for (uint8_t i = 0; i < ZYKLEN_M2M3; i++) {
    zyklusM2M3();
  }

  // danach M4 fuer 8 Sekunden
  motor4->run(FORWARD);
  delay(DAUER_M4);
  stopp();
}
