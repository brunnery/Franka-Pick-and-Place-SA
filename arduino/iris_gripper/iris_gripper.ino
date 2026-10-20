// Iris gripper firmware: serial protocol of docs/PLAN.md Section 6, timing on the board
// (non-blocking, millis()). The PC sends durations and speeds; constants below are only the
// hardware mapping and the safety limits.
//
//   PC -> board:  <id> <CMD> [args]\n
//   board -> PC:  <id> ACK | <id> DONE [info] | <id> ERR <reason> | READY | WATCHDOG
//
//   OPEN <ms> <speed>               iris opens for ms, then off                 -> DONE
//   CLOSE <ms> <speed> <hold_speed> iris closes for ms -> DONE, then keeps closing with
//                                   hold_speed (holding) until OPEN, STOP or the watchdog
//   ROT <+1|-1> <ms> <speed>        M4 for ms, then off -> DONE; the iris keeps holding
//   STOP                            all motors off -> DONE (pending commands: ERR stopped)
//   PING                            -> DONE (resets the watchdog)
//   STATE                           -> DONE iris=<off|opening|closing|holding> rot=<on|off>
//
// Watchdog: no line from the PC for WATCHDOG_MS -> all motors off (the iris lets go; for now it
// then opens once, WATCHDOG_OPENS_IRIS). The PC sends PING well within that time while it holds.
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Port mapping. M1 is blocked: never connect a motor to it.
const uint8_t PORT_BLOCKED = 1;
const uint8_t PORT_IRIS_A = 2;
const uint8_t PORT_IRIS_B = 3;
const uint8_t PORT_ROT = 4;
static_assert(PORT_IRIS_A != PORT_BLOCKED && PORT_IRIS_B != PORT_BLOCKED && PORT_ROT != PORT_BLOCKED,
              "Motor port M1 is blocked, do not use it");

// The iris motors run in opposite directions. Tested 19.10. (test_m2_m3): A (M2) FORWARD +
// B (M3) BACKWARD OPENS the iris, so closing = A BACKWARD, B FORWARD.
const bool CLOSE_IS_A_FORWARD = false;

const unsigned long BAUD = 115200;
const unsigned long WATCHDOG_MS = 2000;
const unsigned long MAX_DURATION_MS = 60000;
const uint8_t LINE_MAX = 64;

// TEMPORARY (20.10., testing convenience): when the watchdog fires while the iris is closed
// or holding (e.g. 2 s after the pick process ends), open it once with these values instead
// of only switching off, so it need not be opened by hand. = gripper.open in tasks.yaml.
const bool WATCHDOG_OPENS_IRIS = true;
const unsigned long WATCHDOG_OPEN_MS = 1500;
const uint8_t WATCHDOG_OPEN_SPEED = 100;

enum IrisState { IRIS_OFF, IRIS_OPENING, IRIS_CLOSING, IRIS_HOLDING };

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *irisA;
Adafruit_DCMotor *irisB;
Adafruit_DCMotor *rot;

IrisState iris = IRIS_OFF;
unsigned long irisStart = 0, irisDuration = 0;
uint8_t holdSpeed = 0;
long irisId = -1;  // command waiting for DONE, -1 = none
bool watchdogOpening = false;  // opening started by the watchdog, not by the PC

bool rotOn = false;
unsigned long rotStart = 0, rotDuration = 0;
long rotId = -1;

unsigned long lastRx = 0;
char line[LINE_MAX + 1];
uint8_t lineLen = 0;

void reply(long id, const char *text) {
  Serial.print(id);
  Serial.print(' ');
  Serial.println(text);
}

void finishPending(long &id, const char *text) {
  if (id >= 0) reply(id, text);
  id = -1;
}

void irisRun(bool close, uint8_t speed) {
  bool aForward = (close == CLOSE_IS_A_FORWARD);
  irisA->setSpeed(speed);
  irisB->setSpeed(speed);
  irisA->run(aForward ? FORWARD : BACKWARD);
  irisB->run(aForward ? BACKWARD : FORWARD);
}

void irisOff() {
  irisA->run(RELEASE);
  irisB->run(RELEASE);
  iris = IRIS_OFF;
  watchdogOpening = false;
}

void rotOff() {
  rot->run(RELEASE);
  rotOn = false;
}

void stopAll(const char *pendingText) {
  irisOff();
  rotOff();
  finishPending(irisId, pendingText);
  finishPending(rotId, pendingText);
}

const char *irisName() {
  switch (iris) {
    case IRIS_OPENING: return "opening";
    case IRIS_CLOSING: return "closing";
    case IRIS_HOLDING: return "holding";
    default: return "off";
  }
}

bool parseRange(const char *token, long lo, long hi, long &out) {
  if (token == NULL) return false;
  char *end;
  out = strtol(token, &end, 10);
  return *end == '\0' && out >= lo && out <= hi;
}

void handleLine(char *text) {
  char *idToken = strtok(text, " ");
  char *cmd = strtok(NULL, " ");
  long id;
  if (!parseRange(idToken, 0, 2147483647L, id) || cmd == NULL) {
    Serial.println("-1 ERR syntax");
    return;
  }
  char *a1 = strtok(NULL, " ");
  char *a2 = strtok(NULL, " ");
  char *a3 = strtok(NULL, " ");
  long v1, v2, v3;
  unsigned long now = millis();

  if (strcmp(cmd, "PING") == 0) {
    reply(id, "DONE");
  } else if (strcmp(cmd, "STATE") == 0) {
    Serial.print(id);
    Serial.print(" DONE iris=");
    Serial.print(irisName());
    Serial.println(rotOn ? " rot=on" : " rot=off");
  } else if (strcmp(cmd, "STOP") == 0) {
    stopAll("ERR stopped");
    reply(id, "DONE");
  } else if (strcmp(cmd, "OPEN") == 0 || strcmp(cmd, "CLOSE") == 0) {
    bool close = cmd[0] == 'C';
    if (!parseRange(a1, 1, MAX_DURATION_MS, v1) || !parseRange(a2, 0, 255, v2) ||
        (close && !parseRange(a3, 0, 255, v3))) {
      reply(id, "ERR args");
      return;
    }
    finishPending(irisId, "ERR replaced");
    reply(id, "ACK");
    watchdogOpening = false;
    irisRun(close, (uint8_t)v2);
    iris = close ? IRIS_CLOSING : IRIS_OPENING;
    holdSpeed = close ? (uint8_t)v3 : 0;
    irisStart = now;
    irisDuration = (unsigned long)v1;
    irisId = id;
  } else if (strcmp(cmd, "ROT") == 0) {
    if (!parseRange(a1, -1, 1, v1) || v1 == 0 || !parseRange(a2, 1, MAX_DURATION_MS, v2) ||
        !parseRange(a3, 0, 255, v3)) {
      reply(id, "ERR args");
      return;
    }
    finishPending(rotId, "ERR replaced");
    reply(id, "ACK");
    rot->setSpeed((uint8_t)v3);
    rot->run(v1 > 0 ? FORWARD : BACKWARD);
    rotOn = true;
    rotStart = now;
    rotDuration = (unsigned long)v2;
    rotId = id;
  } else {
    reply(id, "ERR unknown");
  }
}

void readSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[lineLen] = '\0';
      lastRx = millis();
      if (lineLen > 0) handleLine(line);
      lineLen = 0;
    } else if (lineLen < LINE_MAX) {
      line[lineLen++] = c;
    } else {
      lineLen = 0;  // too long: drop the line
      Serial.println("-1 ERR too long");
    }
  }
}

void updateTimers() {
  unsigned long now = millis();
  if ((iris == IRIS_OPENING || iris == IRIS_CLOSING) && now - irisStart >= irisDuration) {
    if (iris == IRIS_CLOSING) {
      irisRun(true, holdSpeed);  // keep pressing: holding
      iris = IRIS_HOLDING;
    } else {
      irisOff();
    }
    finishPending(irisId, "DONE");
  }
  if (rotOn && now - rotStart >= rotDuration) {
    rotOff();
    finishPending(rotId, "DONE");
  }
  if ((iris != IRIS_OFF || rotOn) && !watchdogOpening && now - lastRx > WATCHDOG_MS) {
    bool wasClosed = iris == IRIS_CLOSING || iris == IRIS_HOLDING;
    stopAll("ERR watchdog");
    Serial.println("WATCHDOG");
    if (WATCHDOG_OPENS_IRIS && wasClosed) {
      irisRun(false, WATCHDOG_OPEN_SPEED);  // ends in the opening branch above -> off
      iris = IRIS_OPENING;
      irisStart = now;
      irisDuration = WATCHDOG_OPEN_MS;
      watchdogOpening = true;
    }
  }
}

void setup() {
  Serial.begin(BAUD);
  if (!AFMS.begin()) {
    Serial.println("-1 ERR motor shield not found");
    while (1);
  }
  irisA = AFMS.getMotor(PORT_IRIS_A);
  irisB = AFMS.getMotor(PORT_IRIS_B);
  rot = AFMS.getMotor(PORT_ROT);
  irisOff();
  rotOff();
  lastRx = millis();
  Serial.println("READY");
}

void loop() {
  readSerial();
  updateTimers();
}
