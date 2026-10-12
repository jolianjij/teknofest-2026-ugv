// Standalone bring-up test for the differential pan/tilt stepper gimbal
// (2x NEMA17 + TMC2208 v1.2, STEP/DIR standalone mode). NOT part of
// esp32_firmware.ino and does not talk to the Pi - this is purely for
// jogging the motors by hand over the Arduino Serial Monitor.
//
// Mixing model CONFIRMED on real hardware (2026-08-26) - not a symmetric
// differential: tilt = motorA - motorB, pan = motorB (independent of
// motorA). Motor A alone is pure tilt; A==B (same direction) is pure
// pan. See panTiltMove() below for the derivation/inverse.
//
// ============================================================
// PIN CONFLICT FIXED HERE - re-wire before flashing if you haven't:
// You said both TMC2208 EN pins go to G4. G4 is already the PCA9685
// !OE line in esp32_firmware/PINOUT.md - the hardware kill switch that
// cuts the DRIVE motors when disarmed/failsafe. Sharing it with the
// gimbal's enable means you can't kill one without the other. This
// sketch uses G14 (confirmed free) instead.
//
// Lower-severity flag: PAN_DIR=G17 is soft-reserved as SBUS_TX (UART2)
// in the main firmware. Nothing is physically wired there today so this
// standalone test is fine, but that pin can't be reused once this gets
// merged alongside SBUS.
// ============================================================
#define EN_PIN     14   // shared enable, both TMC2208 EN tied together - moved from your stated G4
#define PAN_DIR    17
#define PAN_STEP   5
#define TILT_DIR   19
#define TILT_STEP  18

// Most TMC2208 breakouts: EN is active LOW (LOW = driver enabled).
// Flip this if your motors do the opposite of what you expect.
constexpr bool EN_ACTIVE_LOW = true;

// Standalone mode sets microstepping in HARDWARE via the MS1/MS2 pins
// on the TMC2208 board - this code can't change that. For 32 microsteps:
// MS1 = HIGH, MS2 = LOW. This constant only feeds this sketch's own
// step-count math (nothing depends on it for basic jogging to work).
constexpr int MICROSTEPS = 32;
constexpr int FULL_STEPS_PER_REV = 200;  // NEMA17
constexpr long STEPS_PER_REV = (long)FULL_STEPS_PER_REV * MICROSTEPS;

constexpr int MIN_PULSE_US = 5;  // TMC2208 needs ~100ns min - huge margin

long stepDelayUs = 800;  // time between step edges; smaller = faster, tune with 's' command
long posA = 0, posB = 0; // soft step counters - open loop, no encoders, just for tracking during this test

char rxBuf[64];
int rxLen = 0;

void driversEnable(bool on) {
  bool activeLevel = EN_ACTIVE_LOW ? LOW : HIGH;
  bool idleLevel   = EN_ACTIVE_LOW ? HIGH : LOW;
  digitalWrite(EN_PIN, on ? activeLevel : idleLevel);
  Serial.println(on ? "drivers ENABLED" : "drivers disabled");
}

void stepOnly(int stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(MIN_PULSE_US);
  digitalWrite(stepPin, LOW);
}

// Drives motor A and motor B a given (signed) raw step count each,
// pulsed together via simple Bresenham interleaving so both axes move
// simultaneously through the differential rather than sequentially
// (sequential would twist one way then the other instead of a clean
// combined motion). Not acceleration-profiled - fine for a bring-up
// test, revisit with AccelStepper's MultiStepper if motion needs to be
// smoother once this is real firmware.
void rawMove(long stepsA, long stepsB) {
  digitalWrite(PAN_DIR,  stepsA >= 0 ? HIGH : LOW);
  digitalWrite(TILT_DIR, stepsB >= 0 ? HIGH : LOW);
  long remA = abs(stepsA);
  long remB = abs(stepsB);
  long total = max(remA, remB);
  long accA = 0, accB = 0;
  for (long i = 0; i < total; i++) {
    accA += remA;
    accB += remB;
    if (accA >= total) { accA -= total; stepOnly(PAN_STEP); }
    if (accB >= total) { accB -= total; stepOnly(TILT_STEP); }
    delayMicroseconds(stepDelayUs);
  }
  posA += stepsA;
  posB += stepsB;
  Serial.print("posA="); Serial.print(posA);
  Serial.print(" posB="); Serial.println(posB);
}

// ============================================================
// CONFIRMED on real hardware (2026-08-26), not a symmetric differential:
//   tilt = motorA - motorB
//   pan  = motorB               (independent of motorA)
// i.e. motor A alone = pure tilt, A==B (same direction) = pure pan.
// "Opposite directions" is NOT pure tilt - it moves both axes. Inverting
// for a desired (pan, tilt) target:
//   motorA = pan + tilt
//   motorB = pan
// ============================================================
void panTiltMove(long panSteps, long tiltSteps) {
  rawMove(panSteps + tiltSteps, panSteps);
}
void panMove(long n)  { panTiltMove(n, 0); }
void tiltMove(long n) { panTiltMove(0, n); }

void printHelp() {
  Serial.println("--- differential gimbal test ---");
  Serial.println("e              enable drivers");
  Serial.println("x              disable drivers");
  Serial.println("a <steps>      raw diagnostic: motor A alone (no mixing) - this IS pure tilt");
  Serial.println("b <steps>      raw diagnostic: motor B alone (no mixing) - moves BOTH pan+tilt");
  Serial.println("p <steps>      pure pan move (mixed: motorA=n, motorB=n)");
  Serial.println("t <steps>      pure tilt move (mixed: motorA=n, motorB=0) - same as 'a', kept for symmetry");
  Serial.println("m <pan> <tilt> combined move using the confirmed mixing formula");
  Serial.println("s <us>         set per-step delay in microseconds (speed), currently ");
  Serial.print("               "); Serial.println(stepDelayUs);
  Serial.println("z              zero the soft position counters");
  Serial.println("h              this help");
  Serial.println("");
  Serial.println("Confirmed model: tilt = motorA - motorB, pan = motorB (see comment above");
  Serial.println("panTiltMove()). Use p/t/m for real moves - a/b are raw diagnostics only.");
}

void handleLine(const char *line) {
  char cmd;
  long arg1 = 0, arg2 = 0;
  int n = sscanf(line, "%c %ld %ld", &cmd, &arg1, &arg2);
  if (n < 1) return;

  switch (cmd) {
    case 'e': driversEnable(true); break;
    case 'x': driversEnable(false); break;
    case 'a': if (n >= 2) rawMove(arg1, 0); else Serial.println("usage: a <steps>"); break;
    case 'b': if (n >= 2) rawMove(0, arg1); else Serial.println("usage: b <steps>"); break;
    case 'p': if (n >= 2) panMove(arg1); else Serial.println("usage: p <steps>"); break;
    case 't': if (n >= 2) tiltMove(arg1); else Serial.println("usage: t <steps>"); break;
    case 'm': if (n == 3) panTiltMove(arg1, arg2); else Serial.println("usage: m <pan_steps> <tilt_steps>"); break;
    case 's':
      if (n >= 2 && arg1 > 0) { stepDelayUs = arg1; Serial.print("stepDelayUs="); Serial.println(stepDelayUs); }
      else Serial.println("usage: s <microseconds>");
      break;
    case 'z': posA = 0; posB = 0; Serial.println("position counters zeroed"); break;
    case 'h': printHelp(); break;
    default: Serial.println("unknown command, 'h' for help");
  }
}

void pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      rxBuf[rxLen] = '\0';
      if (rxLen > 0) handleLine(rxBuf);
      rxLen = 0;
    } else if (c != '\r' && rxLen < (int)sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(EN_PIN, OUTPUT);
  pinMode(PAN_DIR, OUTPUT);
  pinMode(PAN_STEP, OUTPUT);
  pinMode(TILT_DIR, OUTPUT);
  pinMode(TILT_STEP, OUTPUT);
  driversEnable(false);  // start disabled - explicit 'e' required before anything moves

  Serial.println("Differential gimbal stepper test ready.");
  printHelp();
}

void loop() {
  pollSerial();
}
