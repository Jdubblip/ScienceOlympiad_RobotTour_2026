// =====================================
// PICO 2W + TMC2209 — STEPPER WITH FIXED SPEEDS + S-CURVE RAMPS + ENDSCRIPT
// INDEPENDENT WHEEL DIAMETERS + BRESENHAM SYNC
// MT6701 ENCODER CLOSED-LOOP CORRECTION
// =====================================

// LEFT DRIVER
#define L_STEP 7
#define L_DIR 8
#define L_MS1 10
#define L_MS2 11

#include <Wire.h>

// RIGHT DRIVER
#define R_STEP 6
#define R_DIR 5
#define R_MS1 2
#define R_MS2 3

// BUTTONS
#define GO_BUTTON 28
#define STOP_BUTTON 22

// =====================================
// MT6701 ENCODER CONFIG
// =====================================
// Left encoder:  I2C0 (Wire)  — GP20 SDA, GP21 SCL
// Right encoder: I2C1 (Wire1) — GP26 SDA, GP27 SCL
#define ENC_LEFT_SDA  20
#define ENC_LEFT_SCL  21
#define ENC_RIGHT_SDA 26
#define ENC_RIGHT_SCL 27

#define MT6701_ADDR       0x06
#define MT6701_ANGLE_REG  0x03

// =====================================
// ROBOT CONSTANTS
// =====================================
const float WHEELBASE_CM = 14.525f;
const float ARC_WHEELBASE_CM = 14.61f;
const float AimPt_CenterPt_CM = 3.40f;

const float LEFT_WHEEL_DIAMETER_CM = 6.0938f;
const float RIGHT_WHEEL_DIAMETER_CM = 6.1044f;

const float LEFT_WHEEL_CIRCUM_CM = PI * LEFT_WHEEL_DIAMETER_CM;
const float RIGHT_WHEEL_CIRCUM_CM = PI * RIGHT_WHEEL_DIAMETER_CM;

const int FULL_STEPS_PER_REV = 200;
const int MICROSTEP_DIVISOR = 8;
const int MICROSTEPS_PER_REV = FULL_STEPS_PER_REV * MICROSTEP_DIVISOR;

const float LEFT_CM_PER_STEP = LEFT_WHEEL_CIRCUM_CM / (float)MICROSTEPS_PER_REV;
const float RIGHT_CM_PER_STEP = RIGHT_WHEEL_CIRCUM_CM / (float)MICROSTEPS_PER_REV;

// Encoder: degrees of wheel rotation per cm of travel
const float LEFT_DEG_PER_CM  = 360.0f / LEFT_WHEEL_CIRCUM_CM;
const float RIGHT_DEG_PER_CM = 360.0f / RIGHT_WHEEL_CIRCUM_CM;

// =====================================
// SPEEDS (step delay in us — lower = faster)
// =====================================
const int CRUISE_DELAY_US = 80;
const int BOTTLE_DELAY_US = 120;
const int TURN_DELAY_US = 150;
const int TURN_BOTTLE_DELAY_US = 240;
const double bottleTurnFactor = 1.003;

// =====================================
// RAMP SETTINGS
// =====================================
const float SHORT_RAMP_PERCENT = 35.0f;
const float LONG_RAMP_PERCENT = 14.0f;
const float SHORT_THRESHOLD_CM = 50.0f;
const float TURN_RAMP_PERCENT = 21.0f;
const float BOTTLE_RAMP_PERCENT = 35.0f;
const float START_SPEED = 0.18f;
const float END_SPEED = 0.21f;
const int MAX_DELAY_US = 2500;

// =====================================
// TIMING
// =====================================
const float TARGET_TIME_S = 57.0f;
unsigned long routeStartMs = 0;

// =====================================
// ARC CONSTANTS
// =====================================
const float ARC_RADIUS_CM = 25.0f;
const int ARC_DELAY_US = 120;
const int ARC_BOTTLE_DELAY_US = 180;
const float ARC_DISTANCE_FACTOR = 1.0f;
const float ARC_BOTTLE_DISTANCE_FACTOR_R = 1.0025f;
const float ARC_BOTTLE_DISTANCE_FACTOR_L = 1.004f;

// =====================================
// ODOMETRY
// =====================================
float odoX = 0.0f, odoY = 0.0f, odoHeadingDeg = 0.0f;

// =====================================
// ENCODER CORRECTION SETTINGS
// =====================================
const int   CORRECTION_DELAY_US    = 400;   // slow step rate for correction pulses
const float CORRECTION_DEADBAND_DEG = 1.0f; // ignore encoder errors smaller than this
const int   MAX_CORRECTION_STEPS   = 80;    // safety cap per correction burst
const bool  ENCODER_CORRECTION_ON  = true;  // master switch — false to log-only

// =====================================
// ENCODER STATE
// =====================================
float encLeftAngleBefore  = 0.0f;
float encRightAngleBefore = 0.0f;
float encLeftDelta  = 0.0f;
float encRightDelta = 0.0f;

// =====================================
// RUN LOGGING
// =====================================
struct MoveLogEntry {
  char  label[16];
  float param;
  float encLeftDeg;         // actual encoder rotation
  float encRightDeg;
  float expectedLeftDeg;    // commanded rotation
  float expectedRightDeg;
  int   corrLeftSteps;      // correction steps applied
  int   corrRightSteps;
  float odoX, odoY, odoHead;
  float elapsedS;
};

const int MAX_LOG_ENTRIES = 100;
MoveLogEntry moveLog[MAX_LOG_ENTRIES];
int logCount = 0;
bool runLogReady = false;
bool runLogPrinted = false;

// Temp storage filled during a move
float _logExpectedLeftDeg  = 0.0f;
float _logExpectedRightDeg = 0.0f;
int   _logCorrLeftSteps    = 0;
int   _logCorrRightSteps   = 0;
float _logActualLeftDeg    = 0.0f;
float _logActualRightDeg   = 0.0f;

void logMove(const char* label, float param) {
  if (logCount >= MAX_LOG_ENTRIES) return;
  MoveLogEntry &e = moveLog[logCount];
  strncpy(e.label, label, 15);
  e.label[15] = '\0';
  e.param = param;
  _logCorrLeftSteps  = 0;
  _logCorrRightSteps = 0;
  _logExpectedLeftDeg  = 0;
  _logExpectedRightDeg = 0;
  _logActualLeftDeg    = 0;
  _logActualRightDeg   = 0;
}

void logMoveEnd() {
  if (logCount >= MAX_LOG_ENTRIES) return;
  MoveLogEntry &e = moveLog[logCount];
  e.expectedLeftDeg  = _logExpectedLeftDeg;
  e.expectedRightDeg = _logExpectedRightDeg;
  e.encLeftDeg       = _logActualLeftDeg;
  e.encRightDeg      = _logActualRightDeg;
  e.corrLeftSteps    = _logCorrLeftSteps;
  e.corrRightSteps   = _logCorrRightSteps;
  e.odoX    = odoX;
  e.odoY    = odoY;
  e.odoHead = odoHeadingDeg;
  e.elapsedS = (millis() - routeStartMs) / 1000.0f;
  logCount++;
}

void printRunLog() {
  Serial.println();
  Serial.println("============================================================");
  Serial.println("RUN LOG");
  Serial.println("============================================================");
  Serial.print("Total moves: "); Serial.println(logCount);
  Serial.print("Total time:  ");
  if (logCount > 0) Serial.print(moveLog[logCount - 1].elapsedS, 2);
  else              Serial.print("0");
  Serial.println(" s");
  Serial.println("------------------------------------------------------------");

  for (int i = 0; i < logCount; i++) {
    const MoveLogEntry &e = moveLog[i];
    Serial.print(i);
    Serial.print(") ");
    Serial.print(e.label);
    Serial.print("("); Serial.print(e.param, 2); Serial.println(")");

    Serial.print("   t="); Serial.print(e.elapsedS, 2); Serial.println(" s");

    Serial.print("   L wheel: expected=");
    Serial.print(e.expectedLeftDeg, 2);
    Serial.print(" deg  actual=");
    Serial.print(e.encLeftDeg, 2);
    Serial.print(" deg  err=");
    Serial.print(e.encLeftDeg - e.expectedLeftDeg, 2);
    Serial.print(" deg  corr=");
    Serial.print(e.corrLeftSteps);
    Serial.println(" steps");

    Serial.print("   R wheel: expected=");
    Serial.print(e.expectedRightDeg, 2);
    Serial.print(" deg  actual=");
    Serial.print(e.encRightDeg, 2);
    Serial.print(" deg  err=");
    Serial.print(e.encRightDeg - e.expectedRightDeg, 2);
    Serial.print(" deg  corr=");
    Serial.print(e.corrRightSteps);
    Serial.println(" steps");

    Serial.print("   odo: x="); Serial.print(e.odoX, 2);
    Serial.print("  y="); Serial.print(e.odoY, 2);
    Serial.print("  head="); Serial.print(e.odoHead, 2);
    Serial.println(" deg");
    Serial.println();
  }

  Serial.println("============================================================");
  Serial.println();
}

void serviceDeferredLogPrint() {
  if (runLogReady && !runLogPrinted) {
    printRunLog();
    runLogPrinted = true;
  }
}

// =====================================
// ENCODER FUNCTIONS
// =====================================
float readEncoderAngle(TwoWire &bus) {
  bus.beginTransmission(MT6701_ADDR);
  bus.write(MT6701_ANGLE_REG);
  if (bus.endTransmission() != 0) return -1.0f;

  bus.requestFrom(MT6701_ADDR, 2);
  if (bus.available() == 2) {
    uint8_t highByte = bus.read();
    uint8_t lowByte  = bus.read();
    uint16_t raw = (highByte << 6) | (lowByte >> 2);
    return (raw * 360.0f) / 16384.0f;
  }
  return -1.0f;
}

float readLeftEncoder()  { return readEncoderAngle(Wire);  }
float readRightEncoder() { return readEncoderAngle(Wire1); }

// Signed angular delta handling 0/360 wraparound
float angleDelta(float before, float after) {
  float d = after - before;
  while (d > 180.0f)  d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

// Snapshot encoder positions before a move
void encSnapshot() {
  encLeftAngleBefore  = readLeftEncoder();
  encRightAngleBefore = readRightEncoder();
}

// Measure how far each wheel actually rotated since snapshot
void encMeasure() {
  float leftNow  = readLeftEncoder();
  float rightNow = readRightEncoder();
  encLeftDelta  = angleDelta(encLeftAngleBefore, leftNow);
  encRightDelta = angleDelta(encRightAngleBefore, rightNow);
}

// =====================================
// ENCODER CORRECTION
// =====================================
// After a move completes, compare expected wheel rotation (degrees)
// to what the encoder actually measured. If a wheel under-rotated,
// drive slow correction steps to make up the difference.
//
void correctFromEncoder(float expectedLeftDeg, float expectedRightDeg,
                        int leftDir, int rightDir) {
  _logExpectedLeftDeg  = expectedLeftDeg;
  _logExpectedRightDeg = expectedRightDeg;

  // Read encoder result
  encMeasure();

  float actualLeft  = fabsf(encLeftDelta);
  float actualRight = fabsf(encRightDelta);
  float expLeft     = fabsf(expectedLeftDeg);
  float expRight    = fabsf(expectedRightDeg);

  // Store actual for logging
  _logActualLeftDeg  = actualLeft;
  _logActualRightDeg = actualRight;

  if (!ENCODER_CORRECTION_ON) return;

  float errLeft  = expLeft  - actualLeft;   // positive = under-rotated
  float errRight = expRight - actualRight;

  // Correct left wheel
  if (errLeft > CORRECTION_DEADBAND_DEG) {
    float corrCm = errLeft / LEFT_DEG_PER_CM;
    int steps = cmToStepsLeft(corrCm);
    steps = min(steps, MAX_CORRECTION_STEPS);
    _logCorrLeftSteps = steps;

    digitalWrite(L_DIR, leftDir);
    delay(1);
    for (int i = 0; i < steps; i++) {
      if (stopped()) break;
      digitalWrite(L_STEP, HIGH);
      delayMicroseconds(CORRECTION_DELAY_US);
      digitalWrite(L_STEP, LOW);
      delayMicroseconds(CORRECTION_DELAY_US);
    }
  }

  // Correct right wheel
  if (errRight > CORRECTION_DEADBAND_DEG) {
    float corrCm = errRight / RIGHT_DEG_PER_CM;
    int steps = cmToStepsRight(corrCm);
    steps = min(steps, MAX_CORRECTION_STEPS);
    _logCorrRightSteps = steps;

    digitalWrite(R_DIR, rightDir);
    delay(1);
    for (int i = 0; i < steps; i++) {
      if (stopped()) break;
      digitalWrite(R_STEP, HIGH);
      delayMicroseconds(CORRECTION_DELAY_US);
      digitalWrite(R_STEP, LOW);
      delayMicroseconds(CORRECTION_DELAY_US);
    }
  }
}

// =====================================
// ODOMETRY
// =====================================
void resetOdo() {
  odoX = 0; odoY = 0; odoHeadingDeg = 0;
}

void updateOdoLinear(float cm) {
  float rad = odoHeadingDeg * PI / 180.0f;
  odoX += cm * cosf(rad);
  odoY += cm * sinf(rad);
}

void updateOdoTurn(float deg) {
  odoHeadingDeg += deg;
  while (odoHeadingDeg > 180)  odoHeadingDeg -= 360;
  while (odoHeadingDeg < -180) odoHeadingDeg += 360;
}

// =====================================
// HELPERS
// =====================================
int cmToStepsLeft(float cm)  { return (int)(fabsf(cm) / LEFT_CM_PER_STEP + 0.5f); }
int cmToStepsRight(float cm) { return (int)(fabsf(cm) / RIGHT_CM_PER_STEP + 0.5f); }

float turnArcCm(float deg) {
  return (fabsf(deg) / 360.0f) * PI * WHEELBASE_CM;
}

float straightRampPct(float cm) {
  return (fabsf(cm) <= SHORT_THRESHOLD_CM) ? SHORT_RAMP_PERCENT : LONG_RAMP_PERCENT;
}

bool stopped() { return digitalRead(STOP_BUTTON) == LOW; }

void setMicrostep() {
  digitalWrite(L_MS1, LOW); digitalWrite(L_MS2, LOW);
  digitalWrite(R_MS1, LOW); digitalWrite(R_MS2, LOW);
}

// =====================================
// S-CURVE RAMP
// =====================================
int rampDelay(int step, int totalSteps, int baseDelayUs, float rampPct) {
  int rampStart = min((int)(totalSteps * rampPct / 100.0f), totalSteps / 3);
  int rampEnd   = min((int)(totalSteps * rampPct / 100.0f), totalSteps / 3);

  if (step < rampStart) {
    float t = (float)step / (float)rampStart;
    float s = 0.5f - 0.5f * cosf(t * PI);
    float speed = START_SPEED + (1.0f - START_SPEED) * s;
    return min((int)(baseDelayUs / speed), MAX_DELAY_US);
  }

  int distFromEnd = totalSteps - 1 - step;
  if (distFromEnd < rampEnd) {
    float t = (float)distFromEnd / (float)rampEnd;
    float s = 0.5f - 0.5f * cosf(t * PI);
    float speed = END_SPEED + (1.0f - END_SPEED) * s;
    return min((int)(baseDelayUs / speed), MAX_DELAY_US);
  }

  return baseDelayUs;
}

// =====================================
// BRESENHAM DRIVE VARIANTS
// =====================================
void driveSynced(int leftSteps, int rightSteps,
                 int leftDir, int rightDir,
                 int baseDelayUs, bool useRamp, float rampPct) {
  digitalWrite(L_DIR, leftDir);
  digitalWrite(R_DIR, rightDir);
  delay(2);

  int maxSteps = max(leftSteps, rightSteps);
  int leftAccum = 0, rightAccum = 0;

  for (int i = 0; i < maxSteps; i++) {
    if (stopped()) break;
    leftAccum  += leftSteps;
    rightAccum += rightSteps;
    bool stepL = (leftAccum >= maxSteps);
    bool stepR = (rightAccum >= maxSteps);
    if (stepL) leftAccum  -= maxSteps;
    if (stepR) rightAccum -= maxSteps;

    int d = useRamp ? rampDelay(i, maxSteps, baseDelayUs, rampPct) : baseDelayUs;

    if (stepL) digitalWrite(L_STEP, HIGH);
    if (stepR) digitalWrite(R_STEP, HIGH);
    delayMicroseconds(d);
    digitalWrite(L_STEP, LOW);
    digitalWrite(R_STEP, LOW);
    delayMicroseconds(d);
  }
  delay(50);
}

void driveSyncedRampUp(int leftSteps, int rightSteps,
                       int leftDir, int rightDir,
                       int baseDelayUs, float rampPct) {
  digitalWrite(L_DIR, leftDir);
  digitalWrite(R_DIR, rightDir);
  delay(2);

  int maxSteps = max(leftSteps, rightSteps);
  int rampLen = min((int)(maxSteps * rampPct / 100.0f), maxSteps / 3);
  int leftAccum = 0, rightAccum = 0;

  for (int i = 0; i < maxSteps; i++) {
    if (stopped()) break;
    leftAccum  += leftSteps;
    rightAccum += rightSteps;
    bool stepL = (leftAccum >= maxSteps);
    bool stepR = (rightAccum >= maxSteps);
    if (stepL) leftAccum  -= maxSteps;
    if (stepR) rightAccum -= maxSteps;

    int d = baseDelayUs;
    if (i < rampLen) {
      float t = (float)i / (float)rampLen;
      float s = 0.5f - 0.5f * cosf(t * PI);
      float speed = START_SPEED + (1.0f - START_SPEED) * s;
      d = min((int)(baseDelayUs / speed), MAX_DELAY_US);
    }

    if (stepL) digitalWrite(L_STEP, HIGH);
    if (stepR) digitalWrite(R_STEP, HIGH);
    delayMicroseconds(d);
    digitalWrite(L_STEP, LOW);
    digitalWrite(R_STEP, LOW);
    delayMicroseconds(d);
  }
}

void driveSyncedConstant(int leftSteps, int rightSteps,
                         int leftDir, int rightDir,
                         int baseDelayUs) {
  digitalWrite(L_DIR, leftDir);
  digitalWrite(R_DIR, rightDir);
  delay(2);

  int maxSteps = max(leftSteps, rightSteps);
  int leftAccum = 0, rightAccum = 0;

  for (int i = 0; i < maxSteps; i++) {
    if (stopped()) break;
    leftAccum  += leftSteps;
    rightAccum += rightSteps;
    bool stepL = (leftAccum >= maxSteps);
    bool stepR = (rightAccum >= maxSteps);
    if (stepL) leftAccum  -= maxSteps;
    if (stepR) rightAccum -= maxSteps;

    if (stepL) digitalWrite(L_STEP, HIGH);
    if (stepR) digitalWrite(R_STEP, HIGH);
    delayMicroseconds(baseDelayUs);
    digitalWrite(L_STEP, LOW);
    digitalWrite(R_STEP, LOW);
    delayMicroseconds(baseDelayUs);
  }
}

void driveSyncedRampDown(int leftSteps, int rightSteps,
                         int leftDir, int rightDir,
                         int baseDelayUs, float rampPct) {
  digitalWrite(L_DIR, leftDir);
  digitalWrite(R_DIR, rightDir);
  delay(2);

  int maxSteps = max(leftSteps, rightSteps);
  int rampLen = min((int)(maxSteps * rampPct / 100.0f), maxSteps / 3);
  int leftAccum = 0, rightAccum = 0;

  for (int i = 0; i < maxSteps; i++) {
    if (stopped()) break;
    leftAccum  += leftSteps;
    rightAccum += rightSteps;
    bool stepL = (leftAccum >= maxSteps);
    bool stepR = (rightAccum >= maxSteps);
    if (stepL) leftAccum  -= maxSteps;
    if (stepR) rightAccum -= maxSteps;

    int d = baseDelayUs;
    int distFromEnd = maxSteps - 1 - i;
    if (distFromEnd < rampLen) {
      float t = (float)distFromEnd / (float)rampLen;
      float s = 0.5f - 0.5f * cosf(t * PI);
      float speed = END_SPEED + (1.0f - END_SPEED) * s;
      d = min((int)(baseDelayUs / speed), MAX_DELAY_US);
    }

    if (stepL) digitalWrite(L_STEP, HIGH);
    if (stepR) digitalWrite(R_STEP, HIGH);
    delayMicroseconds(d);
    digitalWrite(L_STEP, LOW);
    digitalWrite(R_STEP, LOW);
    delayMicroseconds(d);
  }
  delay(50);
}

// =====================================
// MOVEMENT FUNCTIONS — WITH ENCODER CORRECTION
// =====================================
const int SPIN_DELAY_US = 80;

void spinL(int steps) {
  digitalWrite(L_DIR, HIGH);
  delay(2);
  for (int i = 0; i < steps; i++) {
    if (stopped()) break;
    digitalWrite(L_STEP, HIGH);
    delayMicroseconds(SPIN_DELAY_US);
    digitalWrite(L_STEP, LOW);
    delayMicroseconds(SPIN_DELAY_US);
  }
}

void spinR(int steps) {
  digitalWrite(R_DIR, LOW);
  delay(2);
  for (int i = 0; i < steps; i++) {
    if (stopped()) break;
    digitalWrite(R_STEP, HIGH);
    delayMicroseconds(SPIN_DELAY_US);
    digitalWrite(R_STEP, LOW);
    delayMicroseconds(SPIN_DELAY_US);
  }
}

void forward(float cm) {
  logMove("forward", cm);
  float expectedLeftDeg  = cm * LEFT_DEG_PER_CM;
  float expectedRightDeg = cm * RIGHT_DEG_PER_CM;

  encSnapshot();

  int lSteps = cmToStepsLeft(cm);
  int rSteps = cmToStepsRight(cm);
  driveSynced(lSteps, rSteps, HIGH, LOW, CRUISE_DELAY_US, true, straightRampPct(cm));

  correctFromEncoder(expectedLeftDeg, expectedRightDeg, HIGH, LOW);

  updateOdoLinear(cm);
  logMoveEnd();
}

void centerF() {
  float centerDistance = AimPt_CenterPt_CM;
  const int CENTER_DELAY_US = 900;
  int lSteps = cmToStepsLeft(centerDistance);
  int rSteps = cmToStepsRight(centerDistance);
  driveSyncedConstant(lSteps, rSteps, HIGH, LOW, CENTER_DELAY_US);
  updateOdoLinear(centerDistance);
}

void centerB() {
  float centerDistance = AimPt_CenterPt_CM;
  const int CENTER_DELAY_US = 900;
  int lSteps = cmToStepsLeft(centerDistance);
  int rSteps = cmToStepsRight(centerDistance);
  driveSyncedConstant(lSteps, rSteps, LOW, HIGH, CENTER_DELAY_US);
  updateOdoLinear(centerDistance);
}

void backward(float cm) {
  logMove("backward", cm);
  float expectedLeftDeg  = cm * LEFT_DEG_PER_CM;
  float expectedRightDeg = cm * RIGHT_DEG_PER_CM;

  encSnapshot();

  int lSteps = cmToStepsLeft(cm);
  int rSteps = cmToStepsRight(cm);
  driveSynced(lSteps, rSteps, LOW, HIGH, CRUISE_DELAY_US, true, straightRampPct(cm));

  correctFromEncoder(expectedLeftDeg, expectedRightDeg, LOW, HIGH);

  updateOdoLinear(-cm);
  logMoveEnd();
}

void forwardBottle(float cm) {
  logMove("fwd bottle", cm);
  float expectedLeftDeg  = cm * LEFT_DEG_PER_CM;
  float expectedRightDeg = cm * RIGHT_DEG_PER_CM;

  encSnapshot();

  int lSteps = cmToStepsLeft(cm);
  int rSteps = cmToStepsRight(cm);
  driveSynced(lSteps, rSteps, HIGH, LOW, BOTTLE_DELAY_US, true, BOTTLE_RAMP_PERCENT);

  correctFromEncoder(expectedLeftDeg, expectedRightDeg, HIGH, LOW);

  updateOdoLinear(cm);
  logMoveEnd();
}

// =====================================
// CHAINED FORWARD VARIANTS (no correction mid-chain)
// =====================================
void fru(float cm) {
  int lSteps = cmToStepsLeft(cm);
  int rSteps = cmToStepsRight(cm);
  driveSyncedRampUp(lSteps, rSteps, HIGH, LOW, ARC_DELAY_US, straightRampPct(cm));
  updateOdoLinear(cm);
}

void fc(float cm) {
  int lSteps = cmToStepsLeft(cm);
  int rSteps = cmToStepsRight(cm);
  driveSyncedConstant(lSteps, rSteps, HIGH, LOW, ARC_DELAY_US);
  updateOdoLinear(cm);
}

void frd(float cm) {
  int lSteps = cmToStepsLeft(cm);
  int rSteps = cmToStepsRight(cm);
  driveSyncedRampDown(lSteps, rSteps, HIGH, LOW, ARC_DELAY_US, straightRampPct(cm));
  updateOdoLinear(cm);
}

void frub(float cm) {
  int lSteps = cmToStepsLeft(cm);
  int rSteps = cmToStepsRight(cm);
  driveSyncedRampUp(lSteps, rSteps, HIGH, LOW, ARC_BOTTLE_DELAY_US, straightRampPct(cm));
  updateOdoLinear(cm);
}

void fcb(float cm) {
  int lSteps = cmToStepsLeft(cm);
  int rSteps = cmToStepsRight(cm);
  driveSyncedConstant(lSteps, rSteps, HIGH, LOW, ARC_BOTTLE_DELAY_US);
  updateOdoLinear(cm);
}

void frdb(float cm) {
  int lSteps = cmToStepsLeft(cm);
  int rSteps = cmToStepsRight(cm);
  driveSyncedRampDown(lSteps, rSteps, HIGH, LOW, ARC_BOTTLE_DELAY_US, straightRampPct(cm));
  updateOdoLinear(cm);
}

// =====================================
// SPIN TURNS — WITH ENCODER CORRECTION
// =====================================
void spinRight(float degrees) {
  logMove("right", degrees);
  float arc = turnArcCm(degrees);
  float expectedLeftDeg  = arc * LEFT_DEG_PER_CM;
  float expectedRightDeg = arc * RIGHT_DEG_PER_CM;

  encSnapshot();

  int lSteps = cmToStepsLeft(arc);
  int rSteps = cmToStepsRight(arc);
  driveSynced(lSteps, rSteps, HIGH, HIGH, TURN_DELAY_US, true, TURN_RAMP_PERCENT);

  correctFromEncoder(expectedLeftDeg, expectedRightDeg, HIGH, HIGH);

  updateOdoTurn(degrees);
  logMoveEnd();
}

void spinLeft(float degrees) {
  logMove("left", degrees);
  float arc = turnArcCm(degrees);
  float expectedLeftDeg  = arc * LEFT_DEG_PER_CM;
  float expectedRightDeg = arc * RIGHT_DEG_PER_CM;

  encSnapshot();

  int lSteps = cmToStepsLeft(arc);
  int rSteps = cmToStepsRight(arc);
  driveSynced(lSteps, rSteps, LOW, LOW, TURN_DELAY_US, true, TURN_RAMP_PERCENT);

  correctFromEncoder(expectedLeftDeg, expectedRightDeg, LOW, LOW);

  updateOdoTurn(-degrees);
  logMoveEnd();
}

void spinRightBottle(float degrees) {
  logMove("right bottle", degrees);
  float adjusted = degrees * bottleTurnFactor;
  float arc = turnArcCm(adjusted);
  float expectedLeftDeg  = arc * LEFT_DEG_PER_CM;
  float expectedRightDeg = arc * RIGHT_DEG_PER_CM;

  encSnapshot();

  int lSteps = cmToStepsLeft(arc);
  int rSteps = cmToStepsRight(arc);
  driveSynced(lSteps, rSteps, HIGH, HIGH, TURN_BOTTLE_DELAY_US, true, BOTTLE_RAMP_PERCENT);

  correctFromEncoder(expectedLeftDeg, expectedRightDeg, HIGH, HIGH);

  updateOdoTurn(degrees);
  logMoveEnd();
}

void spinLeftBottle(float degrees) {
  logMove("left bottle", degrees);
  float adjusted = degrees * bottleTurnFactor;
  float arc = turnArcCm(adjusted);
  float expectedLeftDeg  = arc * LEFT_DEG_PER_CM;
  float expectedRightDeg = arc * RIGHT_DEG_PER_CM;

  encSnapshot();

  int lSteps = cmToStepsLeft(arc);
  int rSteps = cmToStepsRight(arc);
  driveSynced(lSteps, rSteps, LOW, LOW, TURN_BOTTLE_DELAY_US, true, BOTTLE_RAMP_PERCENT);

  correctFromEncoder(expectedLeftDeg, expectedRightDeg, LOW, LOW);

  updateOdoTurn(-degrees);
  logMoveEnd();
}

// =====================================
// ARC TURNS — STANDALONE (with correction)
// =====================================
void als() {
  logMove("arc left", 90);
  float innerR = ARC_RADIUS_CM - (ARC_WHEELBASE_CM / 2.0f);
  float outerR = ARC_RADIUS_CM + (ARC_WHEELBASE_CM / 2.0f);
  float innerArc = (90.0f / 360.0f) * 2.0f * PI * innerR * ARC_DISTANCE_FACTOR;
  float outerArc = (90.0f / 360.0f) * 2.0f * PI * outerR * ARC_DISTANCE_FACTOR;

  float expectedLeftDeg  = innerArc * LEFT_DEG_PER_CM;
  float expectedRightDeg = outerArc * RIGHT_DEG_PER_CM;

  encSnapshot();
  driveSynced(cmToStepsLeft(innerArc), cmToStepsRight(outerArc), HIGH, LOW, ARC_DELAY_US, true, TURN_RAMP_PERCENT);
  correctFromEncoder(expectedLeftDeg, expectedRightDeg, HIGH, LOW);

  updateOdoTurn(-90);
  logMoveEnd();
}

void ars() {
  logMove("arc right", 90);
  float innerR = ARC_RADIUS_CM - (ARC_WHEELBASE_CM / 2.0f);
  float outerR = ARC_RADIUS_CM + (ARC_WHEELBASE_CM / 2.0f);
  float innerArc = (90.0f / 360.0f) * 2.0f * PI * innerR * ARC_DISTANCE_FACTOR;
  float outerArc = (90.0f / 360.0f) * 2.0f * PI * outerR * ARC_DISTANCE_FACTOR;

  float expectedLeftDeg  = outerArc * LEFT_DEG_PER_CM;
  float expectedRightDeg = innerArc * RIGHT_DEG_PER_CM;

  encSnapshot();
  driveSynced(cmToStepsLeft(outerArc), cmToStepsRight(innerArc), HIGH, LOW, ARC_DELAY_US, true, TURN_RAMP_PERCENT);
  correctFromEncoder(expectedLeftDeg, expectedRightDeg, HIGH, LOW);

  updateOdoTurn(90);
  logMoveEnd();
}

// =====================================
// ARC RIGHT — CHAINED VARIANTS (no correction mid-chain)
// =====================================
void arru() {
  float innerR = ARC_RADIUS_CM - (ARC_WHEELBASE_CM / 2.0f);
  float outerR = ARC_RADIUS_CM + (ARC_WHEELBASE_CM / 2.0f);
  float innerArc = (90.0f / 360.0f) * 2.0f * PI * innerR * ARC_DISTANCE_FACTOR;
  float outerArc = (90.0f / 360.0f) * 2.0f * PI * outerR * ARC_DISTANCE_FACTOR;
  driveSyncedRampUp(cmToStepsLeft(outerArc), cmToStepsRight(innerArc), HIGH, LOW, ARC_DELAY_US, TURN_RAMP_PERCENT);
  updateOdoTurn(90);
}

void arc() {
  float innerR = ARC_RADIUS_CM - (ARC_WHEELBASE_CM / 2.0f);
  float outerR = ARC_RADIUS_CM + (ARC_WHEELBASE_CM / 2.0f);
  float innerArc = (90.0f / 360.0f) * 2.0f * PI * innerR * ARC_DISTANCE_FACTOR;
  float outerArc = (90.0f / 360.0f) * 2.0f * PI * outerR * ARC_DISTANCE_FACTOR;
  driveSyncedConstant(cmToStepsLeft(outerArc), cmToStepsRight(innerArc), HIGH, LOW, ARC_DELAY_US);
  updateOdoTurn(90);
}

void arrd() {
  float innerR = ARC_RADIUS_CM - (ARC_WHEELBASE_CM / 2.0f);
  float outerR = ARC_RADIUS_CM + (ARC_WHEELBASE_CM / 2.0f);
  float innerArc = (90.0f / 360.0f) * 2.0f * PI * innerR * ARC_DISTANCE_FACTOR;
  float outerArc = (90.0f / 360.0f) * 2.0f * PI * outerR * ARC_DISTANCE_FACTOR;
  driveSyncedRampDown(cmToStepsLeft(outerArc), cmToStepsRight(innerArc), HIGH, LOW, ARC_DELAY_US, TURN_RAMP_PERCENT);
  updateOdoTurn(90);
}

void arrub() {
  float innerR = ARC_RADIUS_CM - (ARC_WHEELBASE_CM / 2.0f);
  float outerR = ARC_RADIUS_CM + (ARC_WHEELBASE_CM / 2.0f);
  float innerArc = (90.0f / 360.0f) * 2.0f * PI * innerR * ARC_DISTANCE_FACTOR * ARC_BOTTLE_DISTANCE_FACTOR_R;
  float outerArc = (90.0f / 360.0f) * 2.0f * PI * outerR * ARC_DISTANCE_FACTOR * ARC_BOTTLE_DISTANCE_FACTOR_R;
  driveSyncedRampUp(cmToStepsLeft(outerArc), cmToStepsRight(innerArc), HIGH, LOW, ARC_BOTTLE_DELAY_US, TURN_RAMP_PERCENT);
  updateOdoTurn(90);
}

void arcb() {
  float innerR = ARC_RADIUS_CM - (ARC_WHEELBASE_CM / 2.0f);
  float outerR = ARC_RADIUS_CM + (ARC_WHEELBASE_CM / 2.0f);
  float innerArc = (90.0f / 360.0f) * 2.0f * PI * innerR * ARC_DISTANCE_FACTOR * ARC_BOTTLE_DISTANCE_FACTOR_R;
  float outerArc = (90.0f / 360.0f) * 2.0f * PI * outerR * ARC_DISTANCE_FACTOR * ARC_BOTTLE_DISTANCE_FACTOR_R;
  driveSyncedConstant(cmToStepsLeft(outerArc), cmToStepsRight(innerArc), HIGH, LOW, ARC_BOTTLE_DELAY_US);
  updateOdoTurn(90);
}

void arrdb() {
  float innerR = ARC_RADIUS_CM - (ARC_WHEELBASE_CM / 2.0f);
  float outerR = ARC_RADIUS_CM + (ARC_WHEELBASE_CM / 2.0f);
  float innerArc = (90.0f / 360.0f) * 2.0f * PI * innerR * ARC_DISTANCE_FACTOR * ARC_BOTTLE_DISTANCE_FACTOR_R;
  float outerArc = (90.0f / 360.0f) * 2.0f * PI * outerR * ARC_DISTANCE_FACTOR * ARC_BOTTLE_DISTANCE_FACTOR_R;
  driveSyncedRampDown(cmToStepsLeft(outerArc), cmToStepsRight(innerArc), HIGH, LOW, ARC_BOTTLE_DELAY_US, TURN_RAMP_PERCENT);
  updateOdoTurn(90);
}

// =====================================
// ARC LEFT — CHAINED VARIANTS (no correction mid-chain)
// =====================================
void alru() {
  float innerR = ARC_RADIUS_CM - (ARC_WHEELBASE_CM / 2.0f);
  float outerR = ARC_RADIUS_CM + (ARC_WHEELBASE_CM / 2.0f);
  float innerArc = (90.0f / 360.0f) * 2.0f * PI * innerR * ARC_DISTANCE_FACTOR;
  float outerArc = (90.0f / 360.0f) * 2.0f * PI * outerR * ARC_DISTANCE_FACTOR;
  driveSyncedRampUp(cmToStepsLeft(innerArc), cmToStepsRight(outerArc), HIGH, LOW, ARC_DELAY_US, TURN_RAMP_PERCENT);
  updateOdoTurn(-90);
}

void alc() {
  float innerR = ARC_RADIUS_CM - (ARC_WHEELBASE_CM / 2.0f);
  float outerR = ARC_RADIUS_CM + (ARC_WHEELBASE_CM / 2.0f);
  float innerArc = (90.0f / 360.0f) * 2.0f * PI * innerR * ARC_DISTANCE_FACTOR;
  float outerArc = (90.0f / 360.0f) * 2.0f * PI * outerR * ARC_DISTANCE_FACTOR;
  driveSyncedConstant(cmToStepsLeft(innerArc), cmToStepsRight(outerArc), HIGH, LOW, ARC_DELAY_US);
  updateOdoTurn(-90);
}

void alrd() {
  float innerR = ARC_RADIUS_CM - (ARC_WHEELBASE_CM / 2.0f);
  float outerR = ARC_RADIUS_CM + (ARC_WHEELBASE_CM / 2.0f);
  float innerArc = (90.0f / 360.0f) * 2.0f * PI * innerR * ARC_DISTANCE_FACTOR;
  float outerArc = (90.0f / 360.0f) * 2.0f * PI * outerR * ARC_DISTANCE_FACTOR;
  driveSyncedRampDown(cmToStepsLeft(innerArc), cmToStepsRight(outerArc), HIGH, LOW, ARC_DELAY_US, TURN_RAMP_PERCENT);
  updateOdoTurn(-90);
}

void alrub() {
  float innerR = ARC_RADIUS_CM - (ARC_WHEELBASE_CM / 2.0f);
  float outerR = ARC_RADIUS_CM + (ARC_WHEELBASE_CM / 2.0f);
  float innerArc = (90.0f / 360.0f) * 2.0f * PI * innerR * ARC_DISTANCE_FACTOR * ARC_BOTTLE_DISTANCE_FACTOR_L;
  float outerArc = (90.0f / 360.0f) * 2.0f * PI * outerR * ARC_DISTANCE_FACTOR * ARC_BOTTLE_DISTANCE_FACTOR_L;
  driveSyncedRampUp(cmToStepsLeft(innerArc), cmToStepsRight(outerArc), HIGH, LOW, ARC_BOTTLE_DELAY_US, TURN_RAMP_PERCENT);
  updateOdoTurn(-90);
}

void alcb() {
  float innerR = ARC_RADIUS_CM - (ARC_WHEELBASE_CM / 2.0f);
  float outerR = ARC_RADIUS_CM + (ARC_WHEELBASE_CM / 2.0f);
  float innerArc = (90.0f / 360.0f) * 2.0f * PI * innerR * ARC_DISTANCE_FACTOR * ARC_BOTTLE_DISTANCE_FACTOR_L;
  float outerArc = (90.0f / 360.0f) * 2.0f * PI * outerR * ARC_DISTANCE_FACTOR * ARC_BOTTLE_DISTANCE_FACTOR_L;
  driveSyncedConstant(cmToStepsLeft(innerArc), cmToStepsRight(outerArc), HIGH, LOW, ARC_BOTTLE_DELAY_US);
  updateOdoTurn(-90);
}

void alrdb() {
  float innerR = ARC_RADIUS_CM - (ARC_WHEELBASE_CM / 2.0f);
  float outerR = ARC_RADIUS_CM + (ARC_WHEELBASE_CM / 2.0f);
  float innerArc = (90.0f / 360.0f) * 2.0f * PI * innerR * ARC_DISTANCE_FACTOR * ARC_BOTTLE_DISTANCE_FACTOR_L;
  float outerArc = (90.0f / 360.0f) * 2.0f * PI * outerR * ARC_DISTANCE_FACTOR * ARC_BOTTLE_DISTANCE_FACTOR_L;
  driveSyncedRampDown(cmToStepsLeft(innerArc), cmToStepsRight(outerArc), HIGH, LOW, ARC_BOTTLE_DELAY_US, TURN_RAMP_PERCENT);
  updateOdoTurn(-90);
}

// =====================================
// ENDSCRIPT
// =====================================
void endScript(float cm) {
  logMove("end script", cm);
  float elapsedS = (millis() - routeStartMs) / 1000.0f;
  float remainingS = TARGET_TIME_S - elapsedS;

  if (remainingS <= 0.1f) {
    forward(fabsf(cm));
    return;
  }

  float absCm = fabsf(cm);
  float speedCmS = absCm / remainingS;
  float avgCmPerStep = (LEFT_CM_PER_STEP + RIGHT_CM_PER_STEP) / 2.0f;
  float stepsPerSec = speedCmS / avgCmPerStep;
  int delayUs = (int)(1e6f / (2.0f * stepsPerSec));
  delayUs = constrain(delayUs, CRUISE_DELAY_US, 5000);

  float expectedLeftDeg  = absCm * LEFT_DEG_PER_CM;
  float expectedRightDeg = absCm * RIGHT_DEG_PER_CM;

  encSnapshot();

  int lSteps = cmToStepsLeft(absCm);
  int rSteps = cmToStepsRight(absCm);

  bool goForward = (cm >= 0);
  int lDir = goForward ? HIGH : LOW;
  int rDir = goForward ? LOW : HIGH;

  driveSynced(lSteps, rSteps, lDir, rDir, delayUs, true, straightRampPct(cm));
  correctFromEncoder(expectedLeftDeg, expectedRightDeg, lDir, rDir);

  updateOdoLinear(cm);
  logMoveEnd();
}

// =====================================
// SHORT ALIASES
// =====================================
void f(float cm)  { forward(cm); }
void b(float cm)  { backward(cm); }
void fb(float cm) { forwardBottle(cm); }
void l(float deg) { spinLeft(deg); }
void r(float deg) { spinRight(deg); }
void lb(float deg){ spinLeftBottle(deg); }
void rb(float deg){ spinRightBottle(deg); }
void e(float cm)  { endScript(cm); }

// =====================================
// ROUTE
// =====================================
void route() {
  runLogReady = true;

  f(50);
  delay(100);
  b(50);
}

// =====================================
// SETUP
// =====================================
void setup() {
  Serial.begin(115200);

  pinMode(GO_BUTTON, INPUT_PULLUP);
  pinMode(STOP_BUTTON, INPUT_PULLUP);

  pinMode(L_STEP, OUTPUT);
  pinMode(L_DIR, OUTPUT);
  pinMode(L_MS1, OUTPUT);
  pinMode(L_MS2, OUTPUT);
  pinMode(R_STEP, OUTPUT);
  pinMode(R_DIR, OUTPUT);
  pinMode(R_MS1, OUTPUT);
  pinMode(R_MS2, OUTPUT);

  digitalWrite(L_STEP, LOW);
  digitalWrite(R_STEP, LOW);
  setMicrostep();

  // Init left encoder on I2C0
  Wire.setSDA(ENC_LEFT_SDA);
  Wire.setSCL(ENC_LEFT_SCL);
  Wire.begin();

  // Init right encoder on I2C1
  Wire1.setSDA(ENC_RIGHT_SDA);
  Wire1.setSCL(ENC_RIGHT_SCL);
  Wire1.begin();

  // Verify encoders are responding
  delay(100);
  float testL = readLeftEncoder();
  float testR = readRightEncoder();
  Serial.print("Left encoder:  ");
  if (testL >= 0) { Serial.print(testL, 1); Serial.println(" deg - OK"); }
  else            { Serial.println("NOT FOUND"); }
  Serial.print("Right encoder: ");
  if (testR >= 0) { Serial.print(testR, 1); Serial.println(" deg - OK"); }
  else            { Serial.println("NOT FOUND"); }
}

// =====================================
// LOOP
// =====================================
void loop() {
  // Wait for button press-release
  while (digitalRead(GO_BUTTON) == LOW) delay(10);
  delay(50);
  while (digitalRead(GO_BUTTON) != LOW) delay(10);

  logCount = 0;
  runLogReady = false;
  runLogPrinted = false;

  resetOdo();
  routeStartMs = millis();
  delay(200);

  route();
  serviceDeferredLogPrint();
  delay(2000);
}
