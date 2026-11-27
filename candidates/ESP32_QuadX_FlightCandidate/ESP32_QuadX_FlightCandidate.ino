/*
  ESP32 QUAD-X FLIGHT CONTROLLER + READ-ONLY WIFI TELEMETRY
  Revision: Stage E flight-control candidate (bench locked by default)

  !!! PROPELLERS OFF FOR VALIDATION. NOT FLIGHT-VALIDATED !!!
  Full physical-control code exists behind the default BENCH_LOCKOUT=1.
  DO NOT disable this lockout until the 10ms timing-gap fault is resolved,
  receiver and reset diagnostics remain clean, and motor/axis directions
  have been independently verified. An unexpected failsafe can cut thrust.

  AP: ESP32-DRONE   Password: drone1234   URL: http://192.168.4.1

  Preserved from previous sketch (calculation-only in BENCH_LOCKOUT):
    Pins, 250 Hz loop, 500 Hz ESC command frequency, Quad-X mixer,
    original PID gains, 100 ms RX timeout, 1000/1170/1800 us motor limits,
    low->high CH5 arming with throttle low.

  Deliberate corrections and safety changes:
    - Use proven 3-stage bench ISR capture (no shared RX spinlock)
    - Reject bad widths without replacing last valid PWM pulse
    - Report per-channel raw width, accepted width, edge counts and rejects
    - IMU write/read verification and continued reads after a transient failure
    - Explicit failsafe latching and deliberate low-switch recovery
    - True loop interval, execution time, max timing, overrun counts
    - Saturation-aware PID integral freeze and stop-on-low-throttle reset
    - Dashboard retains the LAST fault after the current latch clears
    - Stop repeated PWM writes and PID resets after a latched failsafe
    - Report receiver dropouts, misses, timing and workload stages
    - BENCH LOCKOUT by default: ESCs at 1000us; PID/mixer calculated in shadow
    - FLIGHT-CODE PATH included but NOT certified and NOT yet safe for flight
*/

#include <Arduino.h>
#include <esp_system.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>
#include <soc/gpio_struct.h>

// -------------------- PHYSICAL OUTPUT SAFETY LOCKOUT --------------------
// DEFAULT = 1: ESCs remain at 1000 us, arming disabled, shadow PID runs.
// 0 enables the experimental RC-arming + motor-output control path. WARNING:
// This path is NOT FLIGHT-VALIDATED. A timing failsafe was still observed in
// Stage D-T (11,172 us interval). Do NOT switch to 0 until that is resolved
// and the complete system passes propeller-free physical direction testing.
#define BENCH_LOCKOUT 1
#if (BENCH_LOCKOUT != 0) && (BENCH_LOCKOUT != 1)
#error BENCH_LOCKOUT must be exactly 0 or 1.
#endif

// -------------------- 1. PIN CONFIGURATION --------------------
static constexpr int SDA_PIN = 21;
static constexpr int SCL_PIN = 22;
static constexpr uint8_t MPU_ADDR = 0x68;
static constexpr int BLUE_LED_PIN = 5;
static constexpr int RED_LED_PIN = 4;

static constexpr int MOTOR1_PIN = 13; // Front Right - CCW
static constexpr int MOTOR2_PIN = 12; // Rear Right  - CW (ESP32 strapping pin!)
static constexpr int MOTOR3_PIN = 14; // Rear Left   - CCW
static constexpr int MOTOR4_PIN = 27; // Front Left  - CW

static constexpr int RX_CH1_PIN = 34; // Roll
static constexpr int RX_CH2_PIN = 35; // Pitch
static constexpr int RX_CH3_PIN = 32; // Throttle
static constexpr int RX_CH4_PIN = 33; // Yaw
static constexpr int RX_CH5_PIN = 25; // Arm switch
static constexpr int RX_CH6_PIN = 26; // Auxiliary
static const int rxPins[6] = {
  RX_CH1_PIN, RX_CH2_PIN, RX_CH3_PIN,
  RX_CH4_PIN, RX_CH5_PIN, RX_CH6_PIN
};

// -------------------- 2. FIXED CONFIGURATION --------------------
static constexpr int ESC_FREQUENCY = 500;  // VERIFY ESC firmware accepts 500 Hz input
static constexpr int ESC_MIN = 1000;
static constexpr int ESC_IDLE = 1170;
static constexpr int ESC_MAX = 1800;

static constexpr uint32_t LOOP_PERIOD_US = 4000;   // 250 Hz
static constexpr uint32_t MAX_LOOP_GAP_US = 10000; // Timing fault threshold
static constexpr uint32_t RX_TIMEOUT_US = 100000;  // 100 ms -- unchanged
static constexpr uint16_t RX_MIN_US = 900;
static constexpr uint16_t RX_MAX_US = 2100;
// A valid 50 Hz PWM channel sends one pulse about every 20 ms. Reject
// implausibly rapid repeated pulses, without increasing RX_TIMEOUT_US.
static constexpr uint32_t RX_MIN_FRAME_US = 3000;
static constexpr uint16_t THROTTLE_CUTOFF = 1030;
static constexpr float MAX_ANGLE_DEG = 20.0f;
static constexpr float MAX_RATE_DPS = 250.0f;
static constexpr uint8_t REQUIRED_RX_MASK = 0x3F; // Require ALL six channels
static constexpr uint16_t IMU_RECOVERY_GOOD_LOOPS = 50;    // 0.2 s at 250 Hz
static constexpr uint16_t TIMING_RECOVERY_GOOD_LOOPS = 100; // 0.4 s

// -------------------- 3. ORIGINAL PID GAINS --------------------
static constexpr float ANGLE_KP = 2.0f;
static constexpr float ANGLE_KI = 0.5f;
static constexpr float ANGLE_KD = 0.007f;
static constexpr float RATE_RP_KP = 0.625f;
static constexpr float RATE_RP_KI = 2.1f;
static constexpr float RATE_RP_KD = 0.0088f;
static constexpr float RATE_Y_KP = 4.0f;
static constexpr float RATE_Y_KI = 3.0f;
static constexpr float RATE_Y_KD = 0.0f;

// -------------------- 4. WIFI --------------------
static const char* AP_SSID = "ESP32-DRONE";
static const char* AP_PASS = "drone1234";
WebServer server(80);
Servo motor1, motor2, motor3, motor4;

// -------------------- 4A. RESET DIAGNOSTICS --------------------
// esp_reset_reason() records WHY the previous run ended. A genuine brownout,
// watchdog, or software panic can restart both Wi-Fi and the flight controller.
// We deliberately do not auto-arm after a restart.
uint32_t bootResetReasonCode = 0;

// -------------------- 5. LOCK-FREE RECEIVER (PROVEN BENCH DESIGN) --------------------
// Copied from the 3-stage 10+ minute bench sketch, including its single-core
// noInterrupts() snapshot. NO portMUX in any RX interrupt handler.
// Keep attachInterrupt() and readReceiver() on Arduino's loop core.
static constexpr int RX_COUNT = 6;
static const uint8_t RX_PIN[RX_COUNT] = {34, 35, 32, 33, 25, 26};
volatile uint32_t risingUs[RX_COUNT] = {};
volatile uint32_t pulseUs[RX_COUNT] = {};
volatile uint32_t rawUs[RX_COUNT] = {};
volatile uint32_t acceptedAtUs[RX_COUNT] = {};
volatile uint32_t edges[RX_COUNT] = {};
volatile uint32_t accepted[RX_COUNT] = {};
volatile uint32_t badWidths[RX_COUNT] = {};
volatile uint32_t orphans[RX_COUNT] = {};
volatile uint8_t sawRise[RX_COUNT] = {};

uint16_t rxValue[RX_COUNT] = {};
uint16_t rxRawValue[RX_COUNT] = {};
uint32_t rxAgeUs[RX_COUNT] = {};
uint32_t rxRejects[RX_COUNT] = {};
uint32_t rxOrphans[RX_COUNT] = {};
uint32_t rxWidthBads[RX_COUNT] = {};
uint32_t rxFrameBads[RX_COUNT] = {}; // unused by validated bench decoder; zero
uint32_t rxResyncs[RX_COUNT] = {}; // unused by validated bench decoder; zero
uint32_t rxEdges[RX_COUNT] = {};
uint32_t rxAccepts[RX_COUNT] = {};
uint8_t rxValid[RX_COUNT] = {};
bool receiverHealthy = false;
bool receiverEverHealthy = false;
uint8_t badChannelsMask = 0x3F;
uint32_t rxDropouts = 0;
uint32_t lastRxDropoutMs = 0;
uint8_t lastRxDropoutMask = 0;
uint32_t maxRxPollGapMs = 0;
uint32_t priorRxPollMs = 0;

// Direct ESP32 GPIO register read as in the successful receiver bench test.
static inline bool IRAM_ATTR readLevel(uint8_t pin) {
  if (pin < 32) return (GPIO.in & (1UL << pin)) != 0;
  return (GPIO.in1.val & (1UL << (pin - 32))) != 0;
}

void IRAM_ATTR onReceiverEdge(uint8_t channel) {
  const uint32_t t = micros();
  const bool isHigh = readLevel(RX_PIN[channel]);
  edges[channel]++;
  if (isHigh) {
    risingUs[channel] = t;
    sawRise[channel] = 1;
    return;
  }
  if (!sawRise[channel]) {
    orphans[channel]++;
    return;
  }
  sawRise[channel] = 0;
  const uint32_t w = (uint32_t)(t - risingUs[channel]);
  rawUs[channel] = w;
  if (w >= RX_MIN_US && w <= RX_MAX_US) {
    pulseUs[channel] = w;
    acceptedAtUs[channel] = t;
    accepted[channel]++;
  } else {
    badWidths[channel]++;
  }
}
void IRAM_ATTR rx1() { onReceiverEdge(0); }
void IRAM_ATTR rx2() { onReceiverEdge(1); }
void IRAM_ATTR rx3() { onReceiverEdge(2); }
void IRAM_ATTR rx4() { onReceiverEdge(3); }
void IRAM_ATTR rx5() { onReceiverEdge(4); }
void IRAM_ATTR rx6() { onReceiverEdge(5); }

void setupReceiver() {
  void (*handlers[RX_COUNT])() = {rx1, rx2, rx3, rx4, rx5, rx6};
  for (int i = 0; i < RX_COUNT; i++) {
    pinMode(RX_PIN[i], INPUT); // GPIO34/35 have no internal pull resistors
    attachInterrupt(digitalPinToInterrupt(RX_PIN[i]), handlers[i], CHANGE);
  }
}

void readReceiver() {
  uint32_t p[RX_COUNT], raw[RX_COUNT], ts[RX_COUNT];
  uint32_t nEdge[RX_COUNT], nGood[RX_COUNT], nBad[RX_COUNT], nOrphan[RX_COUNT];
  // As tested: snapshot ISR data on the same core as the attached ISRs.
  // This masks local interrupts for only the short array copy, not a spinlock.
  noInterrupts();
  for (int i = 0; i < RX_COUNT; i++) {
    p[i] = pulseUs[i]; raw[i] = rawUs[i]; ts[i] = acceptedAtUs[i];
    nEdge[i] = edges[i]; nGood[i] = accepted[i];
    nBad[i] = badWidths[i]; nOrphan[i] = orphans[i];
  }
  interrupts();
  const uint32_t nowUs = micros(); // timestamp AFTER snapshot (no underflow race)
  uint8_t currentBad = 0;
  for (int i = 0; i < RX_COUNT; i++) {
    rxValue[i] = (uint16_t)p[i];
    rxRawValue[i] = (uint16_t)(raw[i] > 65535UL ? 65535UL : raw[i]);
    rxAgeUs[i] = nGood[i] ? (uint32_t)(nowUs - ts[i]) : UINT32_MAX;
    rxValid[i] = (nGood[i] && p[i] >= RX_MIN_US && p[i] <= RX_MAX_US &&
                  rxAgeUs[i] <= RX_TIMEOUT_US) ? 1 : 0;
    rxEdges[i] = nEdge[i]; rxAccepts[i] = nGood[i];
    rxOrphans[i] = nOrphan[i]; rxWidthBads[i] = nBad[i];
    rxRejects[i] = nBad[i] + nOrphan[i];
    if ((REQUIRED_RX_MASK & (1u << i)) && !rxValid[i]) currentBad |= (1u << i);
  }
  const bool nowHealthy = (currentBad == 0);
  if (receiverEverHealthy && receiverHealthy && !nowHealthy) {
    rxDropouts++;
    lastRxDropoutMs = millis();
    lastRxDropoutMask = currentBad;
    // No Serial I/O in the timing-critical receiver poll. Use telemetry.
  }
  if (nowHealthy) receiverEverHealthy = true;
  badChannelsMask = currentBad;
  receiverHealthy = nowHealthy;
  const uint32_t nowMs = millis();
  if (priorRxPollMs) {
    const uint32_t gap = (uint32_t)(nowMs - priorRxPollMs);
    if (gap > maxRxPollGapMs) maxRxPollGapMs = gap;
  }
  priorRxPollMs = nowMs;
}

// -------------------- 6. IMU --------------------
struct RawIMU {
  int16_t ax, ay, az, gx, gy, gz;
};

// Explicit prototype: prevents Arduino .ino preprocessing from moving a
// generated declaration above the RawIMU type definition.
bool readRawIMU(RawIMU &r);

bool imuConfigured = false;
bool sensorHealthy = false;
uint8_t imuIdentity = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;
float accX = 0, accY = 0, accZ = 0;
float rollDeg = 0, pitchDeg = 0;
uint16_t imuGoodStreak = 0;
uint32_t imuReadErrors = 0;

bool writeIMUReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readIMUReg(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)1) != 1) return false;
  int b = Wire.read();
  if (b < 0) return false;
  value = (uint8_t)b;
  return true;
}

bool configureIMU() {
  if (!readIMUReg(0x75, imuIdentity)) return false;
  if (imuIdentity != 0x68 && imuIdentity != 0x70) return false;
  if (!writeIMUReg(0x6B, 0x00)) return false;
  delay(100);
  if (!writeIMUReg(0x6B, 0x01)) return false; // PLL gyro clock
  if (!writeIMUReg(0x19, 0x03)) return false; // Sample rate divider
  if (!writeIMUReg(0x1A, 0x03)) return false; // DLPF
  if (!writeIMUReg(0x1B, 0x08)) return false; // +/-500 dps = 65.5 LSB/dps
  if (!writeIMUReg(0x1C, 0x10)) return false; // +/-8g = 4096 LSB/g
  uint8_t gyroCfg = 0, accelCfg = 0;
  if (!readIMUReg(0x1B, gyroCfg) || !readIMUReg(0x1C, accelCfg)) return false;
  return ((gyroCfg & 0x18) == 0x08) && ((accelCfg & 0x18) == 0x10);
}

bool readRawIMU(RawIMU &r) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) return false;
  uint8_t b[14];
  for (int i = 0; i < 14; i++) {
    int v = Wire.read();
    if (v < 0) return false;
    b[i] = (uint8_t)v;
  }
  r.ax = (int16_t)(((uint16_t)b[0] << 8) | b[1]);
  r.ay = (int16_t)(((uint16_t)b[2] << 8) | b[3]);
  r.az = (int16_t)(((uint16_t)b[4] << 8) | b[5]);
  r.gx = (int16_t)(((uint16_t)b[8] << 8) | b[9]);
  r.gy = (int16_t)(((uint16_t)b[10] << 8) | b[11]);
  r.gz = (int16_t)(((uint16_t)b[12] << 8) | b[13]);
  return true;
}

float accelRoll(float x, float y, float z) {
  return atan2f(y, sqrtf(x * x + z * z)) * 57.2957795f;
}
float accelPitch(float x, float y, float z) {
  return -atan2f(x, sqrtf(y * y + z * z)) * 57.2957795f;
}

bool calibrateGyro() {
  int64_t sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < 1000; i++) {
    RawIMU r;
    if (!readRawIMU(r)) return false;
    sx += r.gx; sy += r.gy; sz += r.gz;
    digitalWrite(BLUE_LED_PIN, ((i / 75) & 1) ? HIGH : LOW);
    delay(2); // Keep craft COMPLETELY still during calibration
  }
  gyroOffsetX = ((float)sx / 1000.0f) / 65.5f;
  gyroOffsetY = ((float)sy / 1000.0f) / 65.5f;
  gyroOffsetZ = ((float)sz / 1000.0f) / 65.5f;
  return true;
}

bool readIMU(float dt) {
  RawIMU r;
  if (!readRawIMU(r)) {
    imuReadErrors++;
    return false;
  }
  accX = (float)r.ax / 4096.0f;
  accY = (float)r.ay / 4096.0f;
  accZ = (float)r.az / 4096.0f;
  gyroX = (float)r.gx / 65.5f - gyroOffsetX;
  gyroY = (float)r.gy / 65.5f - gyroOffsetY;
  gyroZ = (float)r.gz / 65.5f - gyroOffsetZ;
  if (!isfinite(accX) || !isfinite(accY) || !isfinite(accZ) ||
      !isfinite(gyroX) || !isfinite(gyroY) || !isfinite(gyroZ)) return false;
  // Keep prior complementary filter proportions at nominal 250 Hz.
  // Exponential form adjusts the same time constant for loop jitter.
  const float alpha = expf(-dt * 2.2602f); // ~= 0.991 at 0.004 s
  rollDeg = alpha * (rollDeg + gyroX * dt) +
            (1.0f - alpha) * accelRoll(accX, accY, accZ);
  pitchDeg = alpha * (pitchDeg + gyroY * dt) +
             (1.0f - alpha) * accelPitch(accX, accY, accZ);
  return isfinite(rollDeg) && isfinite(pitchDeg);
}

// -------------------- 7. PID --------------------
struct PIDState {
  float integral = 0;
  float lastMeasurement = 0;
  float filteredDerivative = 0;
  bool initialized = false;
};

// These manual declarations keep the Arduino sketch preprocessor from
// generating prototypes before PIDState has been declared.
float anglePID(float targetDeg, float angle, float measuredRate, PIDState &pid, float dt, bool integrate);
float ratePID(float target, float measurement, PIDState &pid, float kp, float ki, float kd, float dt, bool integrate);

PIDState angleRollPID, anglePitchPID;
PIDState rateRollPID, ratePitchPID, rateYawPID;

void resetPID() {
  angleRollPID = PIDState(); anglePitchPID = PIDState();
  rateRollPID = PIDState(); ratePitchPID = PIDState();
  rateYawPID = PIDState();
}

float anglePID(float targetDeg, float angle, float measuredRate,
               PIDState &pid, float dt, bool integrate) {
  float error = targetDeg - angle;
  float newIntegral = pid.integral;
  if (integrate) newIntegral = constrain(newIntegral + ANGLE_KI * error * dt,
                                         -100.0f, 100.0f);
  float out = ANGLE_KP * error + newIntegral - ANGLE_KD * measuredRate;
  // Do not accumulate if the PID output is saturated further in same direction.
  if ((out > MAX_RATE_DPS && error > 0) ||
      (out < -MAX_RATE_DPS && error < 0)) {
    newIntegral = pid.integral;
    out = ANGLE_KP * error + newIntegral - ANGLE_KD * measuredRate;
  }
  pid.integral = newIntegral;
  return constrain(out, -MAX_RATE_DPS, MAX_RATE_DPS);
}

float ratePID(float target, float measurement, PIDState &pid,
              float kp, float ki, float kd, float dt, bool integrate) {
  float error = target - measurement;
  float derivative = 0;
  if (pid.initialized) derivative = (measurement - pid.lastMeasurement) / dt;
  pid.lastMeasurement = measurement;
  pid.initialized = true;
  pid.filteredDerivative = 0.7f * pid.filteredDerivative + 0.3f * derivative;
  float proposedIntegral = pid.integral;
  if (integrate) proposedIntegral = constrain(proposedIntegral + ki * error * dt,
                                              -200.0f, 200.0f);
  float out = kp * error + proposedIntegral - kd * pid.filteredDerivative;
  if ((out > 400.0f && error > 0) || (out < -400.0f && error < 0)) {
    proposedIntegral = pid.integral;
    out = kp * error + proposedIntegral - kd * pid.filteredDerivative;
  }
  pid.integral = proposedIntegral;
  return constrain(out, -400.0f, 400.0f);
}

// -------------------- 8. MOTORS --------------------
int motorOutput[4] = {ESC_MIN, ESC_MIN, ESC_MIN, ESC_MIN};
int shadowOutput[4] = {ESC_MIN, ESC_MIN, ESC_MIN, ESC_MIN};
bool mixerClipped = false;
bool pwmOutputInitialized = false;
uint32_t pwmCommandUpdates = 0;

void writeMotors(int a, int b, int c, int d) {
#if BENCH_LOCKOUT
  // Hardware safety interlock. Never allow a value above minimum to ESCs.
  a = b = c = d = ESC_MIN;
#endif
  // Once configured, the ESP32 PWM peripheral continues outputting the last
  // commanded pulse without calling Servo::writeMicroseconds every iteration.
  // Force the initial write immediately after attach; always write when changed.
  if (pwmOutputInitialized && motorOutput[0] == a && motorOutput[1] == b &&
      motorOutput[2] == c && motorOutput[3] == d) return;
  motorOutput[0] = a; motorOutput[1] = b;
  motorOutput[2] = c; motorOutput[3] = d;
  motor1.writeMicroseconds(a);
  motor2.writeMicroseconds(b);
  motor3.writeMicroseconds(c);
  motor4.writeMicroseconds(d);
  pwmOutputInitialized = true;
  pwmCommandUpdates++;
}

void stopMotors() {
  writeMotors(ESC_MIN, ESC_MIN, ESC_MIN, ESC_MIN);
  mixerClipped = false;
}

void initializeMotors() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  motor1.setPeriodHertz(ESC_FREQUENCY);
  motor2.setPeriodHertz(ESC_FREQUENCY);
  motor3.setPeriodHertz(ESC_FREQUENCY);
  motor4.setPeriodHertz(ESC_FREQUENCY);
  motor1.attach(MOTOR1_PIN, ESC_MIN, ESC_MAX);
  motor2.attach(MOTOR2_PIN, ESC_MIN, ESC_MAX);
  motor3.attach(MOTOR3_PIN, ESC_MIN, ESC_MAX);
  motor4.attach(MOTOR4_PIN, ESC_MIN, ESC_MAX);
  stopMotors();
}

void mixQuadX(float throttle, float roll, float pitch, float yaw) {
  // PRESERVED ORIGINAL MOTOR ORDER AND MIXER SIGNS.
  float cmd[4] = {
    throttle - roll - pitch - yaw, // M1 Front Right CCW
    throttle - roll + pitch + yaw, // M2 Rear Right CW
    throttle + roll + pitch - yaw, // M3 Rear Left CCW
    throttle + roll - pitch + yaw  // M4 Front Left CW
  };
  int out[4];
  mixerClipped = false;
  for (int i = 0; i < 4; i++) {
    if (cmd[i] < ESC_IDLE || cmd[i] > ESC_MAX) mixerClipped = true;
    out[i] = (int)lroundf(constrain(cmd[i], (float)ESC_IDLE, (float)ESC_MAX));
  }
  for (int i = 0; i < 4; i++) shadowOutput[i] = out[i];
#if BENCH_LOCKOUT
  // PID/mixer exercise only; no motor response to radio or attitude.
  // Use writeMotors directly to preserve the shadow mixerClipped flag.
  writeMotors(ESC_MIN, ESC_MIN, ESC_MIN, ESC_MIN);
#else
  writeMotors(out[0], out[1], out[2], out[3]);
#endif
}

// Exercise the original angle PID, rate PID and mixer without arming the ESCs.
// In BENCH_LOCKOUT, a synthetic 1350 us throttle is used for SHADOW outputs.
// Actual ESC outputs are forced to ESC_MIN independently in writeMotors().
void runControlCalculation(float dt, bool shadowBench) {
      const float desiredRoll = constrain((rxValue[0] - 1500) *
                  (MAX_ANGLE_DEG / 500.0f), -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
      const float desiredPitch = constrain((rxValue[1] - 1500) *
                  (MAX_ANGLE_DEG / 500.0f), -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
      const float desiredYawRate = constrain((rxValue[3] - 1500) *
                  (MAX_RATE_DPS / 500.0f), -MAX_RATE_DPS, MAX_RATE_DPS);
      const float throttle = shadowBench ? 1350.0f :
               constrain((float)rxValue[2], (float)ESC_IDLE, (float)ESC_MAX);

      bool integrate = !mixerClipped;
      float desiredRollRate = anglePID(desiredRoll, rollDeg, gyroX,
                                      angleRollPID, dt, integrate);
      float desiredPitchRate = anglePID(desiredPitch, pitchDeg, gyroY,
                                       anglePitchPID, dt, integrate);
      float rollCmd = ratePID(desiredRollRate, gyroX, rateRollPID,
                             RATE_RP_KP, RATE_RP_KI, RATE_RP_KD, dt, integrate);
      float pitchCmd = ratePID(desiredPitchRate, gyroY, ratePitchPID,
                              RATE_RP_KP, RATE_RP_KI, RATE_RP_KD, dt, integrate);
      float yawCmd = ratePID(desiredYawRate, gyroZ, rateYawPID,
                            RATE_Y_KP, RATE_Y_KI, RATE_Y_KD, dt, integrate);
      mixQuadX(throttle, rollCmd, pitchCmd, yawCmd);
}

// -------------------- 9. FAILSAFES / ARMING --------------------
enum Failsafe : uint8_t {
  FS_NONE = 0, FS_TIMING = 1, FS_RECEIVER = 2, FS_IMU = 3
};

// Manual declarations for functions accepting the custom Failsafe type.
const char* faultText(Failsafe f);
void triggerFailsafe(Failsafe reason);
Failsafe failsafe = FS_NONE;
Failsafe lastFailsafe = FS_NONE;
bool failsafeLatched = false;
uint8_t faultsSeenInLatch = 0;
uint32_t timingFaults = 0, receiverFaults = 0, imuFaults = 0;
bool armed = false;
bool armSwitchReady = false;
uint16_t timingGoodStreak = 0;

const char* faultText(Failsafe f) {
  switch (f) {
    case FS_TIMING: return "TIMING";
    case FS_RECEIVER: return "RECEIVER";
    case FS_IMU: return "IMU";
    default: return "NONE";
  }
}

void triggerFailsafe(Failsafe reason) {
  const bool wasLatched = failsafeLatched;
  uint8_t bit = (uint8_t)(1u << (uint8_t)reason);
  if (!(faultsSeenInLatch & bit)) {
    faultsSeenInLatch |= bit;
    if (reason == FS_TIMING) timingFaults++;
    if (reason == FS_RECEIVER) receiverFaults++;
    if (reason == FS_IMU) imuFaults++;
  }
  if (reason >= failsafe) failsafe = reason;
  lastFailsafe = reason;
  failsafeLatched = true;
  armed = false;
  armSwitchReady = false;
  // A latched fault must NOT reissue 8 redundant Servo writes per control loop.
  // The PWM hardware keeps sending ESC_MIN after the first command.
  if (!wasLatched) {
    resetPID();
    stopMotors();
  }
}

void clearFailsafe() {
  failsafe = FS_NONE;
  faultsSeenInLatch = 0;
  failsafeLatched = false;
  armed = false;
  armSwitchReady = true; // CH5 confirmed LOW during recovery
  resetPID();
  stopMotors();
}

// -------------------- 10. TIMING DIAGNOSTICS --------------------
uint32_t previousLoopStartUs = 0;
uint32_t loopIntervalUs = LOOP_PERIOD_US;
uint32_t maxLoopIntervalUs = 0;
uint32_t loopExecutionUs = 0;
uint32_t maxLoopExecutionUs = 0;
uint32_t executionOverruns = 0;
uint32_t rxWorkUs = 0, imuWorkUs = 0, controlWorkUs = 0;
uint32_t maxRxWorkUs = 0, maxImuWorkUs = 0, maxControlWorkUs = 0;

// -------------------- 10A. LAST BAD-GAP TRACE --------------------
// Record the PREVIOUS iteration's work and waits at the instant a gap is
// discovered. Capturing only a few integers avoids adding Serial I/O or
// expensive diagnostics to the timing-critical 250 Hz path.
uint32_t delay1LastUs = 0, busyWaitLastUs = 0;
uint32_t maxDelay1Us = 0;
uint32_t timingGapEvents = 0;
uint32_t lastGapAtSec = 0, lastGapIntervalUs = 0;
uint32_t lastGapWorkUs = 0, lastGapDelay1Us = 0;
uint32_t lastGapBusyUs = 0, lastGapOtherUs = 0;


// -------------------- 11. SHARED TELEMETRY --------------------
struct Telemetry {
  float roll, pitch, gyroRoll, gyroPitch, gyroYaw;
  bool armed, receiverOk, sensorOk, failsafeLatched, mixerClipped;
  uint8_t failsafe, lastFailsafe, badMask, whoAmI;
  uint16_t rx[6], raw[6];
  uint32_t rejects[6], edges[6], accepts[6];
  uint32_t orphans[6], widthBads[6], frameBads[6], resyncs[6];
  uint32_t age[6];
  uint8_t valid[6];
  int motor[4], shadowMotor[4];
  bool benchLockout;
  uint32_t rxDropouts, lastRxDropoutMs, maxRxPollGapMs;
  uint8_t lastRxDropoutMask;
  uint32_t intervalUs, maxIntervalUs, executionUs, maxExecutionUs, overruns;
  uint32_t rxFaults, imuFaults, timingFaults, imuErrors;
  uint32_t pwmWrites, rxWorkUs, imuWorkUs, controlWorkUs;
  uint32_t maxRxWorkUs, maxImuWorkUs, maxControlWorkUs;
  uint32_t resetReasonCode, uptimeSeconds;
  uint32_t tickPeriodMs, maxDelay1Us, timingGapEvents;
  uint32_t lastGapAtSec, lastGapIntervalUs, lastGapWorkUs;
  uint32_t lastGapDelay1Us, lastGapBusyUs, lastGapOtherUs;
};
Telemetry liveTelemetry = {};
portMUX_TYPE telemetryMux = portMUX_INITIALIZER_UNLOCKED;

void publishTelemetry() {
  Telemetry t = {};
  t.roll = rollDeg; t.pitch = pitchDeg;
  t.gyroRoll = gyroX; t.gyroPitch = gyroY; t.gyroYaw = gyroZ;
  t.armed = armed; t.receiverOk = receiverHealthy;
  t.sensorOk = sensorHealthy; t.failsafeLatched = failsafeLatched;
  t.mixerClipped = mixerClipped;
  t.failsafe = (uint8_t)failsafe;
  t.lastFailsafe = (uint8_t)lastFailsafe;
  t.badMask = badChannelsMask; t.whoAmI = imuIdentity;
  for (int i = 0; i < 6; i++) {
    t.rx[i] = rxValue[i]; t.raw[i] = rxRawValue[i];
    t.rejects[i] = rxRejects[i]; t.edges[i] = rxEdges[i];
    t.accepts[i] = rxAccepts[i];
    t.orphans[i] = rxOrphans[i]; t.widthBads[i] = rxWidthBads[i];
    t.frameBads[i] = rxFrameBads[i]; t.resyncs[i] = rxResyncs[i];
    t.age[i] = rxAgeUs[i]; t.valid[i] = rxValid[i];
  }
  for (int i = 0; i < 4; i++) {
    t.motor[i] = motorOutput[i];
    t.shadowMotor[i] = shadowOutput[i];
  }
  t.benchLockout = (BENCH_LOCKOUT != 0);
  t.rxDropouts = rxDropouts;
  t.lastRxDropoutMs = lastRxDropoutMs;
  t.lastRxDropoutMask = lastRxDropoutMask;
  t.maxRxPollGapMs = maxRxPollGapMs;
  t.intervalUs = loopIntervalUs; t.maxIntervalUs = maxLoopIntervalUs;
  t.executionUs = loopExecutionUs; t.maxExecutionUs = maxLoopExecutionUs;
  t.overruns = executionOverruns;
  t.rxFaults = receiverFaults; t.imuFaults = imuFaults;
  t.timingFaults = timingFaults; t.imuErrors = imuReadErrors;
  t.pwmWrites = pwmCommandUpdates;
  t.rxWorkUs = rxWorkUs; t.imuWorkUs = imuWorkUs;
  t.controlWorkUs = controlWorkUs;
  t.maxRxWorkUs = maxRxWorkUs; t.maxImuWorkUs = maxImuWorkUs;
  t.maxControlWorkUs = maxControlWorkUs;
  t.resetReasonCode = bootResetReasonCode;
  t.uptimeSeconds = millis() / 1000UL;
  t.tickPeriodMs = portTICK_PERIOD_MS;
  t.maxDelay1Us = maxDelay1Us;
  t.timingGapEvents = timingGapEvents;
  t.lastGapAtSec = lastGapAtSec;
  t.lastGapIntervalUs = lastGapIntervalUs;
  t.lastGapWorkUs = lastGapWorkUs;
  t.lastGapDelay1Us = lastGapDelay1Us;
  t.lastGapBusyUs = lastGapBusyUs;
  t.lastGapOtherUs = lastGapOtherUs;
  portENTER_CRITICAL(&telemetryMux);
  liveTelemetry = t;
  portEXIT_CRITICAL(&telemetryMux);
}

// -------------------- 12. DASHBOARD PAGE --------------------
static const char DASHBOARD_HTML[] PROGMEM = R"DRONE_HTML(
<!DOCTYPE html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 Quad-X | Live Telemetry</title>
<style>
:root{color-scheme:dark;--bg:#080e19;--panel:#121e31;--line:#263751;--ink:#e7f0ff;--muted:#90a5c0;--green:#4ce0aa;--red:#ff667c;--yellow:#ffc56c;--blue:#63b8ff}
*{box-sizing:border-box}body{margin:0;padding:22px;font:14px system-ui,-apple-system,Segoe UI,sans-serif;background:var(--bg);color:var(--ink)}
.wrap{max-width:1140px;margin:auto}header{display:flex;align-items:center;justify-content:space-between;gap:12px;flex-wrap:wrap;margin-bottom:16px}h1{font-size:25px;letter-spacing:.2px;margin:0}h2{font-size:14px;color:#c2d8f3;text-transform:uppercase;letter-spacing:1.2px;margin:0 0 12px}.sub{color:var(--muted);margin-top:5px}.tag{border:1px solid var(--line);border-radius:50px;padding:9px 13px;color:var(--yellow)}.grid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:12px}.card{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:17px;min-width:0}.wide{grid-column:span 2}.all{grid-column:1/-1}.value{font-size:26px;font-weight:700;font-variant-numeric:tabular-nums}.tiny{font-size:12px;color:var(--muted);margin-top:6px}.ok{color:var(--green)}.bad{color:var(--red)}.warn{color:var(--yellow)}.blue{color:var(--blue)}.rxrow{display:grid;grid-template-columns:80px minmax(30px,1fr) 88px 70px 74px 65px 150px;gap:8px;align-items:center;padding:9px 0;border-bottom:1px solid var(--line)}.rxrow:last-child{border:0}.rxrow span{font-variant-numeric:tabular-nums}.bar{height:7px;border-radius:6px;background:#23374e;overflow:hidden}.fill{height:100%;width:0;background:var(--blue);transition:width .18s}.motors{display:grid;grid-template-columns:repeat(4,1fr);gap:8px}.mot{background:#0b1628;padding:13px;border-radius:9px;text-align:center}.mot strong{font-size:22px;display:block;margin-top:7px;font-variant-numeric:tabular-nums}.faults{display:grid;grid-template-columns:repeat(4,1fr);gap:10px}.faults div{padding:12px;background:#0b1628;border-radius:9px}.faults strong{display:block;font-size:24px;font-variant-numeric:tabular-nums;margin-top:4px}.info{color:var(--muted);line-height:1.5}.status{font-weight:700}.metricline{display:flex;gap:14px;flex-wrap:wrap}.metricline div{flex:1;min-width:110px}@media(max-width:850px){.grid{grid-template-columns:repeat(2,minmax(0,1fr))}.wide{grid-column:span 2}}@media(max-width:520px){body{padding:11px}.grid{grid-template-columns:1fr}.wide,.all{grid-column:span 1}.rxrow{grid-template-columns:58px 69px 54px 62px 49px 108px;gap:4px;font-size:11px}.rxrow .bar{display:none}.rxrow span{overflow:hidden}.motors,.faults{grid-template-columns:repeat(2,1fr)}}
</style></head><body><div class="wrap">
<header><div><h1>ESP32 <span class="blue">Quad-X</span> Telemetry</h1><div class="sub" id="modeSub">Stage E candidate · bench lockout enabled · 5 Hz refresh</div></div><div id="link" class="tag">CONNECTING...</div></header>
<div class="grid">
<section class="card"><h2>Arm status</h2><div id="armed" class="value">--</div><div class="tiny" id="armhint">BENCH_LOCKOUT=1: Physical arming disabled</div></section>
<section class="card"><h2>Receiver</h2><div id="receiver" class="value">--</div><div class="tiny">Dropouts: <span id="rxDrops">--</span> · Last: <span id="lastRxDrop">--</span> · Poll gap max: <span id="rxGap">--</span>ms</div></section>
<section class="card"><h2>IMU</h2><div id="imu" class="value">--</div><div class="tiny">WHO_AM_I: <span id="who">--</span></div></section>
<section class="card"><h2>Current failsafe</h2><div id="fault" class="value">--</div><div class="tiny">Last fault: <span id="last">--</span></div></section>
<section class="card wide"><h2>Attitude</h2><div class="metricline"><div><div class="value"><span id="roll">--</span>°</div><div class="tiny">Roll</div></div><div><div class="value"><span id="pitch">--</span>°</div><div class="tiny">Pitch</div></div></div><div class="tiny">Check tilt directions with props removed</div></section>
<section class="card wide"><h2>Gyroscope (°/s)</h2><div class="metricline"><div><div class="value" id="gx">--</div><div class="tiny">Roll rate</div></div><div><div class="value" id="gy">--</div><div class="tiny">Pitch rate</div></div><div><div class="value" id="gz">--</div><div class="tiny">Yaw rate</div></div></div></section>
<section class="card all"><h2>Receiver pulse widths · age and diagnostics</h2><div class="tiny" style="margin-bottom:7px">Accepted pulse (µs), age (ms), last raw high time (µs), rejected edges/pulses, interrupt edges, and accepted pulses. Counters increase from boot; accepted values expire after 100 ms.</div><div id="rxRows"></div><div id="badList" class="tiny">--</div><div id="badReasons" class="tiny">--</div></section>
<section class="card wide"><h2 id="escTitle">ESC outputs (µs): FORCED 1000</h2><div class="motors"><div class="mot">M1 · FR<strong id="m0">1000</strong></div><div class="mot">M2 · RR<strong id="m1">1000</strong></div><div class="mot">M3 · RL<strong id="m2">1000</strong></div><div class="mot">M4 · FL<strong id="m3">1000</strong></div></div><div class="tiny"><span id="shadowCaption">Shadow PID/mixer (NOT sent to ESCs)</span>: <span id="shadow">--</span> · Mixer clipped: <span id="clip">--</span></div></section>
<section class="card wide"><h2>Loop timing (µs)</h2><div class="metricline"><div><div class="value" id="interval">--</div><div class="tiny">Start-to-start interval</div></div><div><div class="value" id="execution">--</div><div class="tiny">Work execution time</div></div><div><div class="value" id="maxInterval">--</div><div class="tiny">Maximum interval</div></div><div><div class="value" id="maxExecution">--</div><div class="tiny">Maximum execution</div></div></div><div class="tiny">Target 4000µs · timing fault if interval exceeds 10000µs · overruns <span id="overruns">--</span></div><div class="tiny">Work stages: RX <span id="rxwork">--</span>µs / IMU <span id="imuwork">--</span>µs / Control <span id="ctlwork">--</span>µs</div><div class="tiny">Maximum: RX <span id="maxrxwork">--</span>µs / IMU <span id="maximuwork">--</span>µs / Control <span id="maxctlwork">--</span>µs · PWM updates <span id="pwmwrites">--</span></div></section>
<section class="card all"><h2>Timing-gap trace</h2><div class="tiny">Read-only diagnostics. Stores the most recent loop start-to-start interval over 10,000 µs. Values refer to the PREVIOUS loop. No safety threshold changes.</div><div class="metricline" style="margin-top:10px"><div><div class="value" id="tickms">--</div><div class="tiny">FreeRTOS tick (ms)</div></div><div><div class="value" id="maxdelay1">--</div><div class="tiny">Max delay(1) duration (µs)</div></div><div><div class="value" id="gapevents">--</div><div class="tiny">Total long-gap events</div></div><div><div class="value" id="lastgapsec">--</div><div class="tiny">Most recent event (uptime s)</div></div></div><div class="tiny" id="lastgapdetails" style="margin-top:10px">No long interval recorded</div></section>
<section class="card all"><h2>Reset diagnostics</h2><div class="metricline"><div><div class="value" id="resetCause">--</div><div class="tiny">Last ESP32 reset reason</div></div><div><div class="value" id="uptime">--</div><div class="tiny">Uptime since current boot (seconds)</div></div></div><p class="info">If uptime drops and receiver counters restart, the ESP32 rebooted. BROWNOUT suggests power instability; TASK WDT / INT WDT suggests a watchdog reset; PANIC suggests a crash. Keep propellers removed.</p></section>
<section class="card all"><h2>Fault counters</h2><div class="faults"><div>Receiver<strong id="rf">0</strong></div><div>IMU failsafe<strong id="if">0</strong></div><div>Timing<strong id="tf">0</strong></div><div>IMU read errors<strong id="ie">0</strong></div></div><p class="info">Faults are latched until RC CH5 is LOW with throttle LOW and inputs recovered. Any recorded timing fault inhibits new arming until reboot. This is an unvalidated flight-control candidate; keep propellers removed.</p></section>
</div></div>
<script>
const $=id=>document.getElementById(id);
const names=['Roll','Pitch','Throttle','Yaw','Arm','Aux'];
const fNames=['NONE','TIMING','RECEIVER','IMU'];
const resetNames={0:'UNKNOWN',1:'POWER ON',2:'EXTERNAL',3:'SOFTWARE',4:'PANIC',5:'INT WDT',6:'TASK WDT',7:'WDT',8:'DEEP SLEEP',9:'BROWNOUT',10:'SDIO',11:'USB',12:'JTAG',13:'EFUSE',14:'POWER GLITCH',15:'CPU LOCKUP'};
for(let i=0;i<6;i++) $('rxRows').insertAdjacentHTML('beforeend',`<div class="rxrow"><span>CH${i+1} ${names[i]}</span><div class="bar"><div id="b${i}" class="fill"></div></div><span id="v${i}">-- µs</span><span id="a${i}">--</span><span id="rw${i}" title="Last measured high width">Raw --</span><span id="rj${i}" title="Rejects (orphan/width/frame/resync)">Bad 0</span><span id="ed${i}" title="Interrupt edge count">Edges 0 / OK 0</span></div>`);
let lastGood=0;
async function tick(){
  try{
    const r=await fetch('/telemetry',{cache:'no-store'});if(!r.ok)throw Error(r.status);
    const d=await r.json();lastGood=Date.now();$('link').textContent='LIVE';$('link').className='tag ok';
    const set=(id,v,cls='')=>{const e=$(id);e.textContent=v;e.className=cls;};
    set('armed',d.benchLockout?'BENCH LOCKED':(d.armed?'ARMED':(d.timingFaults?'ARM BLOCKED':'DISARMED')),d.benchLockout?'value warn':(d.armed?'value warn':(d.timingFaults?'value bad':'value')));
    set('modeSub',d.benchLockout?'Stage E candidate · BENCH LOCKED · ESC=1000 µs · 5 Hz':'UNVALIDATED CONTROL MODE · PHYSICAL ESC OUTPUT ENABLED · 5 Hz');
    set('armhint',d.benchLockout?'Arming disabled; physical outputs forced to 1000 µs':(d.timingFaults?'Timing fault on this boot: arming inhibited until reboot':'CH5 LOW then HIGH with low throttle; NOT flight validated'));
    set('escTitle',d.benchLockout?'ESC outputs (µs): FORCED 1000':'ESC outputs (µs): PHYSICAL OUTPUTS ENABLED');
    set('shadowCaption',d.benchLockout?'Shadow PID/mixer (NOT sent to ESCs)':'PID/mixer calculation (sent only when safely armed)');
    set('rxDrops',d.rxDropouts); set('lastRxDrop',d.lastRxDropoutMs?d.lastRxDropoutMs+'ms (0x'+d.lastRxDropoutMask.toString(16)+')':'None'); set('rxGap',d.maxRxPollGapMs);
    set('receiver',d.receiverOk?'HEALTHY':'INVALID',d.receiverOk?'value ok':'value bad');
    set('imu',d.sensorOk?'HEALTHY':'ERROR',d.sensorOk?'value ok':'value bad');
    set('who','0x'+Number(d.whoAmI).toString(16).toUpperCase().padStart(2,'0'));
    set('fault',fNames[d.failsafe]||'UNKNOWN',d.failsafe?'value bad':'value ok');
    set('last',fNames[d.lastFailsafe]||'UNKNOWN');
    set('resetCause',(resetNames[d.resetReasonCode]||'OTHER')+' ('+d.resetReasonCode+')');
    set('uptime',d.uptimeSeconds);
    set('tickms',d.tickPeriodMs);
    set('maxdelay1',d.maxDelay1Us);
    set('gapevents',d.timingGapEvents);
    set('lastgapsec',d.timingGapEvents?d.lastGapAtSec:'None');
    set('lastgapdetails',d.timingGapEvents?('Last long interval: '+d.lastGapIntervalUs+' µs | previous loop work: '+d.lastGapWorkUs+' µs | delay(1): '+d.lastGapDelay1Us+' µs | busy-wait: '+d.lastGapBusyUs+' µs | other/scheduling: '+d.lastGapOtherUs+' µs'):'No long interval recorded');
    ['roll','pitch','gx','gy','gz'].forEach((id,i)=>set(id,Number([d.roll,d.pitch,d.gyroRoll,d.gyroPitch,d.gyroYaw][i]).toFixed(2)));
    for(let i=0;i<6;i++){
      set('v'+i,d.rx[i]+' µs',d.valid[i]?'ok':'bad');
      set('a'+i,d.age[i]===4294967295?'NEVER':(d.age[i]/1000).toFixed(0)+'ms',d.valid[i]?'ok':'bad');
      set('rw'+i,'Raw '+d.raw[i]);
      set('rj'+i,'Bad '+d.rejects[i]);
      set('ed'+i,'Edges '+d.edges[i]+' / OK '+d.accepts[i]);
      $('b'+i).style.width=Math.max(0,Math.min(100,((d.rx[i]-900)/1200)*100))+'%';
      $('b'+i).style.background=d.valid[i]?'var(--blue)':'var(--red)';
    }
    const bad=[];for(let i=0;i<6;i++)if((d.badMask>>i)&1)bad.push('CH'+(i+1));
    set('badList',bad.length?'INVALID / STALE: '+bad.join(', '):'All required receiver channels fresh');
    set('badReasons','Reject reasons (Orphan / Width / Frame / Resync) by channel: '+d.orphans.map((v,i)=>'CH'+(i+1)+' '+v+'/'+d.widthBads[i]+'/'+d.frameBads[i]+'/'+d.resyncs[i]).join(' · '));
    for(let i=0;i<4;i++)set('m'+i,d.motors[i]);
    set('shadow',d.shadowMotors.join(' / ')+' µs');
    set('clip',d.mixerClipped?'YES':'NO');
    [['interval','intervalUs'],['execution','executionUs'],['maxInterval','maxIntervalUs'],['maxExecution','maxExecutionUs'],['overruns','overruns'],['rf','rxFaults'],['if','imuFaults'],['tf','timingFaults'],['ie','imuErrors'],['pwmwrites','pwmWrites'],['rxwork','rxWorkUs'],['imuwork','imuWorkUs'],['ctlwork','controlWorkUs'],['maxrxwork','maxRxWorkUs'],['maximuwork','maxImuWorkUs'],['maxctlwork','maxControlWorkUs']].forEach(([id,key])=>set(id,d[key]));
  }catch(e){if(Date.now()-lastGood>1000){$('link').textContent='OFFLINE';$('link').className='tag bad';}}
}
setInterval(tick,200);tick();
</script></body></html>
)DRONE_HTML";

// -------------------- 13. HTTP / WIFI TASK --------------------
void serveDashboard() { server.send_P(200, "text/html", DASHBOARD_HTML); }

void addBool(String &s, bool v) { s += (v ? "true" : "false"); }
void serveTelemetry() {
  Telemetry t;
  portENTER_CRITICAL(&telemetryMux);
  t = liveTelemetry;
  portEXIT_CRITICAL(&telemetryMux);

  String s;
  s.reserve(1850);
  s += '{';
  s += "\"roll\":"; s += String(t.roll, 2);
  s += ",\"pitch\":"; s += String(t.pitch, 2);
  s += ",\"gyroRoll\":"; s += String(t.gyroRoll, 2);
  s += ",\"gyroPitch\":"; s += String(t.gyroPitch, 2);
  s += ",\"gyroYaw\":"; s += String(t.gyroYaw, 2);
  s += ",\"armed\":"; addBool(s, t.armed);
  s += ",\"receiverOk\":"; addBool(s, t.receiverOk);
  s += ",\"sensorOk\":"; addBool(s, t.sensorOk);
  s += ",\"mixerClipped\":"; addBool(s, t.mixerClipped);
  s += ",\"failsafeLatched\":"; addBool(s, t.failsafeLatched);
  s += ",\"failsafe\":"; s += (unsigned int)t.failsafe;
  s += ",\"lastFailsafe\":"; s += (unsigned int)t.lastFailsafe;
  s += ",\"badMask\":"; s += (unsigned int)t.badMask;
  s += ",\"whoAmI\":"; s += (unsigned int)t.whoAmI;
  s += ",\"rx\":[";
  for (int i=0;i<6;i++) { if (i) s+=','; s+=t.rx[i]; }
  s += "],\"raw\":[";
  for (int i=0;i<6;i++) { if (i) s+=','; s+=t.raw[i]; }
  s += "],\"rejects\":[";
  for (int i=0;i<6;i++) { if (i) s+=','; s+=t.rejects[i]; }
  s += "],\"orphans\":[";
  for (int i=0;i<6;i++) { if (i) s+=','; s+=t.orphans[i]; }
  s += "],\"widthBads\":[";
  for (int i=0;i<6;i++) { if (i) s+=','; s+=t.widthBads[i]; }
  s += "],\"frameBads\":[";
  for (int i=0;i<6;i++) { if (i) s+=','; s+=t.frameBads[i]; }
  s += "],\"resyncs\":[";
  for (int i=0;i<6;i++) { if (i) s+=','; s+=t.resyncs[i]; }
  s += "],\"edges\":[";
  for (int i=0;i<6;i++) { if (i) s+=','; s+=t.edges[i]; }
  s += "],\"accepts\":[";
  for (int i=0;i<6;i++) { if (i) s+=','; s+=t.accepts[i]; }
  s += "],\"age\":[";
  for (int i=0;i<6;i++) { if (i) s+=','; s+=t.age[i]; }
  s += "],\"valid\":[";
  for (int i=0;i<6;i++) { if (i) s+=','; s+=(unsigned int)t.valid[i]; }
  s += "],\"motors\":[";
  for (int i=0;i<4;i++) { if (i) s+=','; s+=t.motor[i]; }
  s += "]";
  s += ",\"shadowMotors\":[";
  for (int i=0;i<4;i++) { if (i) s+=','; s+=t.shadowMotor[i]; }
  s += ']';
  s += ",\"benchLockout\":"; addBool(s, t.benchLockout);
  s += ",\"rxDropouts\":"; s+=t.rxDropouts;
  s += ",\"lastRxDropoutMs\":"; s+=t.lastRxDropoutMs;
  s += ",\"lastRxDropoutMask\":"; s+=(unsigned int)t.lastRxDropoutMask;
  s += ",\"maxRxPollGapMs\":"; s+=t.maxRxPollGapMs;
  s += ",\"intervalUs\":"; s+=t.intervalUs;
  s += ",\"maxIntervalUs\":"; s+=t.maxIntervalUs;
  s += ",\"executionUs\":"; s+=t.executionUs;
  s += ",\"maxExecutionUs\":"; s+=t.maxExecutionUs;
  s += ",\"overruns\":"; s+=t.overruns;
  s += ",\"rxFaults\":"; s+=t.rxFaults;
  s += ",\"imuFaults\":"; s+=t.imuFaults;
  s += ",\"timingFaults\":"; s+=t.timingFaults;
  s += ",\"imuErrors\":"; s+=t.imuErrors;
  s += ",\"pwmWrites\":"; s+=t.pwmWrites;
  s += ",\"rxWorkUs\":"; s+=t.rxWorkUs;
  s += ",\"imuWorkUs\":"; s+=t.imuWorkUs;
  s += ",\"controlWorkUs\":"; s+=t.controlWorkUs;
  s += ",\"maxRxWorkUs\":"; s+=t.maxRxWorkUs;
  s += ",\"maxImuWorkUs\":"; s+=t.maxImuWorkUs;
  s += ",\"maxControlWorkUs\":"; s+=t.maxControlWorkUs;
  s += ",\"resetReasonCode\":"; s+=t.resetReasonCode;
  s += ",\"uptimeSeconds\":"; s+=t.uptimeSeconds;
  s += ",\"tickPeriodMs\":"; s+=t.tickPeriodMs;
  s += ",\"maxDelay1Us\":"; s+=t.maxDelay1Us;
  s += ",\"timingGapEvents\":"; s+=t.timingGapEvents;
  s += ",\"lastGapAtSec\":"; s+=t.lastGapAtSec;
  s += ",\"lastGapIntervalUs\":"; s+=t.lastGapIntervalUs;
  s += ",\"lastGapWorkUs\":"; s+=t.lastGapWorkUs;
  s += ",\"lastGapDelay1Us\":"; s+=t.lastGapDelay1Us;
  s += ",\"lastGapBusyUs\":"; s+=t.lastGapBusyUs;
  s += ",\"lastGapOtherUs\":"; s+=t.lastGapOtherUs;
  s += '}';
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", s);
}

void wifiServerTask(void *parameter) {
  (void)parameter;
  for (;;) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void startWiFiDashboard() {
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(AP_SSID, AP_PASS)) {
    Serial.println("WiFi AP failed; flight control stays DISARMED until verified.");
    return;
  }
  server.on("/", HTTP_GET, serveDashboard);
  server.on("/telemetry", HTTP_GET, serveTelemetry);
  server.onNotFound([](){ server.send(404, "text/plain", "Not found"); });
  server.begin();
  BaseType_t created = xTaskCreatePinnedToCore(
    wifiServerTask, "WiFiServer", 8192, nullptr, 1, nullptr, 0);
  Serial.printf("WiFi: %s  http://%s  task=%s\n", AP_SSID,
                WiFi.softAPIP().toString().c_str(),
                created == pdPASS ? "OK" : "FAILED");
}

// -------------------- 14. LED LOGIC --------------------
void updateLEDs() {
  uint32_t ms = millis();
  if (failsafeLatched) {
    digitalWrite(BLUE_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, (failsafe == FS_IMU) ? HIGH : ((ms / 180) & 1));
  } else {
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(BLUE_LED_PIN, armed ? ((ms / 220) & 1) : HIGH);
  }
}

// -------------------- 15. SETUP --------------------
void setup() {
  Serial.begin(115200);
  bootResetReasonCode = (uint32_t)esp_reset_reason();
  Serial.printf("\n==== ESP32 BOOT: reset reason code=%lu ====\n",
                (unsigned long)bootResetReasonCode);
  Serial.println("POWER ON=1, SOFTWARE=3, PANIC=4, INT WDT=5, TASK WDT=6, WDT=7, BROWNOUT=9");
  pinMode(BLUE_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  for (int i = 0; i < 3; i++) {
    digitalWrite(BLUE_LED_PIN, HIGH); digitalWrite(RED_LED_PIN, HIGH);
    delay(120);
    digitalWrite(BLUE_LED_PIN, LOW); digitalWrite(RED_LED_PIN, LOW);
    delay(120);
  }

  // Always command all ESCs to minimum from the beginning.
  initializeMotors();
  // Attach and read receiver ISR on the Arduino loop core, as in bench A/B/C.
  Serial.printf("Arduino setup core: %d\n", xPortGetCoreID());
  setupReceiver();

  if (!Wire.begin(SDA_PIN, SCL_PIN, 400000)) {
    Serial.println("I2C initialization failed");
  } else {
    Wire.setTimeOut(5); // Avoid a 50 ms I2C stall masquerading as normal loop
    imuConfigured = configureIMU();
  }

  if (imuConfigured) {
    Serial.printf("IMU WHO_AM_I = 0x%02X; calibrating gyro. KEEP STILL.\n", imuIdentity);
    imuConfigured = calibrateGyro();
  }
  if (imuConfigured) {
    RawIMU r;
    imuConfigured = readRawIMU(r);
    if (imuConfigured) {
      float ax = r.ax/4096.0f, ay = r.ay/4096.0f, az = r.az/4096.0f;
      rollDeg = accelRoll(ax, ay, az);
      pitchDeg = accelPitch(ax, ay, az);
      sensorHealthy = true;
      imuGoodStreak = IMU_RECOVERY_GOOD_LOOPS;
      Serial.printf("Initial angles roll=%.2f pitch=%.2f\n", rollDeg, pitchDeg);
    }
  }
  if (!imuConfigured) {
    Serial.println("IMU startup failure: reboot after fixing sensor/calibration.");
    triggerFailsafe(FS_IMU);
  }

  stopMotors();
  publishTelemetry();
  startWiFiDashboard();
  previousLoopStartUs = micros();
#if BENCH_LOCKOUT
  Serial.println("STAGE E CANDIDATE: BENCH LOCKED. PID in shadow, ESC=1000 us.");
#else
  Serial.println("WARNING: UNVALIDATED PHYSICAL MOTOR CONTROL IS ENABLED.");
  Serial.println("DO NOT FLY UNTIL ALL TIMING, AXIS, MOTOR AND FAILSAFE TESTS PASS.");
#endif
  Serial.println("REMOVE ALL PROPELLERS DURING INITIAL VALIDATION.");
  Serial.printf("FreeRTOS tick period: %lu ms\n", (unsigned long)portTICK_PERIOD_MS);
}

// -------------------- 16. 250 HZ MAIN CONTROL LOOP --------------------
void loop() {
  const uint32_t started = micros();
  loopIntervalUs = (uint32_t)(started - previousLoopStartUs);
  previousLoopStartUs = started;
  if (loopIntervalUs > maxLoopIntervalUs) maxLoopIntervalUs = loopIntervalUs;
  if (loopIntervalUs > MAX_LOOP_GAP_US) {
    timingGapEvents++;
    lastGapAtSec = millis() / 1000UL;
    lastGapIntervalUs = loopIntervalUs;
    lastGapWorkUs = loopExecutionUs; // last completed loop, before waiting
    lastGapDelay1Us = delay1LastUs;
    lastGapBusyUs = busyWaitLastUs;
    const uint32_t accounted = lastGapWorkUs + lastGapDelay1Us + lastGapBusyUs;
    lastGapOtherUs = loopIntervalUs > accounted ? loopIntervalUs - accounted : 0;
  }

  bool timingOk = loopIntervalUs <= MAX_LOOP_GAP_US;
  if (timingOk) {
    if (timingGoodStreak < TIMING_RECOVERY_GOOD_LOOPS) timingGoodStreak++;
  } else {
    timingGoodStreak = 0;
  }

  float dt = constrain(loopIntervalUs * 0.000001f, 0.001f, 0.010f);
  const uint32_t rxStageStartUs = micros();
  readReceiver();
  rxWorkUs = (uint32_t)(micros() - rxStageStartUs);
  if (rxWorkUs > maxRxWorkUs) maxRxWorkUs = rxWorkUs;

  const uint32_t imuStageStartUs = micros();
  // CRITICAL FIX: keep reading the IMU even after a transient read failure.
  if (imuConfigured) {
    sensorHealthy = readIMU(dt);
    if (sensorHealthy) {
      if (imuGoodStreak < IMU_RECOVERY_GOOD_LOOPS) imuGoodStreak++;
    } else {
      imuGoodStreak = 0;
    }
  } else {
    sensorHealthy = false;
    imuGoodStreak = 0;
  }

  imuWorkUs = (uint32_t)(micros() - imuStageStartUs);
  if (imuWorkUs > maxImuWorkUs) maxImuWorkUs = imuWorkUs;
  const uint32_t controlStageStartUs = micros();

  if (!receiverHealthy) triggerFailsafe(FS_RECEIVER);
  if (!timingOk) triggerFailsafe(FS_TIMING);
  if (!sensorHealthy) triggerFailsafe(FS_IMU);
  // Fault clears ONLY after the input and sensor signals recover, CH5 is LOW,
  // and throttle is LOW. A fresh LOW -> HIGH CH5 transition is required to arm.
  // Even after recovery, any timingFaults>0 inhibits arming for this boot.
  if (failsafeLatched && receiverHealthy && sensorHealthy &&
      imuGoodStreak >= IMU_RECOVERY_GOOD_LOOPS &&
      timingGoodStreak >= TIMING_RECOVERY_GOOD_LOOPS &&
      rxValue[4] < 1300 && rxValue[2] < THROTTLE_CUTOFF) {
    clearFailsafe();
  }

#if BENCH_LOCKOUT
  // Keep the original full-rate attitude/PID/mixer computation running in
  // SHADOW mode. No arming ever, and writeMotors() always enforces 1000us.
  armed = false;
  armSwitchReady = false;
  if (!failsafeLatched && receiverHealthy && sensorHealthy && timingOk) {
    runControlCalculation(dt, true);
  } else {
    resetPID();
    mixerClipped = false;
    stopMotors();
  }
#else
  if (!failsafeLatched) {
    const bool armLow = rxValue[4] < 1300;
    const bool armHigh = rxValue[4] > 1700;
    const bool throttleLow = rxValue[2] < THROTTLE_CUTOFF;
    if (armLow) {
      if (armed) resetPID();
      armed = false;
      armSwitchReady = true;
    } else if (armHigh && !armed && armSwitchReady && throttleLow &&
               timingFaults == 0) {
      // Conservative pre-arm inhibit: once a timing fault has occurred during
      // this boot, motor arming is blocked until the cause is fixed and the
      // controller is restarted. Never erase historical fault counts here.
      resetPID();
      armed = true;
      armSwitchReady = false;
    } else if (!armHigh) {
      // Mid-position is not armed; don't accidentally arm on ambiguous switch PWM.
      if (armed) resetPID();
      armed = false;
    }

    if (!armed || throttleLow) {
      stopMotors();
      resetPID();
    } else {
      runControlCalculation(dt, false);
    }
  } else {
    stopMotors();
  }
#endif // BENCH_LOCKOUT

  updateLEDs();
  controlWorkUs = (uint32_t)(micros() - controlStageStartUs);
  if (controlWorkUs > maxControlWorkUs) maxControlWorkUs = controlWorkUs;
  // Telemetry snapshot at 20 Hz; HTTP requests arrive at ~5 Hz.
  // Reported execution time is previous iteration's completed work duration.
  static uint32_t lastPublishUs = 0;
  if ((uint32_t)(started - lastPublishUs) >= 50000UL) {
    lastPublishUs = started;
    publishTelemetry();
  }
  loopExecutionUs = (uint32_t)(micros() - started);
  if (loopExecutionUs > maxLoopExecutionUs) maxLoopExecutionUs = loopExecutionUs;
  if (loopExecutionUs > LOOP_PERIOD_US) executionOverruns++;

  // Period measured from START of this iteration. Never 'catch up' by
  // running successive loops with artificially short dt after an overrun.
  uint32_t elapsed = (uint32_t)(micros() - started);
  // Allow the Arduino-core scheduler to run at least once when time permits.
  // Fine-grained wait afterward maintains the nominal 4 ms target.
  // Preserve the tested Stage D-T scheduler/wait strategy, which still had
  // a measured 11+ ms scheduling gap. We are NOT claiming a timing fix.
  const uint32_t beforeDelay1 = micros();
  if (elapsed + 1800UL < LOOP_PERIOD_US) delay(1);
  delay1LastUs = (uint32_t)(micros() - beforeDelay1);
  if (delay1LastUs > maxDelay1Us) maxDelay1Us = delay1LastUs;
  elapsed = (uint32_t)(micros() - started);
  const uint32_t beforeBusyWait = micros();
  if (elapsed < LOOP_PERIOD_US) delayMicroseconds(LOOP_PERIOD_US - elapsed);
  busyWaitLastUs = (uint32_t)(micros() - beforeBusyWait);
}