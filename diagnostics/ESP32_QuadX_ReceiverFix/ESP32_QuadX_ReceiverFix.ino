/*
  ESP32 QUAD-X FLIGHT CONTROLLER + READ-ONLY WIFI TELEMETRY
  Revision: filtered six-channel ISR receiver + timing / IMU diagnostics

  !!! BENCH TEST ONLY -- REMOVE ALL PROPELLERS !!!
  Not flight validated. Check receiver, IMU axes, motor order, motor direction,
  ESC compatibility, power, and failsafes before considering any flight test.

  AP: ESP32-DRONE   Password: drone1234   URL: http://192.168.4.1

  Preserved from previous sketch:
    Pins, 250 Hz loop, 500 Hz ESC command frequency, Quad-X mixer,
    original PID gains, 100 ms RX timeout, 1000/1170/1800 us motor limits,
    low->high CH5 arming with throttle low.

  Deliberate corrections:
    - Reject invalid/noisy receiver edges without replacing the last good PWM pulse
    - Report per-channel raw width, accepted width, edge counts and rejects
    - IMU write/read verification and continued reads after a transient failure
    - Explicit failsafe latching and deliberate low-switch recovery
    - True loop interval, execution time, max timing, overrun counts
    - Saturation-aware PID integral freeze and stop-on-low-throttle reset
    - Dashboard retains the LAST fault after the current latch clears
*/

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>
#include <driver/gpio.h>

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

// -------------------- 5. RECEIVER ISR STATE --------------------
// ESP32-WROOM: 6 independent CHANGE interrupts; no pulseIn() blocking in loop.
// Each interrupt saves the last complete valid PWM pulse for control.
// Invalid widths and extra edges are counted, but DO NOT overwrite that pulse.
portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t rxRiseUs[6] = {0};
volatile uint16_t rxPulseUs[6] = {0};       // last ACCEPTED pulse width
volatile uint16_t rxRawUs[6] = {0};         // last observed full high interval
volatile uint32_t rxLastUs[6] = {0};        // timestamp of last accepted pulse
volatile uint32_t rxRejectCount[6] = {0};
volatile uint32_t rxEdgeCount[6] = {0};
volatile uint8_t rxHasPulse[6] = {0};
volatile uint8_t rxHighSeen[6] = {0};

uint16_t rxValue[6] = {0};
uint16_t rxRawValue[6] = {0};
uint32_t rxAgeUs[6] = {0};
uint32_t rxRejects[6] = {0};
uint32_t rxEdges[6] = {0};
uint8_t rxValid[6] = {0};
bool receiverHealthy = false;
uint8_t badChannelsMask = 0x3F;

void IRAM_ATTR receiverEdge(uint8_t ch, gpio_num_t pin) {
  // Timestamp BEFORE reading the level or taking the spinlock.
  // No Serial, String, delay, or WiFi calls within the ISR.
  const uint32_t now = micros();
  const bool high = gpio_get_level(pin) != 0;

  portENTER_CRITICAL_ISR(&rxMux);
  rxEdgeCount[ch]++;
  if (high) {
    // A rising edge should start a fresh high pulse.
    // If a falling edge was missed, resynchronise on a later rise.
    if (!rxHighSeen[ch] || (uint32_t)(now - rxRiseUs[ch]) > RX_MAX_US) {
      if (rxHighSeen[ch]) rxRejectCount[ch]++;
      rxRiseUs[ch] = now;
      rxHighSeen[ch] = 1;
    }
  } else {
    if (rxHighSeen[ch]) {
      const uint32_t width = (uint32_t)(now - rxRiseUs[ch]);
      rxHighSeen[ch] = 0;
      rxRawUs[ch] = (uint16_t)(width > 65535UL ? 65535UL : width);

      const bool pulseWidthOk = (width >= RX_MIN_US && width <= RX_MAX_US);
      const bool frameSpacingOk = !rxHasPulse[ch] ||
              (uint32_t)(now - rxLastUs[ch]) >= RX_MIN_FRAME_US;

      if (pulseWidthOk && frameSpacingOk) {
        rxPulseUs[ch] = (uint16_t)width;
        rxLastUs[ch] = now;
        rxHasPulse[ch] = 1;
      } else {
        rxRejectCount[ch]++;
      }
    } else {
      // Orphan falling edge -- possibly noise or delayed interrupt.
      rxRejectCount[ch]++;
    }
  }
  portEXIT_CRITICAL_ISR(&rxMux);
}

void IRAM_ATTR rxISR1() { receiverEdge(0, (gpio_num_t)RX_CH1_PIN); }
void IRAM_ATTR rxISR2() { receiverEdge(1, (gpio_num_t)RX_CH2_PIN); }
void IRAM_ATTR rxISR3() { receiverEdge(2, (gpio_num_t)RX_CH3_PIN); }
void IRAM_ATTR rxISR4() { receiverEdge(3, (gpio_num_t)RX_CH4_PIN); }
void IRAM_ATTR rxISR5() { receiverEdge(4, (gpio_num_t)RX_CH5_PIN); }
void IRAM_ATTR rxISR6() { receiverEdge(5, (gpio_num_t)RX_CH6_PIN); }

void setupReceiver() {
  for (int i = 0; i < 6; i++) pinMode(rxPins[i], INPUT);
  // GPIO34 and GPIO35 have no internal pull-up/down resistor.
  attachInterrupt(digitalPinToInterrupt(RX_CH1_PIN), rxISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RX_CH2_PIN), rxISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RX_CH3_PIN), rxISR3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RX_CH4_PIN), rxISR4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RX_CH5_PIN), rxISR5, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RX_CH6_PIN), rxISR6, CHANGE);
}

void readReceiver(uint32_t nowUs) {
  uint16_t accepted[6], raw[6];
  uint32_t times[6], rejects[6], edges[6];
  uint8_t seen[6];
  // Snapshot all ISR-owned fields coherently, then leave critical section.
  portENTER_CRITICAL(&rxMux);
  for (int i = 0; i < 6; i++) {
    accepted[i] = rxPulseUs[i];
    raw[i] = rxRawUs[i];
    times[i] = rxLastUs[i];
    rejects[i] = rxRejectCount[i];
    edges[i] = rxEdgeCount[i];
    seen[i] = rxHasPulse[i];
  }
  portEXIT_CRITICAL(&rxMux);

  badChannelsMask = 0;
  for (int i = 0; i < 6; i++) {
    rxValue[i] = accepted[i];
    rxRawValue[i] = raw[i];
    rxRejects[i] = rejects[i];
    rxEdges[i] = edges[i];
    rxAgeUs[i] = seen[i] ? (uint32_t)(nowUs - times[i]) : UINT32_MAX;
    rxValid[i] = (seen[i] && accepted[i] >= RX_MIN_US &&
                  accepted[i] <= RX_MAX_US && rxAgeUs[i] <= RX_TIMEOUT_US) ? 1 : 0;
    if ((REQUIRED_RX_MASK & (1u << i)) && !rxValid[i]) {
      badChannelsMask |= (1u << i);
    }
  }
  receiverHealthy = (badChannelsMask == 0);
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
bool mixerClipped = false;

void writeMotors(int a, int b, int c, int d) {
  motorOutput[0] = a; motorOutput[1] = b;
  motorOutput[2] = c; motorOutput[3] = d;
  motor1.writeMicroseconds(a);
  motor2.writeMicroseconds(b);
  motor3.writeMicroseconds(c);
  motor4.writeMicroseconds(d);
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
  writeMotors(out[0], out[1], out[2], out[3]);
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
  resetPID();
  stopMotors();
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

// -------------------- 11. SHARED TELEMETRY --------------------
struct Telemetry {
  float roll, pitch, gyroRoll, gyroPitch, gyroYaw;
  bool armed, receiverOk, sensorOk, failsafeLatched, mixerClipped;
  uint8_t failsafe, lastFailsafe, badMask, whoAmI;
  uint16_t rx[6], raw[6];
  uint32_t rejects[6], edges[6];
  uint32_t age[6];
  uint8_t valid[6];
  int motor[4];
  uint32_t intervalUs, maxIntervalUs, executionUs, maxExecutionUs, overruns;
  uint32_t rxFaults, imuFaults, timingFaults, imuErrors;
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
    t.age[i] = rxAgeUs[i]; t.valid[i] = rxValid[i];
  }
  for (int i = 0; i < 4; i++) t.motor[i] = motorOutput[i];
  t.intervalUs = loopIntervalUs; t.maxIntervalUs = maxLoopIntervalUs;
  t.executionUs = loopExecutionUs; t.maxExecutionUs = maxLoopExecutionUs;
  t.overruns = executionOverruns;
  t.rxFaults = receiverFaults; t.imuFaults = imuFaults;
  t.timingFaults = timingFaults; t.imuErrors = imuReadErrors;
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
.wrap{max-width:1140px;margin:auto}header{display:flex;align-items:center;justify-content:space-between;gap:12px;flex-wrap:wrap;margin-bottom:16px}h1{font-size:25px;letter-spacing:.2px;margin:0}h2{font-size:14px;color:#c2d8f3;text-transform:uppercase;letter-spacing:1.2px;margin:0 0 12px}.sub{color:var(--muted);margin-top:5px}.tag{border:1px solid var(--line);border-radius:50px;padding:9px 13px;color:var(--yellow)}.grid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:12px}.card{background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:17px;min-width:0}.wide{grid-column:span 2}.all{grid-column:1/-1}.value{font-size:26px;font-weight:700;font-variant-numeric:tabular-nums}.tiny{font-size:12px;color:var(--muted);margin-top:6px}.ok{color:var(--green)}.bad{color:var(--red)}.warn{color:var(--yellow)}.blue{color:var(--blue)}.rxrow{display:grid;grid-template-columns:80px minmax(30px,1fr) 88px 70px 74px 65px 75px;gap:8px;align-items:center;padding:9px 0;border-bottom:1px solid var(--line)}.rxrow:last-child{border:0}.rxrow span{font-variant-numeric:tabular-nums}.bar{height:7px;border-radius:6px;background:#23374e;overflow:hidden}.fill{height:100%;width:0;background:var(--blue);transition:width .18s}.motors{display:grid;grid-template-columns:repeat(4,1fr);gap:8px}.mot{background:#0b1628;padding:13px;border-radius:9px;text-align:center}.mot strong{font-size:22px;display:block;margin-top:7px;font-variant-numeric:tabular-nums}.faults{display:grid;grid-template-columns:repeat(4,1fr);gap:10px}.faults div{padding:12px;background:#0b1628;border-radius:9px}.faults strong{display:block;font-size:24px;font-variant-numeric:tabular-nums;margin-top:4px}.info{color:var(--muted);line-height:1.5}.status{font-weight:700}.metricline{display:flex;gap:14px;flex-wrap:wrap}.metricline div{flex:1;min-width:110px}@media(max-width:850px){.grid{grid-template-columns:repeat(2,minmax(0,1fr))}.wide{grid-column:span 2}}@media(max-width:520px){body{padding:11px}.grid{grid-template-columns:1fr}.wide,.all{grid-column:span 1}.rxrow{grid-template-columns:58px 69px 54px 62px 49px 58px;gap:4px;font-size:11px}.rxrow .bar{display:none}.rxrow span{overflow:hidden}.motors,.faults{grid-template-columns:repeat(2,1fr)}}
</style></head><body><div class="wrap">
<header><div><h1>ESP32 <span class="blue">Quad-X</span> Telemetry</h1><div class="sub">250 Hz target control loop · read-only · 5 Hz refresh</div></div><div id="link" class="tag">CONNECTING...</div></header>
<div class="grid">
<section class="card"><h2>Arm status</h2><div id="armed" class="value">--</div><div class="tiny">CH5 LOW → HIGH + low throttle</div></section>
<section class="card"><h2>Receiver</h2><div id="receiver" class="value">--</div><div class="tiny">All six channels required</div></section>
<section class="card"><h2>IMU</h2><div id="imu" class="value">--</div><div class="tiny">WHO_AM_I: <span id="who">--</span></div></section>
<section class="card"><h2>Current failsafe</h2><div id="fault" class="value">--</div><div class="tiny">Last fault: <span id="last">--</span></div></section>
<section class="card wide"><h2>Attitude</h2><div class="metricline"><div><div class="value"><span id="roll">--</span>°</div><div class="tiny">Roll</div></div><div><div class="value"><span id="pitch">--</span>°</div><div class="tiny">Pitch</div></div></div><div class="tiny">Check tilt directions with props removed</div></section>
<section class="card wide"><h2>Gyroscope (°/s)</h2><div class="metricline"><div><div class="value" id="gx">--</div><div class="tiny">Roll rate</div></div><div><div class="value" id="gy">--</div><div class="tiny">Pitch rate</div></div><div><div class="value" id="gz">--</div><div class="tiny">Yaw rate</div></div></div></section>
<section class="card all"><h2>Receiver pulse widths · age and diagnostics</h2><div class="tiny" style="margin-bottom:7px">Accepted pulse (µs), age (ms), last raw high time (µs), rejected edges/pulses and interrupt edges. Counters increase from boot; accepted values expire after 100 ms.</div><div id="rxRows"></div><div id="badList" class="tiny">--</div></section>
<section class="card wide"><h2>Motor outputs (µs)</h2><div class="motors"><div class="mot">M1 · FR<strong id="m0">1000</strong></div><div class="mot">M2 · RR<strong id="m1">1000</strong></div><div class="mot">M3 · RL<strong id="m2">1000</strong></div><div class="mot">M4 · FL<strong id="m3">1000</strong></div></div><div class="tiny">Mixer clipped: <span id="clip">--</span></div></section>
<section class="card wide"><h2>Loop timing (µs)</h2><div class="metricline"><div><div class="value" id="interval">--</div><div class="tiny">Start-to-start interval</div></div><div><div class="value" id="execution">--</div><div class="tiny">Work execution time</div></div><div><div class="value" id="maxInterval">--</div><div class="tiny">Maximum interval</div></div><div><div class="value" id="maxExecution">--</div><div class="tiny">Maximum execution</div></div></div><div class="tiny">Target 4000µs · timing fault if interval exceeds 10000µs · overruns <span id="overruns">--</span></div></section>
<section class="card all"><h2>Fault counters</h2><div class="faults"><div>Receiver<strong id="rf">0</strong></div><div>IMU failsafe<strong id="if">0</strong></div><div>Timing<strong id="tf">0</strong></div><div>IMU read errors<strong id="ie">0</strong></div></div><p class="info">Fault counters increment once per fault type during each latched episode. To clear a fault, fix the cause, set CH5 LOW and throttle LOW, then move CH5 HIGH to re-arm. Never test with propellers fitted.</p></section>
</div></div>
<script>
const $=id=>document.getElementById(id);
const names=['Roll','Pitch','Throttle','Yaw','Arm','Aux'];
const fNames=['NONE','TIMING','RECEIVER','IMU'];
for(let i=0;i<6;i++) $('rxRows').insertAdjacentHTML('beforeend',`<div class="rxrow"><span>CH${i+1} ${names[i]}</span><div class="bar"><div id="b${i}" class="fill"></div></div><span id="v${i}">-- µs</span><span id="a${i}">--</span><span id="rw${i}" title="Last measured high width">Raw --</span><span id="rj${i}" title="Rejected edges / widths">Bad 0</span><span id="ed${i}" title="Interrupt edge count">Edges 0</span></div>`);
let lastGood=0;
async function tick(){
  try{
    const r=await fetch('/telemetry',{cache:'no-store'});if(!r.ok)throw Error(r.status);
    const d=await r.json();lastGood=Date.now();$('link').textContent='LIVE';$('link').className='tag ok';
    const set=(id,v,cls='')=>{const e=$(id);e.textContent=v;e.className=cls;};
    set('armed',d.armed?'ARMED':'DISARMED',d.armed?'value warn':'value');
    set('receiver',d.receiverOk?'HEALTHY':'INVALID',d.receiverOk?'value ok':'value bad');
    set('imu',d.sensorOk?'HEALTHY':'ERROR',d.sensorOk?'value ok':'value bad');
    set('who','0x'+Number(d.whoAmI).toString(16).toUpperCase().padStart(2,'0'));
    set('fault',fNames[d.failsafe]||'UNKNOWN',d.failsafe?'value bad':'value ok');
    set('last',fNames[d.lastFailsafe]||'UNKNOWN');
    ['roll','pitch','gx','gy','gz'].forEach((id,i)=>set(id,Number([d.roll,d.pitch,d.gyroRoll,d.gyroPitch,d.gyroYaw][i]).toFixed(2)));
    for(let i=0;i<6;i++){
      set('v'+i,d.rx[i]+' µs',d.valid[i]?'ok':'bad');
      set('a'+i,d.age[i]===4294967295?'NEVER':(d.age[i]/1000).toFixed(0)+'ms',d.valid[i]?'ok':'bad');
      set('rw'+i,'Raw '+d.raw[i]);
      set('rj'+i,'Bad '+d.rejects[i]);
      set('ed'+i,'Edges '+d.edges[i]);
      $('b'+i).style.width=Math.max(0,Math.min(100,((d.rx[i]-900)/1200)*100))+'%';
      $('b'+i).style.background=d.valid[i]?'var(--blue)':'var(--red)';
    }
    const bad=[];for(let i=0;i<6;i++)if((d.badMask>>i)&1)bad.push('CH'+(i+1));
    set('badList',bad.length?'INVALID / STALE: '+bad.join(', '):'All required receiver channels fresh');
    for(let i=0;i<4;i++)set('m'+i,d.motors[i]);
    set('clip',d.mixerClipped?'YES':'NO');
    [['interval','intervalUs'],['execution','executionUs'],['maxInterval','maxIntervalUs'],['maxExecution','maxExecutionUs'],['overruns','overruns'],['rf','rxFaults'],['if','imuFaults'],['tf','timingFaults'],['ie','imuErrors']].forEach(([id,key])=>set(id,d[key]));
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
  s += "],\"edges\":[";
  for (int i=0;i<6;i++) { if (i) s+=','; s+=t.edges[i]; }
  s += "],\"age\":[";
  for (int i=0;i<6;i++) { if (i) s+=','; s+=t.age[i]; }
  s += "],\"valid\":[";
  for (int i=0;i<6;i++) { if (i) s+=','; s+=(unsigned int)t.valid[i]; }
  s += "],\"motors\":[";
  for (int i=0;i<4;i++) { if (i) s+=','; s+=t.motor[i]; }
  s += ']';
  s += ",\"intervalUs\":"; s+=t.intervalUs;
  s += ",\"maxIntervalUs\":"; s+=t.maxIntervalUs;
  s += ",\"executionUs\":"; s+=t.executionUs;
  s += ",\"maxExecutionUs\":"; s+=t.maxExecutionUs;
  s += ",\"overruns\":"; s+=t.overruns;
  s += ",\"rxFaults\":"; s+=t.rxFaults;
  s += ",\"imuFaults\":"; s+=t.imuFaults;
  s += ",\"timingFaults\":"; s+=t.timingFaults;
  s += ",\"imuErrors\":"; s+=t.imuErrors;
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
  Serial.println("READY FOR PROP-REMOVED BENCH TEST ONLY");
}

// -------------------- 16. 250 HZ MAIN CONTROL LOOP --------------------
void loop() {
  const uint32_t started = micros();
  loopIntervalUs = (uint32_t)(started - previousLoopStartUs);
  previousLoopStartUs = started;
  if (loopIntervalUs > maxLoopIntervalUs) maxLoopIntervalUs = loopIntervalUs;

  bool timingOk = loopIntervalUs <= MAX_LOOP_GAP_US;
  if (timingOk) {
    if (timingGoodStreak < TIMING_RECOVERY_GOOD_LOOPS) timingGoodStreak++;
  } else {
    timingGoodStreak = 0;
  }

  float dt = constrain(loopIntervalUs * 0.000001f, 0.001f, 0.010f);
  readReceiver(started);

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

  if (!receiverHealthy) triggerFailsafe(FS_RECEIVER);
  if (!timingOk) triggerFailsafe(FS_TIMING);
  if (!sensorHealthy) triggerFailsafe(FS_IMU);

  // Fault clears ONLY after the input and sensor signals recover, CH5 is LOW,
  // and throttle is LOW. A fresh LOW -> HIGH CH5 transition is required to arm.
  if (failsafeLatched && receiverHealthy && sensorHealthy &&
      imuGoodStreak >= IMU_RECOVERY_GOOD_LOOPS &&
      timingGoodStreak >= TIMING_RECOVERY_GOOD_LOOPS &&
      rxValue[4] < 1300 && rxValue[2] < THROTTLE_CUTOFF) {
    clearFailsafe();
  }

  if (!failsafeLatched) {
    const bool armLow = rxValue[4] < 1300;
    const bool armHigh = rxValue[4] > 1700;
    const bool throttleLow = rxValue[2] < THROTTLE_CUTOFF;
    if (armLow) {
      if (armed) resetPID();
      armed = false;
      armSwitchReady = true;
    } else if (armHigh && !armed && armSwitchReady && throttleLow) {
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
      const float desiredRoll = constrain((rxValue[0] - 1500) *
                  (MAX_ANGLE_DEG / 500.0f), -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
      const float desiredPitch = constrain((rxValue[1] - 1500) *
                  (MAX_ANGLE_DEG / 500.0f), -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
      const float desiredYawRate = constrain((rxValue[3] - 1500) *
                  (MAX_RATE_DPS / 500.0f), -MAX_RATE_DPS, MAX_RATE_DPS);
      const float throttle = constrain((float)rxValue[2],
                                      (float)ESC_IDLE, (float)ESC_MAX);

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
  } else {
    stopMotors();
  }

  updateLEDs();
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
  if (elapsed < LOOP_PERIOD_US) delayMicroseconds(LOOP_PERIOD_US - elapsed);
}