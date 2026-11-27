/*
  ESP32 Quad-X - Manual PWM ESC endpoint calibration
  For separate, propeller-free maintenance, NOT for flight.

  Pins:
    M1 Front Right GPIO13
    M2 Rear Right  GPIO12
    M3 Rear Left   GPIO14
    M4 Front Left  GPIO27

  CALIBRATION IS OPT-IN, NOT AUTOMATIC ON EVERY STARTUP.
  This sketch ALWAYS boots into safe minimum (1000 us).
  To calibrate ESCs whose firmware supports throttle endpoint learning:
    1. REMOVE ALL PROPELLERS. Secure the frame. DISCONNECT ESC LiPo power.
    2. Power the ESP32 separately, via an appropriately isolated USB supply.
       Ensure any external BEC 5V is disconnected from the USB-powered ESP32.
       The ESC signal ground and ESP32 ground must have a suitable reference.
    3. Open Serial Monitor at 115200 baud.
    4. Type H to send HIGH (2000 us). ONLY do this while LiPo is unplugged.
    5. Connect ESC LiPo; listen for calibration-entry beeps.
    6. Type L promptly at the correct beep to send LOW (1000 us).
    7. Confirm successful save tones for all four ESCs. Power down.
    8. Reflash your usual flight-controller sketch for normal operation.

  DANGER: HIGH could spin motors if an ESC is already armed/powered.
  HIGH automatically reverts to LOW after 30 seconds as a backup only.
  The firmware cannot know whether battery is unplugged.

  The ESP32's GPIO12 (M2) is a boot-strapping pin. If boot fails with ESC
  connected, check boot-strap loading and wiring.
*/

#include <Arduino.h>
#include <ESP32Servo.h>

static constexpr int M1_PIN = 13;
static constexpr int M2_PIN = 12;
static constexpr int M3_PIN = 14;
static constexpr int M4_PIN = 27;

static constexpr int ESC_CALIBRATION_HZ = 50;     // 20 ms PWM frame for endpoint learning
static constexpr int ESC_MIN_US = 1000;
static constexpr int ESC_HIGH_US = 2000;         // Endpoint learning, not normal flight max
static constexpr uint32_t HIGH_TIMEOUT_MS = 30000;

Servo m1, m2, m3, m4;
bool highActive = false;
uint32_t highSinceMs = 0;

void setAllESCs(int pulseUs) {
  m1.writeMicroseconds(pulseUs);
  m2.writeMicroseconds(pulseUs);
  m3.writeMicroseconds(pulseUs);
  m4.writeMicroseconds(pulseUs);
}

void safeLow(const char *why) {
  setAllESCs(ESC_MIN_US);
  highActive = false;
  Serial.print("ESC outputs: 1000 us LOW - ");
  Serial.println(why);
}

void setup() {
  Serial.begin(115200);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  m1.setPeriodHertz(ESC_CALIBRATION_HZ);
  m2.setPeriodHertz(ESC_CALIBRATION_HZ);
  m3.setPeriodHertz(ESC_CALIBRATION_HZ);
  m4.setPeriodHertz(ESC_CALIBRATION_HZ);

  m1.attach(M1_PIN, ESC_MIN_US, ESC_HIGH_US);
  m2.attach(M2_PIN, ESC_MIN_US, ESC_HIGH_US);
  m3.attach(M3_PIN, ESC_MIN_US, ESC_HIGH_US);
  m4.attach(M4_PIN, ESC_MIN_US, ESC_HIGH_US);

  setAllESCs(ESC_MIN_US);
  delay(2000);  // Minimum throttle startup signal. No high throttle on boot.

  Serial.println("\n==== ESC ENDPOINT CALIBRATION: PROPS OFF ====");
  Serial.println("All ESCs output 1000 us at 50 Hz by default.");
  Serial.println("H = send 2000 us HIGH (ONLY when ESC battery is DISCONNECTED)");
  Serial.println("L = send 1000 us LOW after calibration-entry beeps");
  Serial.println("X = emergency return to 1000 us; ? = print help");
  Serial.println("Disconnect ESC battery before sending H!");
  Serial.println("The HIGH state expires after 30 seconds.");
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c >= 'a' && c <= 'z') c -= ('a' - 'A');

    switch (c) {
      case 'H':
        if (highActive) {
          Serial.println("HIGH already active; send L after ESC entry tones.");
        } else {
          Serial.println("WARNING: 2000 us HIGH begins NOW. ESC BATTERY MUST BE UNPLUGGED.");
          setAllESCs(ESC_HIGH_US);
          highSinceMs = millis();
          highActive = true;
          Serial.println("Connect ESC battery now, listen for entry tones, then type L.");
        }
        break;
      case 'L':
        safeLow("requested LOW / endpoint-save step");
        Serial.println("Listen for confirmation beeps. No beep feedback is read by ESP32.");
        break;
      case 'X':
        safeLow("manual emergency stop command");
        break;
      case '?':
        Serial.println("H=HIGH (battery disconnected first), L=LOW, X=LOW, ?=help");
        break;
      default:
        break; // Ignore newline and unsupported characters
    }
  }

  if (highActive && (uint32_t)(millis() - highSinceMs) >= HIGH_TIMEOUT_MS) {
    safeLow("30-second HIGH watchdog expired");
    Serial.println("Calibration may be incomplete; verify ESC beeps.");
  }
  delay(5);
}