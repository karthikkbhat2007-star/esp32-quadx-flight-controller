ESP32 QUAD-X ESC CALIBRATION / STARTUP

Purpose:
  Calibrate PWM ESC throttle endpoints ONCE when necessary.
  Normal ESC startup must use 1000 us LOW; do not automatically send HIGH
  every flight-controller boot. Some ESCs treat boot-time HIGH as a special
  programming signal; others can start motors unexpectedly.

Hardware mapping (unchanged):
  M1 = GPIO13 (front right)
  M2 = GPIO12 (rear right)
  M3 = GPIO14 (rear left)
  M4 = GPIO27 (front left)

BEFORE CALIBRATION:
  Remove all four propellers. Secure the frame.
  Power the ESP32 separately and ensure ESC battery power is disconnected.
  Only one correctly wired 5V supply should power the ESP32; avoid joining
  USB 5V with external BEC 5V without verified isolation.
  Ensure common signal ground and 3.3V-compatible signaling.
  Verify that your specific ESC firmware supports HIGH/LOW endpoint learning.

TO CALIBRATE:
  1. Upload ESP32_QuadX_ESCCalibration.ino using Arduino IDE with ESP32Servo.
  2. Open Serial Monitor at 115200 baud. The sketch boots with 1000us LOW.
  3. With ESC battery still DISCONNECTED, send H.
  4. Connect ESC battery after HIGH is active.
  5. At ESC calibration-entry beep, send L promptly; listen for save tones.
  6. Confirm that all four ESCs saved endpoints; otherwise troubleshoot each
     ESC separately using its exact manufacturer procedure.
  7. Disconnect battery. Re-upload normal flight-controller sketch.

BACKUP:
  X = return to 1000us LOW. HIGH automatically returns LOW after 30 sec.
  The controller cannot verify whether the ESC actually saved calibration.

EVERY STARTUP IN YOUR FLIGHT CONTROLLER:
  Flight candidate has initializeMotors() -> stopMotors(),
  sending 1000us LOW to all ESCs at startup. Keep this behavior.
  Keep receiver/IMU validation and arming disabled during boot/ESC init.
  Do not re-run the 2000us endpoint-calibration sequence automatically.

NOTE ON RANGE:
  Calibration uses 1000..2000us at 50Hz (typically recognized by PWM ESCs).
  Flight candidate limits outputs to 1000..1800us at 500Hz.
  Calibration does not change your 1800us flight command cap.
  Use only verified manufacturer-compatible endpoints/frequency.

DISCLAIMER:
  This is a standalone service sketch, not the currently flying firmware.
  This code does not include flight controls, PID or failsafes for flight.
