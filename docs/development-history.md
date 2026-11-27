# Development log and evidence

These are user-reported bench observations, not independently verified flight-controller certification. The archive recreates version milestones rather than original historical Git commits.

## Earlier debugging

Original firmware used ESP32-WROOM, MPU-compatible IMU, six independent PWM receiver channels, cascaded angle/rate PID and four ESCs. Integrated revisions experienced receiver failures after roughly 60–65 seconds, and some runs reported INT WDT (5) resets. Standalone pulseIn() checks of all six channels worked. A simplified receiver capture architecture improved stability.

## Bench tests

| Stage | Subsystems | User-reported result |
| --- | --- | --- |
| A | Receiver + Wi-Fi | 10 min without dropouts/resets |
| B | + IMU | 658 s, no RX dropouts or IMU errors |
| C | + powered ESCs at 1000 µs | 672 s, no new resets/dropouts |
| D | Integrated PID/mixer, locked outputs | 617 s, zero RX dropouts and IMU errors, one timing fault; peak gap 11,130 µs |
| D-T | Detailed timing trace | 355 s, zero RX dropouts, one timing fault; peak gap 11,172 µs; test stopped because transmitter battery drained |

Later FreeRTOS-task candidate generated but not proven on target hardware. The exact revision reportedly flown is not verified in this archive.

## ESC calibration

The standalone maintenance sketch uses an explicit Serial H/L endpoint learning sequence with propellers removed. Flight-controller startup should send minimum throttle only.
