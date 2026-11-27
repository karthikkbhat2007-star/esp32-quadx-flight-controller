# Hardware reference (from development record)

**Platform:** ESP32-WROOM, Quad-X. Experimental controller, not flight-validated.

| Component | GPIO | Description |
| --- | --- | --- |
| Receiver CH1 | 34 | Roll; input-only |
| Receiver CH2 | 35 | Pitch; input-only |
| Receiver CH3 | 32 | Throttle |
| Receiver CH4 | 33 | Yaw |
| Receiver CH5 | 25 | Arm |
| Receiver CH6 | 26 | Auxiliary |
| ESC M1 | 13 | Front-right CCW |
| ESC M2 | 12 | Rear-right CW; boot-strap pin |
| ESC M3 | 14 | Rear-left CCW |
| ESC M4 | 27 | Front-left CW |
| Blue LED | 5 | Status |
| Red LED | 4 | Fault |
| I²C SDA/SCL | 21 / 22 | IMU at 0x68 |

IMU WHO_AM_I reported 0x70, identity not conclusively verified. Earlier QMC5883L at 0x0D is not used in rate-yaw controller.

Flight sketches generally target 250 Hz (4 ms), ESC minimum 1000 µs, idle 1170 µs, maximum 1800 µs, RX timeout 100 ms. ESC maintenance calibration uses a separate 50 Hz and 1000–2000 µs routine when supported by the exact ESC firmware.

Propellers off for bench work. Use common ground, GPIO-compatible voltage levels, and verified power-supply isolation.
