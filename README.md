# ESP32 Quad-X custom flight controller — development archive

A versioned archive of the Arduino sketches produced during development of an **ESP32-WROOM Quad-X** flight controller. Includes the original user-provided baseline, receiver/watchdog diagnostics, integration bench sketches, experimental flight-control candidates, and standalone ESC endpoint calibration.

> **Safety and status:** These are **experimental hardware-control sketches**, not validated flight firmware. Some sketches include motor outputs capable of spinning propellers. Remove all propellers during diagnostics or calibration. The repository does **not** establish airworthiness, verified failsafe behavior, or flight readiness. The exact firmware revision used for the user's later reported successful flights is **not verified**. The last flight candidates in this archive default to `BENCH_LOCKOUT = 1`.

## Getting started

1. Open one **individual sketch folder** in Arduino IDE. Each `.ino` resides in a matching folder to avoid Arduino sketch naming errors.
2. Select the correct ESP32 board and install the ESP32 Arduino core and `ESP32Servo`. Wi-Fi and Wire are supplied with the ESP32 Arduino core.
3. Confirm hardware wiring and software settings using [hardware.md](docs/hardware.md).
4. For receiver/watchdog checks, start with `bench/ESP32_RC_WDT_Bench/` (three configurations in the same sketch).
5. Use `bench/ESP32_QuadX_StageD_TimingTrace/` for the locked, integrated PID + mixer benchmark.
6. `maintenance/ESP32_QuadX_ESCCalibration/` is a **standalone, opt-in** maintenance program; it must never auto-send maximum throttle at every boot.

## Source organization

| Path | Purpose | Recorded status |
| --- | --- | --- |
| `baseline/Original_User_Flight_Controller/` | Earlier user-provided flight-controller implementation | Historical reference; not revalidated |
| `diagnostics/` | Earlier integrated receiver, timing, and reset attempts | Historical diagnostic versions |
| `bench/ESP32_RC_WDT_Bench/` | Separate receiver + Wi-Fi, IMU, and 1000 µs ESC stress stages | User reported A/B/C >10 min without receiver dropouts |
| `bench/ESP32_QuadX_StageD_BenchIntegration/` | Full PID/mixer calculations with ESC lockout | User reported 617 s, 0 receiver drops, 1 timing fault |
| `bench/ESP32_QuadX_StageD_TimingTrace/` | Adds timing-gap tracing | User reported 355 s, 0 receiver drops, 1 timing fault |
| `candidates/` | Experimental physical motor-control logic behind default lockout | Not proven flight-ready |
| `maintenance/` | Manual four-ESC endpoint calibration | Procedure depends on ESC firmware |

## Code version history

See [docs/development-history.md](docs/development-history.md) for the sequence and observed results. Each generated code version is preserved **as supplied**, without functional edits or merging. Git commits here represent **archive milestones** rather than an exact reproduction of the original development timestamps.

## Known outstanding issues

- Occasional **loop start-to-start gap ~11.1 ms** in locked Stage D builds, crossing the 10 ms timing fault threshold despite <1 ms work execution.
- Earlier integrated versions showed receiver dropouts and `INT WDT (5)` resets; dedicated bench versions improved stability but do not validate the complete flight-control system.
- Motor mixing signs, physical attitude correction, ESC PWM compatibility at 500 Hz, and fault behavior in flight are not independently validated.
- **GPIO12** is an ESP32 boot-strapping pin. Confirm boot behavior with ESC signal attached.

## Wi-Fi caution

Archived sketches contain the **demonstration AP password** used during development (`drone1234`), not an account credential. Change this before using the project outside an isolated test environment. Do not commit personal passwords or tokens.

## Publishing to GitHub

This archive includes full Git history. Unzip it, navigate into `esp32-quadx-flight-controller/` and create a **private** empty GitHub repository with the same name (or another name). Then run:

```bash
git remote add origin https://github.com/YOUR_USERNAME/esp32-quadx-flight-controller.git
git push -u origin main
```

You can make it public only after reviewing code, credentials, safety claims, and licensing. No third-party license is asserted by this archive.