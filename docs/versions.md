# Sketch versions

Each sketch is separate; do not combine all .ino files into one Arduino program.

- **baseline/** — Original user-provided source, historical reference.
- **diagnostics/** — Receiver capture, timing, and reset/debugging versions.
- **bench/** — A/B/C staged stress test and locked Stage D/D-T PID integrations.
- **candidates/** — Experimental full flight-control paths; default bench lockout remains enabled.
- **maintenance/** — Independent manual ESC calibration, not flight firmware.

File names are retained for comparison with earlier downloads. Compiling successfully or passing a bench test does not establish flight readiness.
