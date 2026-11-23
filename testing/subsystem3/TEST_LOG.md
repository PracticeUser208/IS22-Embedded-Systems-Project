# Subsystem 3 — test log

**Subsystem:** Line and Barcode (Buddy 3)  ·  **Student ID:** 2502978  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | (template example, off with `TEMPLATE_DEMO 0`) statistics self-consistent | | auto | `subsystem3_test.c` | `[TEST] S3-U01 PASS` |
| S3-CORE | Buddy 3's core logic: IR calibration/classification, pulse capture, 7-state machine | FR1, FR19 | auto | `subsystem3_test.c` (`s3_check.c`); also on a PC, see `README.md` | `[TEST] S3-CORE PASS` |
| S3-LF | line following logic: automatic calibration, black/white with hysteresis, steering sign and size, line lost after `LF_LOST_MS` (600 ms; was 300 ms), no spin after both sensors saw black | FR1, NFR10 | auto | `subsystem3_test.c` (`line_check.c`); on a PC: `python3 testing/pc_checks.py line` | `[TEST] S3-LF PASS` |
| S3-BC | barcodes: Code 39 letters read in both directions at 10–60 mm/s, faint bars, small spot, speed changes, weaving; no barcode read from the plain line | FR2, FR3 | auto | `subsystem3_test.c`; on a PC: `python3 testing/pc_checks.py barcode` | `[TEST] S3-BC PASS`; PC: `failures: 0`, `0 WRONG` |
| S3-IR | the three IR sensors read through the ADC (GP26–28) | IF ADC | auto (live) | `subsystem3_test.c` | `[TEST] S3-IR PASS` and the raw values printed |
| S3-M01 | wiring and polarity | Week 6 pin map | manual | car idle; "7 Read Pico console"; hold white paper, then black tape under each sensor | `S3: raw` lines: each value changes by several hundred; black higher than white (else set `LF_BLACK_IS_HIGH 0`) |
| S3-M02 | calibration | Week 6 INIT | manual | after power-up slide the car sideways across the line once | console `S3: raw ... calibrated`; telemetry `ir=` bits 1 over black, 0 over white |
| S3-M03 | follows a straight line and gentle curves | FR1, NFR9 | manual | START with the centre sensor on the line's left edge | stays on the line for a full lap; record speed and gains |
| S3-M04 | line lost: sweep and reacquire | FR19 | manual | lift the car off the line while patrolling, put it back next to the line | `LINE_LOST`, the car sweeps, `LINE_REACQUIRED` within 4 s (`LF_SEARCH_MS`; 2 s until 2 Nov), patrols again |
| S3-M05 | line not found: fault | FR19 | manual | start next to (not on) the line, far away | `LINE_TIMEOUT` after 4 s (2 s until 2 Nov), FAULT_ESTOP with fault bit 0x20 (FAULT_LINE_LOST); RESET → IDLE_READY |
| S3-M06 | gaps up to 1 cm | NFR10 | manual | 1 cm gap in the line | carries on across the gap without LINE_LOST |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-23 | S3-CORE | `ad50553` | Buddy 5 (Reuben), for the team | PASS | `../system/logs/2026-11-23_pc_checks.txt` (`[s3]`) | PC run of the core logic: `pure-logic regression failures: 0` |
| 2026-11-23 | S3-LF | `ad50553` | Buddy 5 (Reuben), for the team | PASS | `../system/logs/2026-11-23_pc_checks.txt` (`[line]`) | PC run: `line_follow regression failures: 0` |
| 2026-11-23 | S3-BC | `ad50553` | Buddy 5 (Reuben), for the team | PASS | `../system/logs/2026-11-23_pc_checks.txt` (`[barcode]`) | PC run: Code 39 and BC-09…BC-14 `failures: 0`, 516 of 516 read with a 3 mm and a 5 mm spot, 0 reads on the plain line |
| 2026-11-23 | S3-IR, S3-CORE, S3-LF, S3-BC (on the car) | `ad50553` | Buddy 5 (Reuben), for the team | PASS (builds) | `../system/logs/2026-11-23_build.txt` | the test image compiles with no warnings; run it on the car for the live results |
