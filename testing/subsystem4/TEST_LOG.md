# Subsystem 4 — test log

**Subsystem:** IMU and Terrain (Buddy 4)  ·  **Student ID:** 2503710  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | (template example, off) message buffer max line = S4_SUMMARY_MAX | IF-03 | auto | `subsystem4_test.c` | `[TEST] S4-U01 PASS` |
| S4-U02 | (template example, off) LED pauses and resumes every 6 reports | IF-05 | manual | 1. Flash normal image. 2. Watch LED and console ~20 s. | console shows "LED task paused"/"resumed"; LED stops/starts |
| S4-P01 | the 10 ms poll runs (cyclic handler) | Tribunal 1: drift-free ~10 ms poll | auto (live) | `subsystem4_test.c` | about 100 polls in 1 s, `[TEST] S4-P01 PASS` |
| S4-P02 | IMU calibrates and monitors (or reports a fault if not connected) | statechart 6.4, NFR15 | auto (live) | `subsystem4_test.c` | state MONITORING (IMU wired) or FAULT with fault bits; never stuck in INIT / CALIBRATE |
| S4-P03 | Motion Control's speed / distance snapshot is fresh | IF-20, Tribunal 1 | auto (live) | `subsystem4_test.c` | sequence changes within 100 ms |
| S4-U03 | repeated I2C NACKs raise a blocking NACK fault (Buddy 4's check, from the team repo) | NFR15 | auto | `subsystem4_test.c` | `[TEST] S4-U03 PASS` |
| S4-M01 | wiring: IMU answers on I2C1 GP2/GP3 | Week 6 pin map | manual | power up with the IMU on Grove 2; read the console ("7 Read Pico console") | no `S4: -> imu_evt_q SENSOR_FAULT` at start-up; telemetry `fault=-` |
| S4-M02 | IMU fault stops the car, recovers | NFR15, A8, IMU_FAULT_TRIGGERS_ESTOP | manual | software-forced first (Tribunal 1), then unplug SDA while idle, plug back, send RESET | `SENSOR_FAULT`, estop IMU_FAULT, fault 0x04; after replug: `SENSOR_FAULT payload 0`, IDLE_READY |
| S4-M03 | hump height | FR11, A7 | manual | 10 passes over the practice ramp at patrol speed; compare `VC: hump x.y mm` with a ruler | within ±3 mm (FR11, revised 21 Nov; was ±20 %); fit `HUMP_GAIN` |
| S4-M04 | lifting the car stops it and ends the run (16 Nov: replaces "impact stops the car"; requirement changed 11 Nov, collision detection removed) | FR13 (revised), A12 | manual | while patrolling, lift the car straight up about 15 cm within 1 s; 10 times. Then 5 times tipping it on its side instead | console `S4: -> imu_evt_q LIFTED payload 1` (moved up) or `2` (tipped), `VC: lifted / overturned`; wheels stop at once; FAULT_ESTOP "lifted: run ended", estop IMU_LIFTED; put down, RESET → IDLE_READY |
| S4-M05 | no false "lifted" while driving (16 Nov: replaces "no false impact") | A12 | manual | 10 hump passes, 10 STOPs from patrol speed, one full lap with an obstacle bypass; note the largest `S4: upward speed peak` | no `LIFTED`; peak at most a third of `LIFT_UP_MM_S` |
| S4-L01 | lifted / overturned logic on a PC (made-up sensor data: vibration, humps, lifts, tipping) | FR13 (revised) | auto (PC) | `python3 testing/pc_checks.py lift` (`testing/subsystem4/lift_check.c`) | `LIFT CHECK PASSED` |
| S4-H01 | hump detection on a PC: humps of 3, 5, 10 and 20 mm, wheelbase 100 and 140 mm, the same humps under one wheel only, the course hump (10 mm, long ramps, one wheel) with driving jolts, a wheel then the caster over the same hump, the course hump with the car leaning 2 / 1.5 deg while driving and stopping on the hump, and starting 30 cm before it (20 Nov), and 10 min of flat floor with sensor noise and with jolts | FR (hump height) | auto (PC) | `python3 testing/pc_checks.py hump` | `ALL PASSED`: 10 mm and higher reported once each, no false hump on the flat floor |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-23 | S4-L01 | `ad50553` | Buddy 5 (Reuben), for the team | PASS | `../system/logs/2026-11-23_pc_checks.txt` (`[lift]`) | `LIFT CHECK PASSED`. With 20 mg noise: 0 false stops in 10 min of driving and 40 humps; lifts of 100–200 mm in 0.4–0.8 s stopped 50 of 50, a slow 100 mm lift over 1 s only 18 of 50; tipped by 30° roll, 50° pitch or upside down stopped within 0.6 s. With 100 mg noise: 4 false stops in 10 min and 1 per 20 humps |
| 2026-11-23 | S4-H01 | `ad50553` | Buddy 5 (Reuben), for the team | PASS | `../system/logs/2026-11-23_pc_checks.txt` (`[hump]`) | `ALL PASSED`: the 10 mm course hump reported once (11.0–13.5 mm in the simulation), 0 false humps in 10 min of flat floor |
| 2026-11-23 | S4-P01, S4-P02, S4-P03, S4-U03 | `ad50553` | Buddy 5 (Reuben), for the team | PASS (builds) | `../system/logs/2026-11-23_build.txt` | the test image compiles with no warnings; run it on the car for the live results |
