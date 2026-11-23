# Subsystem 2 — test log

**Subsystem:** Motion Control (Buddy 2)  ·  **Student ID:** 2500261  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | encoder count difference correct across the 0xFFFF → 0 wrap | FR5 | auto | `subsystem2_test.c` | `[TEST] S2-U01 PASS` |
| S2-U02 | counts → mm conversion and one wheel revolution | FR5 | auto | `subsystem2_test.c` | `[TEST] S2-U02 PASS` |
| S2-U03 | turn arc length for a commanded angle | FR8 | auto | `subsystem2_test.c` | `[TEST] S2-U03 PASS` |
| S2-U04 | PID clamps its output and integral (anti-windup, `SPEED_I_LIMIT`) | FR6 | auto | `subsystem2_test.c` | `[TEST] S2-U04 PASS` |
| S2-U05 | speed profile: ramp up, cruise cap, slow down before the end of a move | FR6, FR8 | auto | `subsystem2_test.c` | `[TEST] S2-U05 PASS` |
| S2-U06 | the motion API rejects out-of-range arguments | FR9, IF-08 | auto | `subsystem2_test.c` | `[TEST] S2-U06 PASS` |
| S2-U07 | idle with the motors off at power-up | safety | auto (live) | `subsystem2_test.c` | `[TEST] S2-U07 PASS` |
| S2-U08 | a stop command reaches `motion_task` and DONE is signalled on `flg_motion` | FR9, IF-08, IF-09 | auto (live) | `subsystem2_test.c` | `[TEST] S2-U08 PASS` |
| S2-U09 | both wheels turn and both encoders count | FR5 | auto (live, wheels off the floor) | `subsystem2_test.c`; lift the car so the wheels turn freely | `[TEST] S2-U09 PASS` |
| S2-U10 | the velocity watchdog stops the car when commands stop (`MOTION_VEL_TIMEOUT_MS` 300 ms) | safety, IF-18 | auto (live) | `subsystem2_test.c` | `[TEST] S2-U10 PASS` |
| S2-M01 | drives straight (drift correction) | FR7 | manual | drive 2 m straight (`motion_move_forward(2000)`); measure the sideways drift with tape on the floor | lateral deviation ≤ 3 cm over 2 m (Week 6) |
| S2-M04 | distance from the encoders | FR5 | manual | same 2 m run; compare the odometer in telemetry with a tape measure | error ≤ 5 % over 2 m (Week 6) |
| S2-M05 | speed regulation | FR6 | manual | patrol at 25 mm/s; read `spd=` in telemetry for 10 s, then step the speed | steady speed within ±10 % of the setpoint, settles within 1 s of a step (Week 6) |
| S2-M02 | turns by a commanded angle | FR8 | manual | turn left 90° 10 times (and right 90° 10 times); mark the heading with tape | every turn 90° ± 5° (Week 6, 10 trials) |
| S2-M03 | the emergency-stop flag brakes the motors without the Vehicle Controller | IF-12, Tribunal 1 | manual | while patrolling, hold a hand 10 cm in front of the ultrasonic sensor (sets `ESTOP_OBSTACLE`) | wheels stop within one control tick (20 ms); dashboard `estop=OBSTACLE` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console; **auto (live)** = the check uses the real motors or encoders;
**manual** = a person observes the result (LED, scope, meter, phone app…).

To run S2-U01 … S2-U10: build the test image (`make NET=lwip MQTT=1 TEST=1`, file `*_mqtt_test.uf2`), flash it,
read the console with "7 Read Pico console" and save it as `logs/YYYY-MM-DD_unit.txt`.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-23 | S2-U01–U10 (build) | `ad50553` | Buddy 5 (Reuben), for the team | PASS (builds) | `../system/logs/2026-11-23_build.txt` | the test image with all ten checks compiles with no warnings; the checks themselves still have to be run on the car |
