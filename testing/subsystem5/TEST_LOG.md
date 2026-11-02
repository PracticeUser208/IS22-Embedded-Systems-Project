# Subsystem 5 — test log

**Subsystem:** Obstacle Scan (ultrasonic, scan servo, obstacle profiling)  ·  **Student ID:** 2501748  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | obst_q message pack/unpack at 0, max (511) and over-range | Tribunal 1 curveball, obst_q (IF) | auto | `subsystem5_test.c` | `[TEST] S5-U01 PASS` |
| S5-U02 | scan_q command pack/unpack | scan_q (IF) | auto | `subsystem5_test.c` | `[TEST] S5-U02 PASS` |
| S5-U03 | Planner: 20 cm box at 15 cm, wall on the right | FR16 (width ≤ 20 % error), FR17 | auto | `subsystem5_test.c` (made-up scan) | width 16–24 cm, side LEFT, PASS |
| S5-U04 | Planner: walls close on both sides | FR17, Blocked state | auto | `subsystem5_test.c` | NO_PATH, PASS |
| S5-U05 | Planner: nothing near | FR14 | auto | `subsystem5_test.c` | PATH_CLEAR, PASS |
| S5-U06 | Scan storage size | Tribunal 1 curveball (≈240 → 60 B) | auto | `subsystem5_test.c` prints sizes | scan + plan ≤ 64 B, PASS |
| S5-U07 | Two tasks set/clear different ESTOP bits 2000× each at once | Risk R3, ESTOP_REQ | auto | `subsystem5_test.c` | both bits set at the end (no lost update), PASS |
| S5-U08 | 1 MHz hardware timer counts | echo timing | auto | `subsystem5_test.c` | PASS |
| S5-U09 | Sensor answers (HC-SR04+ connected) | FR14, A9 | auto (needs hardware) | `make TEST=1`, sensor wired | no sensor fault and a distance or "open", PASS |
| S5-U10 | Distance vs ruler | A9 | manual | Box at 10, 20, 30, 50, 100 cm; read `dist=` on the status line; 10 readings each | error ≤ ±1 cm up to 50 cm |
| S5-U11 | Servo centre and direction | statechart 6.5 scans | manual | Watch the sensor during a scan; 0° must face straight ahead, + must turn left. Adjust `SERVO_CENTER_US` / `SERVO_DIR` in `drv_servo.h` | 0° straight, + = left |
| S5-U12 | Stop within 15 cm | FR15, A11 | manual | Move a box slowly towards the sensor | `OBSTACLE_DETECTED flag=1` at ≤ 15 cm, `estop=0x4` |
| S5-U13 | Slow then clear | FR15 | manual | Box to ~40 cm, then away to > 55 cm | `OBSTACLE_DETECTED flag=0`, then `PATH_CLEAR` |
| S5-U14 | Coarse + fine scan, width and side | FR16, FR17, NFR11 | manual | 3 widths × 3 positions; compare `S5 plan: width=` with a ruler; put a wall on one side | width error ≤ 20 %; free side chosen in ≥ 95 % |
| S5-U15 | Sensor fault and recovery | statechart 6.5 Sensor Fault | manual | Unplug the echo wire in PATROL, wait, plug it back | `SENSOR_FAULT` (faults 0x01), `estop=0x4`, `sysfault=0x8`; after replug, stub RESET → `FAULT_CLEARED`, back to PATROL |
| S5-U16 | Retry limit when blocked | Week 6 review feedback (BLOCKED → back up loop had no retry limit); post-Tribunal 1 gap check (bypass retry limit untested) | manual | Box in front, walls ≤ 20 cm on both sides; repeat 10 runs (gap check) | `NO_PATH flag=0` twice, then `NO_PATH flag=1` (gave up), every run |
| S5-U17 | Run-to-block time | Version C (bounded run-to-block) | manual | Run 1 min with scans; read `run_max=` on the status line | record the value; target ≤ 1 ms |
| S5-U18 | Scans with motors running | A10 | manual | Repeat S5-U14 with the wheels spinning | no resets; width / side as in S5-U14 |
| S5-U19 | Planner with the servo's safe travel (+30 .. −45): a side the sensor cannot reach counts as no room | FR17, servo limits (sensor hits the board past them) | auto | `subsystem5_test.c` | wall on the right → NO_PATH; right open → RIGHT; PASS |
| S5-U20 | Servo limits | servo limits in `drv_servo.h` | manual | `us_servo_bringup_v3`: step with l / r (or a / d) until the sensor touches the board; the last angle before that is the limit; copy into `SERVO_LIMIT_LEFT_DEG` / `SERVO_LIMIT_RIGHT_DEG`. Re-run if the mounting changes | sensor never touches the board. Measured 25 Oct: +30° left, −45° right (knocked off at +45° and −60°) |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S5-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
