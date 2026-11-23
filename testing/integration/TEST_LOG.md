# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | (template example, off with `TEMPLATE_DEMO 0`) samples flow from S2 to S3 (≥ 5 in 2 s) | IF-01, IF-02 | auto | `subsystem3_test.c` | `[TEST] INT-01 PASS` |
| INT-02 | (template example, off with `TEMPLATE_DEMO 0`) summary lines flow from S3 to S4 (≥ 2 in 3.5 s) | IF-03 | auto | `subsystem4_test.c` | `[TEST] INT-02 PASS` |
| INT-04 | the IMU task started and left INIT (Buddy 4's check, from the team repo) | IF-14 | auto | `subsystem4_test.c` | `[TEST] INT-04 PASS` |
| INT-05 | a telemetry snapshot from the VC, encoded by S1, decodes the same in the ground station (C and Python agree on every field) | IF-06, NFR19 | auto (PC) | `python3 testing/pc_checks.py test_codec` and `python3 tools/buddy1/is22_ground.py selftest` | both `ALL PASSED` / `SELFTEST PASSED` |
| INT-06 | Obstacle Scan's plan drives the VC's bypass legs through Motion Control (simulated car, real `s5_plan` and VC code) | IF-10, IF-11, IF-18 | auto (PC) | `python3 testing/pc_checks.py bypass` | goes round without touching; gives up only when there is no way round (results also in `vehicle_ctrl/TEST_LOG.md`, VC-S01) |
| INT-07 | remote commands reach the VC and the ACK comes back | IF-07, FR22 | manual | `MQTT setup - GitHub`, "Send to car" START / PAUSE / RESUME / STOP / RESET | ACK `FORWARDED` within 1 s, telemetry state changes to match |
| INT-08 | emergency stop from S4 and S5 brakes S2 without the VC | IF-12 | manual | while patrolling: (a) hand 10 cm in front of the sensor; (b) lift the car | wheels stop at once; telemetry `estop=OBSTACLE` / `IMU_LIFTED`; RESET clears it |
| INT-09 | line events and steering: S3 → VC → S2 | IF-16, IF-17, IF-19 | manual | START on the line; then move the car off the line | follows the line (the two wheel speeds differ in bends); `LINE_LOST` → sweep → `LINE_REACQUIRED` → patrolling |
| INT-10 | Motion Control's speed and distance reach the IMU (hump length and height use them) | IF-20 | auto (live) | `subsystem4_test.c` S4-P03 | `[TEST] S4-P03 PASS`: sequence changes within 100 ms |
| INT-11 | IMU events, lift cause, IMU fault bits and tilt reach the dashboard through the VC and S1 | IF-14, IF-22, IF-06 | manual | dashboard while driving over the hump, tilting the car by hand, and with the IMU SDA wire unplugged | hump height and tilt shown; "IMU fault bits: NACK" when unplugged; lifted vs overturned told apart |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-23 | INT-05 | `ad50553` | Buddy 5 (Reuben), for the team | PASS | `../subsystem1/logs/2026-11-23_host_tests.txt`, `../subsystem1/logs/2026-11-23_ground_selftest.txt` | C codec and the Python decoder agree on the 22 B v4 vector |
| 2026-11-23 | INT-06 | `ad50553` | Buddy 5 (Reuben), for the team | FAIL (known limits) | `../system/logs/2026-11-23_pc_checks.txt` (`[bypass]`) | 702 of 792 simulated runs correct, none wrong with a 100 mm long box; details in `vehicle_ctrl/TEST_LOG.md` VC-S01 |
| 2026-11-23 | INT-04, INT-10 | `ad50553` | Buddy 5 (Reuben), for the team | PASS (builds) | `../system/logs/2026-11-23_build.txt` | in the test image, which compiles with no warnings; run it on the car for the results |
