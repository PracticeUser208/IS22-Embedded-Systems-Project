# System — test log

Tests of the complete product against its requirements, run on the final hardware set-up. Run the full set before every demo and before hand-in.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | Obstacle on the line: the car stops, scans, goes round it on a side with room, rejoins the line and patrols again (Obstacle Scan + Vehicle Controller) | FR15–FR18 | manual | 1. Put the 10 × 10 cm box centred on the line, 1 m ahead of the car. 2. Dashboard: RESET, START. 3. Watch the car and the dashboard event list. | stops 15 cm or less before the box; scan; goes round without touching it; back on the line; state PATROLLING again |
| SYS-02 | Side choice with the obstacle off the line (19 Nov: goes round the edge the obstacle reaches out less to; was always left when both sides were open) | FR17 | manual | 1. Put the box so that most of it is left of the line (its right edge about 2 cm right of the line). 2. RESET, START; note the side. 3. Repeat with most of the box right of the line. | box mostly left: bypass on the right (dashboard plan `TURN_RIGHT`); box mostly right: bypass on the left (`TURN_LEFT`) |
| SYS-03 | Bypass length past a small obstacle (19 Nov: along the lane at least 20 cm, then 5 cm steps; obstacle taken as at least 12 cm deep) | FR16–FR18 | manual | 1. Box centred on the line. 2. Tape the floor where the car turns out and where it turns back towards the line. 3. RESET, START. 4. Measure between the two marks. | at least 20 cm and about 45 cm for the 10 × 10 cm box (about 52 cm before 19 Nov); the car does not touch the box |
| SYS-04 | Blocked on both sides: the car backs up, scans again and gives up | FR17, Week 6 feedback (retry limit) | manual | 1. Box on the line with walls (books) beside it on both sides. 2. RESET, START. | backs up and rescans twice, then `FAULT_ESTOP` with fault `NO_PATH` on the dashboard; RESET → `IDLE_READY` |
| SYS-05 | Lifted and overturned are told apart on the dashboard (19 Nov: telemetry version 3 carries the lift cause from the Vehicle Controller) | FR13 (revised) | manual | Firmware built from `IS22-Template` (`MQTT setup - Template`). 1. While patrolling, lift the car about 15 cm. 2. RESET, START. 3. While patrolling, tip the car onto its side. | car stops both times; event list shows `IMU: lifted (moved up)` the first time and `IMU: overturned (tipped over)` the second; status line says lifted / overturned |
| SYS-06 | IMU fault shown on the dashboard without the USB console (19 Nov: the Vehicle Controller sends the IMU fault bits in telemetry) | FR13 (revised), A12 | manual | 1. Power off. Unplug the IMU's SDA wire (GP2). 2. Power on, open the dashboard, START. 3. Power off and plug the wire back in. | event `IMU fault bits: NACK` (or `CAL_FAIL`) and a red `IMU NACK` chip; the car does not drive (`FAULT_ESTOP`) |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-20 | SYS-02 | `e4e5285` + uncommitted changes | Buddy 5 | FAIL | dashboard (no log saved) | 10 × 10 cm box placed mostly left of the line; scan width read 3 cm, both sides open; car went LEFT. Cause: a flat face only echoes where it is square-on to the sensor, so the scan saw a 3 cm slice in front of the car and the box looked centred (a centred box goes left). The bypass still cleared the box without contact (FR18). Not changed in code: a rule based on a 3 cm slice is still a guess. See deviation D5 below |
| 2026-11-23 | build | `ad50553` | Buddy 5 (Reuben), for the team | PASS | `logs/2026-11-23_build.txt`, `logs/2026-11-23_pc_checks.txt` | GitHub main builds (Wi-Fi + MQTT, and the test image) with no warnings after the 23 Nov sync; all PC checks give the same output as the code on the car. SYS-01…SYS-06 still to be run on the car |

## Design deviations from Week 6 / Tribunal 1 (checked 21 Nov)

Where the code does not do exactly what the Week 6 design review or Tribunal 1 described, and why. Each one was kept on purpose; the Tribunal 2 requirements and block diagram describe what the code does.

| ID | Week 6 / Tribunal 1 said | The code does | Reason |
|---|---|---|---|
| D1 | Priorities Line (2) > Motion (3) > Vehicle Controller (4) > Obstacle Scan = IMU (5) > Telemetry (6) | TEAM.md bands: Vehicle Controller 11 > Telemetry 13 > Motion 14 > Line 16 > Obstacle Scan 18 > IMU 20 (Tribunal 1: Obstacle Scan above IMU) | Each buddy was given a band so tasks could be developed separately. Every task blocks on a timer or event flag and runs briefly, so the order rarely decides anything; the emergency stop does not depend on it (ESTOP_REQ flag read by Motion Control every 20 ms). Telemetry only does non-blocking work (TMO_POL). Changing the order means editing every subsystem's header, so it was left as is |
| D2 | Line and Barcode every ~2 ms | every 10 ms (one kernel tick) | The µT-Kernel tick is 10 ms (Tribunal 1 feedback). Line lost after 600 ms bridges 1 cm gaps at patrol speed (NFR10) |
| D3 | FR15: slow down when an obstacle is beyond 15 cm | the car already patrols at 25 mm/s (since 2 Nov), so the "slow" speed under 50 cm is the same 25 mm/s; it stops within 15 cm | Going slower than 25 mm/s stalls the motors. FR15 now reads "approach at crawl speed (25 mm/s)" |
| D4 | Coarse scan at 5 angles, 30°–150° (±60°) | 4 angles: −45°, −30°, 0°, +30° | The sensor hits the Robo Pico board past +30° left and −45° right (test S5-U20) |
| D5 | FR17: the side with more clearance | room on both sides: the side the obstacle reaches out less to; otherwise the side with more room (19 Nov) | Week 6 rule went left whenever both sides were open. Limit: on a small flat-faced box the scan can see only a slice (SYS-02, 20 Nov) |
| D6 | FR19: line not found within 2 s → LINE_LOST_TIMEOUT | 4 s, event LINE_TIMEOUT | The search sweep needs about 2.9 s to look both ways (2 Nov) |
| D7 | FR20: Line and Barcode is a critical subsystem at start-up | only Motion Control and the obstacle sensor's timer/interrupt stop start-up; without Line and Barcode the car still reaches IDLE_READY and warns on the console | Kept so the car could be integrated and tested before Line and Barcode was merged (it then patrols straight). A line that is not found while driving is still caught as LINE_LOST / LINE_TIMEOUT |
| D8 | FR13: sudden impact | lifted / overturned (16 Nov) | Requirement revised; the motors stop on either |
| D9 | — | the template's example LED task (blinks GP19, priority 12) still starts | Leftover template example code that was not removed. It wakes every 250 ms, uses a 2 KB stack (under 1 % of RAM) and does not affect driving |
