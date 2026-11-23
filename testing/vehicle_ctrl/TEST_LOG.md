# Vehicle Controller — test log

The Vehicle Controller (`app_program/vehicle_ctrl.c`, `task_vc.c`, `vehicle_ctrl.h`) runs the system-level
statechart (Week 6 report, section 5). Owned by the whole team.

## Tests

| ID | What | Traces to | Method | How | Pass when |
|---|---|---|---|---|---|
| VC-U01 | Bypass on the left: legs and distances | FR16–FR18, statechart 6.5 | auto | `vehicle_ctrl_test.c` | `[TEST] VC-U01 PASS` |
| VC-U02 | Bypass on the right mirrors it | FR17 | auto | `vehicle_ctrl_test.c` | PASS |
| VC-U03 | Side-step limited by the measured room | FR17 | auto | `vehicle_ctrl_test.c` | PASS |
| VC-U04 | No side → no bypass | FR17 | auto | `vehicle_ctrl_test.c` | PASS |
| VC-U05 | Return to the original path | FR18, FR19 | auto | `vehicle_ctrl_test.c` | PASS |
| VC-U06 | Barcode turns, U-turn = 2 × left 90 | FR4, FR9 | auto | `vehicle_ctrl_test.c` | PASS |
| VC-U07 | Provisional IMU / line message formats | IF-14, IF-16 | auto | `vehicle_ctrl_test.c` | PASS |
| VC-U08 | Reaches IDLE_READY at start-up | FR20 (≤ 30 s) | auto | `vehicle_ctrl_test.c` | PASS |
| VC-U09 | Bypass: side check, pass, a turn stopped → further along, diagonally back to the line | FR16–FR18 | auto | `vehicle_ctrl_test.c` | PASS |
| VC-U10 | Bypass: stopped in the lane → rescan / move out; an endless obstacle → gives up | FR17, Week 6 feedback (retry limit) | auto | `vehicle_ctrl_test.c` | PASS |
| VC-S01 | Bypass in simulation: boxes 100–130 mm wide and 100–3000 mm long, on and off the line, three echo models, 24 runs each | FR16–FR18 | auto (PC) | `python3 testing/pc_checks.py bypass` (`testing/vehicle_ctrl/vc_bypass_check.c`) | car goes round without touching the obstacle; gives up when there is no way round |
| VC-M01 | START / PAUSE / RESUME / STOP from the ground station | FR22, state gating | manual | `is22_ground.py send …` | car drives / stops; commands in the wrong state print "ignored in this state" |
| VC-M02 | Obstacle stop, scan and bypass | FR15–FR18 | manual | box in the car's path while patrolling | stop ≤ 15 cm, scan, bypass on the side with room, back on the path, patrolling again |
| VC-M03 | Blocked → back up → rescan → give up | Week 6 feedback (retry limit) | manual | box + walls both sides | backs up twice, then FAULT_ESTOP with fault bit 0x40 (FAULT_NO_PATH); RESET → IDLE_READY |
| VC-M04 | Ultrasonic fault and recovery | statechart 5 Fault | manual | unplug the echo wire while patrolling, plug back, send RESET | FAULT_ESTOP, estop 0x4; after RESET → IDLE_READY |
| VC-M05 | Telemetry carries real values | FR21 | manual | ground station `monitor` | sys_state, speeds, encoder counts, obstacle distance, estop and fault bits change with the car |
| VC-M06 | Obstacle distance stays live while standing still | statechart 6.5 (DETECTED waits for the VC) | manual | car IDLE_READY, hand 5 cm in front of the sensor, then take it away | `estop=OBSTACLE` while the hand is there; within ~1 s of removing it, `obst=` shows the new distance and `estop=-` |
| VC-M07 | Patrol follows the line (S3 steering applied) | FR1, IF-19 | manual | START with the car on the line | telemetry `line=FOLLOWING`, the two `spd=` values differ in bends, car stays on the line |
| VC-M08 | Line lost → LINE_RECOVERY sweep → back to PATROLLING | FR19 | manual | move the car off the line while patrolling | sweeps left/right on the spot; PATROLLING again once found |
| VC-M09 | Line search time-out | FR19 | manual | take the car away from the line while patrolling | FAULT_ESTOP, fault 0x20; RESET → IDLE_READY |
| VC-M10 | IMU events (Buddy 4's format): hump recorded with peak; lifted / overturned (16 Nov, was impact) during a run → FAULT_ESTOP "lifted: run ended"; while standing by → no fault, START still works; RESET after a lift → IDLE_READY without waiting; IMU sensor fault → FAULT until the IMU reports cleared | FR11, FR12, FR13, IF-14/IF-15 | manual | ramp pass, soft bump, unplug/replug the IMU while idle | telemetry `hump=` last/peak, `motion=`, state and estop bits change as described |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-23 | VC-S01 | `ad50553` | Buddy 5 (Reuben), for the team | FAIL (known limits) | `../system/logs/2026-11-23_pc_checks.txt` (`[bypass]`) | 792 simulated bypasses, 702 correct; 80 touched the obstacle and 10 gave up although there was a way round, none with a 100 mm long box like the course obstacle. They are in the harder cases kept in on purpose: obstacles 250 mm or longer, odd shapes (a part sticking out, a second box in the lane, a wall beside it), mostly with the two harder echo models (echo only from flat faces seen square on, narrow beam). Same numbers as the code on the car. Real limit also seen on the car: SYS-02 |
| 2026-11-23 | VC-U01–U10 | `ad50553` | Buddy 5 (Reuben), for the team | PASS (builds) | `../system/logs/2026-11-23_build.txt` | the test image compiles with no warnings; run it on the car for the results |
