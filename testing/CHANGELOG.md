# Change log (for testing)

What changed in the code because of feedback from test runs or a change in
the requirements, newest first. Use it to decide which tests to run again.
Test IDs are the ones in the `TEST_LOG.md` files of each folder under
`testing/`. "PC check" means a test program that runs on a computer, named
in the last column.

Entries before 14 Nov are taken from the dated notes in the source files.

## 20 Nov

| Change | Why | Files | Test again |
|---|---|---|---|
| Hump, as asked (20:15): the level at RESET (the IMU calibrates then) is the baseline. A tilt of 2.5 deg or more away from it (nose up or down, or sideways) starts a hump; back within 1.5 deg for 20 mm of driving (`HUMP_CALM_MM`) ends it: one hump, reported at once with its height (larger of pitch x distance and 120 mm x sin(largest roll)), no smallest height. A stop on the hump pauses it; it ends after 1 m (`HUMP_MAX_LEN_MM`) anyway. Replaces the floor-level tracking and the 200 mm merge below. | Test 20 Nov 20:11-20:12: the hump was seen twice (roll -5.5 deg and back), but the report waited 200 mm after it (the merge), and the car was lifted / the IMU faulted before that, so nothing reached the dashboard. PC check: course hump 1 report (11-12.5 mm) also with the car leaning 1 / 0.8 deg and stopping on it; no false hump on the flat floor | `subsystem4/imu.h`, `subsystem4/subsystem4.c`, `testing/subsystem4/hump_check.c` | S4-H01, S4-M03, S4-M05 |
| Hump measured from the floor's own level: the tilt is followed slowly over 500 mm of driving (`HUMP_BASE_MM`) and a hump is the tilt away from that (was: away from the level at calibration). After a stop of 1 s or more, the first 100 mm only set that level (`HUMP_SETTLE_MM`). A stop on the hump only pauses it (it was discarded after 200 ms). Tilted for more than 500 mm (`HUMP_MAX_LEN_MM`): a slope, not reported. The height from the pitch counts either way round. | Test 20 Nov 20:05: no hump reported at all. The dashboard tilt showed the car leaning about 2 deg nose down and 1.5 deg sideways while driving on the flat, so the tilt never went back under the 1 deg end threshold and the hump never finished; and the speed reads 0 for a moment now and then while patrolling. PC check (course hump with the car leaning 2 / 1.5 deg, stops on the hump, a start 30 cm before it): version on the car 0 reports, now 1 (11 mm) | `subsystem4/imu.h`, `subsystem4/subsystem4.c`, `testing/subsystem4/hump_check.c` | S4-H01, S4-M03, S4-M05 |
| Hump: one report per hump. Tilts less than `HUMP_MERGE_MM` (200 mm) apart are one hump, reported (the highest) once the car is 200 mm past the last of them; starts at 2 deg (unchanged), ends below 1 deg (`HUMP_PITCH_EXIT_RAD` 0.015 -> 0.017), no smallest height (`HUMP_MIN_HEIGHT_MM` 2 -> 0, every hump is reported); tilt filter about 100 ms (`PITCH_LPF_ALPHA` 0.3 -> 0.1). | Test 20 Nov: one pass over the course hump (10 mm, one low peak, long ramps, one wheel) gave up to 5 reports, and driving on a flat floor tipped the car 2-3 deg (dashboard tilt), above the 2 deg start. PC check with the course hump and +-2.5 deg driving jolts: before 3 reports and 4 false humps in 10 min; now 1 report (11 mm) and none; a 5 mm hump of the same shape: 1 report (6.5 mm) | `subsystem4/imu.h`, `subsystem4/subsystem4.c`, `testing/subsystem4/hump_check.c` | S4-H01, S4-M03, S4-M05 |
| Hump under one wheel: the IMU now also measures the sideways tilt (roll). A hump starts when the pitch or the roll passes 2 deg; its height is the larger of pitch x distance and `HUMP_TRACK_MM` (120) x sin(largest roll). | Test 20 Nov: a 10 mm hump under the right wheel only was not detected: it tips the car sideways about 5 deg but hardly nose up. PC check `hump_check.c`: one-wheel humps of 5, 10, 20 mm now read 5.5, 10.5, 20.5 mm; no false hump on the flat floor | `subsystem4/imu.h`, `subsystem4/subsystem4.c`, `testing/subsystem4/hump_check.c`, `testing/subsystem4/bringup/imu_port_pico_bare.c` | S4-H01, S4-M03, S4-M05 |
| Telemetry version 4: the car's tilt (pitch and roll, 7 bits each, signed, 0.5 deg steps, the value furthest from 0 in the last 0.5-1 s) added at the end of `is22/tm` (157 -> 171 bits, 20 -> 22 B; `telem_q` message 20 -> 24 B). The IMU hands it to the VC as one 32-bit word (`vc_imu_tilt_word`). Dashboard IMU tile: "tilt: nose x deg, side y deg"; versions 2 and 3 still read. | To see the tilt over a hump on the dashboard (no USB console needed) | `subsystem1/telem_format.h`, `subsystem1/subsystem1.c`, `vc_imu_msg.h`, `task_vc.c`, `tools/buddy1/is22_ground.py`, `testing/subsystem1/host/test_codec.c`, `test_format.c`, `testing/week13/*` | S1 PC checks C1-C4, ground tool `selftest`, Week 13 numbers |

## 19 Nov

| Change | Why | Files | Test again |
|---|---|---|---|
| Telemetry version 3: the lift cause (2 bits: moved up / tipped over) and the IMU fault bits (7 bits) added at the end of `is22/tm` (148 -> 157 bits, 19 -> 20 B). The VC keeps both (lift cause until RESET, fault bits until the IMU reports the fault cleared). The dashboard shows "overturned" or "lifted" and the fault bits; it still reads version 2 frames. | Test 19 Nov: the dashboard showed "lifted" for an overturn, and an IMU fault could only be read on the USB console | `subsystem1/telem_format.h`, `subsystem1/subsystem1.c`, `task_vc.c`, `tools/buddy1/is22_ground.py`, `testing/subsystem1/host/test_codec.c`, `test_format.c` | S1 PC checks C1-C3, ground tool `selftest`, S4-M04 (look at the dashboard) |
| Obstacle bypass: along the lane at least 20 cm (`VC_PASS_FIRST_MM` 300 -> 200), then 5 cm steps while the obstacle is still beside or in the way of the turn back (`VC_PASS_STEP_MM` 100 -> 50); the obstacle is taken to be at least 12 cm deep (`VC_OBST_MIN_DEPTH_MM` 150 -> 120) and the turn back starts 2 cm past the safe point (`IN_EXTRA_MM` 50 -> 20). The turns stay as they were (90 deg out from the line and back). | Test 19 Nov: it drove about 50 cm along the lane for a 10 x 10 cm obstacle | `vehicle_ctrl.h`, `vehicle_ctrl.c` (`vc_bypass.c` section), `testing/vehicle_ctrl/vc_bypass_check.c` | VC-U09 (PC check `bypass`), VC-M02 |
| Obstacle scan, side choice: when both sides have room (`S5_MIN_CLEAR_CM`), the car goes round the edge the obstacle reaches out less to (was the side with more room, and left whenever both were open). | Test 19 Nov: an obstacle placed left of the line, and the car still went round it on the left | `subsystem5/subsystem5.c` (`s5_plan`), `subsystem5_test.c` | S5-U21 (new), S5-U03, S5-U19, VC-M02 |
| Hump: starts at 2 deg of pitch (`HUMP_PITCH_ENTER_RAD` 0.07 -> 0.035), ends below 0.9 deg (`HUMP_PITCH_EXIT_RAD` 0.03 -> 0.015), smallest hump reported 2 mm (`HUMP_MIN_HEIGHT_MM` 3 -> 2). Console: the pitch range while moving every 10 s (`S4: pitch while moving ...`). | Test 19 Nov: the new 5 mm hump was not detected. A 5 mm hump tilts the car only 2-3 deg (PC check `hump_check.c`), below the 4 deg start | `subsystem4/imu.h`, `subsystem4/subsystem4.c`, `testing/subsystem4/hump_check.c` | S4-H01 (PC check), S4-M03, S4-M05 |

## 17 Nov

| Change | Why | Files | Test again |
|---|---|---|---|
| Obstacle bypass, side-step: out by half the measured width (at least `VC_OBST_MIN_WIDTH_MM`) + half the car + 5 cm, at least 15 cm (was the whole width + 10 cm, at least 20 cm). | Feedback (video of the 14:04 run): the scan measured the tub as 14 cm, so the car went 24 cm out and passed 10-12 cm from it: "the clearance is too much" | `vehicle_ctrl/vc_bypass.c`, `vehicle_ctrl.h` (`VC_BYPASS_MARGIN_MM` 100 -> 50, `VC_SIDE_STEP_MM` 200 -> 150) | VC-U09, VC-U10, VC-M02 |
| Obstacle bypass: when the turn back towards the line is stopped because the obstacle is still there, the car only turns back and goes further along the lane, then checks again. No side-step further out any more. | Feedback: undoing the turn and then turning 45 deg out "did a 90 degree turn again as if detecting a new object", and took the car further from the line every time | `vehicle_ctrl/vc_bypass.c`, `vc_bypass.h`, `vehicle_ctrl.h` (`VC_JOG_MM` now only for the watch along the lane) | VC-U09, VC-U10, VC-M02 |
| Obstacle bypass: turns 70 deg towards the line to cross back (was 45 deg). The obstacle is taken to be at least 15 cm long (`VC_OBST_MIN_DEPTH_MM` 100 -> 150). | At 45 deg the car came back onto the line about 27 cm past a 15 cm obstacle, at 70 deg about 18 cm; at 10 cm it turned in beside an obstacle 15 cm deep | `vehicle_ctrl/vc_bypass.h` (`VC_BYP_IN_DEG`), `vc_bypass.c`, `vehicle_ctrl.h` | VC-U09, VC-M02, PC check `vc_bypass_check.c` |
| Barcode turn at a junction: the car turns 10 mm sooner (`VC_JCT_AHEAD_MM` 83 -> 73). | Tuned on the course after the 15 Nov change (turning at the junction); the note in `vehicle_ctrl.h` gives the new value, not a measurement | `vehicle_ctrl/vehicle_ctrl.h` | VC-U06, S3-LF (LF-25, LF-26) |
| Template copy rebuilt in the template's layout: each subsystem in `subsystemN.h`, `subsystemN.c`, `subsystemN_test.c` and one task file (`task_comm.c`, `task_motion.c`, `task_line.c`, `task_monitor.c`, `task_obstacle.c`); subsystem 1 keeps `telem_format.h` and `comm_bridge.h` (the lwIP library includes them), subsystem 4 keeps Buddy 4's IMU headers in `imu.h` (they clash with `telem_format.h`); the Vehicle Controller in `app_program/` (`vehicle_ctrl.h`, `vehicle_ctrl.c`, `vehicle_ctrl_test.c`, `task_vc.c`, `vc_imu_msg.h`); PC checks and the IMU bring-up in `testing/`, built with `python3 testing/pc_checks.py`. Code taken unchanged from the working project; S4-U03 and INT-04 from the team repo added to the test build. No change to what the car does: compiled without inlining, every function and table is the same as the working project's, and the PC checks print the same output | Submission follows the template's files per subsystem | all of `app_program/`, `testing/pc_checks.py` | build (`make -j8`, `make TEST=1`), auto tests (S4-U03, INT-04 new), `python3 testing/pc_checks.py` |

In `telemetry_2026-11-17.txt` the last bypass of the day (15:41) went
round and back to PATROLLING in 57 s; the 14:04 run took 75 s.

## 16 Nov

| Change | Why | Files | Test again |
|---|---|---|---|
| Code moved into the course template (`INF2004_ProjectTemplate`). The template's own comments are back where they had been shortened or dropped; its examples for subsystems 2, 3 and 4 stay in the files, switched off with `TEMPLATE_DEMO 0`. No change to what the car does: the firmware image is byte-for-byte the same as before the move. | Submission has to follow the template | `app_program/subsystem1..5/subsystemN.*`, `subsystem2/task_producer.c` | build (`make -j8`, `make TEST=1`), then the auto tests |
| IMU: collision (impact) detection removed. The IMU now detects the car being lifted or overturned, the motors stop at once, and a lift during a run ends the run (FAULT_ESTOP "lifted: run ended", estop `IMU_LIFTED`). Lifted while idle: nothing ends, START still works. | Requirement changed (lecturer's reply of 11 Nov: collision detection conflicts with collision avoidance; lifting the robot ends the demonstration run) | `subsystem4/imu_logic.c`, `imu_logic.h`, `imu.c`, `imu_cfg.h` (`LIFT_xxx`), `app_fields.h`, `shared_bits.h`, `vehicle_ctrl/task_vc.c`, `subsystem1/telem_format.h`, `tools/buddy1/is22_ground.py` | S4-M04, S4-M05, S4-L01 (PC check `lift_check.c`), VC-M10 |
| Obstacle bypass, side-step: out by the measured width + 10 cm, never less than 20 cm (was half the width + half the car + 12 cm). | Feedback: the car went too far out to the side. The scan measured a 13 cm obstacle as 2-9 cm, so the distance could not depend on the width alone | `vehicle_ctrl/vc_bypass.c`, `vehicle_ctrl.h` (`VC_SIDE_STEP_MM`, `VC_BYPASS_MARGIN_MM`) | VC-U09, VC-U10, VC-M02 |
| Obstacle bypass, passing: 30 cm along the obstacle, then a check to the side with the servo (the car does not turn); still there: 10 cm more and check again; clear: one 45 deg turn towards the line and diagonally back to it. Was about 60 cm before the first look. | Feedback: it ran too far parallel to the line before turning back; "go 30 cm then check" | `vehicle_ctrl/vc_bypass.c`, `task_vc.c`, `vehicle_ctrl.h` (`VC_PASS_FIRST_MM`, `VC_PASS_STEP_MM`), `subsystem5/task_obstacle.c` (`S5_BYP_SIDE_LEFT/RIGHT`), `subsystem5.h` | VC-U09, VC-M02, PC check `vc_bypass_check.c` |
| Obstacle bypass: obstacle still in the way when turning in -> 5 cm further out, then check again. Side-steps after the first one use 45 deg turns, not 90 deg left and right. | Feedback: "if it scans go a further 5 cm out"; no 90 deg turns for the later moves | `vehicle_ctrl/vc_bypass.c`, `vehicle_ctrl.h` (`VC_JOG_MM`) | VC-U09, VC-U10, VC-M02 |
| Line following: following one edge of the line with a single sensor was tried against the zigzag and taken out again the same day. Nothing of it is left in the code. | Feedback: the car shook more with it | none | S3-M03 |

Known limits of the 16 Nov bypass, from the PC check: obstacles about 10 cm
long are passed in every run; from 25 cm long, or with parts sticking out,
the car still touches in some runs. Not tried on the car yet.

## 15 Nov

| Change | Why | Files | Test again |
|---|---|---|---|
| A barcode's turn is made at the next junction, not at the barcode. The side sensor reports the crossing tape as a junction; the car drives its axle onto the junction and turns there. The U-turn is made at the junction too. | Feedback: the junctions on the course are full "+" crosses and the turn belongs at the junction | `subsystem3/line_cfg.h` (`LF_JCT_xxx`), `line_follow.c`, `task_line.c`, `vehicle_ctrl/vehicle_ctrl.h` (`VC_BARCODE_ACT 2`, `VC_NAV_JUNCTION`), `task_vc.c` | S3-LF (LF-25, LF-26), VC-U06 |
| Line sensors taken as 107 mm ahead of the wheels' axle. | Measured on the car: 10.5-11 cm | `vehicle_ctrl/vehicle_ctrl.h` (`VC_IR_AHEAD_MM`) | VC-M02 |
| Obstacle bypass: passes closer (margin 140 -> 120 mm), turns 45 deg towards the line instead of 90 deg, and Obstacle Scan watches that turn. Replaced again on 16 Nov. | Feedback: the bypass went too far out and used only 90 deg turns | `vehicle_ctrl/vc_bypass.c`, `task_vc.c`, `subsystem5/task_obstacle.c` (`S5_BYP_TURN_LEFT/RIGHT`) | VC-U09, VC-M02 |

## 14 Nov

| Change | Why | Files | Test again |
|---|---|---|---|
| No code change. The side sensor read the 1 cm bars but not the 0.4 cm ones; it was fixed lower, close to the floor, and then read barcodes at driving speed. | Feedback from the barcode runs | none (hardware) | S3-BC |

## 12 Nov

| Change | Why | Files | Test again |
|---|---|---|---|
| Obstacle bypass decided one move at a time instead of a fixed 4-move route. Obstacle Scan watches during every move and can stop the car. | The fixed route assumed an obstacle 25 cm long; a longer one, or one with a part sticking out, was driven into | `vehicle_ctrl/vc_bypass.c`, `vc_bypass.h`, `task_vc.c`, `subsystem5/task_obstacle.c` (`S5_CMD_BYPASS`), `subsystem5.h` | VC-U09, VC-U10, VC-M02, VC-M03 |

## 8 Nov

| Change | Why | Files | Test again |
|---|---|---|---|
| Barcode decoding: wide/narrow rule 1.3 x -> 1.2 x. The bar-shape reader was removed. | Tuned on the 154 passes recorded on 4 Nov: K read 11 times instead of 6, never a wrong letter | `subsystem3/s3_code39.c`, `bc_rec.c` | S3-BC (BC-11, BC-12) |

## 4 Nov

| Change | Why | Files | Test again |
|---|---|---|---|
| Stronger steering for tight bends. | The car lost the line on the R110 hairpin and the R150 S-bend every few seconds | `subsystem3/line_cfg.h` (`LF_STEER_HARD` 15 -> 21, `LF_C_KP_PER_1000`) | S3-M03 |
| Bar widths measured by distance travelled, not by time. | The car weaves over the barcode: the left wheel went between 0 and 30 mm/s, so a thin bar could last longer than a thick one | `subsystem3/task_line.c`, `line_cfg.h` (`LF_BC_BY_DISTANCE`) | S3-BC (BC-14) |
| Side sensor counts as black at 20 %. | Printed bars read much lighter than the tape | `subsystem3/line_cfg.h` | S3-BC |
| A recording is still decoded when STOP comes just after the barcode. | A good recording was thrown away by a STOP 4 s later | `subsystem3/task_line.c` | S3-BC |
| No code change: the side sensor's drop-outs came from the centre sensor's DO wire sharing GP26. Wire removed. | Car log | none (hardware) | S3-M01 |

## 3 Nov

| Change | Why | Files | Test again |
|---|---|---|---|
| Side sensor trace sent to the dashboard (`is22/ir`); transmit queue 4 -> 8. | Needed to see what the side sensor reads over a barcode | `subsystem1/telem_format.h`, `subsystem1.h` (`S1_TX_Q_DEPTH`), `subsystem3/task_line.c` | S1 auto tests |
| Barcodes are read only while the car follows the line. | Picking the car up made the side sensor "see" bars | `subsystem3/line_cfg.h` (`LF_BC_READ_STANDING`) | S3-BC |
| Faint printed bars: a second look at the recording with lower thresholds. | The printed bars were much lighter than the tape the sensor calibrated on | `subsystem3/bc_rec.c`, `bc_rec.h`, `line_cfg.h` | S3-BC (BC-10) |

## 2 Nov

| Change | Why | Files | Test again |
|---|---|---|---|
| Each wheel held at its speed by its encoder (speed PID); feed-forward values measured again. | A "20 mm/s" target ran at 150-220 mm/s and the left wheel turned 25 % more than the right | `subsystem2/motion_config.h` (`MOTION_SPEED_FEEDBACK`), `task_motion.c` | S2 auto tests, S3-M03 |
| Timed hard turns that stopped one wheel removed. | Jerky | `subsystem3/line_cfg.h`, `line_follow.c` | S3-M03 |
| Line search time-out 2 s -> 4 s. | FR19 says 2 s; the sweep needs 4 s to cover both sides | `subsystem3/line_cfg.h`, `vehicle_ctrl/vehicle_ctrl.h` (`VC_LINE_TIMEOUT_MS`) | S3-M04, S3-M05, VC-M09 |
| IMU speed limits lowered to 10 mm/s. | They were above the 20 mm/s patrol speed, so humps were never measured | `subsystem4/imu_cfg.h` (`HUMP_MIN_SPEED`, `MOTION_STOP_SPEED`) | S4-M03 |
