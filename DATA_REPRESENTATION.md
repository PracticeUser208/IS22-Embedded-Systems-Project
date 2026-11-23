# IS22 Data Representation

Minimum-bit data representation (Tribunal 1 change request): every state,
event, flag and value the subsystems exchange, with its valid range, the bits
it uses and where it is stored. Values are taken from the code in
`app_program/`; the header named in each table is the one place the value is
defined.

Rule for messages between tasks: one 32-bit word per message, packed with
shifts and masks, never text.

## 1. Shared stop and fault bits (`app_program/shared_bits.h`)

### Stop requests: `flg_estop_req` (3 bits)

| Bit | Name | Set by | Meaning |
|---|---|---|---|
| 0 | `ESTOP_IMU_LIFTED` | IMU (Buddy 4) | car lifted or overturned (since 16 Nov; was IMU impact) |
| 1 | `ESTOP_IMU_FAULT` | IMU (Buddy 4) | IMU not working |
| 2 | `ESTOP_OBSTACLE` | Obstacle Scan (Buddy 5) | obstacle within 15 cm ahead |

Motion Control checks these every 20 ms control tick, before its command.
Only the Vehicle Controller clears them.

### System faults: `flg_sysfault` (8 bits)

| Bit | Name | Owner |
|---|---|---|
| 0 | `FAULT_MOTOR_STALL` | Motion Control (Buddy 2) |
| 1 | `FAULT_ENCODER` | Motion Control (Buddy 2) |
| 2 | `FAULT_IMU` | IMU (Buddy 4) |
| 3 | `FAULT_NO_ECHO` | Obstacle Scan (Buddy 5) |
| 4 | `FAULT_WIFI_LOSS` | Telemetry (Buddy 1) |
| 5 | `FAULT_LINE_LOST` | Line and Barcode (Buddy 3) |
| 6 | `FAULT_NO_PATH` | Vehicle Controller |
| 7 | `FAULT_STARTUP` | Vehicle Controller |

## 2. States and enumerations (`subsystem1/telem_format.h`)

| Value list | Values | Count | Bits |
|---|---|---|---|
| System state | INITIALISING, IDLE_READY, PATROLLING, PAUSED, HANDLING_BARCODE, HANDLE_HUMP, OBSTACLE_AVOIDANCE, LINE_RECOVERY, FAULT_ESTOP, STARTUP_ERROR | 10 | 4 |
| Drive mode | FOLLOW_LINE, OPEN_LOOP | 2 | 1 |
| Line state | INIT, FOLLOWING, BARCODE_SCAN, DECODE, SUSPENDED, SEARCH, FAULT | 7 | 3 |
| Barcode action | NONE, LEFT, RIGHT, STRAIGHT, UTURN, INVALID | 6 | 3 |
| Motion class | STATIONARY, ACCELERATING, TURNING, CLIMBING_HUMP, DESCENDING_HUMP, LIFTED, STEADY | 7 | 3 |
| Obstacle plan | NONE, STOP, CONTINUE, TURN_LEFT, TURN_RIGHT, REVERSE, BLOCKED | 7 | 3 |
| Obstacle state | PATROLLING, DETECTED, COARSE_SCAN, FINE_SCAN, PROFILE_PLAN, BYPASSING, REACQUIRE, BLOCKED, SENSOR_FAULT | 9 | 4 |
| Remote command | NONE, START, STOP, PAUSE, RESUME, RESET | 6 | 3 |
| Command result (ACK) | FORWARDED, DUPLICATE, INVALID, QUEUE_FULL | 4 | 2 |

Each list has a compile-time check (`_Static_assert`) that it fits its field.

## 3. Messages to the PC (MQTT, `subsystem1/telem_format.h`)

Frames are bit-packed binary (`subsystem1/subsystem1.c`, bit packer), LSB first.

| Topic | Direction | Bits | Bytes |
|---|---|---|---|
| `is22/tm` telemetry | car -> PC, every second | 148 | 19 |
| `is22/hb` heartbeat | car -> PC | 51 | 7 |
| `is22/cmd` command | PC -> car | 8 | 1 |
| `is22/ack` command result | car -> PC | 10 | 2 |
| `is22/bc` barcode read | car -> PC | header 15 B + text 4 B + up to 59 bar widths | up to 74 |
| `is22/ir` side-sensor trace | car -> PC | header 4 B + 50 samples | 54 |

### Telemetry frame fields (`is22/tm`)

| Field | Bits | Range / unit |
|---|---|---|
| version | 3 | 2 |
| sequence | 8 | 0-255, wraps |
| system state | 4 | table 2 |
| drive mode | 1 | table 2 |
| speed left, right | 7 + 7 | -64..63 cm/s |
| encoder left, right | 12 + 12 | edge count mod 4096 |
| distance | 14 | 0-16383 cm |
| IR left, centre, side | 1 + 1 + 1 | 1 = on the line |
| line state | 3 | table 2 |
| barcode | 3 | table 2 |
| hump last, highest | 7 + 7 | 0-127 mm |
| hump count | 4 | 0-15 |
| motion class | 3 | table 2 |
| obstacle distance | 9 | 0-510 cm; 511 = no echo |
| obstacle width | 6 | 0-63 cm |
| obstacle angle | 5 | index n = 30 + 5n deg (0-24); 31 = none |
| clearance left, right | 6 + 6 | 0-63 cm (63 = 63 cm or more) |
| obstacle plan | 3 | table 2 |
| obstacle state | 4 | table 2 |
| faults | 8 | `flg_sysfault` bits |
| stop sources | 3 | `flg_estop_req` bits |

### Heartbeat (`is22/hb`)

version 3, sequence 8, system state 4, faults 8, uptime 16 (seconds,
saturates at 65535), frames dropped 8, MQTT reconnects 4.

### Command and ACK

Command: code 3 bits (1-5) + sequence 5 bits. ACK: the command's 8 bits +
result 2 bits.

## 4. Messages between tasks (32-bit words)

### `obst_q`: Obstacle Scan -> Vehicle Controller (`subsystem5/subsystem5.h`)

| Bits | Field | Range |
|---|---|---|
| 31-29 | event type | 0 LOOK, 1 OBSTACLE, 2 PATH_CLEAR, 3 BYPASS_PLAN, 4 NO_PATH, 5 BYPASS_DONE, 6 SENSOR_FAULT, 7 FAULT_CLEARED |
| 28-27 | side | 0-3 |
| 26 | flag | 0-1 |
| 25-17 | distance | 0-511 cm |
| 16-9 | width (or fault bits) | 0-255 |
| 8-0 | clearance | 0-511 cm |

### `scan_q`: Vehicle Controller -> Obstacle Scan

Bits 31-29 command (1 STOP, 2 START_SCAN, 3 MOVE_DONE, 4 RESET, 5 PATROL_OFF,
6 PATROL_ON, 7 BYPASS); bits 28-0 argument (bypass watch or look request).

### Obstacle Scan status word (read by the VC and telemetry, one 32-bit store)

| Bits | Field |
|---|---|
| 2-0 | state (8 states: PATROL, DETECTED, COARSE, FINE, PLAN, BYPASS, BLOCKED, FAULT) |
| 3 | pinging |
| 11-4 | fault bits |
| 20-12 | last distance, 0-511 cm |
| 28-21 | servo angle |
| 30-29 | retries (0-2, `S5_MAX_RETRIES`) |

Ultrasonic reading (16 bits): status 3 bits (OK, OPEN, NONE, BUSY, STUCK) +
distance 9 bits. Scan table: 25 slots (-60..+60 deg in 5 deg steps), each a
9-bit distance stored in 16 bits: 50 bytes (was about 240).

### `imu_evt_q`: IMU -> Vehicle Controller (`vc_imu_msg.h`)

| Bits | Field | Range |
|---|---|---|
| 1-0 | event type | 0 HUMP, 1 MOTION, 2 LIFTED, 3 SENSOR_FAULT |
| 8-2 | payload | HUMP: height in 0.5 mm steps (127 = over range); MOTION: 0 STATIONARY, 1 ACCELERATING, 2 TURNING, 3 STEADY; LIFTED: cause bits (1 upward, 2 tipped) |
| 31-16 | time stamp | ms, low 16 bits |

`imu_q` (VC -> IMU): one command, 0 = RESET (1 bit).

### Motion Control snapshot for the IMU (`subsystem4/subsystem4.h`)

One aligned 32-bit word, read-only for the IMU: speed 10 bits (0-1023 mm/s),
distance 16 bits (mm, wraps), sequence 6 bits (moves every 20 ms tick, so the
IMU can tell a stale value).

### `nav_q`: Line and Barcode -> Vehicle Controller (`vehicle_ctrl.h`)

Bits 31-29 event (1 BARCODE, 2 LINE_LOST, 3 LINE_REACQUIRED, 4 LINE_TIMEOUT,
5 SENSOR_FAULT, 6 JUNCTION); bits 2-0 barcode action (table 2).

### `line_cmd_q`: Vehicle Controller -> Line and Barcode

Bits 31-29 command: 1 SUSPEND, 2 SEARCH, 3 FOLLOW, 4 RESET.

### Motion commands (`mbf_motion_cmd`, `subsystem2/subsystem2.h`)

`motion_cmd_t`: type (STOP, MOVE, TURN, VELOCITY), value (signed mm or deg),
cruise speed and left / right wheel speeds (16-bit mm/s). The buffer holds 4
commands. Motion Control's own status is read with `motion_get_status()`.

## 5. IMU internal fields (`subsystem4/imu.h`, section `app_fields.h`)

Buddy 4's table of the IMU's own fields, each with its bit width and a
compile-time check: motion state 3 bits, speed 10 bits (mm/s), turn angle 8
bits (0-180 deg), distance and encoder counts 16 bits, PWM duty 12 bits, IR raw
and threshold 12 bits, steering offset 8 bits (-127..127), obstacle distance 9
bits (0-400 cm), echo time 16 bits (0-38000 us), scan indexes 3 bits (coarse)
and 5 bits (fine), obstacle width 7 bits, IMU faults, lift causes and IMU
states.
