# Team and subsystems

Fill this in at the start of the project and keep it up to date. It is the
single place that says **who owns what**, so that five people can work in
one firmware image without clashing over pins, priorities, or kernel objects.

A **subsystem** is one student's part of the product, with its own folder.
It may contain several tasks and own several peripherals (see section 3).

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 1 | Telemetry / Communications: binary telemetry, heartbeat and remote commands over Wi-Fi / MQTT | 2500510 | magiccc-abc | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Motion Control: motors, wheel encoders, PID speed control, move / turn API | 2500261 | Bungo | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | Line and Barcode: IR line following, barcode decoding, line search | 2502978 | ashwinkkkk | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | IMU and Terrain: hump detection and height, motion state, lifted / overturned | 2503710 | PracticeUser208 | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Obstacle Scan: ultrasonic sensor, scan servo, obstacle profiling, bypass side | 2501748 | ReubenjHoh | `app_program/subsystem5/` | `testing/subsystem5/` |
| VC | Vehicle Controller: system state machine, runs the other subsystems, builds telemetry, acts on commands | whole team | - | `app_program/vehicle_ctrl.c`, `task_vc.c`, `vehicle_ctrl.h`, `vc_imu_msg.h` | `testing/vehicle_ctrl/`, `testing/system/` |

Shared files (`app_program/app_main.c`, `app_program/app.h`, this file)
are changed only by agreement of the team. The exception is claiming an empty
row in this file (a pin, a peripheral, an interface ID), which anyone may do
(section 2).

## 2. Pins

One owner per pin. Claim a pin here **before** you wire or code it.

**Why:** two subsystems using the same pin is one of the most common and
hardest-to-find mistakes in a team project. The code builds without any
error, but the hardware misbehaves. Each subsystem can pass its own tests
and then fail once they are combined, and two outputs driving the same pin
against each other can damage it. Writing the claim down first means a clash
is spotted when someone edits this table, not hours later on the bench.

**No instructor check needed.** The team keeps this table itself. You do not
need the instructor, or the rest of the team, to check or approve a claim:
if the row is empty, fill it in and commit. If two people claim the same
pin, they agree between themselves who moves. Only the *reserved* rows are
fixed.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | *crash report* | UART0 TX, only in a `CRASH_REPORT=1` build; otherwise free |
| GP1 | *crash report* | UART0 RX, only in a `CRASH_REPORT=1` build; otherwise free |
| GP2 | S4 | IMU I2C1 SDA (Maker / Grove 2 port) - Week 6 pin map |
| GP3 | S4 | IMU I2C1 SCL (Maker / Grove 2 port) - Week 6 pin map |
| GP4 | S5 | HC-SR04+ TRIG (Grove 3) |
| GP5 | S5 | HC-SR04+ ECHO (Grove 3, 3V3 sensor, no divider; edge interrupt) |
| GP6 | - | free |
| GP7 | - | free |
| GP8 | S2 | left motor M1A (on-board driver, PWM slice 4A) |
| GP9 | S2 | left motor M1B (PWM slice 4B) |
| GP10 | S2 | right motor M2A (PWM slice 5A) |
| GP11 | S2 | right motor M2B (PWM slice 5B) |
| GP12 | S5 | scan servo signal (servo port S1, PWM slice 6A, 50 Hz) |
| GP13 | - | free |
| GP14 | - | free |
| GP15 | - | free |
| GP16 | S2 | left wheel encoder (Grove 4, rising-edge interrupt) |
| GP17 | S2 | right wheel encoder (Grove 4, rising-edge interrupt) |
| GP18 | - | free |
| GP19 | S1 | template example LED, still blinked by `led_task` (LED + 330 Ω to GND; not in the Week 6 map, free it if someone needs the pin) |
| GP20 | - | free |
| GP21 | - | free |
| GP22 | - | free |
| GP26 | S3 | IR_LEFT, ADC0 (Grove 6) - Week 6 pin map. MH IR module A0, module on 3.3 V. Grove 5 shares GP26: leave its GP26 wire unconnected |
| GP27 | S3 | IR_CENTER, ADC1 (Grove 6) - Week 6 pin map. MH IR module A0, module on 3.3 V |
| GP28 | S3 | IR_BARCODE, ADC2 (Grove 7) - Week 6 pin map. barcode sensor, barcodes decoded by `line_task` |

## 3. Peripherals

**Subsystems, tasks and peripherals are three different things:**

| Term | What it is | Example |
|---|---|---|
| **Subsystem** | one student's part of the product, one folder | "environment sensing", "motor control", "user interface" |
| **Task** | one thread of execution. A subsystem may have **as many tasks as it needs** | a sampling task and a filtering task, both in "environment sensing" |
| **Peripheral** | one hardware block on the RP2040 | I2C0, UART1, the ADC |

A driver is **not** a subsystem. It is code *inside* the subsystem that
needs that hardware: the "environment sensing" subsystem contains the I2C
sensor code, and owns I2C0. Write it as `drv_<name>.c` / `drv_<name>.h` in
that subsystem's folder, never under `device/`, and call it only from the
peripheral's owning task. The kernel's own drivers in `device/` (serial,
I2C, ADC) may be used instead; switching one on in `config/config_device.h`
changes shared configuration, so record it in the table below first.

**The rule: each peripheral is driven by exactly one task, its *owning
task*.** That task can be in any subsystem, and one subsystem may own
several peripherals (one owning task each, or one task owning all of them).
Every other task, in the same subsystem or another one, asks the owning task
through a kernel object (section 5) and never touches that peripheral itself.

*Why:* this port runs tasks on both cores at the same time. If two tasks
drive the same I2C or UART block at once, their transfers get mixed up, and
the drivers have no locking to stop it.

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| UART1 | - | not used | - |
| I2C0 | - | not used | - |
| I2C1 | S4 | `imu_task` (own I2C1 driver in `subsystem4/subsystem4.c`: the kernel I2C driver is off, `DEVCNF_USE_IIC 0`, and its I2C1 pins are GP6/GP7) | GP2, GP3 |
| SPI0 | - | not used | - |
| SPI1 | - | not used | - |
| ADC | S3 | `line_task` (kernel driver `adca`, opened by `ir_init()` in `subsystem3/subsystem3.c`) | GP26–GP28 |
| PWM slices (list) | S2: slices 4, 5; S5: slice 6 | `motion_task`; `s5_scan` | GP8–GP11; GP12 |
| IO_IRQ_BANK0 (all GPIO interrupts) | shared: one handler, in `subsystem5/subsystem5.c` (`gpio_irq_attach()` in `subsystem5.h`; move it when the team picks a place for shared code) | interrupt handler, no task | GP5 echo (S5), GP16/GP17 encoders (S2). Anyone else needing a GPIO interrupt uses `gpio_irq_attach()` too |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

### WiFi and MQTT are special

The radio and the network stack belong to the kernel's built-in WiFi service
task, `cyw43_utk_task` (`lib/libwifi/sysdepend/pico_rp2040/`). **No student
task may call `cyw43_...`, lwIP or MQTT functions, not even from a
"WiFi subsystem".** The calls are not safe from any other task, on either core.

What students *can* do:

- **Read the connection state** from any task, which is always safe:
  ```c
  #include "cyw43_utk.h"
  T_CYW43_UTK_STATUS st;
  cyw43_utk_get_status(&st);   /* st.ready, st.link_status, st.ip_addr, ... */
  ```
- **Send or receive data over the network** by adding code that *runs
  inside the service task*. The MQTT client is in
  `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`. It publishes a heartbeat
  today. A "cloud / reporting" subsystem would add a kernel object
  (e.g. a message buffer) that application tasks fill. The MQTT code then
  reads it with `TMO_POL` (never blocking) and publishes it.
  Because this changes shared library code, **agree it with the team and
  the instructor first**, and record it as an interface in section 5.
  It needs the `NET=lwip MQTT=1` build (README section 3).

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4). Each subsystem gets its own band by default;
change it only by team agreement. Write down every task you create.

| Subsystem | Priority band | Tasks (name – priority – core) |
|---|---|---|
| 1 | 12–13 | led_task – 12 – any (template example LED, still running); comm_task – 13 – any; sim_vc_task – 12 – any (stand-in VC, only built with `S1_SIM_VC 1`; off) |
| 2 | 14–15 | motion_task – 14 – any (woken every 20 ms by `cyc_motion`) |
| 3 | 16–17 | line_task – 16 – any (every 10 ms, one kernel tick; Week 6 asked ~2 ms, the kernel tick is 10 ms) |
| 4 | 20–21 | imu_task – 20 – any (woken every 10 ms by `cyc_imu`). Band swapped with S5 (Tribunal 1 change log: "Obstacle Scan is given a higher priority than IMU") |
| 5 | 18–19 | s5_scan – 18 – any (wakes every 60 ms to ping); s5_stub_vc – 19 – any (bench-test stand-in in `task_obstacle.c`, off: `S5_STUB_VC 0`). Tribunal 1: Obstacle Scan runs above the IMU (bands swapped with S4). |
| VC | 10–11 | vc_task – 11 – any (Vehicle Controller, `app_program/vehicle_ctrl.c` + `task_vc.c`, owned by the whole team; wakes every 20 ms) |

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any".

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID in its own `subsystemN.h`; the **user** only
sends to it or waits on it. Agree on the data format before coding.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-01 – IF-04 | - | - | - | template examples, removed when Motion Control replaced the producer | - |
| IF-05 | S4 → S1 | task control | `tid_led` | `tk_sus_tsk` / `tk_rsm_tsk` / `tk_chg_pri` | *example* (nothing uses it now) |
| IF-06 | VC → S1 | message buffer (owned by S1) | `mbf_telem_q` | `telem_snapshot_t` in `subsystem1/telem_format.h`, posted by `vc_task` once a second with `TMO_POL` | in use (S1) |
| IF-07 | S1 → VC | message buffer (created by S1 for now) | `mbf_remote_cmd_q` | `remote_cmd_t` in `subsystem1/telem_format.h` | in use (S1) |
| IF-08 | any → S2 | API → message buffer (owned by S2) | `mbf_motion_cmd` | call `motion_move_forward()` etc. in `subsystem2.h`, never the buffer directly | in use (S2) |
| IF-09 | S2 → any | event flag (owned by S2) | `flg_motion` | `MOTION_FLG_DONE` / `MOTION_FLG_FAULT`; `motion_wait_done()` | in use (S2) |
| IF-10 | S5 → VC | message buffer, 4-byte packed words (owned by the VC, created in `vc_init`) | `obst_q` | `S5_OBST_MSG` via `s5_obst_pack/unpack` in `subsystem5.h` | in use (S5) |
| IF-11 | VC → S5 | message buffer, 4-byte packed words (owned by S5) | `scan_q` | `S5_CMD_xxx` via `s5_cmd_pack` in `subsystem5.h` | in use (S5) |
| IF-12 | S4, S5 → S2; VC clears | event flag, created by S2 | `flg_estop_req` | `ESTOP_IMU_LIFTED` 0x01 (car lifted or overturned; was `ESTOP_IMU_IMPACT` until 16 Nov, collision detection removed), `ESTOP_IMU_FAULT` 0x02, `ESTOP_OBSTACLE` 0x04 in `app_program/shared_bits.h`. S2 brakes while any bit is set | Tribunal 1 |
| IF-13 | every subsystem → telemetry | event flag (owned by the VC) | `flg_sysfault` | `FAULT_xxx` bits in `app_program/shared_bits.h`; each subsystem sets and clears only its own | Tribunal 1 (bits from S1's telemetry format) |
| IF-14 | S4 → VC | message buffer, 4-byte packed words (owned by the VC) | `imu_evt_q` | Buddy 4's `IMU_MSG_PACK` (`subsystem4/imu.h:403`): [1:0] type HUMP 0 / MOTION 1 / LIFTED 2 (was IMPACT until 16 Nov) / SENSOR_FAULT 3, [8:2] payload (hump height in 0.5 mm, motion class, why lifted: 1 moved up / 2 tipped over, fault bits; SENSOR_FAULT 0 = cleared), [31:16] ms timestamp. VC copy in `app_program/vc_imu_msg.h`, checked at compile time | Buddy 4 |
| IF-15 | VC → S4 | message buffer, 4-byte words (owned by S4, created in `subsystem4_init`) | `imu_q` | `IMU_CMD_RESET` = 0: recalibrate (accepted only while the car is stopped) | Buddy 4 |
| IF-16 | S3 → VC | message buffer, 4-byte packed words (owned by the VC) | `nav_q` | `vc_nav_pack()`: BARCODE (barcode_t, sent when one is read while following; the VC turns only with `VC_BARCODE_ACT` 1), LINE_LOST, LINE_REACQUIRED, LINE_TIMEOUT, SENSOR_FAULT. Sent by `line_task` | in use (S3 / VC) |
| IF-17 | VC → S3 | message buffer, 4-byte words (owned by S3, created in `subsystem3_init`) | `line_cmd_q` | SUSPEND, SEARCH, FOLLOW, RESET (`VC_LINE_CMD_xxx`, bits 31..29) | in use (S3 / VC) |
| IF-18 | VC → S2 | motion API (IF-08) | `motion_set_velocity()`, `motion_move_forward()`, `motion_turn_left/right()`, `motion_stop()` | patrol speed refreshed every 20 ms, steered by IF-19 (left = speed − steer, right = speed + steer); line search sweep; bypass / barcode manoeuvres leg by leg, each checked on `flg_motion` | VC |
| IF-19 | S3 → VC | mutex + latest-value mailbox (owned by S3; Week 6 `steer_mtx`) | `steer_mtx` | `S3_STEER` in `subsystem3/subsystem3.h`: steering mm/s (+ = left), search side, line state, IR bits, flags. Read only with `s3_steer_get()` (bounded 10 ms wait) | in use (S3 / VC) |
| IF-20 | S2 → S4 | read-only status (owned by S2, mutex inside the API) | `motion_get_status()` | speed, odometer distance and `seq` (+1 every 20 ms publish) packed in `subsystem4/task_monitor.c` into Buddy 4's `MC_SNAP_PACK` word (`subsystem4/imu.h:414`). Tribunal 1: IMU needs timestamped speed and distance from Motion Control | in use (S2 / S4) |
| IF-22 | S4 → VC | shared 32-bit word, one store every 0.5 s (single writer: `imu_task`) | `vc_imu_tilt_word` | pitch and roll in 0.5° steps, 7 bits each (`VC_IMU_TILT_PACK / PITCH / ROLL` in `app_program/vc_imu_msg.h`), copied into telemetry by the VC | in use (S4 / VC, 20 Nov) |
| IF-21 | S3 → S1 | function, queues into `mbf_s1_tx` (never blocks) | `s1_publish_barcode()` | one message per barcode read on MQTT topic `is22/bc`: command, flags (standing / abandoned), bar+space count, the letters, each width in 10 ms units (layout in `subsystem1/telem_format.h`). Shown by the dashboard. The telemetry frame is unchanged | 2 Nov (barcode reading on) |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there. The numbers below are what the IS22 code creates.

| Object | Limit | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|
| Task | 32 | 2 (`led_task`, `comm_task`) | 1 (`motion_task`) | 1 (`line_task`) | 1 (`imu_task`) | 1 (`s5_scan`) |
| Semaphore | 16 | 0 | 0 | 0 | 0 | 0 |
| Event flag | 16 | 0 | 2 (`flg_motion`, `flg_estop_req`) | 0 | 0 | 1 (echo flag) |
| Mailbox | 12 | 0 | 0 | 0 | 0 | 0 |
| Mutex | 12 | 0 | 1 (`mtx_motion_status`) | 1 (`steer_mtx`) | 0 | 0 |
| Message buffer | 12 | 3 (`mbf_telem_q`, `mbf_remote_cmd_q`, `mbf_s1_tx`) | 1 (`mbf_motion_cmd`) | 1 (`line_cmd_q`) | 1 (`imu_q`) | 1 (`scan_q`) |
| Fixed memory pool | 12 | 0 | 0 | 0 | 0 | 0 |
| Variable memory pool | 12 | 0 | 0 | 0 | 0 | 0 |
| Cyclic handler | 12 | 1 (`cyc_led`) | 1 (`cyc_motion`) | 0 | 1 (`cyc_imu`) | 0 |
| Alarm handler | 12 | 0 | 0 | 0 | 0 | 0 |

Vehicle Controller (`app_program/vehicle_ctrl.c`): 1 task (`vc_task`), 3 message buffers (`obst_q`, `imu_evt_q`, `nav_q`), 1 event flag (`flg_sysfault`).
Total: 8 tasks, 4 event flags, 2 mutexes, 10 message buffers, 3 cyclic handlers, all well under the limits.

(The kernel's USB and WiFi service tasks also use a few of these.)
