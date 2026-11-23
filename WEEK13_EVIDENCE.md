# IS22 Week 13: Minimum-Bit Data Representation Evidence

The four items the Week 13 submission asks for, for the Tribunal 1 change request
(minimum-bit data representation).

| Item | Where |
|---|---|
| 1. Data representation table | [`DATA_REPRESENTATION.md`](DATA_REPRESENTATION.md) |
| 2. RAM usage comparison | section 2 below; numbers in [`testing/week13/results.md`](testing/week13/results.md) |
| 3. Message size comparison | section 3 below; same file |
| 4. Correctness and performance | section 4 below |

All numbers come from `testing/week13/week13_measure.py`, which uses the firmware's own
compiler (`arm-none-eabi-gcc` 13.2, Cortex-M0+, `-O2`) and the firmware's own source.
To measure again after a build:

```
python3 testing/week13/week13_measure.py
python3 testing/pc_checks.py
```

## 1. Data representation table

[`DATA_REPRESENTATION.md`](DATA_REPRESENTATION.md) lists every state, event, flag and
value the subsystems exchange: its valid range, the bits it uses and where it is
defined. The rules used everywhere:

- **States and value lists**: each list has a `_COUNT` entry and a `_Static_assert`
  that it fits its field (for example 10 system states in 4 bits, 7 line states in 3).
- **Fault and stop conditions**: one bit each in a shared bitmask (`flg_sysfault`
  8 bits, `flg_estop_req` 3 bits, IMU faults 7 bits), never separate variables.
- **Messages between tasks**: one 32-bit word, packed with shifts and masks
  (`obst_q`, `scan_q`, `imu_evt_q`, `nav_q`, `line_cmd_q`, `imu_q`). The telemetry
  snapshot (`telem_q`) is a C bit-field struct of 24 bytes.
- **MQTT**: binary frames packed LSB-first with explicit masks and shifts
  (`bitpack.c`), never text. Bit-fields are not used on the wire because their
  layout is up to the compiler.

## 2. RAM usage comparison

### Where "before" comes from

The bit-packed format goes back to the first telemetry commit (27 Oct, "format v2,
provisional widths"), so there is no older firmware build to measure. The "before"
column is the same data written in plain C, compiled with the same compiler and
flags, and measured the same way (`testing/week13/ram_compare.c`):

- an `enum` for each state;
- a `bool` for each flag;
- the smallest standard integer type that holds each value;
- `float` where the value is a measurement.

This is a fair "before". It does not use `int` for everything: the compiler already
makes enums 1 byte, so the savings shown come only from the packing.

### Data items (bytes)

| Requirement area | Data item | Before | After | Saved |
|---|---|---:|---:|---:|
| 1. Motion control | Motion Control -> IMU snapshot (speed, distance, sequence) | 12 | 4 | 8 |
| 2. Line and barcode | `nav_q` message (event, barcode action) | 2 | 4 | -2 |
| 2. Line and barcode | barcode recording: drop-out flag per sample (3072 samples) | 3072 | 384 | 2688 |
| 3. Obstacle detection | `obst_q` message | 10 | 4 | 6 |
| 3. Obstacle detection | Obstacle Scan status (state, pinging, 8 faults, distance, servo, retries) | 16 | 4 | 12 |
| 3. Obstacle detection | scan table, 25 bearings | 200 | 50 | 150 |
| 4. Hump estimation | `imu_evt_q` message (hump height / motion class / lift cause) | 12 | 4 | 8 |
| 5. Vehicle and controller states | six state values | 6 | 6 | 0 |
| 6. ISR -> task | echo result written by the GPIO interrupt | 8 | 4 | 4 |
| 6. Shared fault bits | system faults (8 conditions) | 8 | 1 | 7 |
| 6. Shared stop bits | stop requests (3 sources) | 3 | 1 | 2 |
| 7. Inter-task messages | `scan_q` message | 8 | 4 | 4 |
| 7. Inter-task messages | `telem_q` message (31 fields) | 60 | 24 | 36 |
| 7. Inter-task messages | `remote_cmd_q` message | 2 | 1 | 1 |
| | **Total** | **3419** | **495** | **2924 (86%)** |

Two rows show no saving:

- **`nav_q`**: one 32-bit word is the team's rule for every queue. The kernel rounds
  every message up to 4 bytes anyway, so the queue's RAM is the same (table below).
- **States**: the variables were already 1 byte each. The saving for states is in the
  messages and frames, where each state takes 3 or 4 bits instead of a byte.

### Message buffers (bytes; each slot is a 4-byte header plus the message rounded up to 4)

| Queue | Slots | Message before | Message after | RAM before | RAM after |
|---|---:|---:|---:|---:|---:|
| `telem_q` (VC -> Telemetry) | 4 | 60 | 24 | 256 | 112 |
| `remote_cmd_q` (Telemetry -> VC) | 4 | 2 | 1 | 32 | 32 |
| `obst_q` (Obstacle Scan -> VC) | 8 | 10 | 4 | 128 | 64 |
| `imu_evt_q` (IMU -> VC) | 8 | 12 | 4 | 128 | 64 |
| `nav_q` (Line -> VC) | 8 | 2 | 4 | 64 | 64 |
| `scan_q` (VC -> Obstacle Scan) | 8 | 8 | 4 | 96 | 64 |
| S1 TX queue (frames waiting for MQTT) | 8 | 329 | 75 | 2688 | 640 |
| **Total** | | | | **3392** | **1040** |

The S1 TX queue holds the largest frame the car sends. With text telemetry that
would be the 328-byte JSON frame. With binary frames it is the 74-byte barcode
record, so packing the telemetry is what shrinks it.

**Together:** 6811 bytes before, 1535 bytes after: 5276 bytes (77%) less.

### Whole firmware (`arm-none-eabi-size`, MQTT build)

| | Bytes |
|---|---:|
| static RAM (`.data` + `.bss` + vectors) | 99 344 of 264 KB (38%) |
| of which the application modules (`app_program`, from the link map) | 16 305 |
| of which the barcode recording (`bcrec`) | 14 228 |
| of which lwIP and the Wi-Fi driver (packet pools, heap, buffers) | about 64 000 |

The 19 Nov change (telemetry version 3, below) added 12 bytes of static RAM
(99 308 -> 99 320 bytes); the `telem_q` message stayed at 20 bytes, because its fifth
32-bit word had room for the 9 new bits. The 20 Nov change (version 4, the car's
tilt) added 24 bytes (99 320 -> 99 344): the 14 new bits needed a sixth word, so the
`telem_q` message is now 24 bytes.

## 3. Message size comparison

### MQTT (bytes on the wire)

| Message | Bits | Binary (now) | As JSON text | Smaller by |
|---|---:|---:|---:|---:|
| `is22/tm` telemetry, every second | 171 | 22 | 328 | 15x |
| `is22/hb` heartbeat | 51 | 7 | 74 | 11x |
| `is22/cmd` command | 8 | 1 | 23 | 23x |
| `is22/ack` command result | 10 | 2 | 44 | 22x |

These are the same test values in both forms (binary and text are in
`testing/week13/results.md`). The JSON uses short keys and numbers, so it is the
smallest text form, not a padded one. At one telemetry frame a second, the car sends
22 B/s instead of 328 B/s.

### Between tasks (bytes per message)

| Message | Before | After |
|---|---:|---:|
| `telem_q` telemetry snapshot | 60 | 24 |
| `obst_q` obstacle event | 10 | 4 |
| `imu_evt_q` IMU event | 12 | 4 |
| `scan_q` scan command | 8 | 4 |
| `nav_q` line event | 2 | 4 (same RAM, see above) |
| `remote_cmd_q` command | 2 | 1 |

### Change on 19 Nov: telemetry version 3

The lift cause (2 bits) and the IMU fault bits (7 bits) were added at the end of
`is22/tm`: 148 -> 157 bits, 19 -> 20 bytes. Without them the dashboard could not
tell "lifted" from "overturned", and an IMU fault could only be read on the USB
console. Both fields use the fewest bits that hold their range.

### Change on 20 Nov: telemetry version 4

The car's tilt was added at the end: pitch (nose up) and roll (side up), each 7
bits signed in 0.5° steps (-32 to +31.5°), the value furthest from 0 in the last
0.5-1 s. 157 -> 171 bits, 20 -> 22 bytes. Without it the hump check could not be
tuned from the dashboard, and a hump under one wheel (which tips the car sideways,
hardly nose up) could not be seen at all. The IMU hands it to the VC as one 32-bit
word (`vc_imu_tilt_word`, `vc_imu_msg.h`).

## 4. Correctness and performance

### No loss of required functionality

| Evidence | Result |
|---|---|
| Compile-time checks: every value list fits its field, every frame is 256 B or less, `telem_q` message 24 B or less, `remote_cmd_t` 1 B | the build stops if one is broken; all three builds (MQTT, TEST=1, plain) pass with no warnings |
| Telemetry codec on a PC (`testing/subsystem1/host/`): test vectors worked out separately in Python, round trips, rejects wrong length and wrong version, saturation at range limits | all pass (`python3 testing/pc_checks.py telemetry`) |
| Ground station decoder (`tools/buddy1/is22_ground.py selftest`): the same vectors, version 3 and version 2 | passes |
| PC checks of the other subsystems: line following, barcode decoding (Code 39), lift and overturn detection, hump detection | `s3`, `line`, `barcode`, `lift`, `hump` pass |
| Obstacle bypass simulation (`bypass`, 792 runs, including hard cases that are expected to fail) | same output, byte for byte, as before the change |
| Moving the code into the template layout (17 Nov): every compiled function compared with the working project | 666 of 666 sections identical: the firmware does the same thing |
| On the car, 4-19 Nov: MQTT logs in `MQTT setup/logs` | telemetry loss 0.2-1.1% (target 5% or less); barcodes, line following, obstacle bypass, lift and overturn all logged |
| On the car, 19 Nov, USB console | lift reported as "moved up", overturn as "tipped over"; both stopped the car |

### No unacceptable loss of timing or performance

Packing one telemetry frame was measured on an emulated Cortex-M0+: the
instructions executed were counted, with the Cortex-M0+ cycle times for each
instruction.

| Step | Instructions | Cycles | Time at 125 MHz |
|---|---:|---:|---:|
| pack the 22-byte frame (`tc_encode_tm`) | 3839 | 5333 | 42.7 us |
| write the same values as JSON (`snprintf`) | 21054 | 33972 | 271.8 us |
| unpack the frame (`tc_decode_tm`, PC side and tests) | 5013 | 6561 | 52.5 us |

- **Telemetry:** once a second, so packing costs 0.004% of one core, about 6 times
  less than writing it as text would.
- **32-bit queue messages:** packing or unpacking one takes a few shift-and-mask
  instructions, far below the tightest loops: the IMU every 10 ms, Motion Control
  every 20 ms, the line task every 10 ms.
- **Timing faults:** the IMU raises `TIMING` if 10 polls in a row are more than 3 ms
  late. It was not seen in the 19 Nov console run.
- **Packet loss:** telemetry loss stayed at or below 1.1% in every log.

### Shared packed data: no read-modify-write races

| Shared data | Writers -> readers | How it is kept safe |
|---|---|---|
| `flg_sysfault`, `flg_estop_req` (fault and stop bits) | several tasks -> several tasks | µT-Kernel event flags. `tk_set_flg` / `tk_clr_flg` change bits inside the kernel, atomically. Only the VC clears stop bits. |
| Obstacle Scan status word | S5 task -> VC, telemetry | built in a local variable, then written with one aligned 32-bit store (`s5_status`, `volatile UW`). Readers read it once. |
| IMU tilt word (pitch, roll) | IMU task -> VC | built in local variables, then written with one aligned 32-bit store (`vc_imu_tilt_word`, `volatile`); the VC reads it once a second |
| Echo result | GPIO interrupt -> S5 task | one 32-bit store in the interrupt (`us_result`), read once by the task |
| Motion Control status (speed, distance) | Motion Control -> IMU, VC | read under Motion Control's mutex (`motion_get_status`). The IMU packs its own 32-bit copy. |
| Messages between tasks | sender -> receiver | copied by the kernel's message buffers; no memory is shared |
| Bit-field structs (`telem_snapshot_t`, `vc_imu_rep`, the IMU's `hump_t` and fault mask) | one task each | only ever written by their own task, so a read-modify-write cannot be interrupted by another writer |

### Where the rule is not applied yet

| Data | Now | Why not packed |
|---|---|---|
| Barcode recording: raw side-sensor samples (3072) | 12-bit ADC values in `uint16_t` (6 KB) | packing 12-bit values would save 1.5 KB but adds work in the 10 ms line loop; the drop-out flags next to them are 1 bit each |
| `motion_cmd_t` (VC -> Motion Control) | 16 B: type, value, three speeds | Motion Control's interface, carries signed mm, degrees and three speeds; the buffer holds 4 |
| `is22/bc`, `is22/ir` | byte-sized binary records | debug channels for the barcode reader; still binary, not text |

## 5. Demonstration (Week 13)

1. **Binary telemetry:** open the dashboard. The log lines show the decoded fields of
   each 22-byte frame, and the event list shows state changes.
2. **Measurements:** run `python3 testing/week13/week13_measure.py` and
   `python3 testing/pc_checks.py`, and show the tables and `PC checks passed`.
3. **Fault and stop bits:**
   - lift the car while it patrols: the wheels stop, and the dashboard shows "IMU: lifted (moved up)".
   - after RESET, tip the car over: the dashboard shows "overturned".
   - both come from the 2-bit lift cause.
4. **Rubric run:** start on the line and pass the hump (hump count, last and peak in
   the IMU tile), then a barcode (decoded and acted on), then the obstacle (width
   measured, the car goes round and rejoins the line). Finally STOP from the
   dashboard; the ACK comes back in the event list.
