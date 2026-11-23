# Subsystem 1 — test log

**Subsystem:** Telemetry / Communications (Buddy 1)  ·  **Student ID:** 2500510  ·  **Code:** `app_program/subsystem1/`, MQTT bridge in `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | (template LED example, still running) cyclic handler is running | | auto | `subsystem1_test.c` | `[TEST] S1-U01 PASS` |
| S1-U02 | (template LED example) LED task sleeps between ticks (not busy) | NFR6 | auto | `subsystem1_test.c` | `[TEST] S1-U02 PASS` |
| S1-U03 | (template LED example) LED on GP19 toggles every 250 ms | | manual | 1. Flash normal image. 2. Watch LED / measure with scope. | 2 Hz blink, 50 % duty |
| S1-H01 | bit packing: LSB-first writer / reader, signed fields, overflow flagged and sticky | NFR16 | auto (PC) | `python3 testing/pc_checks.py test_bitpack` (`testing/subsystem1/host/test_bitpack.c`) | `ALL PASSED` |
| S1-H02 | telemetry v4, heartbeat, command and ACK encode / decode against fixed byte vectors; wrong length, unknown version and invalid commands rejected; values saturate instead of wrapping | FR21, FR22, FR23, NFR19 | auto (PC) | `python3 testing/pc_checks.py test_codec` (`host/test_codec.c`) | `ALL PASSED`; telemetry vector `54 11 99 3E … 41 A1 07` (22 B) |
| S1-H03 | frame sizes: telemetry 22 B, heartbeat 7 B, command 1 B, ACK 2 B; telem_q snapshot ≤ 24 B | NFR16, NFR19 | auto (PC) | `python3 testing/pc_checks.py test_format` (`host/test_format.c`) | `ALL PASSED` |
| S1-H04 | the ground station's Python decoder agrees with the C codec | NFR19, IF-06 | auto (PC) | `python3 tools/buddy1/is22_ground.py selftest` | `SELFTEST PASSED` |
| S1-M01 | car online: heartbeat every 5 s, telemetry every second | FR21, FR23 | manual | `MQTT setup - GitHub`: 4 Start MQTT broker, 6 Check car is online | `is22/hb` about every 5 s, `is22/tm` about every 1 s, both decoded |
| S1-M02 | remote commands: START, STOP, PAUSE, RESUME, RESET acted on within 1 s, ACKed, a resent duplicate not run twice, a bad payload rejected | FR22 | manual | `MQTT setup - GitHub/Send to car/*.cmd`; resend the same command within 2 s | ACK `FORWARDED`, state changes within 1 s; resend → ACK `DUPLICATE`; 2-byte payload → ACK `INVALID` |
| S1-M03 | Wi-Fi lost while driving: the car keeps going and reconnects | FR24 | manual | while patrolling, switch the laptop hotspot off for 20 s, then on | the car keeps following the line; heartbeat `reconnects` goes up by 1; telemetry resumes |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console; **auto (PC)** = runs on a PC, no car needed;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-23 | S1-H01 | `ad50553` | Buddy 5 (Reuben), for the team | PASS | `logs/2026-11-23_host_tests.txt` | B1–B6 pass. Commit = GitHub main after the telemetry merge, plus the 23 Nov sync of `testing/` from the template |
| 2026-11-23 | S1-H02 | `ad50553` | Buddy 5 (Reuben), for the team | PASS | `logs/2026-11-23_host_tests.txt` | C1–C8 pass, telemetry v4 vector 22 B |
| 2026-11-23 | S1-H03 | `ad50553` | Buddy 5 (Reuben), for the team | PASS | `logs/2026-11-23_host_tests.txt` | 22 / 7 / 1 / 2 B; snapshot within 24 B |
| 2026-11-23 | S1-H04 | `ad50553` | Buddy 5 (Reuben), for the team | PASS | `logs/2026-11-23_ground_selftest.txt` | Python decoder matches the C vectors |
