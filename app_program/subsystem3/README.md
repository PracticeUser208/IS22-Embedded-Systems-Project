# Buddy 3: Line and Barcode core

Implemented hardware-independent C modules. These are **not yet connected to the
running robot task**. `subsystem3.c` and `task_consumer.c` still provide the course
template consumer used by other example subsystems. The supplied Make build and
kernel tick are unchanged.

## Modules

| Files | Responsibility |
| --- | --- |
| `s3_ir.c/.h` | Calibration from measured white/black references, polarity-independent classification with hysteresis, normalized darkness 0..1000 |
| `s3_line.c/.h` | Two-sensor pattern lookup using a caller-supplied presence/steering table; no assumed track geometry or steering units |
| `s3_pulse.c/.h` | Bounded 64-run barcode pulse capture and caller-supplied duration-window matching; rejects gaps, oversized runs and overflow |
| `s3_state.c/.h` | Seven-state navigation logic, suspension, fault/reset and 2-second reacquisition deadline |
| `s3_check.c/.h`, `s3_main.c/.h` | Regression tests and host-only runner; also exposed through the existing `make TEST=1` test hook |

Pulse matching is a building block, not a complete barcode decoder. Frame start,
end, symbol patterns, speed compensation and command mappings remain unspecified.
Do not substitute the 20 mm lateral line/barcode separation for a quiet zone.

## Calling contract

One task owns each mutable context; these functions provide no cross-core
synchronization. Initialize state with `s3_state_init`, calibration with
`s3_ir_calibrate`, and capture with `s3_pulse_begin` before using them. Check each
boolean result. The steering lookup clears its output on invalid configuration.
Never publish invalid steering as a valid correction.

Supply actual acquisition timestamps in microseconds modulo 2^32. Call the state
machine regularly; it does not run a timer or independently enforce a deadline.
Timeout notification latency therefore includes scheduling/poll latency. Poll
intervals must be well below one timer wrap. Pulse limits must be below 2^31 us;
samples must arrive within the configured gap. Missing samples invalidate capture,
rather than generating synthetic historical samples.

The proposed state policy gives timeout precedence over LINE_FOUND exactly at
2 seconds; RESET, sensor fault and SUSPEND are handled first. Suspend cancels the
active search; resume starts a fresh search if calibrated. These policies need
team review before integration. Calibration quality, loss debounce and confidence
must be established by the adapter before sending state inputs. Internal input
and event values are **not** a `line_cmd_q` or `nav_q` message format.

Line interpretation uses only left/centre sensors. GP28 is the documented separate
barcode channel. All wiring remains unverified. No module writes PWM or accesses
hardware, µT-Kernel objects, network interfaces or dynamic allocation.

## Integration gates

Before replacing the example task, confirm:

1. Actual sensor models, AO/DO connections, supply/output voltage, polarity,
   mounting geometry, and GP28/VBAT jumper status.
2. Measured black/white distributions, hysteresis and all four line patterns;
   steering units, sign, range and stale-value behavior with Motion.
3. Barcode symbology, framing, symbol-to-command map and supported speed range.
4. Concrete `steer_mtx`, `line_cmd_q`, `nav_q` types, payloads, ownership, bounded
   waits, overflow handling and suspend/search handoff with Vehicle/Motion.
5. ADC ownership and bounded conversion path; reviewed hardware-alarm release
   using interrupt-safe RTOS integration. The configured 10 ms tick cannot provide
   a 2 ms delay. Do not change it. Record acquisition jitter and overruns under SMP
   load; no decoding, mutex waits or printing in the ISR.

## Representation

State uses 3 state bits and 1 calibration bit in one byte. Pulse status uses four
bits in one byte; count 0..64 needs seven bits in a byte. Line results use two
pattern bits plus presence and validity bits in one byte. Masks/shifts are used,
not compiler bit-field wire layouts. Timestamps and run durations use uint32_t;
steering uses a provisional int16_t caller domain. Native structure padding is not
a serialization format. Full minimum-bit compliance and RAM/message savings are
not established: final ranges, baseline comparison and shared-update concurrency
tests are still required.

## Verification

From this directory on Linux/WSL:

```sh
gcc -std=c11 -Wall -Wextra -Werror -Wconversion -Wshadow -pedantic \
  -fsanitize=address,undefined -DS3_HOST_TEST \
  s3_ir.c s3_line.c s3_pulse.c s3_state.c s3_check.c s3_main.c \
  -o /tmp/is22-s3-tests
/tmp/is22-s3-tests
```

On 2026-10-28, GCC 13.3.0 produced **0 regression failures**, with address and
undefined-behavior sanitizers enabled. Tests cover inverse sensor polarity,
hysteresis, invalid calibration, synthetic four-pattern steering, pulse overflow,
gaps and timestamp rollover, state transitions, suspension, fault/reset and the
exact reacquisition deadline. Synthetic steering fixtures are not calibration.

The four core modules also compiled individually using Arm GNU GCC 13.2.1 with
`-std=c11 -mcpu=cortex-m0plus -mthumb -ffreestanding` and the warning flags above.
This verifies compilation, not firmware linking or target execution. No on-car
sampling, decoding accuracy, motor response, timing or resource targets have been
validated. `make TEST=1` tests were added but have not been run on hardware.
