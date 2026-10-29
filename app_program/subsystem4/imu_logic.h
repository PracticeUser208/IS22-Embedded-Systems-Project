/*
 * imu_logic.h -- pure logic (no hardware, no kernel calls). Compiles and tests on a PC.
 *   - fault detection / recovery        - 2-of-3 impact voting
 *   - hump height (pitch x distance)    - motion classification
 * State is bit-packed per the minimum-bit requirement; widths are tied to the tuning
 * constants by _Static_assert in imu_logic.c, so changing a threshold cannot silently overflow a field.
 */
#ifndef IMU_LOGIC_H
#define IMU_LOGIC_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"
#include "imu_cfg.h"

typedef struct { int16_t x, y, z; bool fresh; } raw3_t;          /* 12-bit, right-aligned */
typedef enum { RD_OK = 0, RD_NACK, RD_TIMEOUT } read_result_t;

/* ---------------- faults (19 bits) ---------------- */
typedef struct {
    unsigned mask      : 7;     /* IMU_FAULT_* bits   */
    unsigned nack_run  : 2;
    unsigned stale_run : 2;
    unsigned sat_run   : 2;
    unsigned good_run  : 2;
    unsigned late_run  : 4;
} fault_t;

void fault_init(fault_t *f);
/* raw may be NULL (sensor-init failures). Returns true if a BLOCKING fault is active. */
bool fault_on_read(fault_t *f, read_result_t r, const raw3_t *raw);
void fault_on_lateness(fault_t *f, uint32_t late_us);
void fault_set_cal_fail(fault_t *f);
void fault_note_queue(fault_t *f, bool posted);
bool fault_recovered(const fault_t *f);
void fault_clear_blocking(fault_t *f);

/* ---------------- 2-of-3 impact voting ----------------
 * VOTE_ACCEL : |a| deviates from 1 g by more than a threshold (instantaneous spike)
 * VOTE_DECEL : sustained forward deceleration (>= IMPACT_DECEL_POLLS polls)
 * VOTE_WHEEL : encoder speed collapses within one poll
 * DECEL and WHEEL are ignored while Motion Control is braking (a controlled stop must not
 * vote itself into an E-stop), and WHEEL is ignored while the speed snapshot is stale.
 * A hump gives the spike vote only, so it cannot confirm on its own. */
typedef struct { uint8_t packed; } vote_win_t;       /* 3 x 2-bit countdowns + 2-bit decel run = 8 bits */
typedef struct {
    float    a_dev_mg;        /* |a| - 1 g                                  */
    float    fwd_mg;          /* forward acceleration, body frame           */
    uint16_t speed, prev_speed;
    bool     braking;         /* Motion Control controlled stop in progress */
    bool     in_hump;
    bool     wheel_valid;     /* false while the speed snapshot is stale    */
} vote_in_t;

void    vote_init(vote_win_t *w);
uint8_t vote_step(vote_win_t *w, const vote_in_t *in);   /* mask of votes active inside the window */
bool    vote_confirmed(uint8_t mask);                    /* >= 2 distinct votes                    */

/* ---------------- hump (8 bits of state + two floats) ---------------- */
typedef struct { float h_mm, h_max_mm; unsigned active : 1; unsigned quiet : 2; unsigned stall : 5; } hump_t;
void  hump_init(hump_t *h);
float pitch_from_accel(float ax_mg, float ay_mg, float az_mg);
/* Returns true when a hump completes; *height_x2 = height in 0.5 mm steps (0..127, 127 = overrange). */
bool  hump_step(hump_t *h, float pitch_rad, uint16_t ds_mm, bool moving,
                float gain, uint8_t *height_x2);

/* ---------------- motion (9 bits) ---------------- */
typedef struct { unsigned stopped_run : 3; unsigned cand_run : 2; unsigned cand : 2; unsigned cur : 2; } motion_t;
void motion_init(motion_t *m);
/* Returns true when the (debounced) class changes; *out holds the new class. */
bool motion_step(motion_t *m, float fwd_mg, float lat_mg, uint16_t speed,
                 uint16_t prev_speed, motion_class_t *out);

#endif /* IMU_LOGIC_H */
