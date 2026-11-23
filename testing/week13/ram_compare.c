/*
 *  ram_compare.c
 *  Week 13 evidence: the size of each application data item before and
 *  after the minimum-bit change, compiled by arm-none-eabi-gcc for the
 *  Pico (Cortex-M0+, same flags as the firmware) and measured from the
 *  object file (arm-none-eabi-nm -S).  Not part of the firmware.
 *
 *  AFTER  = the type the firmware uses (telem_format.h, or one 32-bit word).
 *  BEFORE = the same fields in plain C: an enum for each state, a bool for
 *           each flag, the smallest standard integer that holds each value,
 *           float where the value is a measurement.
 *  Every BEFORE_x / AFTER_x pair below is measured as a char array of
 *  sizeof(type) bytes, so the symbol sizes are the type sizes.
 */

#include <stdbool.h>
#include <stdint.h>
#include "telem_format.h"

#define MEASURE(name, type) char name[sizeof(type)]

typedef enum { ST_A, ST_B, ST_C } plain_enum_t;     /* any state list: 4 bytes on ARM EABI */

/* ---- 1. motion control and encoder data: Motion Control -> IMU snapshot ---- */
typedef struct { uint16_t speed_mm_s; int32_t dist_mm; uint8_t seq; } before_mc_snap_t;

/* ---- 2. line and barcode: nav_q message ---- */
typedef struct { plain_enum_t event; plain_enum_t barcode; } before_nav_msg_t;

/* ---- 2. line and barcode: drop-out mask of the barcode recording (3072 samples) ---- */
#define BC_SAMPLES 3072u

/* ---- 3. obstacle detection ---- */
typedef struct {                       /* obst_q message */
    plain_enum_t type; plain_enum_t side; bool flag;
    uint16_t dist_cm; uint16_t width_cm; uint16_t clear_cm;
} before_obst_msg_t;
typedef struct {                       /* Obstacle Scan status word */
    plain_enum_t state; bool pinging;
    bool f_no_echo, f_stuck, f_open, f_servo, f_timeout, f_busy, f_range, f_other;
    uint16_t last_cm; int16_t servo_deg; uint8_t retries;
} before_s5_status_t;
typedef struct { int16_t deg; float cm; } before_scan_slot_t;   /* scan table entry */
typedef struct { plain_enum_t status; uint16_t cm; uint32_t width_us; } before_echo_t;  /* ISR -> task */
typedef struct { plain_enum_t cmd; uint32_t arg; } before_scan_cmd_t;  /* scan_q message */

/* ---- 4. hump estimation: imu_evt_q message ---- */
typedef struct { plain_enum_t type; float payload; uint32_t ts_ms; } before_imu_evt_t;

/* ---- 5. vehicle and controller states ---- */
typedef struct { plain_enum_t vc_state; plain_enum_t line_state; plain_enum_t obst_state;
                 plain_enum_t motion; plain_enum_t barcode; plain_enum_t plan; } before_states_t;

/* ---- 6. shared fault and stop bits ---- */
typedef struct { bool motor_stall, encoder, imu, no_echo, wifi_loss, line_lost, no_path, startup; } before_faults_t;
typedef struct { bool imu_lifted, imu_fault, obstacle; } before_estops_t;
typedef struct { bool nack, stale, bus_stuck, saturation, cal_fail, timing, queue_full; } before_imu_faults_t;

/* ---- 7. inter-task messages: telem_q and remote_cmd_q ---- */
typedef struct {
    plain_enum_t sys_state; plain_enum_t drive_mode;
    int16_t speed_l, speed_r; uint16_t enc_l, enc_r; uint16_t dist_cm;
    bool ir_left, ir_center, ir_barcode;
    plain_enum_t line_state; plain_enum_t barcode;
    uint8_t hump_last, hump_peak, hump_count; plain_enum_t motion;
    uint16_t obst_dist; uint8_t obst_width, obst_angle, clear_l, clear_r;
    plain_enum_t obst_plan; plain_enum_t obst_state;
    before_faults_t fault; before_estops_t estop;
    plain_enum_t lift_cause; before_imu_faults_t imu_fault;
    float pitch_deg, roll_deg;
} before_telem_snapshot_t;
typedef struct { plain_enum_t code; uint8_t seq; } before_remote_cmd_t;

MEASURE(BEFORE_mc_snap, before_mc_snap_t);          MEASURE(AFTER_mc_snap, uint32_t);
MEASURE(BEFORE_nav_msg, before_nav_msg_t);          MEASURE(AFTER_nav_msg, uint32_t);
MEASURE(BEFORE_dead_mask, bool[BC_SAMPLES]);       MEASURE(AFTER_dead_mask, uint8_t[(BC_SAMPLES + 7u) / 8u]);
MEASURE(BEFORE_obst_msg, before_obst_msg_t);        MEASURE(AFTER_obst_msg, uint32_t);
MEASURE(BEFORE_s5_status, before_s5_status_t);      MEASURE(AFTER_s5_status, uint32_t);
MEASURE(BEFORE_scan_table, before_scan_slot_t[25]); MEASURE(AFTER_scan_table, uint16_t[25]);
MEASURE(BEFORE_echo_result, before_echo_t);         MEASURE(AFTER_echo_result, uint32_t);
MEASURE(BEFORE_scan_cmd, before_scan_cmd_t);        MEASURE(AFTER_scan_cmd, uint32_t);
MEASURE(BEFORE_imu_evt, before_imu_evt_t);          MEASURE(AFTER_imu_evt, uint32_t);
MEASURE(BEFORE_states, before_states_t);            MEASURE(AFTER_states, uint8_t[6]);
MEASURE(BEFORE_faults, before_faults_t);            MEASURE(AFTER_faults, uint8_t);
MEASURE(BEFORE_estops, before_estops_t);            MEASURE(AFTER_estops, uint8_t);
MEASURE(BEFORE_telem_snapshot, before_telem_snapshot_t); MEASURE(AFTER_telem_snapshot, telem_snapshot_t);
MEASURE(BEFORE_remote_cmd, before_remote_cmd_t);    MEASURE(AFTER_remote_cmd, remote_cmd_t);
