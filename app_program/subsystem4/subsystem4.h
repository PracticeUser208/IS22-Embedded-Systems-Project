/*
 *	subsystem4.h
 *	Subsystem 4 -- Monitor (reporting).  Student ID: <student ID>.
 *
 *	Public interface of subsystem 4.  Other subsystems include it as
 *	"../subsystem4/subsystem4.h".
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"

#define S4_PRI_MONITOR		18	/* priority band 18-19 (TEAM.md) */
#define S4_ALARM_MS		2000	/* report anyway if nothing for 2 s */
#define S4_LED_TOGGLE_REPORTS	6	/* pause/resume the LED every N reports */

/* IF-03: send one text line (at most S4_SUMMARY_MAX bytes, no '\0'
   needed) to mbf_summary, then set FLG_BATCH in flg_events. */
#define S4_SUMMARY_MAX		48
#define FLG_BATCH		0x0002	/* set by subsystem 3 */
#define FLG_ALARM		0x0001	/* private: set by our alarm handler */

IMPORT ID	mbf_summary;
IMPORT ID	flg_events;

/* Private to subsystem 4 */
IMPORT ID	alm_monitor;
IMPORT UW	s4_lines_received;	/* for the tests */
IMPORT void	monitor_task(INT stacd, void *exinf);
IMPORT void	monitor_alarm_handler(void *exinf);

/* ---- IMU / Terrain (Buddy 4): merged from imu_cfg.h, imu.h, imu_logic.h, imu_port.h ---- */
#include "app_types.h"	/* shared header (pulls in app_fields.h) */
#define S4_PRI_IMU		18	/* PLACEHOLDER: take the real band from TEAM.md */

/* ======== from imu_cfg.h ======== */
/*
 * imu_cfg.h -- IMU tunables.
 * EVERY NUMBER HERE IS A PLACEHOLDER until tuned from logged data (A7, A8, A12).
 * Sensor: LSM303D (the GY-511 on Buddy 5's car; WHO_AM_I 0x0F reads 0x49). Accelerometer AND magnetometer
 * share one address, 0x1D (SA0 high). The older LSM303DLHC GY-511 answers at 0x19 and uses different
 * control registers, so this driver does not run on it unchanged.
 * VERIFY against your board's datasheet before trusting any of this.
 */
#ifndef IMU_CFG_H
#define IMU_CFG_H

/* ---- sensor ---- */
#define IMU_I2C_ADDR_ACCEL        0x1D     /* LSM303D, SA0 high (was 0x19 for the LSM303DLHC) */
#define IMU_POLL_PERIOD_US        10000
#define IMU_SENS_MG_PER_LSB       4.0f     /* +-8 g. The LSM303D is really 3.904 mg/count (readings ~2.5% high);
                                              calibration absorbs it. Use 3.904f for exact values. */
#define IMU_RAW_FULLSCALE         2047
/* Mounting. Which sensor axis (0=X, 1=Y, 2=Z) is the car's forward / lateral / up axis,
 * and the sign that makes each point the right way:
 *   forward axis: reads POSITIVE when the car pushes forward (accelerating)
 *   up axis     : reads POSITIVE (~ +1 g) when the car sits level
 * Use the bring-up firmware's 'p' command to check these (see README). */
#define IMU_AXIS_FWD   0
#define IMU_AXIS_LAT   1
#define IMU_AXIS_UP    2
/* Buddy 5's car: the IMU is mounted upside down and facing backwards, so
 *   fwd = -X, lat = +Y, up = -Z.
 * LAT is not yet verified on the car (lift the right side: lat should go negative).
 * The #ifndef lets the PC tests override these with -D (their fake sensor uses a +X/+Y/+Z mounting). */
#ifndef IMU_SIGN_FWD
#define IMU_SIGN_FWD   (-1.0f)
#endif
#ifndef IMU_SIGN_LAT
#define IMU_SIGN_LAT   (1.0f)
#endif
#ifndef IMU_SIGN_UP
#define IMU_SIGN_UP    (-1.0f)
#endif
/* Flip to -1 if a nose-UP tilt reports a NEGATIVE pitch. */
#define IMU_PITCH_SIGN 1.0f

/* ---- calibration (car must be stopped) ---- */
#define CAL_SAMPLES               50
#define CAL_NOISE_MAX_MG          60.0f
#define CAL_MAX_RETRIES           3

/* ---- fault detection ---- */
#define NACK_POLLS_TO_FAULT       2        /* 2 x 10 ms                               */
#define STALE_POLLS_TO_FAULT      3        /* 3 x 10 ms = 30 ms (NFR15 proposal)      */
#define SAT_POLLS_TO_FLAG         2
#define GOOD_READS_TO_RECOVER     3
#define LATE_POLL_US              3000
#define LATE_POLLS_TO_FAULT       10
/* Team decision pending: should an IMU sensor fault stop the car? 1 = yes. */
#define IMU_FAULT_TRIGGERS_ESTOP  1

/* ---- impact voting (2 of 3) ---- */
#define IMPACT_ACCEL_DEV_MG       2500.0f  /* |a| deviation from 1 g                  */
#define IMPACT_ACCEL_DEV_HUMP_MULT 2.0f    /* raise threshold while crossing a hump   */
#define IMPACT_DECEL_MG           600.0f   /* sustained forward deceleration          */
#define IMPACT_DECEL_POLLS        2
#define IMPACT_WHEEL_MIN_SPEED    100      /* mm/s before a wheel drop can count      */
#define IMPACT_WHEEL_DROP_PCT     40       /* % speed lost within one poll            */
#define IMPACT_WINDOW_POLLS       3        /* votes must fall within this window      */
#define IMPACT_COOLDOWN_POLLS     50

/* ---- hump (pitch x encoder distance) ---- */
#define HUMP_PITCH_ENTER_RAD      0.07f    /* about 4 degrees                         */
#define HUMP_PITCH_EXIT_RAD       0.03f
#define HUMP_END_POLLS            3
#define HUMP_MIN_SPEED            80       /* mm/s                                    */
#define HUMP_ABORT_STALL_POLLS    20       /* discard hump if the car stalls on it    */
#define HUMP_MIN_HEIGHT_MM        3.0f
#define HUMP_GAIN                 1.0f     /* fit from calibration passes (A7)        */
#define PITCH_LPF_ALPHA           0.3f

/* ---- Motion Control snapshot health ---- */
#define SNAP_STALE_POLLS          5        /* sequence unchanged this long = Motion Control not updating */
#define SNAP_DS_SAFETY_MULT       4        /* a distance jump bigger than this x expected is a discontinuity */

/* ---- debug read-out (bring-up build only; keep 0 in the kernel build) ---- */
#ifndef IMU_ENABLE_DEBUG
#define IMU_ENABLE_DEBUG          0
#endif

/* ---- motion classification ---- */
#define MOTION_STOP_SPEED         20       /* mm/s                                    */
#define MOTION_STOP_POLLS         5
#define MOTION_TURN_LAT_MG        250.0f
#define MOTION_ACCEL_FWD_MG       150.0f
#define MOTION_ACCEL_DSPEED       8        /* mm/s gained per poll                    */
#define MOTION_DEBOUNCE_POLLS     3

#endif /* IMU_CFG_H */

/* ======== from imu.h ======== */
#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

/* Diagnostic counters (saturating, not application data). */
typedef struct {
    uint16_t polls, impacts, single_votes, humps, drops, resets, reset_rejected;
    uint16_t snap_stale, ds_jumps;
} imu_stats_t;

void        imu_init(void);
void        imu_step(uint32_t late_us);   /* one poll cycle (called by imu_run, or by tests) */
void        imu_run(void);                /* task body: never returns                        */
imu_state_t imu_get_state(void);
uint8_t     imu_get_fault_mask(void);
imu_stats_t imu_get_stats(void);

#if IMU_ENABLE_DEBUG
typedef struct {
    float    body_mg[3];      /* forward, lateral, up (mg)            */
    float    a_dev_mg;        /* |a| - 1 g                            */
    float    pitch_rad;       /* filtered pitch, relative to baseline */
    uint8_t  votes;           /* VOTE_* mask active this poll         */
    bool     hump_active;
    float    hump_mm;         /* running height integral (gain applied) */
    uint16_t speed_mm_s;
} imu_debug_t;
imu_debug_t imu_get_debug(void);
#endif

#endif /* IMU_H */

/* ======== from imu_logic.h ======== */
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

/* ======== from imu_port.h ======== */
/*
 * imu_port.h -- the ONLY hardware/kernel boundary used by imu.c.
 * imu_port_pico_utk.c implements it for the Pico W + uT-Kernel build.
 * test/fake_port.c implements it for PC testing.
 */
#ifndef IMU_PORT_H
#define IMU_PORT_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

typedef enum { PORT_I2C_OK = 0, PORT_I2C_NACK = 1, PORT_I2C_TIMEOUT = 2 } port_i2c_rc_t;

port_i2c_rc_t port_i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val);
port_i2c_rc_t port_i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n);
void          port_i2c_recover(void);              /* clock out a stuck bus, re-init I2C1 */

uint64_t      port_time_us(void);                  /* 1 MHz timestamp                      */
uint32_t      port_time_ms(void);                  /* milliseconds since boot (wraps)      */
uint32_t      port_wait_next_poll(void);           /* block to next 10 ms tick; returns lateness in us */

bool          port_post_event(uint32_t msg);       /* imu_evt_q, ZERO timeout; false if full */
bool          port_recv_cmd(uint8_t *cmd);         /* imu_q, non-blocking                   */
uint32_t      port_read_mc_snapshot(void);         /* one aligned 32-bit load               */
bool          port_mc_braking(void);               /* Motion Control controlled stop active */
void          port_estop_set(uint32_t src_bits);   /* set bits in ESTOP_REQ (atomic)        */
void          port_sysfault_set(uint32_t bits);    /* set  OUR bit in the shared fault bitmask (atomic) */
void          port_sysfault_clear(uint32_t bits);  /* clear OUR bit only                     */

#endif /* IMU_PORT_H */

IMPORT void	imu_task(INT stacd, void *exinf);	/* wraps imu_run() */
#endif /* SUBSYSTEM4_H */