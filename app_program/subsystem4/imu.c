/*
 * imu.c -- IMU / Terrain task (Buddy 4).
 *
 * States: INIT -> CALIBRATE -> MONITORING -> FAULT -> CALIBRATE
 *
 * Rules kept from the design:
 *   - Only this task touches I2C1.
 *   - The IMU never writes PWM and never posts to telem_q.
 *   - The IMU only SETS ESTOP_REQ bits; the Vehicle Controller clears them.
 *   - The IMU owns ONE bit of the shared system fault bitmask (SYSFAULT_IMU_FAULT): it sets it on
 *     entering FAULT and clears it when it is back in MONITORING.
 *   - Event posts use a zero timeout. A full queue drops the event and is counted.
 *   - In FAULT nothing is published except SENSOR_FAULT (retried until the post succeeds).
 *   - Impact voting runs only in MONITORING (blind in CALIBRATE/FAULT: CALIBRATE only happens with the
 *     car stopped, and entering FAULT raises ESTOP).
 */
#include "imu.h"
#include "imu_cfg.h"
#include "imu_logic.h"
#include "imu_port.h"
#include <math.h>
#include <string.h>

/* LSM303D accelerometer registers (GY-511 on Buddy 5's car, address 0x1D).
 * NOT the LSM303DLHC map: on the LSM303D, 0x23 (CTRL4) only routes interrupts to INT2, so never write it
 * for range. The range lives in CTRL2 (0x21). */
#define REG_CTRL1       0x20u   /* ODR + block data update + axis enable    */
#define REG_CTRL2       0x21u   /* full scale + anti-alias filter           */
#define REG_STATUS_A    0x27u   /* STATUS_REG_A                             */
#define REG_AUTO_INC    0x80u   /* set bit 7 of the sub-address to auto-inc */
#define STATUS_ZYXDA    0x08u   /* new X/Y/Z data available                 */
#define CTRL1_VALUE     0x7Fu   /* 200 Hz, block data update on, X/Y/Z on   (0x67 would be only 100 Hz here) */
#define CTRL2_VALUE     0x58u   /* +-8 g, 194 Hz anti-alias filter          */
#define RAW_SHIFT       4       /* data is left-justified 12-bit            */
#define CAL_SENTINEL_MG 1.0e9f

_Static_assert(CAL_SAMPLES < 64,              "n is 6 bits");
_Static_assert(CAL_MAX_RETRIES + 1 < 8,       "retries is 3 bits");
_Static_assert(IMPACT_COOLDOWN_POLLS < 64,    "impact_cd is 6 bits");
_Static_assert(SNAP_STALE_POLLS < 8,          "snap_stale is 3 bits");
_Static_assert(IMU_ST_FAULT < (1 << IMU_STATE_BITS), "imu state does not fit its field");

typedef struct {
    /* working floats (arithmetic values stay native width) */
    float    sum[3], mn[3], mx[3], base_mg[3];
    float    g_mg, pitch0, pitch_f;
    /* packed sub-state */
    fault_t    f;
    vote_win_t v;
    hump_t     h;
    motion_t   m;
    uint16_t   prev_dist16;
    /* packed task state: 2+1+7+6+2+3+6+10+6+3+1 = 47 bits */
    unsigned state             : IMU_STATE_BITS;
    unsigned fault_posted      : 1;
    unsigned fault_posted_mask : IMU_FAULT_MASK_BITS;
    unsigned impact_cd         : 6;
    unsigned fault_age         : 2;
    unsigned retries           : 3;
    unsigned n                 : 6;
    unsigned prev_speed        : SPEED_MMPS_BITS;
    unsigned snap_seq          : 6;
    unsigned snap_stale        : 3;
    unsigned sysfault_set      : 1;
    imu_stats_t stats;
} imu_ctx_t;

static imu_ctx_t g;

#if IMU_ENABLE_DEBUG
static imu_debug_t dbg;
#endif

static void stat_inc(uint16_t *c)
{
    if (*c < UINT16_MAX) {
        (*c)++;
    }
}

/* sensor axes -> car body axes (forward, lateral, up), in mg */
static void to_body(const raw3_t *raw, float b[3])
{
    float a[3];
    a[0] = (float)raw->x * IMU_SENS_MG_PER_LSB;
    a[1] = (float)raw->y * IMU_SENS_MG_PER_LSB;
    a[2] = (float)raw->z * IMU_SENS_MG_PER_LSB;
    b[0] = IMU_SIGN_FWD * a[IMU_AXIS_FWD];
    b[1] = IMU_SIGN_LAT * a[IMU_AXIS_LAT];
    b[2] = IMU_SIGN_UP  * a[IMU_AXIS_UP];
}

/* ------------------------------------------------------------------ helpers */
static bool post_evt(imu_evt_type_t type, uint8_t payload)
{
    uint16_t ts16 = (uint16_t)(port_time_ms() & 0xFFFFu);
    bool ok = port_post_event(IMU_MSG_PACK(type, payload, ts16));
    fault_note_queue(&g.f, ok);
    if (!ok) {
        stat_inc(&g.stats.drops);
    }
    return ok;
}

static void reset_accumulators(void)
{
    for (int i = 0; i < 3; i++) {
        g.sum[i] = 0.0f;
        g.mn[i] = CAL_SENTINEL_MG;
        g.mx[i] = -CAL_SENTINEL_MG;
    }
    g.n = 0u;
}

static void start_calibration(void)
{
    reset_accumulators();
    g.retries = 0u;
    vote_init(&g.v);
    hump_init(&g.h);
    motion_init(&g.m);
    g.impact_cd = 0u;
    g.state = (unsigned)IMU_ST_CALIBRATE;
}

static void enter_fault(void)
{
    g.state = (unsigned)IMU_ST_FAULT;
    g.fault_age = 0u;
    vote_init(&g.v);
    hump_init(&g.h);
    g.fault_posted = 0u;
    port_sysfault_set(SYSFAULT_IMU_FAULT);             /* our bit in the shared fault bitmask */
    g.sysfault_set = 1u;
#if IMU_FAULT_TRIGGERS_ESTOP
    port_estop_set(ESTOP_SRC_IMU_FAULT);
#endif
}

static read_result_t read_sample(raw3_t *raw)
{
    uint8_t b[7];                                      /* status + 6 data bytes */
    port_i2c_rc_t rc = port_i2c_read_regs(IMU_I2C_ADDR_ACCEL,
                                          (uint8_t)(REG_STATUS_A | REG_AUTO_INC), b, sizeof b);
    if (rc == PORT_I2C_TIMEOUT) {
        return RD_TIMEOUT;
    }
    if (rc != PORT_I2C_OK) {
        return RD_NACK;
    }
    raw->fresh = ((b[0] & STATUS_ZYXDA) != 0u);
    raw->x = (int16_t)((int16_t)(b[1] | (b[2] << 8)) >> RAW_SHIFT);
    raw->y = (int16_t)((int16_t)(b[3] | (b[4] << 8)) >> RAW_SHIFT);
    raw->z = (int16_t)((int16_t)(b[5] | (b[6] << 8)) >> RAW_SHIFT);
    return RD_OK;
}

/* The ONE place the chip is configured. Called at startup and again from fault recovery,
 * so the two can never drift apart. */
static read_result_t init_sensor(void)
{
    port_i2c_rc_t rc = port_i2c_write_reg(IMU_I2C_ADDR_ACCEL, REG_CTRL1, CTRL1_VALUE);
    if (rc == PORT_I2C_OK) {
        rc = port_i2c_write_reg(IMU_I2C_ADDR_ACCEL, REG_CTRL2, CTRL2_VALUE);
    }
    if (rc == PORT_I2C_TIMEOUT) {
        return RD_TIMEOUT;
    }
    return (rc == PORT_I2C_OK) ? RD_OK : RD_NACK;
}

static void handle_cmd(uint16_t speed)
{
    uint8_t cmd;
    if ((!port_recv_cmd(&cmd)) || (cmd != IMU_CMD_RESET)) {
        return;
    }
    if (speed != 0u) {                                 /* recalibrate only when stopped */
        stat_inc(&g.stats.reset_rejected);
        return;
    }
    stat_inc(&g.stats.resets);
    if (g.state == (unsigned)IMU_ST_FAULT) {
        fault_clear_blocking(&g.f);
    }
    start_calibration();
}

/* ------------------------------------------------------------------ public */
void imu_init(void)
{
    memset(&g, 0, sizeof g);
    fault_init(&g.f);
    vote_init(&g.v);
    hump_init(&g.h);
    motion_init(&g.m);
    g.state = (unsigned)IMU_ST_INIT;
}

imu_state_t imu_get_state(void)      { return (imu_state_t)g.state; }
uint8_t     imu_get_fault_mask(void) { return (uint8_t)g.f.mask; }
imu_stats_t imu_get_stats(void)      { return g.stats; }
#if IMU_ENABLE_DEBUG
imu_debug_t imu_get_debug(void)      { return dbg; }
#endif

static void step_calibrate(const raw3_t *raw, uint16_t speed)
{
    if (speed != 0u) {                                 /* wait for the car to stop; not a retry */
        reset_accumulators();
        return;
    }
    float a[3];
    to_body(raw, a);
    for (int i = 0; i < 3; i++) {
        g.sum[i] += a[i];
        if (a[i] < g.mn[i]) {
            g.mn[i] = a[i];
        }
        if (a[i] > g.mx[i]) {
            g.mx[i] = a[i];
        }
    }
    g.n++;
    if (g.n < CAL_SAMPLES) {
        return;
    }

    bool noisy = false;
    for (int i = 0; i < 3; i++) {
        if ((g.mx[i] - g.mn[i]) > CAL_NOISE_MAX_MG) {
            noisy = true;
        }
    }
    if (noisy) {
        g.retries++;
        if (g.retries > CAL_MAX_RETRIES) {
            fault_set_cal_fail(&g.f);
            return;
        }
        reset_accumulators();
        return;
    }

    for (int i = 0; i < 3; i++) {
        g.base_mg[i] = g.sum[i] / (float)g.n;
    }
    g.g_mg    = sqrtf((g.base_mg[0] * g.base_mg[0]) + (g.base_mg[1] * g.base_mg[1]) + (g.base_mg[2] * g.base_mg[2]));
    g.pitch0  = pitch_from_accel(g.base_mg[0], g.base_mg[1], g.base_mg[2]);
    g.pitch_f = 0.0f;
    g.state   = (unsigned)IMU_ST_MONITORING;

    if (g.sysfault_set != 0u) {                        /* recovered: drop our system fault bit, tell the VC */
        port_sysfault_clear(SYSFAULT_IMU_FAULT);
        g.sysfault_set = 0u;
        (void)post_evt(IMU_EVT_SENSOR_FAULT, 0u);      /* payload 0 = fault cleared */
    }
}

static void step_monitoring(const raw3_t *raw, uint16_t speed, uint16_t prev_speed,
                            uint16_t ds, bool snap_ok)
{
    if (!raw->fresh) {
        return;                                        /* stale polls are counted by the fault logic */
    }

    float a[3];
    to_body(raw, a);                                   /* a[0]=forward a[1]=lateral a[2]=up */
    float mag   = sqrtf((a[0] * a[0]) + (a[1] * a[1]) + (a[2] * a[2]));
    float fwd   = a[0] - g.base_mg[0];
    float lat   = a[1] - g.base_mg[1];
    float pitch = IMU_PITCH_SIGN * (pitch_from_accel(a[0], a[1], a[2]) - g.pitch0);
    g.pitch_f += PITCH_LPF_ALPHA * (pitch - g.pitch_f);

    /* ---- impact: 2-of-3 vote. Stays active even while saturated (saturation is itself a
     *      symptom of a hard hit). ---- */
    vote_in_t vin;
    vin.a_dev_mg    = mag - g.g_mg;
    vin.fwd_mg      = fwd;
    vin.speed       = speed;
    vin.prev_speed  = prev_speed;
    vin.braking     = port_mc_braking();
    vin.in_hump     = (g.h.active != 0u);
    vin.wheel_valid = snap_ok;
    uint8_t votes = vote_step(&g.v, &vin);
#if IMU_ENABLE_DEBUG
    dbg.body_mg[0] = a[0]; dbg.body_mg[1] = a[1]; dbg.body_mg[2] = a[2];
    dbg.a_dev_mg = vin.a_dev_mg; dbg.pitch_rad = g.pitch_f; dbg.votes = votes; dbg.speed_mm_s = speed;
#endif
    if (g.impact_cd != 0u) {
        g.impact_cd--;
    }
    if (vote_confirmed(votes) && (g.impact_cd == 0u)) {
        port_estop_set(ESTOP_SRC_IMU_IMPACT);          /* fast path: flag, not queue */
        (void)post_evt(IMU_EVT_IMPACT, votes);         /* the VC still gets the event */
        g.impact_cd = IMPACT_COOLDOWN_POLLS;
        stat_inc(&g.stats.impacts);
    } else if (votes != 0u) {
        stat_inc(&g.stats.single_votes);               /* one vote alone never stops the car */
    }

    if ((g.f.mask & IMU_FAULT_SATURATION) != 0u) {
        return;                                        /* mute hump/motion while saturated */
    }

    uint8_t h2 = 0u;
    bool moving = snap_ok && (speed >= HUMP_MIN_SPEED);
    if (hump_step(&g.h, g.pitch_f, ds, moving, HUMP_GAIN, &h2)) {
        (void)post_evt(IMU_EVT_HUMP, h2);
        stat_inc(&g.stats.humps);
    }
#if IMU_ENABLE_DEBUG
    dbg.hump_active = (g.h.active != 0u); dbg.hump_mm = g.h.h_mm;
#endif
    motion_class_t cls;
    if (motion_step(&g.m, fwd, lat, speed, prev_speed, &cls)) {
        (void)post_evt(IMU_EVT_MOTION, (uint8_t)cls);
    }
}

void imu_step(uint32_t late_us)
{
    stat_inc(&g.stats.polls);

    /* ---- Motion Control snapshot: one aligned load, then sanity-check it ---- */
    uint32_t snap  = port_read_mc_snapshot();
    uint16_t speed = MC_SNAP_SPEED(snap);
    uint16_t dist  = MC_SNAP_DIST(snap);
    unsigned seq   = MC_SNAP_SEQ(snap);
    if (seq == g.snap_seq) {
        if (g.snap_stale < 7u) {
            g.snap_stale++;
        }
    } else {
        g.snap_stale = 0u;
        g.snap_seq = seq;
    }
    bool snap_ok = (g.snap_stale < SNAP_STALE_POLLS);
    if (!snap_ok) {
        stat_inc(&g.stats.snap_stale);                 /* Motion Control is not updating: ignore wheel data */
    }

    uint16_t ds = (uint16_t)(dist - g.prev_dist16);    /* modular: handles the 16-bit wrap */
    uint16_t max_ds = (uint16_t)(((speed / 100u) * SNAP_DS_SAFETY_MULT) + 8u);
    if (ds > max_ds) {                                 /* distance was reset or jumped: do not integrate it */
        ds = 0u;
        stat_inc(&g.stats.ds_jumps);
    }
    uint16_t prev_speed = (uint16_t)g.prev_speed;
    g.prev_dist16 = dist;
    g.prev_speed  = speed;

    handle_cmd(speed);

    if (g.state == (unsigned)IMU_ST_INIT) {
        read_result_t r = init_sensor();
        bool blocking = fault_on_read(&g.f, r, NULL);
        if (r == RD_TIMEOUT) {
            port_i2c_recover();
        }
        if (r == RD_OK) {
            start_calibration();
        } else if (blocking) {
            enter_fault();
        }
        return;
    }

    raw3_t raw = { 0, 0, 0, false };
    read_result_t rr = read_sample(&raw);
    if (rr == RD_TIMEOUT) {
        port_i2c_recover();                            /* one bus-recovery attempt per timeout */
    }
    (void)fault_on_read(&g.f, rr, (rr == RD_OK) ? &raw : NULL);
    fault_on_lateness(&g.f, late_us);
    bool blocking = ((g.f.mask & IMU_FAULT_BLOCKING) != 0u);

    if ((g.state != (unsigned)IMU_ST_FAULT) && blocking) {
        enter_fault();
    }

    switch ((imu_state_t)g.state) {
    case IMU_ST_FAULT:
        if ((g.fault_posted == 0u) || (g.fault_posted_mask != g.f.mask)) {   /* once per mask change */
            if (post_evt(IMU_EVT_SENSOR_FAULT, (uint8_t)g.f.mask)) {
                g.fault_posted = 1u;                   /* if the queue was full we retry next poll */
                g.fault_posted_mask = g.f.mask;
            }
        }
        if (g.fault_age < 3u) {
            g.fault_age++;
        }
        if ((g.fault_age >= GOOD_READS_TO_RECOVER) && (rr == RD_OK) &&
            fault_recovered(&g.f) && (speed == 0u)) {
            read_result_t ir = init_sensor();          /* re-configure: the sensor may have lost power */
            if (ir == RD_OK) {
                fault_clear_blocking(&g.f);            /* ESTOP bit stays set: only the VC clears it */
                start_calibration();
            } else {
                (void)fault_on_read(&g.f, ir, NULL);
            }
        }
        break;
    case IMU_ST_CALIBRATE:
        if ((rr == RD_OK) && raw.fresh) {
            step_calibrate(&raw, speed);
        }
        if ((g.f.mask & IMU_FAULT_CAL_FAIL) != 0u) {
            enter_fault();
        }
        break;
    case IMU_ST_MONITORING:
        if (rr == RD_OK) {
            step_monitoring(&raw, speed, prev_speed, ds, snap_ok);
        }
        break;
    default:
        break;
    }
}

void imu_run(void)
{
    imu_init();
    for (;;) {
        uint32_t late_us = port_wait_next_poll();
        imu_step(late_us);
    }
}
