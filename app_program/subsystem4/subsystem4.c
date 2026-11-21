/*
 *	subsystem4.c
 *	Subsystem 4 -- IMU and Terrain: creates its objects and starts its
 *	task (see subsystem4.h).
 *
 *	Objects (TEAM.md sections 5 and 6):
 *	  imu_q     message buffer   VC -> IMU, 4-byte words (IF-15)
 *	  cyc_imu   cyclic handler   10 ms poll tick (Tribunal 1: drift-free)
 *	  imu_task  task             runs Buddy 4's imu_run()
 */

#include "subsystem4.h"

#if !TEMPLATE_DEMO

#include "subsystem4.h"
#include "imu.h"

#define S4_CMD_DEPTH		4	/* 4-byte commands */

EXPORT ID	imu_q;			/* IF-15, replaces the VC's weak 0 */
EXPORT ID	tid_imu;
EXPORT ID	cyc_imu;

EXPORT ER subsystem4_init(void)
{
	T_CMBF	cmbf = {0};
	T_CCYC	ccyc = {0};

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = (SZ)(S4_CMD_DEPTH * 8);	/* 4-byte message + header */
	cmbf.maxmsz = (INT)sizeof(UW);
	imu_q = tk_cre_mbf(&cmbf);
	if (imu_q < E_OK) {
		return imu_q;
	}

	/* Wakes imu_task every 10 ms, however long one poll takes (Tribunal 1:
	   "drift-free by using a cyclic handler").  Created stopped; started
	   in subsystem4_start() once the task exists. */
	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)imu_cyclic_handler;
	ccyc.cyctim = S4_POLL_MS;
	ccyc.cycphs = S4_POLL_MS;
	cyc_imu = tk_cre_cyc(&ccyc);
	if (cyc_imu < E_OK) {
		return cyc_imu;
	}

	i2c1_init();				/* GP2/GP3, 400 kHz */
	return E_OK;
}

EXPORT ER subsystem4_start(void)
{
	tid_imu = app_start_task(imu_task, S4_PRI_IMU, 0, "imu_task");
	if (tid_imu < E_OK) {
		return tid_imu;
	}
	return tk_sta_cyc(cyc_imu);
}

EXPORT void imu_cyclic_handler(void *exinf)
{
	/* Interrupt context: keep it short, never wait */
	(void)exinf;
	(void)tk_wup_tsk(tid_imu);
}

EXPORT void imu_task(INT stacd, void *exinf)
{
	(void)stacd; (void)exinf;

	tm_printf((UB*)"S4: IMU task up (I2C1 SDA GP%d SCL GP%d, LSM303D 0x%02x), every %d ms\n",
		  (INT)I2C1_SDA_PIN, (INT)I2C1_SCL_PIN, (INT)IMU_I2C_ADDR_ACCEL, (INT)S4_POLL_MS);
	imu_run();				/* Buddy 4's task body, never returns */
}

#else /* TEMPLATE_DEMO: the template's monitor example */

EXPORT ID	mbf_summary;	/* IF-03 */
EXPORT ID	flg_events;	/* IF-03 */
EXPORT ID	alm_monitor;

EXPORT ER subsystem4_init(void)
{
	T_CFLG	cflg = {0};
	T_CMBF	cmbf = {0};
	T_CALM	calm = {0};

	/* Event flag: a 32-bit pattern of bits.  Tasks wait for a bit (or a
	   combination of bits) to be set. */
	cflg.flgatr  = TA_TFIFO | TA_WMUL;	/* allow several waiters */
	cflg.iflgptn = 0;			/* all bits clear to begin with */
	flg_events = tk_cre_flg(&cflg);
	if (flg_events < E_OK) {
		return flg_events;
	}

	/* Message buffer: COPIES variable-length data into a kernel ring
	   buffer.  Here it carries short text lines from subsystem 3. */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = 4 * S4_SUMMARY_MAX;	/* total buffer size, bytes */
	cmbf.maxmsz = S4_SUMMARY_MAX;		/* largest single message */
	mbf_summary = tk_cre_mbf(&cmbf);
	if (mbf_summary < E_OK) {
		return mbf_summary;
	}

	/* Alarm handler: called ONCE, a given time after tk_sta_alm().
	   The monitor task re-arms it each time round its loop. */
	calm.almatr = TA_HLNG;
	calm.almhdr = (FP)monitor_alarm_handler;
	alm_monitor = tk_cre_alm(&calm);
	return (alm_monitor < E_OK) ? alm_monitor : E_OK;
}

EXPORT ER subsystem4_start(void)
{
	/* No affinity: the scheduler may run it on either core */
	ID tid = app_start_task(monitor_task, S4_PRI_MONITOR, 0, "monitor_task");
	return (tid < E_OK) ? tid : E_OK;
}

#endif /* TEMPLATE_DEMO */

/* ======== from imu.c ======== */
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
 *   - Lifted / overturned detection (16 Nov; replaces the impact voting, requirement changed 11 Nov)
 *     runs only in MONITORING (blind in CALIBRATE/FAULT: CALIBRATE only happens with the car stopped,
 *     and entering FAULT raises ESTOP).  It sets ESTOP_SRC_IMU_LIFTED at once, so Motion Control
 *     stops the motors on its next control tick.
 */
#include "imu.h"
#include <math.h>
#include <stdlib.h>
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
_Static_assert(SNAP_STALE_POLLS < 8,          "snap_stale is 3 bits");
_Static_assert(IMU_ST_FAULT < (1 << IMU_STATE_BITS), "imu state does not fit its field");

typedef struct {
    /* working floats (arithmetic values stay native width) */
    float    sum[3], mn[3], mx[3], base_mg[3];
    float    g_mg, pitch0, pitch_f;
    float    roll0, roll_f;                            /* 20 Nov: sideways tilt, like the pitch */
    int8_t   tilt_p[2], tilt_r[2];                     /* 20 Nov: furthest from 0 this / last 0.5 s, 0.5 deg */
    uint8_t  tilt_polls;
    /* packed sub-state */
    fault_t    f;
    lift_t     l;
    hump_t     h;
    motion_t   m;
    uint16_t   prev_dist16;
    /* packed task state: 2+1+7+2+3+6+10+6+3+1 = 41 bits */
    unsigned state             : IMU_STATE_BITS;
    unsigned fault_posted      : 1;
    unsigned fault_posted_mask : IMU_FAULT_MASK_BITS;
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
    lift_init(&g.l);
    hump_init(&g.h);
    motion_init(&g.m);
    g.state = (unsigned)IMU_ST_CALIBRATE;
}

static void enter_fault(void)
{
    g.state = (unsigned)IMU_ST_FAULT;
    g.fault_age = 0u;
    lift_init(&g.l);
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
    lift_init(&g.l);
    hump_init(&g.h);
    motion_init(&g.m);
    g.state = (unsigned)IMU_ST_INIT;
}

imu_state_t imu_get_state(void)      { return (imu_state_t)g.state; }
uint8_t     imu_get_fault_mask(void) { return (uint8_t)g.f.mask; }
int16_t     imu_take_up_peak(void)
{
    int16_t p = g.stats.up_peak_mm_s;
    g.stats.up_peak_mm_s = 0;
    return p;
}
void        imu_take_pitch_peaks(int16_t *p_max_x10, int16_t *p_min_x10)
{
    *p_max_x10 = g.stats.pitch_max_x10;
    *p_min_x10 = g.stats.pitch_min_x10;
    g.stats.pitch_max_x10 = 0;
    g.stats.pitch_min_x10 = 0;
}
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
    g.roll0   = roll_from_accel(g.base_mg[0], g.base_mg[1], g.base_mg[2]);
    g.roll_f  = 0.0f;
    g.state   = (unsigned)IMU_ST_MONITORING;

    if (g.sysfault_set != 0u) {                        /* recovered: drop our system fault bit, tell the VC */
        port_sysfault_clear(SYSFAULT_IMU_FAULT);
        g.sysfault_set = 0u;
        (void)post_evt(IMU_EVT_SENSOR_FAULT, 0u);      /* payload 0 = fault cleared */
    }
}

/* 20 Nov: the tilt for the dashboard: pitch and roll in 0.5 deg steps, the value furthest from 0 in
 * this and the last 0.5 s window, published as one 32-bit word (port_tilt_publish) */
static int8_t tilt_x2(float rad)
{
    float v = rad * (360.0f / 3.14159265f);            /* rad -> 0.5 deg */
    if (v > 63.0f) {
        v = 63.0f;
    }
    if (v < -64.0f) {
        v = -64.0f;
    }
    return (int8_t)((v < 0.0f) ? (v - 0.5f) : (v + 0.5f));
}

static int8_t tilt_further(int8_t a, int8_t b)
{
    return (abs(b) > abs(a)) ? b : a;
}

static void tilt_track(void)
{
    g.tilt_p[0] = tilt_further(g.tilt_p[0], tilt_x2(g.pitch_f));
    g.tilt_r[0] = tilt_further(g.tilt_r[0], tilt_x2(g.roll_f));
    g.tilt_polls++;
    if (g.tilt_polls >= 50u) {                         /* 0.5 s at the 10 ms poll */
        port_tilt_publish(tilt_further(g.tilt_p[0], g.tilt_p[1]), tilt_further(g.tilt_r[0], g.tilt_r[1]));
        g.tilt_p[1] = g.tilt_p[0];
        g.tilt_r[1] = g.tilt_r[0];
        g.tilt_p[0] = 0;
        g.tilt_r[0] = 0;
        g.tilt_polls = 0u;
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
#if IMU_ENABLE_DEBUG
    float mag   = sqrtf((a[0] * a[0]) + (a[1] * a[1]) + (a[2] * a[2]));
#endif
    float fwd   = a[0] - g.base_mg[0];
    float lat   = a[1] - g.base_mg[1];
    float pitch = IMU_PITCH_SIGN * (pitch_from_accel(a[0], a[1], a[2]) - g.pitch0);
    g.pitch_f += PITCH_LPF_ALPHA * (pitch - g.pitch_f);
    float roll = roll_from_accel(a[0], a[1], a[2]) - g.roll0;
    g.roll_f += PITCH_LPF_ALPHA * (roll - g.roll_f);
    tilt_track();

    /* ---- lifted / overturned (16 Nov).  Stays active even while saturated.  The flag first: Motion
     *      Control stops the motors on its next control tick, whatever the queue does. ---- */
    uint8_t lifted = lift_step(&g.l, a, g.base_mg, g.g_mg);
#if IMU_ENABLE_DEBUG
    dbg.body_mg[0] = a[0]; dbg.body_mg[1] = a[1]; dbg.body_mg[2] = a[2];
    dbg.a_dev_mg = mag - g.g_mg; dbg.pitch_rad = g.pitch_f; dbg.lift = lifted; dbg.up_mm_s = g.l.v_mm_s;
    dbg.speed_mm_s = speed;
#endif
    if ((g.l.fired == 0u) && (g.l.v_mm_s > (float)g.stats.up_peak_mm_s)) {
        g.stats.up_peak_mm_s = (int16_t)g.l.v_mm_s;    /* tuning aid: how close driving gets to LIFT_UP_MM_S */
    }
    if (lifted != 0u) {
        port_estop_set(ESTOP_SRC_IMU_LIFTED);          /* fast path: flag, not queue */
        (void)post_evt(IMU_EVT_LIFTED, lifted);        /* the VC still gets the event (payload: why) */
        stat_inc(&g.stats.lifts);
    }

    if ((g.f.mask & IMU_FAULT_SATURATION) != 0u) {
        return;                                        /* mute hump/motion while saturated */
    }

    uint8_t h2 = 0u;
    bool moving = snap_ok && (speed >= HUMP_MIN_SPEED);
    if (moving) {                                      /* tuning aid: the pitch the hump check sees */
        int16_t p10 = (int16_t)(g.pitch_f * (1800.0f / 3.14159265f));
        if (p10 > g.stats.pitch_max_x10) {
            g.stats.pitch_max_x10 = p10;
        }
        if (p10 < g.stats.pitch_min_x10) {
            g.stats.pitch_min_x10 = p10;
        }
    }
    if (hump_step(&g.h, g.pitch_f, g.roll_f, ds, moving, HUMP_GAIN, &h2)) {
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

/* ======== from imu_logic.c ======== */
#include <math.h>
#include <string.h>

/* Every bit-field width is tied to the constant that limits it. */
_Static_assert(NACK_POLLS_TO_FAULT   <= 3,  "nack_run is 2 bits");
_Static_assert(STALE_POLLS_TO_FAULT  <= 3,  "stale_run is 2 bits");
_Static_assert(SAT_POLLS_TO_FLAG     <= 3,  "sat_run is 2 bits");
_Static_assert(GOOD_READS_TO_RECOVER <= 3,  "good_run is 2 bits");
_Static_assert(LATE_POLLS_TO_FAULT   <= 15, "late_run is 4 bits");
_Static_assert(LIFT_UP_POLLS         <= 3,  "up_run is 2 bits");
_Static_assert(LIFT_TILT_POLLS       <= 31, "tilt_run is 5 bits");
_Static_assert(LIFT_REARM_POLLS      <= 127, "calm_run is 7 bits");
_Static_assert(HUMP_END_POLLS        <= 3,  "quiet is 2 bits");
_Static_assert(HUMP_ABORT_STALL_POLLS <= 31, "stall is 5 bits");
_Static_assert(MOTION_STOP_POLLS     <= 7,  "stopped_run is 3 bits");
_Static_assert(MOTION_DEBOUNCE_POLLS <= 3,  "cand_run is 2 bits");
_Static_assert(IMU_FAULT_QUEUE_FULL < (1u << IMU_FAULT_MASK_BITS), "fault mask is 7 bits");

#define BF_INC(f, mx)   do { if ((f) < (mx)) { (f)++; } } while (0)

static int abs_i(int v)
{
    return (v < 0) ? -v : v;
}

/* ======================= faults ======================= */
void fault_init(fault_t *f)
{
    memset(f, 0, sizeof *f);
}

bool fault_on_read(fault_t *f, read_result_t r, const raw3_t *raw)
{
    if (r == RD_TIMEOUT) {
        f->mask |= IMU_FAULT_BUS_STUCK;              /* bus hang: fault immediately */
        f->nack_run = 0u;
        f->good_run = 0u;
    } else if (r == RD_NACK) {
        BF_INC(f->nack_run, 3u);
        f->good_run = 0u;
        if (f->nack_run >= NACK_POLLS_TO_FAULT) {
            f->mask |= IMU_FAULT_NACK;
        }
    } else {
        f->nack_run = 0u;
        if (raw != NULL) {
            if (raw->fresh) {
                f->stale_run = 0u;
                BF_INC(f->good_run, 3u);
                bool sat = (abs_i(raw->x) >= IMU_RAW_FULLSCALE) ||
                           (abs_i(raw->y) >= IMU_RAW_FULLSCALE) ||
                           (abs_i(raw->z) >= IMU_RAW_FULLSCALE);
                if (sat) {
                    BF_INC(f->sat_run, 3u);
                    if (f->sat_run >= SAT_POLLS_TO_FLAG) {
                        f->mask |= IMU_FAULT_SATURATION;
                    }
                } else {
                    f->sat_run = 0u;
                    f->mask &= ~IMU_FAULT_SATURATION;
                }
            } else {                                  /* no new sample since the last read */
                BF_INC(f->stale_run, 3u);
                f->good_run = 0u;
                if (f->stale_run >= STALE_POLLS_TO_FAULT) {
                    f->mask |= IMU_FAULT_STALE;
                }
            }
        }
    }
    return (f->mask & IMU_FAULT_BLOCKING) != 0u;
}

void fault_on_lateness(fault_t *f, uint32_t late_us)
{
    if (late_us > LATE_POLL_US) {
        BF_INC(f->late_run, 15u);
        if (f->late_run >= LATE_POLLS_TO_FAULT) {
            f->mask |= IMU_FAULT_TIMING;
        }
    } else {
        f->late_run = 0u;
    }
}

void fault_set_cal_fail(fault_t *f)
{
    f->mask |= IMU_FAULT_CAL_FAIL;
}

void fault_note_queue(fault_t *f, bool posted)
{
    if (posted) {
        f->mask &= ~IMU_FAULT_QUEUE_FULL;
    } else {
        f->mask |= IMU_FAULT_QUEUE_FULL;
    }
}

bool fault_recovered(const fault_t *f)
{
    if ((f->mask & IMU_FAULT_CAL_FAIL) != 0u) {
        return false;                                 /* only a RESET clears this one */
    }
    return (f->good_run >= GOOD_READS_TO_RECOVER) && (f->late_run == 0u) &&
           (f->stale_run == 0u) && (f->nack_run == 0u);
}

void fault_clear_blocking(fault_t *f)
{
    f->mask &= ~IMU_FAULT_BLOCKING;
    f->nack_run = 0u;
    f->stale_run = 0u;
    f->good_run = 0u;
    f->late_run = 0u;
}

/* ======================= lifted / overturned ======================= */
#define LIFT_DT_S          ((float)IMU_POLL_PERIOD_US / 1.0e6f)
#define MG_TO_MM_S2        9.80665f               /* 1 mg = 9.80665 mm/s^2 */
#define DEG_TO_RAD         0.017453293f

void lift_init(lift_t *l)
{
    memset(l, 0, sizeof *l);
}

uint8_t lift_step(lift_t *l, const float a[3], const float base[3], float g_mg)
{
    uint8_t cause = 0u;

    if (g_mg < 1.0f) {
        return 0u;                                    /* not calibrated */
    }
    if (l->seeded == 0u) {                            /* first poll: start from the calibrated gravity */
        l->f[0] = base[0]; l->f[1] = base[1]; l->f[2] = base[2];
        l->seeded = 1u;
    }

    /* ---- UP: acceleration along the calibrated "up", minus 1 g, summed to a speed ---- */
    float up_mg = (((a[0] * base[0]) + (a[1] * base[1]) + (a[2] * base[2])) / g_mg) - g_mg;
    l->v_mm_s += up_mg * MG_TO_MM_S2 * LIFT_DT_S;
    l->v_mm_s -= l->v_mm_s * (LIFT_DT_S / LIFT_V_TAU_S);
    if (l->v_mm_s < -LIFT_V_FLOOR_MM_S) {
        l->v_mm_s = -LIFT_V_FLOOR_MM_S;
    }
    bool up = (l->v_mm_s >= LIFT_UP_MM_S);
    if (up) {
        BF_INC(l->up_run, 3u);
    } else {
        l->up_run = 0u;
    }

    /* ---- TILT: where gravity points now (smoothed) against where it pointed when calibrated ---- */
    for (int i = 0; i < 3; i++) {
        l->f[i] += LIFT_LPF_ALPHA * (a[i] - l->f[i]);
    }
    float fm = sqrtf((l->f[0] * l->f[0]) + (l->f[1] * l->f[1]) + (l->f[2] * l->f[2]));
    bool tilt = false;
    if (fm > (0.3f * g_mg)) {                         /* in free fall there is no direction to compare */
        float cos_t = ((l->f[0] * base[0]) + (l->f[1] * base[1]) + (l->f[2] * base[2])) / (fm * g_mg);
        float roll  = (l->f[1] / fm) - (base[1] / g_mg);          /* sine of the sideways tip */
        tilt = (cos_t < cosf(LIFT_TILT_DEG * DEG_TO_RAD)) || (fabsf(roll) > sinf(LIFT_ROLL_DEG * DEG_TO_RAD));
    }
    if (tilt) {
        BF_INC(l->tilt_run, 31u);
    } else {
        l->tilt_run = 0u;
    }

    if (l->fired != 0u) {                             /* reported: quiet until level and still again */
        if ((!tilt) && (fabsf(l->v_mm_s) < (LIFT_UP_MM_S / 4.0f))) {
            BF_INC(l->calm_run, 127u);
            if (l->calm_run >= LIFT_REARM_POLLS) {
                l->fired = 0u;
                l->up_run = 0u;
                l->tilt_run = 0u;
            }
        } else {
            l->calm_run = 0u;
        }
        return 0u;
    }

    if (l->up_run >= LIFT_UP_POLLS) {
        cause |= (uint8_t)LIFT_CAUSE_UP;
    }
    if (l->tilt_run >= LIFT_TILT_POLLS) {
        cause |= (uint8_t)LIFT_CAUSE_TILT;
    }
    if (cause != 0u) {
        l->fired = 1u;
        l->calm_run = 0u;
    }
    return cause;
}

/* ======================= hump ======================= */
void hump_init(hump_t *h)
{
    memset(h, 0, sizeof *h);
}

float pitch_from_accel(float ax, float ay, float az)
{
    return atan2f(ax, sqrtf((ay * ay) + (az * az)));
}

float roll_from_accel(float ax, float ay, float az)
{
    return atan2f(ay, sqrtf((ax * ax) + (az * az)));
}

/* 20 Nov (20:15), as asked: the level when RESET is pressed (the IMU calibrates then; pitch and
 * roll are measured from it) is the baseline.  A tilt away from it (HUMP_PITCH_ENTER_RAD, nose
 * up or down, or sideways) starts a hump; back within HUMP_PITCH_EXIT_RAD of it for HUMP_CALM_MM
 * of driving ends it: one hump, reported at once with its height (no smallest height).  A stop on
 * the hump only pauses it; after HUMP_MAX_LEN_MM it ends anyway. */
bool hump_step(hump_t *h, float pitch, float roll, uint16_t ds_mm, bool moving, float gain, uint8_t *height_x2)
{
    if (h->active == 0u) {
        if (moving && ((fabsf(pitch) >= HUMP_PITCH_ENTER_RAD) || (fabsf(roll) >= HUMP_PITCH_ENTER_RAD))) {
            h->active = 1u;
            h->h_mm = 0.0f;
            h->h_max_mm = 0.0f;
            h->roll_max = 0.0f;
            h->len_mm = 0.0f;
            h->calm_mm = 0.0f;
            h->stall = 0u;
        } else {
            return false;
        }
    }
    if (!moving) {                                    /* stopped on the hump: wait */
        BF_INC(h->stall, 31u);
        return false;
    }
    h->stall = 0u;

    h->h_mm += sinf(pitch) * (float)ds_mm * gain;     /* height gained = sin(pitch) x distance */
    if (fabsf(h->h_mm) > h->h_max_mm) {
        h->h_max_mm = fabsf(h->h_mm);                 /* either way round: the sign depends on
                                                         which wheels go over first */
    }
    if (fabsf(roll) > h->roll_max) {
        h->roll_max = fabsf(roll);                    /* one wheel on the hump: tipped sideways */
    }
    h->len_mm += (float)ds_mm;

    if ((fabsf(pitch) < HUMP_PITCH_EXIT_RAD) && (fabsf(roll) < HUMP_PITCH_EXIT_RAD)) {
        h->calm_mm += (float)ds_mm;                   /* back at the baseline */
    } else {
        h->calm_mm = 0.0f;
    }
    if ((h->calm_mm < HUMP_CALM_MM) && (h->len_mm < HUMP_MAX_LEN_MM)) {
        return false;
    }
    h->active = 0u;
    float height = HUMP_TRACK_MM * sinf(h->roll_max) * gain;
    if (h->h_max_mm > height) {
        height = h->h_max_mm;
    }
    if (height < HUMP_MIN_HEIGHT_MM) {
        return false;
    }
    float v = (height * 2.0f) + 0.5f;                 /* round to 0.5 mm steps */
    if (v > (float)HUMP_HEIGHT_X2_MAX) {
        v = (float)HUMP_HEIGHT_X2_MAX;                /* overrange: reported as the maximum */
    }
    *height_x2 = (uint8_t)v;
    return true;
}

/* ======================= motion ======================= */
void motion_init(motion_t *m)
{
    memset(m, 0, sizeof *m);
    m->cur  = (unsigned)MOTION_STATIONARY;
    m->cand = (unsigned)MOTION_STATIONARY;
}

bool motion_step(motion_t *m, float fwd_mg, float lat_mg, uint16_t speed,
                 uint16_t prev_speed, motion_class_t *out)
{
    motion_class_t raw;
    if (speed <= MOTION_STOP_SPEED) {
        BF_INC(m->stopped_run, 7u);
    } else {
        m->stopped_run = 0u;
    }

    if (m->stopped_run >= MOTION_STOP_POLLS) {
        raw = MOTION_STATIONARY;
    } else if (speed <= MOTION_STOP_SPEED) {
        raw = (motion_class_t)m->cur;                 /* settling: hold the current class */
    } else if (fabsf(lat_mg) >= MOTION_TURN_LAT_MG) {
        raw = MOTION_TURNING;
    } else if ((((int)speed - (int)prev_speed) >= MOTION_ACCEL_DSPEED) || (fwd_mg >= MOTION_ACCEL_FWD_MG)) {
        raw = MOTION_ACCELERATING;
    } else {
        raw = MOTION_STEADY;
    }

    if ((unsigned)raw == m->cand) {
        BF_INC(m->cand_run, 3u);
    } else {
        m->cand = (unsigned)raw;
        m->cand_run = 1u;
    }

    if ((m->cand != m->cur) && (m->cand_run >= MOTION_DEBOUNCE_POLLS)) {
        m->cur = m->cand;
        *out = (motion_class_t)m->cur;
        return true;
    }
    return false;
}

/* ======== from imu_port_pico_utk.c ======== */
/*
 *	imu_port_pico_utk.c
 *	Subsystem 4 -- imu_port.h for the Pico W + micro T-Kernel build (the
 *	file Buddy 4's imu_port.h refers to).  Buddy 4's imu.c and
 *	imu_logic.c are unchanged: this file only connects them to the car,
 *	as the Week 6 design and the Tribunal 1 changes describe:
 *	  I2C1 GP2/GP3, owned by the IMU task only        drv_i2c1.c
 *	  ~10 ms poll, drift-free: cyclic handler + 1 MHz timestamps
 *	  events to the VC on imu_evt_q, zero timeout; full = dropped, counted
 *	  commands from the VC on imu_q, non-blocking
 *	  lifted / IMU fault: SET own bit in ESTOP_REQ (only the VC clears)
 *	  own bit in the shared fault mask (FAULT_IMU), set and cleared
 *	  speed + distance from Motion Control, read-only  imu_mc_snap.c
 *	The IMU never writes PWM (single-writer rule).
 *
 *	imu_port.h needs the toolchain's size_t (<stddef.h>), which clashes
 *	with the kernel's own typedef: tk/syslib.h's PROHIBIT_DEF_SIZE_T guard
 *	is for exactly this (same as lib/liblwip/.../sys_arch.c does).
 */

#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include "subsystem4.h"

#if !TEMPLATE_DEMO

#include "imu.h"
#include "imu.h"
#include "imu.h"
#include "imu.h"
#include "subsystem4.h"
#include "subsystem4.h"
#include "../vc_imu_msg.h"

#define S4_WAIT_TMO_MS		100	/* backstop if the cyclic stops */
#define S4_IMU_ESTOP_BITS	(ESTOP_IMU_LIFTED | ESTOP_IMU_FAULT)

/* ---- Buddy 4's encodings must match the team's shared ones ---- */
_Static_assert(ESTOP_SRC_IMU_LIFTED == ESTOP_IMU_LIFTED, "ESTOP lifted bit differs");
_Static_assert(ESTOP_SRC_IMU_FAULT == ESTOP_IMU_FAULT, "ESTOP IMU fault bit differs");
_Static_assert(ESTOP_SRC_OBSTACLE_15 == ESTOP_OBSTACLE, "ESTOP obstacle bit differs");
_Static_assert(SYSFAULT_IMU_FAULT == FAULT_IMU, "IMU fault-mask bit differs");
/* ... and the Vehicle Controller's copy of his message formats */
_Static_assert(VC_IMU_EVT_HUMP == (unsigned)IMU_EVT_HUMP, "imu_evt_q HUMP differs");
_Static_assert(VC_IMU_EVT_MOTION == (unsigned)IMU_EVT_MOTION, "imu_evt_q MOTION differs");
_Static_assert(VC_IMU_EVT_LIFTED == (unsigned)IMU_EVT_LIFTED, "imu_evt_q LIFTED differs");
_Static_assert(VC_IMU_EVT_SENSOR_FAULT == (unsigned)IMU_EVT_SENSOR_FAULT, "imu_evt_q FAULT differs");
_Static_assert(VC_IMU_MOT_STATIONARY == (unsigned)MOTION_STATIONARY, "motion class differs");
_Static_assert(VC_IMU_MOT_ACCELERATING == (unsigned)MOTION_ACCELERATING, "motion class differs");
_Static_assert(VC_IMU_MOT_TURNING == (unsigned)MOTION_TURNING, "motion class differs");
_Static_assert(VC_IMU_MOT_STEADY == (unsigned)MOTION_STEADY, "motion class differs");
_Static_assert(VC_IMU_CMD_RESET == (unsigned)IMU_CMD_RESET, "imu_q RESET differs");
_Static_assert(VC_IMU_MSG_PACK(3u, 0x55u, 0xBEEFu) == IMU_MSG_PACK(3u, 0x55u, 0xBEEFu),
	       "imu_evt_q word layout differs");
_Static_assert(S4_SNAP_PACK(1023u, 0xABCDu, 63u) == MC_SNAP_PACK(1023u, 0xABCDu, 63u),
	       "Motion Control snapshot layout differs");
_Static_assert(S4_POLL_MS * 1000 == IMU_POLL_PERIOD_US, "poll period differs");

/* Owned by the Vehicle Controller; 0 if the firmware is built without it */
__attribute__((weak)) ID imu_evt_q = 0;

/* ---- I2C1 ---- */
LOCAL port_i2c_rc_t map_rc(INT rc)
{
	if (rc == I2C1_TIMEOUT) {
		return PORT_I2C_TIMEOUT;
	}
	return (rc == I2C1_OK) ? PORT_I2C_OK : PORT_I2C_NACK;
}

port_i2c_rc_t port_i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
	UB buf[2];

	buf[0] = reg;
	buf[1] = val;
	return map_rc(i2c1_write(addr, buf, 2, FALSE));
}

port_i2c_rc_t port_i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n)
{
	INT rc = i2c1_write(addr, &reg, 1, TRUE);	/* register address, no STOP */

	if (rc != I2C1_OK) {
		return map_rc(rc);
	}
	return map_rc(i2c1_read(addr, buf, (INT)n, FALSE));	/* repeated START */
}

void port_i2c_recover(void)
{
	i2c1_recover();
}

/* ---- time ---- */
uint64_t port_time_us(void)
{
	UW hi;
	UW lo;

	do {				/* raw registers: no latch, so re-read */
		hi = in_w(TIMER_TIMERAWH);
		lo = in_w(TIMER_TIMERAWL);
	} while (hi != in_w(TIMER_TIMERAWH));
	return ((uint64_t)hi << 32) | (uint64_t)lo;
}

uint32_t port_time_ms(void)
{
	SYSTIM t;

	(void)tk_get_tim(&t);
	return (uint32_t)t.lo;
}

/* Sleep until the cyclic handler's next 10 ms wake-up.  Lateness is
   measured against an ideal 10 ms grid on the 1 MHz timer, as the
   bring-up port does (resync after a long stall). */
uint32_t port_wait_next_poll(void)
{
	static uint64_t	next;
	uint64_t	now;

#if LIFT_REPORT_S > 0
	{
		static UW polls;

		polls++;
		if (polls >= ((UW)LIFT_REPORT_S * (1000000U / IMU_POLL_PERIOD_US))) {
			int16_t	pmax, pmin;

			polls = 0U;
			/* tuning aid: LIFT_UP_MM_S has to stay well above this
			   while the car drives */
			imu_take_pitch_peaks(&pmax, &pmin);
			tm_printf((UB*)"S4: upward speed peak %d mm/s in the last %d s (lifted at %d)\n",
				  (INT)imu_take_up_peak(), (INT)LIFT_REPORT_S, (INT)LIFT_UP_MM_S);
			/* 19 Nov, hump tuning: pitch while moving (hump starts at
			   HUMP_PITCH_ENTER_RAD) */
			tm_printf((UB*)"S4: pitch while moving %d to %d (0.1 deg; hump from %d)\n",
				  (INT)pmin, (INT)pmax, (INT)(HUMP_PITCH_ENTER_RAD * 572.958f));
		}
	}
#endif
	(void)tk_slp_tsk(S4_WAIT_TMO_MS);
	now = port_time_us();
	if ((next == 0U) || (now > (next + (5U * IMU_POLL_PERIOD_US)))) {
		next = now;			/* first poll, or after a stall */
	} else {
		next += IMU_POLL_PERIOD_US;
	}
	return (now > next) ? (uint32_t)(now - next) : 0U;
}

/* ---- queues ---- */
LOCAL const char *const s4_evt_names[4] = { "HUMP", "MOTION", "LIFTED", "SENSOR_FAULT" };

bool port_post_event(uint32_t msg)
{
	UW	w = msg;
	bool	ok = (imu_evt_q > 0) &&
		     (tk_snd_mbf(imu_evt_q, &w, (INT)sizeof(w), TMO_POL) == E_OK);

	/* console evidence for the test log (not application data) */
	tm_printf((UB*)"S4: -> imu_evt_q %s payload %d%s\n",
		  s4_evt_names[VC_IMU_MSG_TYPE(msg)], (INT)VC_IMU_MSG_PAYLOAD(msg),
		  ok ? "" : " (DROPPED: queue full)");
	return ok;
}

bool port_recv_cmd(uint8_t *cmd)
{
	UW w;

	if ((cmd == NULL) || (imu_q <= 0) ||
	    (tk_rcv_mbf(imu_q, &w, TMO_POL) != (INT)sizeof(w))) {
		return false;
	}
	*cmd = (uint8_t)(w & 0xFFU);
	return true;
}

/* ---- Motion Control (read-only) ---- */
uint32_t port_read_mc_snapshot(void)
{
	return s4_mc_snapshot();
}

bool port_mc_braking(void)
{
	return s4_mc_braking();
}

/* ---- shared flags: the IMU sets only its own bits ---- */
/* 20 Nov: the car's tilt for the dashboard, one aligned 32-bit store (vc_imu_msg.h) */
volatile uint32_t vc_imu_tilt_word;

void port_tilt_publish(int8_t pitch_x2, int8_t roll_x2)
{
	vc_imu_tilt_word = VC_IMU_TILT_PACK(pitch_x2, roll_x2);
}

void port_estop_set(uint32_t src_bits)
{
	UINT b = (UINT)src_bits & S4_IMU_ESTOP_BITS;

	if ((b != 0U) && (flg_estop_req > 0)) {
		(void)tk_set_flg(flg_estop_req, b);	/* never cleared here */
	}
}

void port_sysfault_set(uint32_t bits)
{
	UINT b = (UINT)bits & FAULT_IMU;

	if ((b != 0U) && (flg_sysfault > 0)) {
		(void)tk_set_flg(flg_sysfault, b);
	}
}

void port_sysfault_clear(uint32_t bits)
{
	UINT b = (UINT)bits & FAULT_IMU;

	if ((b != 0U) && (flg_sysfault > 0)) {
		(void)tk_clr_flg(flg_sysfault, ~b);
	}
}

#endif /* !TEMPLATE_DEMO */

/* ======== from drv_i2c1.c ======== */
/*
 *	drv_i2c1.c
 *	Subsystem 4 -- I2C1 master on GP2/GP3 (see drv_i2c1.h).
 *
 *	RP2040 I2C (Synopsys DW_apb_i2c), polled.  The sequences follow the
 *	pico-sdk's i2c_write_blocking_until / i2c_read_blocking_until, which
 *	Buddy 4's bring-up port (bringup/imu_port_pico_bare.c) calls, so the
 *	IMU code sees the same behaviour on the car as on the bench:
 *	  - a write with nostop leaves the bus claimed; the next transfer
 *	    starts with a repeated START (register read = write the register
 *	    address with nostop, then read);
 *	  - an abort (no ACK) is NACK, no progress within I2C1_TIMEOUT_US is
 *	    TIMEOUT;
 *	  - recovery clocks SCL up to 9 times until the slave lets go of SDA.
 *	Timing for 400 kHz at clk_sys = 125 MHz (cpu_clock.c), computed as
 *	the pico-sdk does: HCNT 126, LCNT 187, spike length 11, SDA hold 38
 *	(the same numbers as the kernel's own I2C driver).
 */


/* I2C1 registers (RP2040 datasheet, section 4.3) */
#define IC_BASE			0x40048000U
#define IC_CON			(IC_BASE + 0x00U)
#define IC_TAR			(IC_BASE + 0x04U)
#define IC_DATA_CMD		(IC_BASE + 0x10U)
#define IC_FS_SCL_HCNT		(IC_BASE + 0x1CU)
#define IC_FS_SCL_LCNT		(IC_BASE + 0x20U)
#define IC_INTR_MASK		(IC_BASE + 0x30U)
#define IC_RAW_INTR_STAT	(IC_BASE + 0x34U)
#define IC_RX_TL		(IC_BASE + 0x38U)
#define IC_TX_TL		(IC_BASE + 0x3CU)
#define IC_CLR_TX_ABRT		(IC_BASE + 0x54U)
#define IC_CLR_STOP_DET		(IC_BASE + 0x60U)
#define IC_ENABLE		(IC_BASE + 0x6CU)
#define IC_TXFLR		(IC_BASE + 0x74U)
#define IC_RXFLR		(IC_BASE + 0x78U)
#define IC_SDA_HOLD		(IC_BASE + 0x7CU)
#define IC_TX_ABRT_SOURCE	(IC_BASE + 0x80U)
#define IC_FS_SPKLEN		(IC_BASE + 0xA0U)

#define CON_MASTER_MODE		(1U << 0)
#define CON_SPEED_FAST		(2U << 1)
#define CON_RESTART_EN		(1U << 5)
#define CON_SLAVE_DISABLE	(1U << 6)
#define CON_TX_EMPTY_CTRL	(1U << 8)

#define CMD_READ		(1U << 8)
#define CMD_STOP		(1U << 9)
#define CMD_RESTART		(1U << 10)

#define RAW_TX_EMPTY		(1U << 4)
#define RAW_STOP_DET		(1U << 9)

#define TX_FIFO_DEPTH		16U

/* 400 kHz from 125 MHz (pico-sdk i2c_set_baudrate arithmetic) */
#define CLK_SYS_HZ		125000000U
#define BAUD_PERIOD		((CLK_SYS_HZ + (I2C1_HZ / 2U)) / I2C1_HZ)	/* 313 */
#define BAUD_LCNT		((BAUD_PERIOD * 3U) / 5U)			/* 187 */
#define BAUD_HCNT		(BAUD_PERIOD - BAUD_LCNT)			/* 126 */
#define BAUD_SPKLEN		((BAUD_LCNT < 16U) ? 1U : (BAUD_LCNT / 16U))	/* 11  */
#define BAUD_SDA_HOLD		(((CLK_SYS_HZ * 3U) / 10000000U) + 1U)		/* 38  */

#define PAD_I2C			(GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT)

LOCAL BOOL	restart_next;	/* last transfer ended without STOP */

EXPORT UW i2c1_now_us(void)
{
	return in_w(TIMER_TIMERAWL);
}

LOCAL BOOL expired(UW deadline)
{
	return ((W)(i2c1_now_us() - deadline) > 0) ? TRUE : FALSE;
}

LOCAL void wait_us(UW us)
{
	UW t0 = i2c1_now_us();

	while ((i2c1_now_us() - t0) < us) {
		/* a few microseconds, bus recovery only */
	}
}

LOCAL void pins_i2c(void)
{
	out_w(GPIO_CTRL(I2C1_SDA_PIN), GPIO_CTRL_FUNCSEL_I2C);
	out_w(GPIO(I2C1_SDA_PIN), PAD_I2C);
	out_w(GPIO_CTRL(I2C1_SCL_PIN), GPIO_CTRL_FUNCSEL_I2C);
	out_w(GPIO(I2C1_SCL_PIN), PAD_I2C);
}

EXPORT void i2c1_init(void)
{
	/* Reset the block (it may be left in any state by a previous run) */
	set_w(RESETS_RESET, RESETS_RESET_I2C1);
	clr_w(RESETS_RESET, RESETS_RESET_I2C1);
	while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_I2C1) == 0U) {
		/* a few clock cycles */
	}

	out_w(IC_ENABLE, 0U);
	out_w(IC_CON, CON_SPEED_FAST | CON_MASTER_MODE | CON_SLAVE_DISABLE |
		      CON_RESTART_EN | CON_TX_EMPTY_CTRL);
	out_w(IC_TX_TL, 0U);
	out_w(IC_RX_TL, 0U);
	out_w(IC_INTR_MASK, 0U);		/* polled, no interrupts */
	out_w(IC_FS_SCL_HCNT, BAUD_HCNT);
	out_w(IC_FS_SCL_LCNT, BAUD_LCNT);
	out_w(IC_FS_SPKLEN, BAUD_SPKLEN);
	out_w(IC_SDA_HOLD, (in_w(IC_SDA_HOLD) & 0xFFFF0000U) | BAUD_SDA_HOLD);
	out_w(IC_ENABLE, 1U);

	pins_i2c();
	restart_next = FALSE;
}

LOCAL void set_target(UB addr)
{
	out_w(IC_ENABLE, 0U);
	out_w(IC_TAR, (UW)addr & 0x7FU);
	out_w(IC_ENABLE, 1U);
}

EXPORT INT i2c1_write(UB addr, const UB *p_src, INT len, BOOL nostop)
{
	UW	deadline = i2c1_now_us() + I2C1_TIMEOUT_US;
	BOOL	abort = FALSE;
	BOOL	timeout = FALSE;
	INT	i;

	if ((p_src == NULL) || (len <= 0)) {
		return I2C1_NACK;
	}
	set_target(addr);

	for (i = 0; i < len; i++) {
		BOOL first = (i == 0) ? TRUE : FALSE;
		BOOL last  = (i == (len - 1)) ? TRUE : FALSE;

		out_w(IC_DATA_CMD, (((first != FALSE) && (restart_next != FALSE)) ? CMD_RESTART : 0U) |
				   (((last != FALSE) && (nostop == FALSE)) ? CMD_STOP : 0U) |
				   (UW)p_src[i]);

		/* wait for the byte to leave the FIFO */
		while ((in_w(IC_RAW_INTR_STAT) & RAW_TX_EMPTY) == 0U) {
			if (expired(deadline) != FALSE) {
				timeout = TRUE;
				break;
			}
		}
		if (timeout != FALSE) {
			abort = TRUE;
		} else {
			if (in_w(IC_TX_ABRT_SOURCE) != 0U) {	/* no ACK */
				(void)in_w(IC_CLR_TX_ABRT);
				abort = TRUE;
			}
			if ((abort != FALSE) || ((last != FALSE) && (nostop == FALSE))) {
				while ((in_w(IC_RAW_INTR_STAT) & RAW_STOP_DET) == 0U) {
					if (expired(deadline) != FALSE) {
						timeout = TRUE;
						abort = TRUE;
						break;
					}
				}
				if (timeout == FALSE) {
					(void)in_w(IC_CLR_STOP_DET);
				}
			}
		}
		if (abort != FALSE) {
			break;
		}
	}

	restart_next = nostop;
	if (timeout != FALSE) {
		return I2C1_TIMEOUT;
	}
	return (abort != FALSE) ? I2C1_NACK : I2C1_OK;
}

EXPORT INT i2c1_read(UB addr, UB *p_dst, INT len, BOOL nostop)
{
	UW	deadline = i2c1_now_us() + I2C1_TIMEOUT_US;
	BOOL	abort = FALSE;
	BOOL	timeout = FALSE;
	INT	i;

	if ((p_dst == NULL) || (len <= 0)) {
		return I2C1_NACK;
	}
	set_target(addr);

	for (i = 0; i < len; i++) {
		BOOL first = (i == 0) ? TRUE : FALSE;
		BOOL last  = (i == (len - 1)) ? TRUE : FALSE;

		while (in_w(IC_TXFLR) >= TX_FIFO_DEPTH) {	/* room to queue the read */
			if (expired(deadline) != FALSE) {
				timeout = TRUE;
				break;
			}
		}
		if (timeout != FALSE) {
			break;
		}
		out_w(IC_DATA_CMD, (((first != FALSE) && (restart_next != FALSE)) ? CMD_RESTART : 0U) |
				   (((last != FALSE) && (nostop == FALSE)) ? CMD_STOP : 0U) |
				   CMD_READ);

		while (in_w(IC_RXFLR) == 0U) {
			(void)in_w(IC_TX_ABRT_SOURCE);
			if (in_w(IC_CLR_TX_ABRT) != 0U) {	/* reading clears it */
				abort = TRUE;
				break;
			}
			if (expired(deadline) != FALSE) {
				timeout = TRUE;
				break;
			}
		}
		if ((abort != FALSE) || (timeout != FALSE)) {
			break;
		}
		p_dst[i] = (UB)(in_w(IC_DATA_CMD) & 0xFFU);
	}

	restart_next = nostop;
	if (timeout != FALSE) {
		return I2C1_TIMEOUT;
	}
	return (abort != FALSE) ? I2C1_NACK : I2C1_OK;
}

/* A slave can hold SDA low if a transfer was cut off mid-byte: clock
   SCL (open drain: drive low, release) until it lets go, then restart
   the controller.  Same idea as the bring-up port's recovery. */
EXPORT void i2c1_recover(void)
{
	INT k;

	out_w(IC_ENABLE, 0U);
	out_w(GPIO_OUT_CLR, (1U << I2C1_SCL_PIN) | (1U << I2C1_SDA_PIN));
	out_w(GPIO_OE_CLR, (1U << I2C1_SCL_PIN) | (1U << I2C1_SDA_PIN));
	out_w(GPIO_CTRL(I2C1_SCL_PIN), GPIO_CTRL_FUNCSEL_SIO);
	out_w(GPIO_CTRL(I2C1_SDA_PIN), GPIO_CTRL_FUNCSEL_SIO);

	for (k = 0; (k < 9) && ((in_w(GPIO_IN) & (1U << I2C1_SDA_PIN)) == 0U); k++) {
		out_w(GPIO_OE_SET, 1U << I2C1_SCL_PIN);		/* SCL low  */
		wait_us(5U);
		out_w(GPIO_OE_CLR, 1U << I2C1_SCL_PIN);		/* SCL high */
		wait_us(5U);
	}
	i2c1_init();
}
