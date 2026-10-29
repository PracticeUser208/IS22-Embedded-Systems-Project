#include "imu_logic.h"
#include <math.h>
#include <string.h>

/* Every bit-field width is tied to the constant that limits it. */
_Static_assert(NACK_POLLS_TO_FAULT   <= 3,  "nack_run is 2 bits");
_Static_assert(STALE_POLLS_TO_FAULT  <= 3,  "stale_run is 2 bits");
_Static_assert(SAT_POLLS_TO_FLAG     <= 3,  "sat_run is 2 bits");
_Static_assert(GOOD_READS_TO_RECOVER <= 3,  "good_run is 2 bits");
_Static_assert(LATE_POLLS_TO_FAULT   <= 15, "late_run is 4 bits");
_Static_assert(IMPACT_WINDOW_POLLS   <= 3,  "vote countdown is 2 bits");
_Static_assert(IMPACT_DECEL_POLLS    <= 3,  "decel run is 2 bits");
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

/* ======================= 2-of-3 voting ======================= */
#define VW_DECEL_SHIFT  6u
#define VW_FIELD_MASK   0x3u

static unsigned vw_get(const vote_win_t *w, unsigned i)
{
    return ((unsigned)w->packed >> (2u * i)) & VW_FIELD_MASK;
}

static void vw_set(vote_win_t *w, unsigned i, unsigned v)
{
    unsigned cleared = (unsigned)w->packed & ~(VW_FIELD_MASK << (2u * i));
    w->packed = (uint8_t)(cleared | ((v & VW_FIELD_MASK) << (2u * i)));
}

static unsigned vw_decel(const vote_win_t *w)
{
    return ((unsigned)w->packed >> VW_DECEL_SHIFT) & VW_FIELD_MASK;
}

static void vw_set_decel(vote_win_t *w, unsigned v)
{
    unsigned cleared = (unsigned)w->packed & ~(VW_FIELD_MASK << VW_DECEL_SHIFT);
    w->packed = (uint8_t)(cleared | ((v & VW_FIELD_MASK) << VW_DECEL_SHIFT));
}

void vote_init(vote_win_t *w)
{
    w->packed = 0u;
}

uint8_t vote_step(vote_win_t *w, const vote_in_t *in)
{
    float t0 = IMPACT_ACCEL_DEV_MG * (in->in_hump ? IMPACT_ACCEL_DEV_HUMP_MULT : 1.0f);
    if (fabsf(in->a_dev_mg) >= t0) {
        vw_set(w, 0u, IMPACT_WINDOW_POLLS);
    }

    if (in->fwd_mg <= -IMPACT_DECEL_MG) {
        if (vw_decel(w) < 3u) {
            vw_set_decel(w, vw_decel(w) + 1u);
        }
    } else {
        vw_set_decel(w, 0u);
    }
    if ((!in->braking) && (vw_decel(w) >= IMPACT_DECEL_POLLS)) {
        vw_set(w, 1u, IMPACT_WINDOW_POLLS);
    }

    if ((!in->braking) && in->wheel_valid && (in->prev_speed >= IMPACT_WHEEL_MIN_SPEED) &&
        (in->speed < in->prev_speed)) {
        uint32_t drop = (uint32_t)(in->prev_speed - in->speed);
        if ((drop * 100u) >= ((uint32_t)in->prev_speed * IMPACT_WHEEL_DROP_PCT)) {
            vw_set(w, 2u, IMPACT_WINDOW_POLLS);
        }
    }

    uint8_t mask = 0u;
    for (unsigned i = 0u; i < 3u; i++) {
        unsigned cd = vw_get(w, i);
        if (cd > 0u) {
            mask |= (uint8_t)(1u << i);
            vw_set(w, i, cd - 1u);
        }
    }
    return mask;
}

bool vote_confirmed(uint8_t mask)
{
    unsigned n = (mask & 1u) + ((mask >> 1) & 1u) + ((mask >> 2) & 1u);
    return n >= 2u;
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

bool hump_step(hump_t *h, float pitch, uint16_t ds_mm, bool moving, float gain, uint8_t *height_x2)
{
    if (h->active == 0u) {
        if (moving && (pitch >= HUMP_PITCH_ENTER_RAD)) {
            h->active = 1u;
            h->h_mm = 0.0f;
            h->h_max_mm = 0.0f;
            h->quiet = 0u;
            h->stall = 0u;
        } else {
            return false;
        }
    }
    if (!moving) {                                    /* stalled on the hump: height unreliable */
        BF_INC(h->stall, 31u);
        if (h->stall >= HUMP_ABORT_STALL_POLLS) {
            h->active = 0u;
        }
        return false;
    }
    h->stall = 0u;

    h->h_mm += sinf(pitch) * (float)ds_mm * gain;     /* height gained = sin(pitch) x distance */
    if (h->h_mm > h->h_max_mm) {
        h->h_max_mm = h->h_mm;
    }

    if (fabsf(pitch) < HUMP_PITCH_EXIT_RAD) {
        BF_INC(h->quiet, 3u);
        if (h->quiet >= HUMP_END_POLLS) {
            h->active = 0u;
            if (h->h_max_mm < HUMP_MIN_HEIGHT_MM) {
                return false;
            }
            float v = (h->h_max_mm * 2.0f) + 0.5f;    /* round to 0.5 mm steps */
            if (v > (float)HUMP_HEIGHT_X2_MAX) {
                v = (float)HUMP_HEIGHT_X2_MAX;        /* overrange: reported as the maximum */
            }
            *height_x2 = (uint8_t)v;
            return true;
        }
    } else {
        h->quiet = 0u;
    }
    return false;
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
