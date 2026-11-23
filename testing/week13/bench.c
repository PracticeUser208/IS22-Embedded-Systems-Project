/*
 *  bench.c
 *  Week 13 evidence: CPU cost of one telemetry frame on the Pico's
 *  Cortex-M0+.  week13_measure.py compiles this with the firmware's
 *  compiler and flags, together with the firmware's own bitpack.c and
 *  telem_codec.c, runs each bench_x() in an instruction-level emulator and
 *  counts the instructions it executes.  Not part of the firmware.
 *
 *  bench_pack():  the snapshot packed into the 22-byte binary frame (what
 *                 the firmware does, tc_encode_tm).
 *  bench_text():  the same values written as compact JSON text with
 *                 snprintf (the text representation it replaces).
 *  bench_unpack(): the binary frame decoded again (tc_decode_tm).
 */

#include <stdio.h>
#include <string.h>
#include "telem_codec.h"

static telem_snapshot_t snap;
static uint8_t          frame[TF_TM_LEN];
static char             text[512];
volatile int            result;

static void fill(telem_snapshot_t *p_s)
{
    (void)memset(p_s, 0, sizeof *p_s);
    p_s->sys_state  = SYS_PATROLLING;
    p_s->drive_mode = DRIVE_FOLLOW_LINE;
    tc_snap_set_speed(p_s, 25, -3);
    tc_snap_set_enc(p_s, 5000u, 4990u);
    tc_snap_set_dist(p_s, 350u);
    p_s->ir_left    = 1u;
    p_s->ir_barcode = 1u;
    p_s->line_state = LINE_FOLLOWING;
    p_s->barcode    = BARCODE_RIGHT;
    tc_snap_set_hump(p_s, 18u, 30u, 2u);
    p_s->motion     = MOTION_CLIMBING_HUMP;
    tc_snap_set_obst(p_s, true, 45u, 12u, 95u, 30u, 70u);
    p_s->obst_plan  = OBST_PLAN_TURN_LEFT;
    p_s->obst_state = OBST_PROFILE_PLAN;
    p_s->fault      = FAULT_WIFI_LOSS;
    p_s->lift_cause = TF_LIFT_TIPPED;
    p_s->imu_fault  = 0x05u;
    p_s->pitch_x2   = 10;
    p_s->roll_x2    = -6;
}

void bench_init(void)
{
    fill(&snap);
}

void bench_pack(void)
{
    result = (int)tc_encode_tm(&snap, 42u, frame, (uint16_t)sizeof frame);
}

void bench_unpack(void)
{
    telem_snapshot_t back;
    uint8_t          seq;

    result = tc_decode_tm(frame, TF_TM_LEN, &back, &seq) ? 1 : 0;
}

void bench_text(void)
{
    result = snprintf(text, sizeof text,
        "{\"v\":%u,\"seq\":%u,\"state\":%u,\"mode\":%u,\"spd_l\":%d,\"spd_r\":%d,"
        "\"enc_l\":%u,\"enc_r\":%u,\"dist\":%u,\"ir_l\":%u,\"ir_c\":%u,\"ir_b\":%u,"
        "\"line\":%u,\"bc\":%u,\"hump_last\":%u,\"hump_peak\":%u,\"hump_n\":%u,"
        "\"motion\":%u,\"obst\":%u,\"width\":%u,\"angle\":%u,\"clear_l\":%u,"
        "\"clear_r\":%u,\"plan\":%u,\"scan\":%u,\"fault\":%u,\"estop\":%u,"
        "\"lift\":%u,\"imu_fault\":%u,\"pitch\":%d,\"roll\":%d}",
        (unsigned)TF_VERSION, 42u, (unsigned)snap.sys_state, (unsigned)snap.drive_mode,
        (int)snap.speed_l, (int)snap.speed_r, (unsigned)snap.enc_l, (unsigned)snap.enc_r,
        (unsigned)snap.dist_cm, (unsigned)snap.ir_left, (unsigned)snap.ir_center,
        (unsigned)snap.ir_barcode, (unsigned)snap.line_state, (unsigned)snap.barcode,
        (unsigned)snap.hump_last, (unsigned)snap.hump_peak, (unsigned)snap.hump_count,
        (unsigned)snap.motion, (unsigned)snap.obst_dist, (unsigned)snap.obst_width,
        (unsigned)snap.obst_angle, (unsigned)snap.clear_l, (unsigned)snap.clear_r,
        (unsigned)snap.obst_plan, (unsigned)snap.obst_state, (unsigned)snap.fault,
        (unsigned)snap.estop, (unsigned)snap.lift_cause, (unsigned)snap.imu_fault,
        (int)snap.pitch_x2, (int)snap.roll_x2);
}

const char *bench_text_buf(void)
{
    return text;
}
