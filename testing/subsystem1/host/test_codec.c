/*
 *  test_codec.c
 *  PC unit tests for subsystem1/telem_codec.c (Buddy 1). Not in the Pico image.
 *  Build and run from testing/subsystem1/host:
 *    gcc -Wall -Wextra -I../../../app_program/subsystem1 test_codec.c \
 *        ../../../app_program/subsystem1/telem_codec.c \
 *        ../../../app_program/subsystem1/bitpack.c -o test_codec
 *    ./test_codec
 *  Expected bytes were computed independently (Python, same LSB-first rule).
 */

#include <stdio.h>
#include <string.h>
#include "telem_codec.h"

static int g_fail = 0;

static void check(char const *p_name, bool b_ok)
{
    printf("%s  %s\n", b_ok ? "PASS" : "FAIL", p_name);
    if (!b_ok)
    {
        g_fail++;
    }
}

static void dump(char const *p_label, uint8_t const *p_buf, uint16_t len)
{
    uint16_t i;

    printf("      %s:", p_label);
    for (i = 0u; i < len; i++)
    {
        printf(" %02X", p_buf[i]);
    }
    printf("\n");
}

/* The same snapshot as the agreed v2 test vector */
static void make_vector_snapshot(telem_snapshot_t *p_s)
{
    (void)memset(p_s, 0, sizeof *p_s);
    p_s->sys_state  = SYS_PATROLLING;
    p_s->drive_mode = DRIVE_FOLLOW_LINE;
    tc_snap_set_speed(p_s, 25, -3);
    tc_snap_set_enc(p_s, 5000u, 4990u);          /* > 4095: wraps */
    tc_snap_set_dist(p_s, 350u);
    p_s->ir_left    = 1u;
    p_s->ir_center  = 0u;
    p_s->ir_barcode = 1u;
    p_s->line_state = LINE_FOLLOWING;
    p_s->barcode    = BARCODE_RIGHT;
    tc_snap_set_hump(p_s, 18u, 30u, 2u);
    p_s->motion     = MOTION_CLIMBING_HUMP;
    /* echo at 45 cm, 12 cm wide, bearing 95 deg (index 13),
       left clearance 30 cm, right 70 cm (saturates to 63) */
    tc_snap_set_obst(p_s, true, 45u, 12u, 95u, 30u, 70u);
    p_s->obst_plan  = OBST_PLAN_TURN_LEFT;
    p_s->obst_state = OBST_PROFILE_PLAN;
    p_s->fault      = FAULT_WIFI_LOSS;
    p_s->estop      = 0u;
}

int main(void)
{
    static uint8_t const k_tm[TF_TM_LEN] =
        {0x52,0x11,0x99,0x3E,0xE2,0xF8,0x8D,0x57,0xD0,0x48,
         0xE2,0x91,0xB5,0x60,0x9A,0xF7,0x8F,0x20,0x00};
    static uint8_t const k_hb[TF_HB_LEN] = {0x3A,0x10,0x88,0x3E,0x80,0x81,0x00};

    telem_snapshot_t snap;
    telem_snapshot_t back;
    telem_hb_t       hb;
    remote_cmd_t     cmd;
    uint8_t          out[32];
    uint8_t          seq = 0u;
    uint16_t         n;
    ack_result_t     r;

    /* C1: telemetry encode matches the reference vector */
    make_vector_snapshot(&snap);
    n = tc_encode_tm(&snap, 42u, out, sizeof out);
    dump("tm", out, n);
    check("C1 telemetry v2 == 52 11 99 3E ... 20 00 (19 B)", (TF_TM_LEN == n) && (0 == memcmp(out, k_tm, TF_TM_LEN)));

    /* C2: telemetry round trip (decode what we encoded) */
    check("C2 telemetry decode ok", tc_decode_tm(out, n, &back, &seq));
    check("C2 round trip: seq, speeds, enc wrap, fault, hump",
          (42u == seq) && (25 == back.speed_l) && (-3 == back.speed_r) &&
          ((5000u & 0xFFFu) == back.enc_l) && (FAULT_WIFI_LOSS == back.fault) &&
          (30u == back.hump_peak) && (1u == back.ir_barcode) &&
          (MOTION_CLIMBING_HUMP == back.motion));
    check("C2 round trip: obstacle profile 45cm/12cm/idx13/30/63/TURN_LEFT",
          (45u == back.obst_dist) && (12u == back.obst_width) && (13u == back.obst_angle) &&
          (30u == back.clear_l) && (63u == back.clear_r) &&
          (OBST_PLAN_TURN_LEFT == back.obst_plan) && (OBST_PROFILE_PLAN == back.obst_state));

    /* C3: decoder rejects wrong length and wrong version */
    check("C3 reject wrong length", !tc_decode_tm(out, (uint16_t)(n - 1u), &back, &seq));
    out[0] = (uint8_t)((out[0] & 0xF8u) | 1u);   /* old version 1 */
    check("C3 reject unknown version", !tc_decode_tm(out, n, &back, &seq));

    /* C4: heartbeat encode matches the reference vector */
    (void)memset(&hb, 0, sizeof hb);
    hb.sys_state  = SYS_PATROLLING;
    hb.fault      = FAULT_WIFI_LOSS;
    hb.uptime_s   = 125u;
    hb.drops      = 3u;
    hb.reconnects = 1u;
    n = tc_encode_hb(&hb, 7u, out, sizeof out);
    dump("hb", out, n);
    check("C4 heartbeat v2 == 3A 10 88 3E 80 81 00", (TF_HB_LEN == n) && (0 == memcmp(out, k_hb, TF_HB_LEN)));

    /* C5: command decode: valid, invalid code, wrong length */
    out[0] = 0x29u;
    r = tc_decode_cmd(out, 1u, &cmd);
    check("C5 0x29 -> START seq 5, FORWARDED", (ACK_FORWARDED == r) && (CMD_START == cmd.code) && (5u == cmd.seq));
    out[0] = 0x32u;
    r = tc_decode_cmd(out, 1u, &cmd);
    check("C5 0x32 -> STOP seq 6, FORWARDED", (ACK_FORWARDED == r) && (CMD_STOP == cmd.code) && (6u == cmd.seq));
    out[0] = 0x06u;
    check("C5 code 6 -> INVALID", ACK_INVALID == tc_decode_cmd(out, 1u, &cmd));
    out[0] = 0x00u;
    check("C5 code 0 -> INVALID", ACK_INVALID == tc_decode_cmd(out, 1u, &cmd));
    out[0] = 0x29u;
    check("C5 2-byte payload -> INVALID", ACK_INVALID == tc_decode_cmd(out, 2u, &cmd));
    check("C5 empty payload -> INVALID", ACK_INVALID == tc_decode_cmd(out, 0u, &cmd));

    /* C6: ACK encode: START seq 5 FORWARDED -> 29 00 */
    cmd.code = CMD_START;
    cmd.seq  = 5u;
    n = tc_encode_ack(cmd, ACK_FORWARDED, out, sizeof out);
    check("C6 ACK == 29 00", (TF_ACK_LEN == n) && (0x29u == out[0]) && (0x00u == out[1]));
    n = tc_encode_ack(cmd, ACK_DUPLICATE, out, sizeof out);
    check("C6 ACK DUPLICATE == 29 01", (0x29u == out[0]) && (0x01u == out[1]));

    /* C7: saturation at the range limits */
    (void)memset(&snap, 0, sizeof snap);
    tc_snap_set_speed(&snap, 100, -100);
    check("C7 speed 100/-100 -> 63/-64", (63 == snap.speed_l) && (-64 == snap.speed_r));
    tc_snap_set_hump(&snap, 200u, 300u, 99u);
    check("C7 hump 200/300/99 -> 127/127/15", (127u == snap.hump_last) && (127u == snap.hump_peak) && (15u == snap.hump_count));
    tc_snap_set_obst(&snap, true, 600u, 80u, 150u, 99u, 0u);
    check("C7 obstacle 600 cm echo -> 510 (never 511), 150 deg -> idx 24",
          (510u == snap.obst_dist) && (63u == snap.obst_width) && (24u == snap.obst_angle) &&
          (63u == snap.clear_l) && (0u == snap.clear_r));
    tc_snap_set_obst(&snap, false, 0u, 0u, 0u, 0u, 0u);
    check("C7 no echo -> 511, no bearing -> 31", (TF_OBST_NO_ECHO == snap.obst_dist) && (TF_OBST_ANGLE_NONE == snap.obst_angle));
    tc_snap_set_obst(&snap, true, 50u, 5u, 32u, 0u, 0u);
    check("C7 32 deg rounds to 30 deg (idx 0)", 0u == snap.obst_angle);
    tc_snap_set_obst(&snap, true, 50u, 5u, 33u, 0u, 0u);
    check("C7 33 deg rounds to 35 deg (idx 1)", 1u == snap.obst_angle);
    tc_snap_set_obst(&snap, true, 50u, 5u, 151u, 0u, 0u);
    check("C7 151 deg (outside servo range) -> no bearing", TF_OBST_ANGLE_NONE == snap.obst_angle);
    tc_snap_set_dist(&snap, 99999u);
    check("C7 distance 99999 -> 16383", 16383u == snap.dist_cm);

    /* C8: output buffer too small -> 0 bytes, nothing claimed */
    check("C8 small buffer -> 0", 0u == tc_encode_tm(&snap, 0u, out, (uint16_t)(TF_TM_LEN - 1u)));

    printf("\n%s: %d failure(s)\n", (0 == g_fail) ? "ALL PASSED" : "FAILED", g_fail);
    return (0 == g_fail) ? 0 : 1;
}
