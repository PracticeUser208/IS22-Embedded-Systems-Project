/*
 *  telem_codec.c
 *  Subsystem 1 (Buddy 1) -- MQTT payload codec (see telem_codec.h).
 *
 *  Wire layout: LSB-first via bitpack.c; field order = order of the
 *  bs_put() calls below. Change both encoder and decoder together, and
 *  bump TF_VERSION.
 */

#include <string.h>
#include "bitpack.h"
#include "telem_codec.h"

/* ================= Range helpers ================================== */

uint32_t tc_sat_u(uint32_t value, uint32_t max)
{
    return (value > max) ? max : value;
}

int32_t tc_sat_s(int32_t value, int32_t min, int32_t max)
{
    int32_t result = value;

    if (value < min)
    {
        result = min;
    }
    else if (value > max)
    {
        result = max;
    }
    else
    {
        /* in range */
    }

    return result;
}

/* ================= Snapshot setters =============================== */

void tc_snap_set_speed(telem_snapshot_t *p_snap, int32_t left_cm_s, int32_t right_cm_s)
{
    p_snap->speed_l = tc_sat_s(left_cm_s,  TF_SPEED_MIN, TF_SPEED_MAX);
    p_snap->speed_r = tc_sat_s(right_cm_s, TF_SPEED_MIN, TF_SPEED_MAX);
}

void tc_snap_set_enc(telem_snapshot_t *p_snap, uint32_t left_count, uint32_t right_count)
{
    /* Modulo, not saturate: the PC unwraps it from successive frames */
    p_snap->enc_l = left_count  & ((1u << TF_ENC_BITS) - 1u);
    p_snap->enc_r = right_count & ((1u << TF_ENC_BITS) - 1u);
}

void tc_snap_set_dist(telem_snapshot_t *p_snap, uint32_t dist_cm)
{
    p_snap->dist_cm = tc_sat_u(dist_cm, TF_DIST_MAX);
}

void tc_snap_set_hump(telem_snapshot_t *p_snap, uint32_t last_mm, uint32_t peak_mm, uint32_t count)
{
    p_snap->hump_last  = tc_sat_u(last_mm, TF_HUMP_MM_MAX);
    p_snap->hump_peak  = tc_sat_u(peak_mm, TF_HUMP_MM_MAX);
    p_snap->hump_count = tc_sat_u(count,   TF_HUMP_COUNT_MAX);
}

void tc_snap_set_obst(telem_snapshot_t *p_snap, bool b_echo, uint32_t dist_cm,
                      uint32_t width_cm, uint32_t angle_deg,
                      uint32_t clear_l_cm, uint32_t clear_r_cm)
{
    uint32_t max_deg = TF_OBST_ANGLE_MIN_DEG + (TF_OBST_ANGLE_MAX_IDX * TF_OBST_ANGLE_STEP_DEG);

    /* "No echo" must never look like a real distance (or like 0 = touching) */
    p_snap->obst_dist  = b_echo ? tc_sat_u(dist_cm, TF_OBST_DIST_MAX) : TF_OBST_NO_ECHO;
    p_snap->obst_width = tc_sat_u(width_cm, TF_OBST_WIDTH_MAX);

    /* Bearing as a servo-step index (nearest 5 deg); outside 30..150 = none */
    if ((angle_deg < TF_OBST_ANGLE_MIN_DEG) || (angle_deg > max_deg))
    {
        p_snap->obst_angle = TF_OBST_ANGLE_NONE;
    }
    else
    {
        p_snap->obst_angle = (angle_deg - TF_OBST_ANGLE_MIN_DEG + (TF_OBST_ANGLE_STEP_DEG / 2u))
                             / TF_OBST_ANGLE_STEP_DEG;
    }

    p_snap->clear_l = tc_sat_u(clear_l_cm, TF_CLEAR_MAX);
    p_snap->clear_r = tc_sat_u(clear_r_cm, TF_CLEAR_MAX);
}

/* ================= Encoders ======================================= */

uint16_t tc_encode_tm(telem_snapshot_t const *p_snap, uint8_t seq,
                      uint8_t *p_out, uint16_t out_len)
{
    bitstream_t bs;
    uint16_t    n = 0u;

    if (out_len >= TF_TM_LEN)
    {
        bs_init_write(&bs, p_out, TF_TM_LEN);
        bs_put(&bs, TF_VERSION,                   TF_VERSION_BITS);
        bs_put(&bs, seq,                          TF_SEQ_BITS);
        bs_put(&bs, p_snap->sys_state,            TF_SYS_STATE_BITS);
        bs_put(&bs, p_snap->drive_mode,           TF_DRIVE_MODE_BITS);
        bs_put(&bs, (uint32_t)p_snap->speed_l,    TF_SPEED_BITS);
        bs_put(&bs, (uint32_t)p_snap->speed_r,    TF_SPEED_BITS);
        bs_put(&bs, p_snap->enc_l,                TF_ENC_BITS);
        bs_put(&bs, p_snap->enc_r,                TF_ENC_BITS);
        bs_put(&bs, p_snap->dist_cm,              TF_DIST_BITS);
        bs_put(&bs, p_snap->ir_left,              TF_IR_BITS);
        bs_put(&bs, p_snap->ir_center,            TF_IR_BITS);
        bs_put(&bs, p_snap->ir_barcode,           TF_IR_BITS);
        bs_put(&bs, p_snap->line_state,           TF_LINE_STATE_BITS);
        bs_put(&bs, p_snap->barcode,              TF_BARCODE_BITS);
        bs_put(&bs, p_snap->hump_last,            TF_HUMP_MM_BITS);
        bs_put(&bs, p_snap->hump_peak,            TF_HUMP_MM_BITS);
        bs_put(&bs, p_snap->hump_count,           TF_HUMP_COUNT_BITS);
        bs_put(&bs, p_snap->motion,               TF_MOTION_BITS);
        bs_put(&bs, p_snap->obst_dist,            TF_OBST_DIST_BITS);
        bs_put(&bs, p_snap->obst_width,           TF_OBST_WIDTH_BITS);
        bs_put(&bs, p_snap->obst_angle,           TF_OBST_ANGLE_BITS);
        bs_put(&bs, p_snap->clear_l,              TF_CLEAR_BITS);
        bs_put(&bs, p_snap->clear_r,              TF_CLEAR_BITS);
        bs_put(&bs, p_snap->obst_plan,            TF_OBST_PLAN_BITS);
        bs_put(&bs, p_snap->obst_state,           TF_OBST_STATE_BITS);
        bs_put(&bs, p_snap->fault,                TF_FAULT_BITS);
        bs_put(&bs, p_snap->estop,                TF_ESTOP_BITS);

        if (false == bs.b_overflow)
        {
            n = TF_TM_LEN;
        }
    }

    return n;
}

uint16_t tc_encode_hb(telem_hb_t const *p_hb, uint8_t seq,
                      uint8_t *p_out, uint16_t out_len)
{
    bitstream_t bs;
    uint16_t    n = 0u;

    if (out_len >= TF_HB_LEN)
    {
        bs_init_write(&bs, p_out, TF_HB_LEN);
        bs_put(&bs, TF_VERSION,         TF_VERSION_BITS);
        bs_put(&bs, seq,                TF_SEQ_BITS);
        bs_put(&bs, p_hb->sys_state,    TF_SYS_STATE_BITS);
        bs_put(&bs, p_hb->fault,        TF_FAULT_BITS);
        bs_put(&bs, p_hb->uptime_s,     TF_UPTIME_BITS);
        bs_put(&bs, p_hb->drops,        TF_DROPS_BITS);
        bs_put(&bs, p_hb->reconnects,   TF_RECONNECT_BITS);

        if (false == bs.b_overflow)
        {
            n = TF_HB_LEN;
        }
    }

    return n;
}

uint16_t tc_encode_ack(remote_cmd_t cmd, ack_result_t result,
                       uint8_t *p_out, uint16_t out_len)
{
    bitstream_t bs;
    uint16_t    n = 0u;

    if (out_len >= TF_ACK_LEN)
    {
        /* Byte 0 is laid out exactly like the command byte it answers */
        bs_init_write(&bs, p_out, TF_ACK_LEN);
        bs_put(&bs, cmd.code,           TF_CMD_CODE_BITS);
        bs_put(&bs, cmd.seq,            TF_CMD_SEQ_BITS);
        bs_put(&bs, (uint32_t)result,   TF_ACK_RESULT_BITS);

        if (false == bs.b_overflow)
        {
            n = TF_ACK_LEN;
        }
    }

    return n;
}

/* ================= Decoders ======================================= */

ack_result_t tc_decode_cmd(uint8_t const *p_in, uint16_t len, remote_cmd_t *p_cmd)
{
    bitstream_t  bs;
    uint8_t      buf[TF_CMD_LEN];
    ack_result_t result = ACK_INVALID;

    p_cmd->code = (uint8_t)CMD_NONE;
    p_cmd->seq  = 0u;

    if ((len > 0u) && (NULL != p_in))
    {
        /* copy so the stream never writes into the caller's const buffer */
        buf[0] = p_in[0];
        bs_init_read(&bs, buf, TF_CMD_LEN);
        p_cmd->code = (uint8_t)bs_get(&bs, TF_CMD_CODE_BITS);
        p_cmd->seq  = (uint8_t)bs_get(&bs, TF_CMD_SEQ_BITS);

        if ((TF_CMD_LEN == len) &&
            (p_cmd->code >= (uint8_t)CMD_START) && (p_cmd->code <= (uint8_t)CMD_RESET))
        {
            result = ACK_FORWARDED;
        }
    }

    return result;
}

bool tc_decode_tm(uint8_t const *p_in, uint16_t len,
                  telem_snapshot_t *p_snap, uint8_t *p_seq)
{
    bitstream_t bs;
    uint8_t     buf[TF_TM_LEN];
    bool        b_ok = false;

    if ((TF_TM_LEN == len) && (NULL != p_in))
    {
        (void)memcpy(buf, p_in, TF_TM_LEN);
        bs_init_read(&bs, buf, TF_TM_LEN);

        if (TF_VERSION == bs_get(&bs, TF_VERSION_BITS))
        {
            (void)memset(p_snap, 0, sizeof *p_snap);
            *p_seq             = (uint8_t)bs_get(&bs, TF_SEQ_BITS);
            p_snap->sys_state  = bs_get(&bs, TF_SYS_STATE_BITS);
            p_snap->drive_mode = bs_get(&bs, TF_DRIVE_MODE_BITS);
            p_snap->speed_l    = bs_get_signed(&bs, TF_SPEED_BITS);
            p_snap->speed_r    = bs_get_signed(&bs, TF_SPEED_BITS);
            p_snap->enc_l      = bs_get(&bs, TF_ENC_BITS);
            p_snap->enc_r      = bs_get(&bs, TF_ENC_BITS);
            p_snap->dist_cm    = bs_get(&bs, TF_DIST_BITS);
            p_snap->ir_left    = bs_get(&bs, TF_IR_BITS);
            p_snap->ir_center  = bs_get(&bs, TF_IR_BITS);
            p_snap->ir_barcode = bs_get(&bs, TF_IR_BITS);
            p_snap->line_state = bs_get(&bs, TF_LINE_STATE_BITS);
            p_snap->barcode    = bs_get(&bs, TF_BARCODE_BITS);
            p_snap->hump_last  = bs_get(&bs, TF_HUMP_MM_BITS);
            p_snap->hump_peak  = bs_get(&bs, TF_HUMP_MM_BITS);
            p_snap->hump_count = bs_get(&bs, TF_HUMP_COUNT_BITS);
            p_snap->motion     = bs_get(&bs, TF_MOTION_BITS);
            p_snap->obst_dist  = bs_get(&bs, TF_OBST_DIST_BITS);
            p_snap->obst_width = bs_get(&bs, TF_OBST_WIDTH_BITS);
            p_snap->obst_angle = bs_get(&bs, TF_OBST_ANGLE_BITS);
            p_snap->clear_l    = bs_get(&bs, TF_CLEAR_BITS);
            p_snap->clear_r    = bs_get(&bs, TF_CLEAR_BITS);
            p_snap->obst_plan  = bs_get(&bs, TF_OBST_PLAN_BITS);
            p_snap->obst_state = bs_get(&bs, TF_OBST_STATE_BITS);
            p_snap->fault      = bs_get(&bs, TF_FAULT_BITS);
            p_snap->estop      = bs_get(&bs, TF_ESTOP_BITS);

            b_ok = (false == bs.b_overflow);
        }
    }

    return b_ok;
}
