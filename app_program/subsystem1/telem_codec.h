/*
 *  telem_codec.h
 *  Subsystem 1 (Buddy 1) -- encode/decode MQTT payloads, plus helpers that
 *  clamp application values into their field ranges.
 *
 *  Pure functions: no kernel calls and no globals, so any task may call
 *  them on its own data at the same time, and they build on a PC.
 *  Field order on the wire is defined here and must match TF_TM_BITS /
 *  TF_HB_BITS in telem_format.h.
 */

#ifndef TELEM_CODEC_H
#define TELEM_CODEC_H

#include <stdint.h>
#include <stdbool.h>
#include "telem_format.h"

/* Heartbeat contents (Buddy 1's own data, filled by the Telemetry task) */
typedef struct
{
    uint32_t sys_state   : TF_SYS_STATE_BITS;
    uint32_t fault       : TF_FAULT_BITS;
    uint32_t drops       : TF_DROPS_BITS;
    uint32_t reconnects  : TF_RECONNECT_BITS;
    uint16_t uptime_s;                          /* full 16 bits */
} telem_hb_t;

/* ---- Range helpers: saturate a value into its field --------------- */
uint32_t tc_sat_u(uint32_t value, uint32_t max);
int32_t  tc_sat_s(int32_t value, int32_t min, int32_t max);

/* ---- Snapshot setters (used by the Vehicle Controller) ------------- */
void tc_snap_set_speed(telem_snapshot_t *p_snap, int32_t left_cm_s, int32_t right_cm_s);
void tc_snap_set_enc(telem_snapshot_t *p_snap, uint32_t left_count, uint32_t right_count);
void tc_snap_set_dist(telem_snapshot_t *p_snap, uint32_t dist_cm);
void tc_snap_set_hump(telem_snapshot_t *p_snap, uint32_t last_mm, uint32_t peak_mm, uint32_t count);
/* Obstacle profile (Buddy 5). b_echo = false means no echo / nothing ahead.
   angle_deg: bearing of the closest point, 30..150 deg; pass a value
   outside 30..150 (e.g. 0) if there is no bearing yet. */
void tc_snap_set_obst(telem_snapshot_t *p_snap, bool b_echo, uint32_t dist_cm,
                      uint32_t width_cm, uint32_t angle_deg,
                      uint32_t clear_l_cm, uint32_t clear_r_cm);

/* ---- Encoders: return bytes written, or 0 if p_out is too small ---- */
uint16_t tc_encode_tm(telem_snapshot_t const *p_snap, uint8_t seq,
                      uint8_t *p_out, uint16_t out_len);
uint16_t tc_encode_hb(telem_hb_t const *p_hb, uint8_t seq,
                      uint8_t *p_out, uint16_t out_len);
uint16_t tc_encode_ack(remote_cmd_t cmd, ack_result_t result,
                       uint8_t *p_out, uint16_t out_len);

/* ---- Decoders ------------------------------------------------------ */
/* Command from the PC. Returns ACK_FORWARDED if the payload is valid
   (caller then forwards it), or ACK_INVALID. p_cmd is always filled with
   whatever could be read, so an INVALID ACK can still echo it. */
ack_result_t tc_decode_cmd(uint8_t const *p_in, uint16_t len, remote_cmd_t *p_cmd);

/* Telemetry (used by PC tests / tools). false if wrong length or version. */
bool tc_decode_tm(uint8_t const *p_in, uint16_t len,
                  telem_snapshot_t *p_snap, uint8_t *p_seq);

#endif /* TELEM_CODEC_H */
