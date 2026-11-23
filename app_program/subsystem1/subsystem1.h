/*
 *	subsystem1.h
 *	Subsystem 1 -- WiFi / MQTT communication, command and telemetry.
 *	Buddy 1, student ID 2500510.
 *
 *	This header is subsystem 1's PUBLIC interface: only what other
 *	subsystems are allowed to use.  Other subsystems include it as
 *	"../subsystem1/subsystem1.h".  Data formats: telem_format.h.
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"
#include "telem_format.h"

/* ---- Build switch ---------------------------------------------------
 * 1 = run a SIMULATED Vehicle Controller inside subsystem 1, so the
 *     telemetry and command path can be shown before integration.
 * 0 = the real Vehicle Controller posts to telem_q and reads remote_cmd_q.
 */
#ifndef S1_SIM_VC
#define S1_SIM_VC		1
#endif

/* ---- Priorities: subsystem 1 band 12-13 (TEAM.md section 4) -------- */
#define S1_PRI_LED		12	/* template LED example, kept for IF-05 */
#define S1_PRI_SIM_VC		12	/* simulated VC (only if S1_SIM_VC)     */
#define S1_PRI_COMM		13	/* Telemetry / Communications task      */

#define S1_LED_PIN		19	/* GP19: LED + 330R resistor to GND */
#define S1_TICK_MS		250	/* LED toggles every tick */

/* ---- Timing -------------------------------------------------------- */
#define S1_TELEM_PERIOD_MS	1000	/* FR21: VC posts >= 1 snapshot/s */
#define S1_HB_PERIOD_MS		5000	/* FR23: heartbeat every 5 s      */

/* ---- Queue depths -------------------------------------------------- */
#define S1_TELEM_Q_DEPTH	4	/* snapshots; poster drops if full */
#define S1_CMD_Q_DEPTH		4	/* decoded remote commands         */
#define S1_TX_Q_DEPTH		8	/* frames waiting for the MQTT client
					   (4 -> 8 on 3 Nov: + the IR trace) */

/* Message-buffer size in bytes.  The kernel stores a 4-byte header with
   every message and rounds the message up to 4 bytes (messagebuf.c). */
#define S1_MBF_SIZE(depth, msgsz) \
	((depth) * (4u + ((((UW)(msgsz)) + 3u) & ~3u)))

/* IF-06  VC -> S1: telemetry snapshots (telem_snapshot_t, 16 B).
 *   Owned by S1 (the receiver).  ONE writer only: the Vehicle Controller.
 *   Post:  tk_snd_mbf(mbf_telem_q, &snap, sizeof snap, TMO_POL)
 *          never blocks; E_TMOUT means full, so drop and count it.     */
IMPORT ID	mbf_telem_q;

/* IF-07  S1 -> VC: decoded remote commands (remote_cmd_t, 1 B).
 *   Created by S1 for now; moves to the VC owner once its folder exists.
 *   Read:  tk_rcv_mbf(mbf_remote_cmd_q, &cmd, tmout)                    */
IMPORT ID	mbf_remote_cmd_q;

/* Private: frames queued for the MQTT client (comm_bridge.h) */
IMPORT ID	mbf_s1_tx;

/* IF-05 (template example): the LED task's ID, so S4 may pause/resume it */
IMPORT ID	tid_led;

/* ---- Private to subsystem 1, shared between its own .c files ------- */
IMPORT ID	cyc_led;
IMPORT void	led_task(INT stacd, void *exinf);
IMPORT void	led_cyclic_handler(void *exinf);
IMPORT void	comm_task(INT stacd, void *exinf);
IMPORT UW	s1_now_ms(void);

/* IF-21  S3 (Line and Barcode) -> PC: one barcode read, published on
 *   TF_TOPIC_BC (layout in telem_format.h).  code = barcode_t, flags =
 *   TF_BC_xxx, p_text = the characters ("" = not read), p_widths_us =
 *   count bar/space widths in microseconds, p_levels = 4 raw levels
 *   (telem_format.h) or NULL.  Never blocks: dropped if the queue is
 *   full or there is no MQTT in this build.                            */
IMPORT void	s1_publish_barcode(UB code, UB flags, const char *p_text,
				   const uint32_t *p_widths_us, UB count,
				   const uint16_t *p_levels);

/* IF-22  S3 -> PC: TF_IR_TRACE_N side sensor samples (raw / 16) on
 *   TF_TOPIC_IR (layout in telem_format.h).  Never blocks.             */
IMPORT void	s1_publish_ir_trace(UB state, UB seq, UB cal_white16,
				    UB cal_black16, const UB *p_samples);
#if S1_SIM_VC
IMPORT void	sim_vc_task(INT stacd, void *exinf);
#endif

/* ======== from bitpack.h ======== */
/*
 *  bitpack.h
 *  Subsystem 1 (Buddy 1) -- bit-level writer/reader for compact MQTT payloads.
 *
 *  Packing rule: LSB-first. The first field starts at bit 0 of byte 0;
 *  each later field starts at the next free bit and may cross bytes.
 *  Only masks and shifts are used, so the layout is the same on the Pico,
 *  on a PC and in any ground-station script.
 *
 *  Uses only <stdint.h>/<stdbool.h> so it also builds on a PC for tests.
 */

#ifndef BITPACK_H
#define BITPACK_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint8_t  *p_buf;        /* byte buffer being written or read       */
    uint16_t  cap_bits;     /* buffer size in bits                     */
    uint16_t  pos;          /* next bit to write or read               */
    bool      b_overflow;   /* set on any out-of-range access; sticky  */
} bitstream_t;

/* Start writing: clears the buffer, then pos = 0 */
void     bs_init_write(bitstream_t *p_bs, uint8_t *p_buf, uint16_t len_bytes);

/* Start reading: does NOT touch the buffer, pos = 0 */
void     bs_init_read(bitstream_t *p_bs, uint8_t *p_buf, uint16_t len_bytes);

/* Write the low nbits (1..32) of value. Negative numbers: pass the value
   cast to uint32_t; the low nbits are its two's-complement form. */
void     bs_put(bitstream_t *p_bs, uint32_t value, uint8_t nbits);

/* Read nbits (1..32) as an unsigned value */
uint32_t bs_get(bitstream_t *p_bs, uint8_t nbits);

/* Read nbits (1..32) as a two's-complement signed value */
int32_t  bs_get_signed(bitstream_t *p_bs, uint8_t nbits);

/* Whole bytes touched so far (bits rounded up) */
uint16_t bs_bytes_used(bitstream_t const *p_bs);

#endif /* BITPACK_H */

/* ======== from telem_codec.h ======== */
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


#endif /* SUBSYSTEM1_H */