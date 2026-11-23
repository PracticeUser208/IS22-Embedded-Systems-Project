/*
 *	subsystem1.c
 *	Subsystem 1 -- WiFi / MQTT / telemetry: creates its kernel objects
 *	and starts its tasks.  Called from usermain() (app_main.c).
 */

#include "subsystem1.h"
#include "comm_bridge.h"

EXPORT ID	tid_led;		/* IF-05 */
EXPORT ID	cyc_led;
EXPORT ID	mbf_telem_q;		/* IF-06 */
EXPORT ID	mbf_remote_cmd_q;	/* IF-07 */
EXPORT ID	mbf_s1_tx;		/* Private: frames queued for the MQTT client */
/* Milliseconds since power-on (operating time, never set by anyone) */
EXPORT UW s1_now_ms(void)
{
	SYSTIM	t;

	(void)tk_get_otm(&t);
	return (UW)t.lo;
}

/* Create one message buffer for `depth` messages of `msgsz` bytes */
LOCAL ID make_mbf(UW depth, UW msgsz)
{
	T_CMBF	cmbf = {0};

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = (SZ)S1_MBF_SIZE(depth, msgsz);
	cmbf.maxmsz = (INT)msgsz;
	return tk_cre_mbf(&cmbf);
}

EXPORT ER subsystem1_init(void)
{
	T_CCYC	ccyc = {0};

	mbf_telem_q = make_mbf(S1_TELEM_Q_DEPTH, sizeof(telem_snapshot_t));
	if (mbf_telem_q < E_OK) {
		return mbf_telem_q;
	}

	mbf_remote_cmd_q = make_mbf(S1_CMD_Q_DEPTH, sizeof(remote_cmd_t));
	if (mbf_remote_cmd_q < E_OK) {
		return mbf_remote_cmd_q;
	}
	
	
	mbf_s1_tx = make_mbf(S1_TX_Q_DEPTH, S1_TX_MSG_MAX);
	if (mbf_s1_tx < E_OK) {
		return mbf_s1_tx;
	}

	/* LED example (template): cyclic handler wakes led_task */
	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)led_cyclic_handler;
	ccyc.cyctim = S1_TICK_MS;
	ccyc.cycphs = S1_TICK_MS;
	cyc_led = tk_cre_cyc(&ccyc);
	return (cyc_led < E_OK) ? cyc_led : E_OK;
}

EXPORT ER subsystem1_start(void)
{
	ID	tid;

	tid_led = app_start_task(led_task, S1_PRI_LED, 0, "led_task");
	if (tid_led < E_OK) {
		return tid_led;
	}

	tid = app_start_task(comm_task, S1_PRI_COMM, 0, "comm_task");
	if (tid < E_OK) {
		return tid;
	}

#if S1_SIM_VC
	tid = app_start_task(sim_vc_task, S1_PRI_SIM_VC, 0, "sim_vc_task");
	if (tid < E_OK) {
		return tid;
	}
#endif

	return tk_sta_cyc(cyc_led);
}

/* ======== from bitpack.c ======== */
/*
 *  bitpack.c
 *  Subsystem 1 (Buddy 1) -- bit-level writer/reader (see bitpack.h).
 *
 *  Works one bit at a time: simple, no shift by 32 (undefined in C),
 *  and fast enough for a 16-byte frame once per second.
 */

#include <string.h>

/* true if nbits more bits fit and the stream has not already failed */
static bool bs_room(bitstream_t const *p_bs, uint8_t nbits)
{
    bool b_ok = false;

    if ((false == p_bs->b_overflow) && (nbits > 0u) && (nbits <= 32u) &&
        (((uint32_t)p_bs->pos + nbits) <= p_bs->cap_bits))
    {
        b_ok = true;
    }

    return b_ok;
}

void bs_init_read(bitstream_t *p_bs, uint8_t *p_buf, uint16_t len_bytes)
{
    p_bs->p_buf      = p_buf;
    p_bs->cap_bits   = (uint16_t)(len_bytes * 8u);
    p_bs->pos        = 0u;
    p_bs->b_overflow = false;
}

void bs_init_write(bitstream_t *p_bs, uint8_t *p_buf, uint16_t len_bytes)
{
    (void)memset(p_buf, 0, len_bytes);
    bs_init_read(p_bs, p_buf, len_bytes);
}

void bs_put(bitstream_t *p_bs, uint32_t value, uint8_t nbits)
{
    uint8_t i;

    if (false == bs_room(p_bs, nbits))
    {
        p_bs->b_overflow = true;    /* write nothing, keep pos */
    }
    else
    {
        for (i = 0u; i < nbits; i++)
        {
            if ((value & 1u) != 0u)
            {
                p_bs->p_buf[p_bs->pos / 8u] |= (uint8_t)(1u << (p_bs->pos % 8u));
            }
            value >>= 1;
            p_bs->pos++;
        }
    }
}

uint32_t bs_get(bitstream_t *p_bs, uint8_t nbits)
{
    uint32_t value = 0u;
    uint8_t  i;

    if (false == bs_room(p_bs, nbits))
    {
        p_bs->b_overflow = true;    /* return 0, keep pos */
    }
    else
    {
        for (i = 0u; i < nbits; i++)
        {
            if ((p_bs->p_buf[p_bs->pos / 8u] & (uint8_t)(1u << (p_bs->pos % 8u))) != 0u)
            {
                value |= ((uint32_t)1u << i);
            }
            p_bs->pos++;
        }
    }

    return value;
}

int32_t bs_get_signed(bitstream_t *p_bs, uint8_t nbits)
{
    uint32_t raw = bs_get(p_bs, nbits);

    /* sign-extend: if the top bit of the field is 1, set all higher bits */
    if ((nbits > 0u) && (nbits < 32u) &&
        ((raw & ((uint32_t)1u << (nbits - 1u))) != 0u))
    {
        raw |= ~(((uint32_t)1u << nbits) - 1u);
    }

    return (int32_t)raw;
}

uint16_t bs_bytes_used(bitstream_t const *p_bs)
{
    return (uint16_t)((p_bs->pos + 7u) / 8u);
}

/* ======== from telem_codec.c ======== */
/*
 *  telem_codec.c
 *  Subsystem 1 (Buddy 1) -- MQTT payload codec (see telem_codec.h).
 *
 *  Wire layout: LSB-first via bitpack.c; field order = order of the
 *  bs_put() calls below. Change both encoder and decoder together, and
 *  bump TF_VERSION.
 */

#include <string.h>

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
        bs_put(&bs, p_snap->lift_cause,           TF_LIFT_CAUSE_BITS);
        bs_put(&bs, p_snap->imu_fault,            TF_IMU_FAULT_BITS);
        bs_put(&bs, (uint32_t)p_snap->pitch_x2,   TF_TILT_BITS);
        bs_put(&bs, (uint32_t)p_snap->roll_x2,    TF_TILT_BITS);

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
            p_snap->lift_cause = bs_get(&bs, TF_LIFT_CAUSE_BITS);
            p_snap->imu_fault  = bs_get(&bs, TF_IMU_FAULT_BITS);
            p_snap->pitch_x2   = bs_get_signed(&bs, TF_TILT_BITS);
            p_snap->roll_x2    = bs_get_signed(&bs, TF_TILT_BITS);

            b_ok = (false == bs.b_overflow);
        }
    }

    return b_ok;
}

/* ======== from comm_bridge.c ======== */
/*
 *	comm_bridge.c
 *	Subsystem 1 (Buddy 1) -- hook functions called by the MQTT client in
 *	the kernel's WiFi service task (see comm_bridge.h).
 *
 *	Shared data and who writes it (single-writer rule, so no lock):
 *	  mbf_s1_tx            kernel message buffer: the kernel serialises it
 *	  online, sessions     written only by the WiFi task (hooks below);
 *	                       comm_task only reads them (aligned 32-bit
 *	                       words: one load, cannot be torn)
 *	  last_code/seq/time   read and written only by the WiFi task
 *
 *	FR22: a valid command reaches remote_cmd_q in the same poll that
 *	received it (no extra task hop), then an ACK is published.
 */

#include "subsystem1.h"
#include "comm_bridge.h"

#define DUP_WINDOW_MS	10000u	/* same code+seq within 10 s = a resend */

LOCAL volatile UW	online;		/* 1 while the MQTT session is up  */
LOCAL volatile UW	sessions;	/* sessions accepted since boot    */

LOCAL UB	last_code;		/* last command FORWARDED          */
LOCAL UB	last_seq;
LOCAL UW	last_ms;
LOCAL BOOL	have_last;

/* ---- outbound ------------------------------------------------------- */

EXPORT uint16_t s1_bridge_take_tx(char const **pp_topic, uint8_t *p_buf, uint16_t cap)
{
	UB	msg[S1_TX_MSG_MAX];
	INT	rsz;
	uint16_t i;
	uint16_t n = 0u;

	rsz = tk_rcv_mbf(mbf_s1_tx, msg, TMO_POL);
	if ((rsz > 1) && ((uint16_t)(rsz - 1) <= cap)) {
		n = (uint16_t)(rsz - 1);
		for (i = 0u; i < n; i++) {	/* no <string.h>: clashes with tkernel.h size_t */
			p_buf[i] = msg[i + 1u];
		}
		*pp_topic = (msg[0] == S1_TX_ID_HB) ? TF_TOPIC_HB :
			    ((msg[0] == S1_TX_ID_BC) ? TF_TOPIC_BC :
			     ((msg[0] == S1_TX_ID_IR) ? TF_TOPIC_IR : TF_TOPIC_TM));
	}

	return n;
}

/* ---- inbound -------------------------------------------------------- */

EXPORT uint16_t s1_bridge_on_cmd(uint8_t const *p_data, uint16_t len,
                                 uint8_t *p_ack, uint16_t cap)
{
	remote_cmd_t	cmd;
	ack_result_t	result;
	UW		now = s1_now_ms();

	result = tc_decode_cmd(p_data, len, &cmd);

	if (result == ACK_FORWARDED) {
		if (have_last && (cmd.code == last_code) && (cmd.seq == last_seq) &&
		    ((now - last_ms) < DUP_WINDOW_MS)) {
			/* The PC resent because it missed our ACK: do not act twice */
			result = ACK_DUPLICATE;
		} else if (tk_snd_mbf(mbf_remote_cmd_q, &cmd, sizeof cmd, TMO_POL) != E_OK) {
			result = ACK_QUEUE_FULL;	/* VC not draining: never block */
		} else {
			have_last = TRUE;
			last_code = cmd.code;
			last_seq  = cmd.seq;
			last_ms   = now;
		}
	}

	tm_printf((UB*)"S1 RX cmd code %d seq %d -> ACK %d\n",
		  (INT)cmd.code, (INT)cmd.seq, (INT)result);

	return tc_encode_ack(cmd, result, p_ack, cap);
}

/* ---- session state -------------------------------------------------- */

EXPORT void s1_bridge_set_online(uint8_t b_online)
{
	UB	discard[S1_TX_MSG_MAX];

	if ((b_online != 0u) && (online == 0u)) {
		sessions++;
		/* Stale frames from before the outage have no value: drop them
		   so the PC sees fresh data first (Tribunal: no stale telemetry) */
		while (tk_rcv_mbf(mbf_s1_tx, discard, TMO_POL) > 0) {
			/* discard */
		}
	}
	online = (b_online != 0u) ? 1u : 0u;
}

EXPORT uint8_t s1_bridge_is_online(void)
{
	return (uint8_t)online;
}

EXPORT uint32_t s1_bridge_reconnects(void)
{
	UW	s = sessions;

	return (s > 0u) ? (s - 1u) : 0u;	/* first session is not a REconnect */
}
