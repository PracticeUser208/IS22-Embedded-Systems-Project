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
#include "telem_codec.h"
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
		*pp_topic = (msg[0] == S1_TX_ID_HB) ? TF_TOPIC_HB : TF_TOPIC_TM;
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