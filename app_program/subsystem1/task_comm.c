/*
 *	task_comm.c
 *	Subsystem 1 -- Telemetry / Communications task (Week 6 statechart 6.1).
 *
 *	Drains telem_q, encodes each snapshot to the 16-byte binary frame,
 *	builds the heartbeat every 5 s, and queues both in mbf_s1_tx.  The
 *	MQTT client in the kernel's WiFi service task publishes them
 *	(comm_bridge.c / mqtt_utk.c).  Without MQTT=1 the frames are only
 *	printed on the console.
 *
 *	FR21 telemetry >= 1 Hz   FR23 heartbeat every 5 s
 *	NFR6 blocks on tk_rcv_mbf with a timeout: no busy-wait
 *	Only this task writes the variables below, so no lock is needed.
 */

#include "subsystem1.h"
#include "telem_codec.h"
#include "comm_bridge.h"

#define FRAME_MAX	((TF_TM_LEN > TF_HB_LEN) ? TF_TM_LEN : TF_HB_LEN)

LOCAL UB	telem_seq;	/* +1 per telemetry frame, wraps at 255 */
LOCAL UB	hb_seq;		/* +1 per heartbeat, wraps at 255       */
LOCAL telem_hb_t hb;		/* heartbeat contents                   */

/* Print a frame on the console (debug evidence for the test log) */
LOCAL void print_frame(const char *topic, const UB *p_buf, UH len, const char *note)
{
	UH	i;

	tm_printf((UB*)"S1 TX [%s] %d B:", topic, (INT)len);
	for (i = 0; i < len; i++) {
		tm_printf((UB*)" %02X", (UINT)p_buf[i]);
	}
	tm_printf((UB*)"%s\n", note);
}

/* Queue one frame for the MQTT client.  Never blocks (FR24): if the
   client is offline or behind and the queue is full, drop and count. */
LOCAL void publish(UB topic_id, const char *topic, const UB *p_buf, UH len)
{
#if TM_MQTT
	UB	msg[S1_TX_MSG_MAX];
	UH	i;

	msg[0] = topic_id;
	for (i = 0; i < len; i++) {	/* no <string.h>: clashes with tkernel.h size_t */
		msg[i + 1u] = p_buf[i];
	}
	if (tk_snd_mbf(mbf_s1_tx, msg, (INT)(len + 1u), TMO_POL) == E_OK) {
		print_frame(topic, p_buf, len, s1_bridge_is_online() ? "" : " (queued, offline)");
	} else {
		hb.drops = tc_sat_u((UW)hb.drops + 1u, 0xFFu);
		print_frame(topic, p_buf, len, " (DROPPED: tx queue full)");
	}
#else
	(void)topic_id;
	print_frame(topic, p_buf, len, " (no MQTT in this build)");
#endif
}

LOCAL void send_heartbeat(void)
{
	UB	frame[FRAME_MAX];
	UH	n;

	hb.uptime_s   = (uint16_t)tc_sat_u(s1_now_ms() / 1000u, 0xFFFFu);
	hb.reconnects = tc_sat_u(s1_bridge_reconnects(), (1u << TF_RECONNECT_BITS) - 1u);
	n = tc_encode_hb(&hb, hb_seq, frame, sizeof frame);
	hb_seq++;
	if (n > 0u) {
		publish(S1_TX_ID_HB, TF_TOPIC_HB, frame, n);
	}
}

LOCAL void send_telemetry(const telem_snapshot_t *p_snap)
{
	UB	frame[FRAME_MAX];
	UH	n;

	/* the heartbeat repeats the latest state and faults */
	hb.sys_state = p_snap->sys_state;
	hb.fault     = p_snap->fault;

	n = tc_encode_tm(p_snap, telem_seq, frame, sizeof frame);
	telem_seq++;
	if (n > 0u) {
		publish(S1_TX_ID_TM, TF_TOPIC_TM, frame, n);
	} else {
		hb.drops = tc_sat_u((UW)hb.drops + 1u, 0xFFu);
	}
}

EXPORT void comm_task(INT stacd, void *exinf)
{
	telem_snapshot_t snap;
	UW	next_hb;
	UW	now;
	INT	left;
	INT	rsz;

	(void)stacd; (void)exinf;

	tm_printf((UB*)"S1 comm_task started: telemetry %d B, heartbeat %d B\n",
		  (INT)TF_TM_LEN, (INT)TF_HB_LEN);

	next_hb = s1_now_ms() + S1_HB_PERIOD_MS;

	while (1) {
		/* Sleep until a snapshot arrives or the heartbeat is due */
		now  = s1_now_ms();
		left = (INT)(next_hb - now);
		rsz  = tk_rcv_mbf(mbf_telem_q, &snap, (left > 0) ? (TMO)left : TMO_POL);

		if (rsz == (INT)sizeof snap) {
			send_telemetry(&snap);
		}

		if ((INT)(s1_now_ms() - next_hb) >= 0) {
			next_hb += S1_HB_PERIOD_MS;
			send_heartbeat();
		}
	}
}