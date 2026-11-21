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

EXPORT void s1_publish_barcode(UB code, UB flags, const char *p_text,
				const uint32_t *p_widths_us, UB count,
				const uint16_t *p_levels)
{
#if TM_MQTT
	UB	msg[1u + TF_BC_LEN_MAX];
	UW	i;
	UW	n = count;
	BOOL	end = (p_text == NULL) ? TRUE : FALSE;

	if (n > TF_BC_WIDTHS_MAX) {
		n = TF_BC_WIDTHS_MAX;
	}
	if (p_widths_us == NULL) {
		n = 0u;
	}
	msg[0] = S1_TX_ID_BC;
	msg[1] = code;
	msg[2] = (UB)(flags | TF_BC_LEVELS);
	msg[3] = count;
	for (i = 0u; i < 4u; i++) {			/* 4 x uint16, little-endian */
		UH v = (p_levels != NULL) ? (UH)p_levels[i] : 0u;

		msg[8u + (2u * i)] = (UB)(v & 0xFFu);
		msg[9u + (2u * i)] = (UB)(v >> 8);
	}
	for (i = 0u; i < TF_BC_TEXT_LEN; i++) {
		if ((end == FALSE) && (p_text[i] == '\0')) {
			end = TRUE;
		}
		msg[4u + i] = (end != FALSE) ? 0u : (UB)p_text[i];
	}
	for (i = 0u; i < n; i++) {
		UW w = (p_widths_us[i] + 5000u) / 10000u;	/* 10 ms units */

		msg[1u + TF_BC_HDR_LEN + i] = (w > 255u) ? 255u : (UB)w;
	}
	if ((mbf_s1_tx <= 0) ||
	    (tk_snd_mbf(mbf_s1_tx, msg, (INT)(1u + TF_BC_HDR_LEN + n), TMO_POL) != E_OK)) {
		tm_printf((UB*)"S1: barcode message DROPPED (tx queue full)\n");
	}
#else
	(void)code; (void)flags; (void)p_text; (void)p_widths_us; (void)count;
	(void)p_levels;
#endif
}

EXPORT void s1_publish_ir_trace(UB state, UB seq, UB cal_white16,
				 UB cal_black16, const UB *p_samples)
{
#if TM_MQTT
	UB	msg[1u + TF_IR_LEN];
	UW	i;

	msg[0] = S1_TX_ID_IR;
	msg[1] = state;
	msg[2] = seq;
	msg[3] = cal_white16;
	msg[4] = cal_black16;
	for (i = 0u; i < TF_IR_TRACE_N; i++) {
		msg[1u + TF_IR_HDR_LEN + i] = p_samples[i];
	}
	if (mbf_s1_tx > 0) {
		(void)tk_snd_mbf(mbf_s1_tx, msg, (INT)(1u + TF_IR_LEN), TMO_POL);	/* drop if full */
	}
#else
	(void)state; (void)seq; (void)cal_white16; (void)cal_black16; (void)p_samples;
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

/* ======== from task_led.c ======== */
/*
 *	task_led.c
 *	Subsystem 1 -- the LED task and its cyclic handler.
 *
 *	APIs shown: tk_slp_tsk, tk_wup_tsk, tk_can_wup, cyclic handler,
 *	            gpio_set_pin / gpio_set_val
 *
 *	Instead of tk_dly_tsk(), this task sleeps until a cyclic handler
 *	wakes it.  The handler is called by the kernel's timer at a fixed
 *	rate, so the blink rate stays exact however long the task's own work
 *	takes.
 */

#include "subsystem1.h"

/*
 * Cyclic handler.  Runs in interrupt context, NOT as a task, so it must
 * be short and must never wait (no tk_slp_tsk, tk_wai_sem, tm_printf...).
 * Signalling a task is the normal thing to do here.
 */
EXPORT void led_cyclic_handler(void *exinf)
{
	(void)exinf;
	tk_wup_tsk(tid_led);	/* wake the LED task (the request queues
				   up if the task is not asleep yet) */
}

EXPORT void led_task(INT stacd, void *exinf)
{
	UINT	level = 0;

	(void)stacd; (void)exinf;

	gpio_set_pin(S1_LED_PIN, GPIO_MODE_OUT);	/* make the pin an output */

	while (1) {
		/* Sleep until led_cyclic_handler calls tk_wup_tsk() */
		tk_slp_tsk(TMO_FEVR);

		/* tk_can_wup: discard wake-ups that queued up while we were
		   not sleeping (e.g. while subsystem 4 had us suspended), so
		   the LED does not "catch up" with a burst of fast blinks. */
		tk_can_wup(TSK_SELF);

		level = !level;
		gpio_set_val(S1_LED_PIN, level);
	}
}

/* ======== from sim_vc.c ======== */
/*
 *	sim_vc.c
 *	Subsystem 1 -- SIMULATED Vehicle Controller (only when S1_SIM_VC = 1).
 *
 *	Stands in for the real Vehicle Controller so Buddy 1 can test the
 *	telemetry and command path before integration:
 *	  - posts one fake snapshot per second to telem_q (never blocks),
 *	  - reads remote_cmd_q and applies START/STOP/PAUSE/RESUME/RESET with
 *	    the same state rules as the Week 6 system statechart.
 *	Delete it (or build with S1_SIM_VC = 0) once the real VC exists.
 */

#include "subsystem1.h"

#if S1_SIM_VC

LOCAL sys_state_t state = SYS_IDLE_READY;
LOCAL UW	enc_l;		/* fake encoder counts, keep growing */
LOCAL UW	enc_r;
LOCAL UW	dist_cm;
LOCAL UW	posts_dropped;

LOCAL const char * const cmd_name[] = {
	"NONE", "START", "STOP", "PAUSE", "RESUME", "RESET"
};

/* Same rules the real VC must follow: a command only acts in the
   states where it makes sense, otherwise it is ignored. */
LOCAL void apply_cmd(remote_cmd_t cmd)
{
	sys_state_t old = state;

	switch (cmd.code) {
	case CMD_START:
		if (state == SYS_IDLE_READY) { state = SYS_PATROLLING; }
		break;
	case CMD_STOP:
		if ((state == SYS_PATROLLING) || (state == SYS_PAUSED)) { state = SYS_IDLE_READY; }
		break;
	case CMD_PAUSE:
		if (state == SYS_PATROLLING) { state = SYS_PAUSED; }
		break;
	case CMD_RESUME:
		if (state == SYS_PAUSED) { state = SYS_PATROLLING; }
		break;
	case CMD_RESET:
		state = SYS_IDLE_READY;
		enc_l   = 0u;
		enc_r   = 0u;
		dist_cm = 0u;
		break;
	default:
		break;
	}

	tm_printf((UB*)"SIM_VC: %s (seq %d) -> state %d -> %d%s\n",
		  cmd_name[(cmd.code < CMD_CODE_COUNT) ? cmd.code : 0],
		  (INT)cmd.seq, (INT)old, (INT)state,
		  (old == state) ? " (ignored in this state)" : "");
}

LOCAL void post_snapshot(void)
{
	telem_snapshot_t snap = {0};
	BOOL	moving = (state == SYS_PATROLLING);

	if (moving) {
		enc_l   += 300u;
		enc_r   += 296u;
		dist_cm += 20u;
	}

	snap.sys_state  = state;
	snap.drive_mode = DRIVE_FOLLOW_LINE;
	tc_snap_set_speed(&snap, moving ? 20 : 0, moving ? 19 : 0);
	tc_snap_set_enc(&snap, enc_l, enc_r);
	tc_snap_set_dist(&snap, dist_cm);
	snap.ir_left    = 0u;
	snap.ir_center  = moving ? 1u : 0u;
	snap.line_state = moving ? LINE_FOLLOWING : LINE_INIT;
	snap.barcode    = BARCODE_NONE;
	tc_snap_set_hump(&snap, 0u, 0u, 0u);
	snap.motion     = moving ? MOTION_ACCELERATING : MOTION_STATIONARY;
	snap.ir_barcode = 0u;
	tc_snap_set_obst(&snap, FALSE, 0u, 0u, 0u, 0u, 0u);	/* nothing ahead */
	snap.obst_plan  = OBST_PLAN_NONE;
	snap.obst_state = OBST_PATROLLING;
	/* The shared stop and fault bits are real even in this simulation:
	   read them from the team flags (app.h) so the ground station shows
	   an obstacle or IMU stop and any subsystem fault as it happens. */
	{
		T_RFLG rflg;

		snap.fault = ((flg_sysfault > 0) && (tk_ref_flg(flg_sysfault, &rflg) == E_OK))
			     ? (rflg.flgptn & 0xFFu) : 0u;
		snap.estop = ((flg_estop_req > 0) && (tk_ref_flg(flg_estop_req, &rflg) == E_OK))
			     ? (rflg.flgptn & ESTOP_ALL) : 0u;
	}

	/* The real VC must post exactly like this: zero timeout, drop if full */
	if (tk_snd_mbf(mbf_telem_q, &snap, sizeof snap, TMO_POL) != E_OK) {
		posts_dropped++;
		tm_printf((UB*)"SIM_VC: telem_q full, snapshot dropped (%d)\n",
			  (INT)posts_dropped);
	}
}

EXPORT void sim_vc_task(INT stacd, void *exinf)
{
	remote_cmd_t cmd;
	UW	next_post;
	INT	left;

	(void)stacd; (void)exinf;

	tm_printf((UB*)"SIM_VC started (simulated Vehicle Controller)\n");
	next_post = s1_now_ms() + S1_TELEM_PERIOD_MS;

	while (1) {
		/* Sleep until a command arrives or the next post is due */
		left = (INT)(next_post - s1_now_ms());
		if (tk_rcv_mbf(mbf_remote_cmd_q, &cmd, (left > 0) ? (TMO)left : TMO_POL)
		    == (INT)sizeof cmd) {
			apply_cmd(cmd);
		}

		if ((INT)(s1_now_ms() - next_post) >= 0) {
			next_post += S1_TELEM_PERIOD_MS;
			post_snapshot();
		}
	}
}

#endif /* S1_SIM_VC */
