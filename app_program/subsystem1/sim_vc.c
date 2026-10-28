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
#include "telem_codec.h"

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
	snap.fault      = 0u;
	snap.estop      = 0u;

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
