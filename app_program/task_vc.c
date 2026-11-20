/*
 *	task_vc.c
 *	Vehicle Controller -- the system-level statechart (Week 6 report,
 *	section 5).  See vehicle_ctrl.h for the states and interfaces.
 *
 *	How it runs
 *	-----------
 *	One task, woken every VC_TICK_MS (20 ms).  Each tick it
 *	  1. drains its input queues (remote commands, Obstacle Scan, IMU,
 *	     Line and Barcode) and reacts to each message,
 *	  2. runs the current state: refreshes the patrol speed (steered by
 *	     Line and Barcode), sweeps to find a lost line, drives the
 *	     next move of a bypass or a barcode turn, checks time-outs and
 *	     the stop flag,
 *	  3. once a second, posts a telemetry snapshot to telem_q.
 *	It never blocks on another subsystem: every post uses TMO_POL and
 *	every manoeuvre leg is checked, not waited for.  The fast safety
 *	stop does not go through the VC at all: Obstacle Scan and the IMU
 *	set ESTOP_REQ and Motion Control brakes on its next 20 ms tick.
 *	The VC then decides what happens next and is the only task that
 *	clears ESTOP bits.
 *
 *	Going round an obstacle (12 Nov)
 *	--------------------------------
 *	Before: one scan, then a fixed route driven blind (side-step, pass
 *	an obstacle assumed 250 mm deep, cut back in by distance).  Now
 *	vc_bypass.c decides after every move what to do next, Obstacle Scan
 *	watches during every forward move and looks towards the line before
 *	the car goes back, and the line sensors end the way back.  This
 *	file carries the moves out ("Going round an obstacle" below).
 */

#include "vehicle_ctrl.h"
#include "subsystem1/subsystem1.h"
#include "subsystem2/subsystem2.h"
#include "subsystem2/subsystem2.h"	/* TRACK_WIDTH_MM */
#include "subsystem3/subsystem3.h"
#include "subsystem3/subsystem3.h"		/* LF_CH_PAIR_xxx */
#include "subsystem5/subsystem5.h"

#if VEHICLE_CONTROLLER

/* Phases inside a state */
#define PH_NONE		0U
#define PH_SETTLE	1U	/* stopped; let the car settle            */
#define PH_SCAN		2U	/* Obstacle Scan is scanning               */
#define PH_LEGS		3U	/* driving a barcode turn, leg by leg      */
#define PH_MOVE		4U	/* bypass: driving one move                */
#define PH_REST		5U	/* bypass: the move ended, the car settles */
#define PH_SEARCH	6U	/* Line and Barcode searches for the line  */
#define PH_LOOK		7U	/* bypass: Obstacle Scan looks ahead       */
#define PH_JCT_FWD	8U	/* junction: axle onto its middle          */
#define PH_JCT_BLIND	9U	/* junction: turning off the old line      */
#define PH_JCT_SEEK	10U	/* junction: turning until the new line    */
#define PH_JCT_STOP	11U	/* junction: on it, stopping               */

/* Why the car is in FAULT_ESTOP (it leaves when all are cleared) */
#define CAUSE_S5	0x01U	/* ultrasonic sensor fault         */
#define CAUSE_IMU	0x02U	/* lifted / overturned, or IMU fault */
#define CAUSE_NO_PATH	0x04U	/* blocked, retry limit reached    */
#define CAUSE_MOVE	0x08U	/* a manoeuvre failed or timed out */
#define CAUSE_LINE	0x10U	/* line not found / IR fault       */

#define VC_INIT_WAIT_MS		1000	/* let the subsystems start */
#define VC_LOOK_TIMEOUT_MS	6000	/* Obstacle Scan's look (3 angles) */
#define VC_LINE_TICKS		2U	/* ticks in a row on the line = found */
#define VC_MOVE_RETRIES		2U	/* a turn cut short: carry on, at most */
#define VC_SEARCH_TIMEOUT_MS	(VC_LINE_TIMEOUT_MS + 3000)

/* ---- State (this task only) ---- */
LOCAL UB	vc_st = (UB)SYS_INITIALISING;
LOCAL UB	vc_ph = PH_NONE;
LOCAL UW	vc_ph_ms;
LOCAL UB	vc_causes;
LOCAL VC_LEGS	vc_legs;
#if VC_BARCODE_ACT == 2
LOCAL UB	vc_jct_cmd = (UB)BARCODE_NONE;	/* for the next junction  */
LOCAL W		vc_jct_odo;		/* odometer when it was read       */
LOCAL UB	vc_jct_line_n;		/* ticks on the new line           */
#endif
LOCAL UB	vc_leg_i;
LOCAL BOOL	vc_leg_busy;
LOCAL BOOL	vc_slow;
/* Going round an obstacle (vc_bypass.c decides, this task drives) */
LOCAL vc_byp_t	vc_byp;
LOCAL vc_byp_act_t vc_act;		/* the move being carried out        */
LOCAL W		vc_byp_l0;		/* wheel travel (mm) when it began   */
LOCAL W		vc_byp_r0;
LOCAL W		vc_want_deg;		/* where its turns add up to, + left */
LOCAL W		vc_move_l0;		/* wheel travel when this move began */
LOCAL W		vc_move_r0;
LOCAL W		vc_move_want0;		/* vc_want_deg before this move      */
LOCAL UB	vc_move_end;		/* VC_BYP_EV_ARRIVED / _LINE         */
LOCAL BOOL	vc_stop_seen;		/* Obstacle Scan stopped the car     */
LOCAL UB	vc_stop_ray;		/* ... VC_BYP_RAY_xxx: where it saw  */
LOCAL UH	vc_stop_cm;		/* ... and how far away, cm          */
LOCAL UB	vc_line_n;		/* ticks in a row on the line        */
LOCAL UB	vc_retry;
LOCAL UB	vc_backed_n;		/* times in a row it only backed away */
LOCAL motion_status_t vc_ms;		/* scratch for the bypass functions (not
					   on the stack: this task's is 2 KB and
					   tm_printf takes about half of it)   */
LOCAL B		vc_search_dir = -1;	/* +1 sweep left first, -1 right */
LOCAL BOOL	vc_imu_faulty;		/* IMU SENSOR_FAULT not cleared yet */
#if VC_AUTOSTART_S > 0
LOCAL BOOL	vc_autostarted;
LOCAL UW	vc_boot_ms;
#endif

/* ---- What telemetry reports (filled from events) ---- */
LOCAL UB	vc_obst_plan = (UB)OBST_PLAN_NONE;
LOCAL UB	vc_obst_width;		/* cm */
LOCAL UH	vc_clear_l;		/* cm */
LOCAL UH	vc_clear_r;
LOCAL UB	vc_hump_last;		/* mm */
LOCAL UB	vc_hump_peak;
LOCAL UB	vc_hump_count;
LOCAL UB	vc_motion_class = (UB)MOTION_STATIONARY;
LOCAL UB	vc_barcode = (UB)BARCODE_NONE;
LOCAL UB	vc_line_state = (UB)LINE_INIT;
LOCAL struct {				/* 19 Nov: for telemetry v3 (VC task only) */
	UH	lift_cause : TF_LIFT_CAUSE_BITS;	/* last lift since RESET */
	UH	imu_fault  : TF_IMU_FAULT_BITS;		/* last IMU fault bits, 0 = none */
} vc_imu_rep;
EXPORT UW	vc_telem_dropped;

LOCAL const char *const vc_names[SYS_STATE_COUNT] = {
	"INITIALISING", "IDLE_READY", "PATROLLING", "PAUSED",
	"HANDLING_BARCODE", "HANDLE_HUMP", "OBSTACLE_AVOIDANCE",
	"LINE_RECOVERY", "FAULT_ESTOP", "STARTUP_ERROR"
};
LOCAL const char *const vc_cmd_names[CMD_CODE_COUNT] = {
	"NONE", "START", "STOP", "PAUSE", "RESUME", "RESET"
};

EXPORT const char *vc_state_name(UB st)
{
	return (st < (UB)SYS_STATE_COUNT) ? vc_names[st] : "?";
}

EXPORT UB vc_state(void)
{
	return vc_st;
}

/* =====================================================================
 *  Small helpers
 * ===================================================================== */
LOCAL UW vc_now_ms(void)
{
	SYSTIM t;

	(void)tk_get_tim(&t);
	return t.lo;
}

LOCAL void vc_set_state(UB st, const char *why)
{
	if (st != vc_st) {
		tm_printf((UB*)"VC: %s -> %s (%s)\n", vc_state_name(vc_st),
			  vc_state_name(st), why);
	}
	vc_st = st;
	vc_ph = PH_NONE;
	vc_ph_ms = vc_now_ms();
}

LOCAL BOOL vc_line_present(void)
{
	return (line_cmd_q > 0) ? TRUE : FALSE;
}

/* Latest steering from Line and Barcode, if it is fresh */
LOCAL BOOL vc_s3(S3_STEER *p)
{
	if ((vc_line_present() == FALSE) || (s3_steer_get(p) != E_OK)) {
		return FALSE;
	}
	return ((vc_now_ms() - p->stamp_ms) <= (UW)VC_STEER_STALE_MS) ? TRUE : FALSE;
}

/* Line search: S3 says which side it last saw the line on */
LOCAL void vc_begin_search(void)
{
	S3_STEER s;

	vc_search_dir = ((vc_s3(&s) != FALSE) && (s.search_dir > 0)) ? (B)1 : (B)-1;
	vc_ph = PH_SEARCH;
	vc_ph_ms = vc_now_ms();
}

LOCAL void vc_post(ID q, UW w)
{
	if (q > 0) {
		(void)tk_snd_mbf(q, &w, (INT)sizeof(w), TMO_POL);	/* never block */
	}
}

LOCAL void vc_to_s5(UB cmd)
{
	vc_post(scan_q, s5_cmd_pack(cmd, 0U));
}

LOCAL void vc_to_s5_arg(UB cmd, UB arg)
{
	vc_post(scan_q, s5_cmd_pack(cmd, arg));
}

LOCAL void vc_to_line(UB cmd)
{
	vc_post(line_cmd_q, ((UW)cmd & 0x7U) << 29);
}

LOCAL void vc_to_imu(UB cmd)
{
	vc_post(imu_q, (UW)cmd);		/* Buddy 4's imu_cmd_t */
}

LOCAL UINT vc_flag(ID flg)
{
	T_RFLG r;

	return ((flg > 0) && (tk_ref_flg(flg, &r) == E_OK)) ? r.flgptn : 0U;
}

LOCAL void vc_estop_clear(UINT bits)
{
	if (flg_estop_req > 0) {
		(void)tk_clr_flg(flg_estop_req, ~bits);	/* only the VC clears */
	}
}

LOCAL void vc_fault_bit(UINT bit, BOOL on)
{
	if (flg_sysfault > 0) {
		if (on != FALSE) {
			(void)tk_set_flg(flg_sysfault, bit);
		} else {
			(void)tk_clr_flg(flg_sysfault, ~bit);
		}
	}
}

LOCAL BOOL vc_moving_state(void)
{
	return ((vc_st == (UB)SYS_PATROLLING) || (vc_st == (UB)SYS_HANDLE_HUMP) ||
		(vc_st == (UB)SYS_LINE_RECOVERY) ||
		(vc_st == (UB)SYS_HANDLING_BARCODE)) ? TRUE : FALSE;
}

/* =====================================================================
 *  Barcode turns: one leg at a time through Motion Control
 * ===================================================================== */
#if VC_BARCODE_ACT == 1
LOCAL void vc_legs_begin(UB phase)
{
	vc_leg_i    = 0U;
	vc_leg_busy = FALSE;
	vc_ph       = phase;
	vc_ph_ms    = vc_now_ms();
}
#endif

LOCAL ER vc_issue(const VC_LEG *p)
{
	switch (p->kind) {
	case VC_LEG_FWD:
		return motion_move_forward((int32_t)p->amount, (int16_t)VC_BYPASS_MM_S);
	case VC_LEG_BACK:
		return motion_move_backward((int32_t)p->amount, (int16_t)VC_BYPASS_MM_S);
	case VC_LEG_LEFT:
		return motion_turn_left((int32_t)p->amount);
	case VC_LEG_RIGHT:
		return motion_turn_right((int32_t)p->amount);
	default:
		return E_PAR;
	}
}

/* 0 = still driving, 1 = all legs done, -1 = a leg failed */
LOCAL INT vc_legs_step(void)
{
	if (vc_leg_busy != FALSE) {
		UINT f = vc_flag(flg_motion);

		if ((f & MOTION_FLG_DONE) == 0U) {
			return 0;
		}
		vc_leg_busy = FALSE;
		if ((f & MOTION_FLG_FAULT) != 0U) {
			return -1;
		}
		vc_leg_i++;
	}
	if (vc_leg_i >= vc_legs.n) {
		return 1;
	}
	if (vc_issue(&vc_legs.leg[vc_leg_i]) != E_OK) {
		return -1;
	}
	vc_leg_busy = TRUE;
	return 0;
}

#if VC_BARCODE_ACT == 1
LOCAL void vc_print_legs(const char *what)
{
	static const char *const kind[5] = { "?", "fwd", "back", "left", "right" };
	UB i;

	tm_printf((UB*)"VC: %s:", what);
	for (i = 0U; i < vc_legs.n; i++) {
		tm_printf((UB*)" %s %d%s", kind[(vc_legs.leg[i].kind < 5U) ? vc_legs.leg[i].kind : 0U],
			  (INT)vc_legs.leg[i].amount,
			  ((vc_legs.leg[i].kind == VC_LEG_LEFT) ||
			   (vc_legs.leg[i].kind == VC_LEG_RIGHT)) ? "deg" : "mm");
	}
	tm_printf((UB*)"\n");
}
#endif

/* =====================================================================
 *  State changes
 * ===================================================================== */
LOCAL void vc_go_idle(const char *why)
{
	(void)motion_stop();
	vc_to_line(VC_LINE_CMD_SUSPEND);
	vc_backed_n = 0U;
	vc_slow = FALSE;
	vc_legs.n = 0U;
#if VC_BARCODE_ACT == 2
	vc_jct_cmd = (UB)BARCODE_NONE;
#endif
	vc_set_state((UB)SYS_IDLE_READY, why);
}

LOCAL void vc_go_fault(UB cause, const char *why)
{
	(void)motion_stop();
	vc_to_line(VC_LINE_CMD_SUSPEND);
	vc_causes |= cause;
	if ((cause & CAUSE_NO_PATH) != 0U) {
		vc_fault_bit(FAULT_NO_PATH, TRUE);
	}
	vc_legs.n = 0U;
	vc_set_state((UB)SYS_FAULT_ESTOP, why);
}

/* rearm = coming from IDLE / PAUSED: Obstacle Scan may be sitting in
   DETECTED from something seen while the car stood still, so put it
   back to patrolling and clear the obstacle stop bit. */
LOCAL void vc_go_patrol(BOOL rearm, const char *why)
{
	if (rearm != FALSE) {
		if ((vc_causes & CAUSE_S5) != 0U) {
			vc_go_fault(0U, "obstacle sensor still faulty");
			return;
		}
		if ((vc_flag(flg_estop_req) & (ESTOP_IMU_LIFTED | ESTOP_IMU_FAULT)) != 0U) {
			vc_go_fault(CAUSE_IMU, "IMU stop still set: send RESET");
			return;
		}
		vc_to_s5(S5_CMD_STOP);
		vc_estop_clear(ESTOP_OBSTACLE);
	}
	vc_slow = FALSE;
	vc_obst_plan = (UB)OBST_PLAN_NONE;
	vc_to_line(VC_LINE_CMD_FOLLOW);
	vc_set_state((UB)SYS_PATROLLING, why);
}

/* Stopped for an obstacle: let the car settle, then scan (PH_SETTLE in
   vc_run_state starts the bypass) */
LOCAL void vc_enter_avoid(UH dist_cm)
{
	(void)motion_stop();			/* ESTOP already braked it */
	vc_to_line(VC_LINE_CMD_SUSPEND);
	vc_obst_plan = (UB)OBST_PLAN_STOP;
	vc_slow = FALSE;
	tm_printf((UB*)"VC: obstacle at %d cm, stopped\n", (INT)dist_cm);
	vc_set_state((UB)SYS_OBSTACLE_AVOIDANCE, "obstacle within 15 cm");
	vc_ph = PH_SETTLE;
}

/* The obstacle is passed (or was never in the way): Line and Barcode
   finds the line again.  dir: sweep this way first (+1 left, -1 right),
   0 = where the line was last seen. */
LOCAL void vc_finish_avoid(const char *why, B dir)
{
	vc_obst_plan = (UB)OBST_PLAN_CONTINUE;
	if (vc_line_present() != FALSE) {
		vc_set_state((UB)SYS_LINE_RECOVERY, why);
		vc_to_line(VC_LINE_CMD_SEARCH);
		vc_begin_search();
		if (dir != 0) {
			vc_search_dir = dir;
		}
	} else {
		vc_go_patrol(FALSE, why);
	}
}

/* =====================================================================
 *  Going round an obstacle (12 Nov).  vc_bypass.c decides after every
 *  move what comes next; the functions here carry it out:
 *    a move       Motion Control drives it; Obstacle Scan watches during
 *                 a forward move (S5_CMD_BYPASS WATCH) and stops the car
 *                 (ESTOP_OBSTACLE) if something is in its way; on the
 *                 way back the line sensors stop it
 *    a look       Obstacle Scan measures how far it is free ahead
 *    a scan       the full scan and plan, as at the start
 *  After each one the result goes to vc_byp_event(), which answers
 *  with the next.
 * ===================================================================== */
LOCAL void vc_byp_do(UW now);

/* How far the car has turned since the bypass began, by the wheel
   encoders: (right - left wheel travel) / track width.  0.1 deg, + left */
LOCAL W vc_turned_ddeg(const motion_status_t *p_ms)
{
	W d = (p_ms->right_mm - vc_byp_r0) - (p_ms->left_mm - vc_byp_l0);

	return (d * 5730) / ((W)TRACK_WIDTH_MM * 10);	/* 573.0 x 0.1 deg per rad */
}

/* The two middle line sensors: is one of them on the line? */
LOCAL BOOL vc_on_line(void)
{
	S3_STEER s;

	if ((vc_s3(&s) == FALSE) || ((s.flags & S3_STEER_CALIBRATED) == 0U)) {
		return FALSE;
	}
	return ((s.ir_bits & (UB)((1U << LF_CH_PAIR_LEFT) | (1U << LF_CH_PAIR_RIGHT))) != 0U)
		? TRUE : FALSE;
}

/* A new obstacle: the car stands in front of it */
LOCAL void vc_byp_begin(void)
{
	static const vc_byp_cfg_t cfg = VC_BYP_CFG_INIT;	/* vehicle_ctrl.h */

	(void)motion_get_status(&vc_ms);
	vc_byp_l0 = vc_ms.left_mm;
	vc_byp_r0 = vc_ms.right_mm;
	vc_want_deg = 0;
	vc_stop_seen = FALSE;
	vc_byp_start(&vc_byp, &cfg);
	vc_act.kind = VC_BYP_ACT_SCAN;
	vc_act.flags = 0U;
	vc_act.amount = 0U;
}

/* Feed what happened to vc_bypass.c and start what it answers with */
LOCAL void vc_byp_step(UB ev, UW a, UW b, UW c, UW d)
{
	vc_byp_event(&vc_byp, ev, a, b, c, d, &vc_act);
	vc_byp_do(vc_now_ms());
}

/* Start the move in vc_act.  again: it was cut short by a stop that
   was not meant for it (below): drive what is left of it. */
LOCAL void vc_move_begin(UW now, BOOL again)
{
	W	amount = (W)vc_act.amount;
	UB	watch = S5_BYP_QUIET;
	BOOL	none = FALSE;
	ER	er = E_OK;

	(void)motion_get_status(&vc_ms);
	if (again == FALSE) {
		vc_move_l0 = vc_ms.left_mm;
		vc_move_r0 = vc_ms.right_mm;
		vc_move_want0 = vc_want_deg;
		vc_retry = 0U;
		if (vc_act.kind == VC_BYP_ACT_LEFT) {
			vc_want_deg += amount;
		} else if (vc_act.kind == VC_BYP_ACT_RIGHT) {
			vc_want_deg -= amount;
		} else {
			/* straight: the direction stays */
		}
	}
	vc_stop_seen = FALSE;
	vc_line_n = 0U;
	vc_move_end = VC_BYP_EV_ARRIVED;

	/* Obstacle Scan: watch a forward move; WIDE = keep extra room at
	   the corner on the obstacle's side (bypass on the left: it is on
	   the right).  A guarded turn (15 Nov, the 45 deg towards the
	   line): watch the side it turns to. */
	if ((vc_act.kind == VC_BYP_ACT_FWD) && ((vc_act.flags & VC_BYP_F_GUARD) != 0U)) {
		watch = ((vc_act.flags & VC_BYP_F_WIDE) == 0U) ? S5_BYP_WATCH :
			((vc_byp.side > 0) ? S5_BYP_WATCH_RIGHT : S5_BYP_WATCH_LEFT);
	} else if (((vc_act.kind == VC_BYP_ACT_LEFT) || (vc_act.kind == VC_BYP_ACT_RIGHT)) &&
		   ((vc_act.flags & VC_BYP_F_GUARD) != 0U)) {
		watch = (vc_act.kind == VC_BYP_ACT_LEFT) ? S5_BYP_TURN_LEFT : S5_BYP_TURN_RIGHT;
	} else {
		/* nothing to watch */
	}
	vc_estop_clear(ESTOP_OBSTACLE);	/* the car stands: Obstacle Scan
					   sets it again if it has to */
	vc_to_s5_arg(S5_CMD_BYPASS, watch);

	switch (vc_act.kind) {
	case VC_BYP_ACT_FWD:
		if (amount <= 0) {
			none = TRUE;
		} else {
			er = motion_move_forward((int32_t)amount, (int16_t)VC_BYPASS_MM_S);
		}
		break;
	case VC_BYP_ACT_BACK:
		if (again != FALSE) {		/* what is left of it */
			W done = ((vc_ms.left_mm - vc_move_l0) + (vc_ms.right_mm - vc_move_r0)) / 2;

			amount -= (done < 0) ? -done : done;
		}
		if (amount <= 0) {
			none = TRUE;
		} else {
			er = motion_move_backward((int32_t)amount, (int16_t)VC_BYPASS_MM_S);
		}
		break;
	default: {	/* LEFT / RIGHT */
		/* Turn to where the turns so far add up to (vc_want_deg), not
		   by this turn's own angle: a turn that came out short or long,
		   or a straight move that drifted, is made up for by the next
		   turn, as far as the wheel encoders saw it.  The correction is
		   kept within VC_TURN_FIX_MAX_DEG of the turn asked for. */
		W nominal = (vc_act.kind == VC_BYP_ACT_LEFT) ? amount : -amount;
		W ddeg = (vc_want_deg * 10) - vc_turned_ddeg(&vc_ms);
		W cmd = (ddeg >= 0) ? ((ddeg + 5) / 10) : -((5 - ddeg) / 10);

		if (again == FALSE) {
			if (cmd > (nominal + VC_TURN_FIX_MAX_DEG)) {
				cmd = nominal + VC_TURN_FIX_MAX_DEG;
			} else if (cmd < (nominal - VC_TURN_FIX_MAX_DEG)) {
				cmd = nominal - VC_TURN_FIX_MAX_DEG;
			} else {
				/* as worked out */
			}
		}
		if (((nominal > 0) && (cmd < 1)) || ((nominal < 0) && (cmd > -1)) || (nominal == 0)) {
			none = TRUE;		/* already there */
		} else if (cmd > 0) {
			er = motion_turn_left((int32_t)cmd);
		} else {
			er = motion_turn_right((int32_t)(-cmd));
		}
		amount = (cmd < 0) ? -cmd : cmd;
		break;
	}
	}

	{
		static const char *const names[5] = { "?", "forward", "back", "turn left", "turn right" };
		BOOL turn = ((vc_act.kind == VC_BYP_ACT_LEFT) || (vc_act.kind == VC_BYP_ACT_RIGHT)) ? TRUE : FALSE;

		tm_printf((UB*)"VC: bypass [%s] %s %d %s%s%s%s\n", vc_byp_phase_name(&vc_byp),
			  names[(vc_act.kind < 5U) ? vc_act.kind : 0U], (INT)amount,
			  (turn != FALSE) ? "deg" : "mm",
			  (watch != S5_BYP_QUIET) ? ", watched" : "",
			  ((vc_act.flags & VC_BYP_F_LINE) != 0U) ? ", until the line" : "",
			  (again != FALSE) ? " (the rest of it)" : "");
	}
	if (er != E_OK) {
		vc_go_fault(CAUSE_MOVE, "bypass move refused by Motion Control");
		return;
	}
	if (none != FALSE) {
		(void)motion_stop();		/* nothing to drive: as if arrived */
	}
	vc_ph = PH_MOVE;
	vc_ph_ms = now;
}

/* The bypass is over: the car should be on the line past the obstacle
   (or never left it) */
LOCAL void vc_byp_finished(void)
{
	BOOL	backed = vc_byp_backed_off(&vc_byp);
	B	dir = 0;

	vc_to_s5(S5_CMD_STOP);			/* back to PATROL */
	vc_estop_clear(ESTOP_OBSTACLE);
	if (backed != FALSE) {
		/* no room on either side, backed away, and now the scan no
		   longer reaches it: it is still there */
		vc_backed_n++;
		if (vc_backed_n >= (UB)VC_BACKOFF_MAX) {
			vc_obst_plan = (UB)OBST_PLAN_BLOCKED;
			vc_go_fault(CAUSE_NO_PATH, "no way round (backed away twice)");
			return;
		}
	} else {
		vc_backed_n = 0U;
	}
	if ((vc_byp.side != 0) && (vc_byp.line_seen == false)) {
		/* drove across where the line should be without seeing it:
		   it is behind the car now, on the side it came from */
		dir = (vc_byp.side > 0) ? (B)1 : (B)-1;
	}
	tm_printf((UB*)"VC: bypass done: %d mm along, line %s\n", (INT)vc_byp.x_mm,
		  (vc_byp.side == 0) ? "never left" :
		  ((vc_byp.line_seen != false) ? "found by the line sensors" : "NOT seen: searching"));
	vc_finish_avoid((vc_byp.side == 0) ? "scan: path clear" : "round the obstacle", dir);
}

LOCAL void vc_byp_failed(void)
{
	static const char *const why[5] = {
		"bypass gave up",
		"no way round (no room beside the obstacle)",
		"no way round (the obstacle does not end)",
		"no way round (stopped and scanned too often)",
		"bypass: unexpected event"
	};
	UB f = (vc_byp.fail < 5U) ? vc_byp.fail : 0U;

	vc_to_s5_arg(S5_CMD_BYPASS, S5_BYP_QUIET);
	if (f == VC_BYP_FAIL_LOGIC) {
		vc_go_fault(CAUSE_MOVE, why[f]);
	} else {
		vc_obst_plan = (UB)OBST_PLAN_BLOCKED;
		vc_go_fault(CAUSE_NO_PATH, why[f]);
	}
}

/* Start what vc_bypass.c answered with (vc_act) */
LOCAL void vc_byp_do(UW now)
{
	switch (vc_act.kind) {
	case VC_BYP_ACT_WAIT:			/* the scan is still running */
		vc_ph = PH_SCAN;
		break;
	case VC_BYP_ACT_SCAN:
		tm_printf((UB*)"VC: bypass [%s] scan\n", vc_byp_phase_name(&vc_byp));
		vc_obst_plan = (UB)OBST_PLAN_STOP;
		vc_to_s5(S5_CMD_START_SCAN);
		vc_ph = PH_SCAN;
		vc_ph_ms = now;
		break;
	case VC_BYP_ACT_LOOK:
		tm_printf((UB*)"VC: bypass [%s] look\n", vc_byp_phase_name(&vc_byp));
		if ((vc_act.flags & VC_BYP_F_SIDE) != 0U) {	/* the check in the lane */
			vc_to_s5_arg(S5_CMD_BYPASS, (vc_act.amount == VC_BYP_LEFT) ? S5_BYP_SIDE_LEFT
										    : S5_BYP_SIDE_RIGHT);
		} else {
			vc_to_s5_arg(S5_CMD_BYPASS, (vc_act.amount == VC_BYP_LEFT) ? S5_BYP_LOOK_LEFT
										    : S5_BYP_LOOK_RIGHT);
		}
		vc_ph = PH_LOOK;
		vc_ph_ms = now;
		break;
	case VC_BYP_ACT_DONE:
		vc_byp_finished();
		break;
	case VC_BYP_ACT_FAIL:
		vc_byp_failed();
		break;
	default:				/* FWD, BACK, LEFT, RIGHT */
		vc_move_begin(now, FALSE);
		break;
	}
}

/* PH_MOVE, every tick: has the move ended? */
LOCAL void vc_move_tick(UW now, UINT estop)
{
	UINT f;

	/* the way back: stop on the line */
	if ((vc_act.kind == VC_BYP_ACT_FWD) && ((vc_act.flags & VC_BYP_F_LINE) != 0U) &&
	    (vc_move_end != VC_BYP_EV_LINE)) {
		if (vc_on_line() != FALSE) {
			vc_line_n++;
		} else {
			vc_line_n = 0U;
		}
		if (vc_line_n >= VC_LINE_TICKS) {
			(void)motion_stop();
			vc_move_end = VC_BYP_EV_LINE;
			return;			/* DONE comes once it has stopped */
		}
	}
	f = vc_flag(flg_motion);
	if ((f & MOTION_FLG_DONE) == 0U) {
		return;				/* driving */
	}
	if ((estop & (ESTOP_IMU_LIFTED | ESTOP_IMU_FAULT)) != 0U) {
		vc_go_fault(CAUSE_IMU, ((estop & ESTOP_IMU_LIFTED) != 0U) ? "lifted: run ended" : "IMU stop");
		return;
	}
	if (((f & MOTION_FLG_FAULT) != 0U) && ((estop & ESTOP_OBSTACLE) == 0U) &&
	    (vc_stop_seen == FALSE) && (vc_move_end != VC_BYP_EV_LINE)) {
		vc_go_fault(CAUSE_MOVE, "bypass move failed (stalled?)");
		return;
	}
	vc_to_s5_arg(S5_CMD_BYPASS, S5_BYP_QUIET);	/* stop watching */
	vc_ph = PH_REST;
	vc_ph_ms = now;
}

/* PH_REST is over: the car stands.  Tell vc_bypass.c how the move ended */
LOCAL void vc_move_ended(UW now, UINT estop)
{
	BOOL	straight = ((vc_act.kind == VC_BYP_ACT_FWD) || (vc_act.kind == VC_BYP_ACT_BACK)) ? TRUE : FALSE;
	BOOL	stopped = ((vc_stop_seen != FALSE) || ((estop & ESTOP_OBSTACLE) != 0U)) ? TRUE : FALSE;
	W	mm;

	(void)motion_get_status(&vc_ms);
	mm = ((vc_ms.left_mm - vc_move_l0) + (vc_ms.right_mm - vc_move_r0)) / 2;
	mm = (straight == FALSE) ? 0 : ((mm < 0) ? -mm : mm);

	if ((stopped != FALSE) && (straight == FALSE) && ((vc_act.flags & VC_BYP_F_GUARD) != 0U)) {
		/* a guarded turn (towards the line) stopped by Obstacle Scan:
		   the direction is now where it got to; tell vc_bypass.c how
		   far it turned, by the wheel encoders, so it can turn back
		   by just that */
		W d = (vc_ms.right_mm - vc_move_r0) - (vc_ms.left_mm - vc_move_l0);
		W deg = (d * 573) / ((W)TRACK_WIDTH_MM * 10);	/* 57.3 deg per rad */

		vc_want_deg = vc_move_want0 + deg;
		deg = (deg < 0) ? -deg : deg;
		tm_printf((UB*)"VC: bypass: turn stopped by Obstacle Scan after %d deg (%d cm away)\n",
			  (INT)deg, (INT)vc_stop_cm);
		vc_byp_step(VC_BYP_EV_STOPPED, (UW)deg, (UW)VC_BYP_RAY_INNER, (UW)vc_stop_cm, 0U);
		return;
	}

	if (vc_move_end == VC_BYP_EV_LINE) {
		tm_printf((UB*)"VC: bypass: line found after %d mm\n", (INT)mm);
		vc_byp_step(VC_BYP_EV_LINE, (UW)mm, 0U, 0U, 0U);
	} else if ((stopped != FALSE) && (vc_act.kind == VC_BYP_ACT_FWD) &&
		   ((vc_act.flags & VC_BYP_F_GUARD) != 0U)) {
		UB ray = (vc_stop_seen != FALSE) ? vc_stop_ray : (UB)VC_BYP_RAY_AHEAD;
		UH cm = (vc_stop_seen != FALSE) ? vc_stop_cm : 0U;

		tm_printf((UB*)"VC: bypass: stopped by Obstacle Scan after %d mm (%d cm away, %s)\n",
			  (INT)mm, (INT)cm, (ray == VC_BYP_RAY_AHEAD) ? "ahead" :
			  ((ray == VC_BYP_RAY_INNER) ? "front corner on the obstacle's side"
						     : "the other front corner"));
		vc_byp_step(VC_BYP_EV_STOPPED, (UW)mm, (UW)ray, (UW)cm, 0U);
	} else if (stopped != FALSE) {
		/* a turn or backing up, cut short by a stop from the watch of
		   the move before it (it comes at most VC_MOVE_REST_MS late, so
		   this should not happen): drive what is left.  More than
		   VC_MOVE_RETRIES times: something is wrong, stop here. */
		if (vc_retry < VC_MOVE_RETRIES) {
			vc_retry++;
			vc_move_begin(now, TRUE);
		} else {
			vc_go_fault(CAUSE_MOVE, "bypass move stopped again and again");
		}
	} else {
		vc_byp_step(VC_BYP_EV_ARRIVED, (UW)mm, 0U, 0U, 0U);
	}
}

/* Startup check: are the critical subsystems up? */
LOCAL void vc_startup_check(void)
{
	UW s5 = s5_status_word();
	UB s5f = S5_STW_FAULTS(s5);

	if ((tid_motion <= 0) || (flg_motion <= 0) || (flg_estop_req <= 0)) {
		vc_set_state((UB)SYS_STARTUP_ERROR, "Motion Control did not start");
		return;
	}
	if ((s5f & (S5_FLT_TIMER | S5_FLT_IRQ)) != 0U) {
		vc_set_state((UB)SYS_STARTUP_ERROR, "obstacle sensor timer / interrupt");
		return;
	}
	if (vc_line_present() == FALSE) {
		tm_printf((UB*)"VC: no Line and Barcode yet: PATROLLING drives straight\n");
	}
	if (imu_q <= 0) {
		tm_printf((UB*)"VC: no IMU yet: no hump or lifted events\n");
	}
	if (S5_STW_STATE(s5) == S5_ST_FAULT) {
		vc_go_fault(CAUSE_S5, "obstacle sensor fault at start-up");
		return;
	}
	vc_set_state((UB)SYS_IDLE_READY, "ready, waiting for START");
}

/* RESET: clear what can be cleared now, ask the faulty subsystems to
   recover, and wait for their FAULT_CLEARED for the rest. */
LOCAL void vc_do_reset(void)
{
	(void)motion_stop();
	vc_legs.n = 0U;
	vc_backed_n = 0U;
	vc_slow = FALSE;
	vc_hump_last = 0U;
	vc_hump_peak = 0U;			/* new run: peak starts again */
	vc_imu_rep.lift_cause = 0U;
	vc_hump_count = 0U;
	vc_barcode = (UB)BARCODE_NONE;
	vc_obst_plan = (UB)OBST_PLAN_NONE;
	vc_obst_width = 0U;
	vc_clear_l = 0U;
	vc_clear_r = 0U;

	vc_causes &= (UB)~(CAUSE_NO_PATH | CAUSE_MOVE | CAUSE_LINE);
	vc_fault_bit(FAULT_NO_PATH, FALSE);
	vc_fault_bit(FAULT_LINE_LOST, FALSE);
	vc_to_line(VC_LINE_CMD_RESET);

	if ((vc_causes & CAUSE_S5) != 0U) {
		vc_to_s5(S5_CMD_RESET);		/* answers FAULT_CLEARED when OK */
	} else {
		vc_to_s5(S5_CMD_STOP);
		vc_estop_clear(ESTOP_OBSTACLE);
	}
	if (imu_q > 0) {
		vc_to_imu(VC_IMU_CMD_RESET);	/* recalibrate (car stopped) */
	}
	if (vc_imu_faulty == FALSE) {
		/* lifted, or no IMU: nothing to wait for */
		vc_causes &= (UB)~CAUSE_IMU;
		vc_estop_clear(ESTOP_IMU_LIFTED | ESTOP_IMU_FAULT);
	}
	/* else: IMU sensor fault; it says SENSOR_FAULT 0 (cleared) once it
	   works again, and the VC leaves FAULT then */

	if (vc_st == (UB)SYS_STARTUP_ERROR) {
		vc_startup_check();
	} else if (vc_causes == 0U) {
		vc_go_idle("RESET");
	} else {
		vc_set_state((UB)SYS_FAULT_ESTOP, "RESET: waiting for the faulty sensor");
	}
}

/* =====================================================================
 *  Inputs
 * ===================================================================== */
LOCAL void vc_on_remote(remote_cmd_t c)
{
	UB	code = (UB)c.code;
	BOOL	used = FALSE;

	switch (code) {
	case CMD_START:
		if (vc_st == (UB)SYS_IDLE_READY) {
			vc_go_patrol(TRUE, "START");
			used = TRUE;
		}
		break;
	case CMD_STOP:
		if ((vc_moving_state() != FALSE) || (vc_st == (UB)SYS_PAUSED) ||
		    (vc_st == (UB)SYS_OBSTACLE_AVOIDANCE)) {
			if (vc_st == (UB)SYS_OBSTACLE_AVOIDANCE) {
				vc_to_s5(S5_CMD_STOP);	/* abort the scan */
			}
			vc_go_idle("STOP");
			used = TRUE;
		}
		break;
	case CMD_PAUSE:
		if ((vc_st == (UB)SYS_PATROLLING) || (vc_st == (UB)SYS_HANDLE_HUMP)) {
			(void)motion_stop();
			vc_to_line(VC_LINE_CMD_SUSPEND);	/* RESUME sends FOLLOW */
			vc_set_state((UB)SYS_PAUSED, "PAUSE");
			used = TRUE;
		}
		break;
	case CMD_RESUME:
		if (vc_st == (UB)SYS_PAUSED) {
			vc_go_patrol(TRUE, "RESUME");
			used = TRUE;
		}
		break;
	case CMD_RESET:
		vc_do_reset();
		used = TRUE;
		break;
	default:
		break;
	}
	tm_printf((UB*)"VC: remote %s (seq %d)%s\n",
		  (code < (UB)CMD_CODE_COUNT) ? vc_cmd_names[code] : "?",
		  (INT)c.seq, (used != FALSE) ? "" : " ignored in this state");
}

LOCAL void vc_on_obst(UW w)
{
	S5_OBST_MSG m;
	BOOL	avoiding = (vc_st == (UB)SYS_OBSTACLE_AVOIDANCE) ? TRUE : FALSE;

	s5_obst_unpack(w, &m);
	switch (m.type) {
	case S5_EVT_OBSTACLE:
		if (avoiding != FALSE) {
			/* stopped while watching a bypass move (ESTOP_OBSTACLE has
			   braked the car): PH_REST passes it on once the car stands.
			   side = the front corner it was seen at; the obstacle is on
			   the right of a bypass on the left */
			if (m.flag != 0U) {
				UB inner = (vc_byp.side > 0) ? S5_SIDE_RIGHT : S5_SIDE_LEFT;

				vc_stop_seen = TRUE;
				vc_stop_cm = m.dist_cm;
				vc_stop_ray = (m.side == S5_SIDE_NONE) ? (UB)VC_BYP_RAY_AHEAD :
					      ((m.side == inner) ? (UB)VC_BYP_RAY_INNER
								 : (UB)VC_BYP_RAY_OUTER);
			}
			break;
		}
		if (vc_moving_state() == FALSE) {
			break;			/* standing still: nothing to do */
		}
		if (m.flag == 0U) {
			vc_slow = TRUE;		/* closer than 50 cm: crawl */
		} else {
			vc_enter_avoid(m.dist_cm);
		}
		break;

	case S5_EVT_PATH_CLEAR:
		vc_slow = FALSE;
		if ((avoiding != FALSE) && (vc_ph == PH_SCAN)) {
			vc_estop_clear(ESTOP_OBSTACLE);
			vc_byp_step(VC_BYP_EV_CLEAR, 0U, 0U, 0U, 0U);
		}
		break;

	case S5_EVT_BYPASS_PLAN:
		vc_obst_width = m.width_cm;
		vc_clear_l = (m.side == S5_SIDE_LEFT) ? m.clear_cm : 0U;
		vc_clear_r = (m.side == S5_SIDE_RIGHT) ? m.clear_cm : 0U;
		if ((avoiding != FALSE) && (vc_ph == PH_SCAN)) {
			tm_printf((UB*)"VC: scan: %d cm wide, %d cm away, %d cm of room on the %s\n",
				  (INT)m.width_cm, (INT)m.dist_cm, (INT)m.clear_cm,
				  (m.side == S5_SIDE_LEFT) ? "LEFT" : "RIGHT");
			vc_byp_step(VC_BYP_EV_PLAN, (UW)m.side, (UW)m.dist_cm, (UW)m.width_cm,
				    (UW)m.clear_cm);
			if (vc_st == (UB)SYS_OBSTACLE_AVOIDANCE) {	/* the side it goes round on */
				vc_obst_plan = (vc_byp.side > 0) ? (UB)OBST_PLAN_TURN_LEFT
								 : (UB)OBST_PLAN_TURN_RIGHT;
			}
		}
		break;

	case S5_EVT_NO_PATH:
		if (m.flag != 0U) {
			vc_obst_plan = (UB)OBST_PLAN_BLOCKED;
			if (vc_st != (UB)SYS_FAULT_ESTOP) {
				vc_go_fault(CAUSE_NO_PATH, "no way round (retry limit)");
			}
		} else if ((avoiding != FALSE) && (vc_ph == PH_SCAN)) {
			tm_printf((UB*)"VC: scan: no room on either side, backing up\n");
			vc_byp_step(VC_BYP_EV_NO_PATH, 0U, 0U, 0U, 0U);
			if (vc_st == (UB)SYS_OBSTACLE_AVOIDANCE) {
				vc_obst_plan = (UB)OBST_PLAN_REVERSE;
			}
		} else {
			/* not expected here */
		}
		break;

	case S5_EVT_LOOK:
		if ((avoiding != FALSE) && (vc_ph == PH_LOOK)) {
			if (m.flag != 0U) {
				tm_printf((UB*)"VC: side check: nearest %d cm sideways, reaches %d cm ahead\n",
					  (INT)m.dist_cm, (INT)m.clear_cm);
			} else if (m.dist_cm == S5_CM_OPEN) {
				tm_printf((UB*)"VC: look: nothing ahead\n");
			} else {
				tm_printf((UB*)"VC: look: %d cm free ahead (%d mm from the line)\n",
					  (INT)m.dist_cm, (INT)((vc_byp.y_mm < 0) ? -vc_byp.y_mm : vc_byp.y_mm));
			}
			vc_byp_step(VC_BYP_EV_LOOK, (UW)m.dist_cm,
				    (m.flag != 0U) ? (UW)m.clear_cm : 0U, 0U, 0U);
		}
		break;

	case S5_EVT_BYPASS_DONE:	/* only sent to the bench stub's MOVE_DONE */
		break;

	case S5_EVT_SENSOR_FAULT:
		vc_causes |= CAUSE_S5;
		if ((vc_st != (UB)SYS_FAULT_ESTOP) && (vc_st != (UB)SYS_STARTUP_ERROR)) {
			vc_go_fault(CAUSE_S5, "ultrasonic sensor fault");
		}
		break;

	case S5_EVT_FAULT_CLEARED:
		vc_causes &= (UB)~CAUSE_S5;
		vc_estop_clear(ESTOP_OBSTACLE);
		if ((vc_st == (UB)SYS_FAULT_ESTOP) && (vc_causes == 0U)) {
			vc_go_idle("obstacle sensor works again");
		}
		break;

	default:
		break;
	}
}

/* Buddy 4's motion class -> telemetry motion_t (telem_format.h) */
LOCAL UB vc_motion_of(UW cls)
{
	switch (cls) {
	case VC_IMU_MOT_ACCELERATING:
		return (UB)MOTION_ACCELERATING;
	case VC_IMU_MOT_TURNING:
		return (UB)MOTION_TURNING;
	case VC_IMU_MOT_STEADY:
		return (UB)MOTION_STEADY;
	default:
		return (UB)MOTION_STATIONARY;
	}
}

/* imu_evt_q, Buddy 4's format (vc_imu_msg.h) */
LOCAL void vc_on_imu(UW w)
{
	UW	p = VC_IMU_MSG_PAYLOAD(w);
	UB	h;

	switch (VC_IMU_MSG_TYPE(w)) {
	case VC_IMU_EVT_HUMP:			/* finished; height in 0.5 mm */
		h = (UB)((p + 1U) / 2U);	/* whole mm (127 = over range) */
		vc_hump_last = h;
		if (h > vc_hump_peak) {
			vc_hump_peak = h;	/* Tribunal 1: peak kept by the VC */
		}
		if (vc_hump_count < 15U) {
			vc_hump_count++;
		}
		tm_printf((UB*)"VC: hump %d.%d mm (peak %d mm)%s\n", (INT)(p / 2U),
			  ((p & 1U) != 0U) ? 5 : 0, (INT)vc_hump_peak,
			  (p >= 127U) ? " OVER RANGE" : "");
		if (vc_st == (UB)SYS_HANDLE_HUMP) {
			vc_go_patrol(FALSE, "hump done");
		}
		break;
	case VC_IMU_EVT_MOTION:
		vc_motion_class = vc_motion_of(p);
		break;
	case VC_IMU_EVT_LIFTED:			/* ESTOP_IMU_LIFTED already set */
		/* 16 Nov (requirement changed 11 Nov): the car was lifted or
		   overturned.  Motion Control has stopped the motors already
		   (the stop bit).  During a run this ends the run: everything
		   stays stopped until RESET.  Standing by (carried to the
		   start, say) there is nothing to end: only the stop bit is
		   cleared again so that START works. */
		vc_motion_class = (UB)MOTION_LIFTED;
		vc_imu_rep.lift_cause = (UH)(p & (TF_LIFT_UP | TF_LIFT_TIPPED));
		tm_printf((UB*)"VC: lifted / overturned (%s%s)\n",
			  ((p & 1U) != 0U) ? "moved up " : "", ((p & 2U) != 0U) ? "tipped over" : "");
		if ((vc_moving_state() != FALSE) || (vc_st == (UB)SYS_PAUSED) ||
		    (vc_st == (UB)SYS_OBSTACLE_AVOIDANCE)) {
			if (vc_st == (UB)SYS_OBSTACLE_AVOIDANCE) {
				vc_to_s5(S5_CMD_STOP);	/* abort the scan */
			}
			vc_go_fault(CAUSE_IMU, "lifted: run ended");
		} else if ((vc_st != (UB)SYS_FAULT_ESTOP) && (vc_imu_faulty == FALSE)) {
			vc_estop_clear(ESTOP_IMU_LIFTED);
		} else {
			/* stopped for something else: RESET clears it */
		}
		break;
	case VC_IMU_EVT_SENSOR_FAULT:
		if (p != 0U) {			/* ESTOP_IMU_FAULT already set */
			vc_imu_faulty = TRUE;
			vc_imu_rep.imu_fault = (UH)(p & 0x7FU);
			tm_printf((UB*)"VC: IMU fault bits 0x%02x\n", (INT)p);
			vc_go_fault(CAUSE_IMU, "IMU sensor fault");
		} else {			/* 0 = cleared, recalibrated */
			vc_imu_faulty = FALSE;
			vc_imu_rep.imu_fault = 0U;
			vc_causes &= (UB)~CAUSE_IMU;
			vc_estop_clear(ESTOP_IMU_LIFTED | ESTOP_IMU_FAULT);
			if ((vc_st == (UB)SYS_FAULT_ESTOP) && (vc_causes == 0U)) {
				vc_go_idle("IMU works again");
			}
		}
		break;
	default:
		break;
	}
}

#if VC_BARCODE_ACT == 2
LOCAL void vc_leg_failed(const char *what);

/* =====================================================================
 *  Barcode turns at the next junction (15 Nov, VC_BARCODE_ACT 2)
 * ===================================================================== */
LOCAL const char *vc_bc_name(UB bc)
{
	static const char *const nm[6] = { "-", "LEFT", "RIGHT", "STRAIGHT", "U-TURN", "INVALID" };

	return (bc < 6U) ? nm[bc] : "?";
}

LOCAL W vc_odometer(void)
{
	return (motion_get_status(&vc_ms) == E_OK) ? (W)vc_ms.odometer_mm : 0;
}

/* A barcode was read: its command waits for the next junction */
LOCAL void vc_jct_remember(UB bc)
{
	if ((bc == (UB)BARCODE_LEFT) || (bc == (UB)BARCODE_RIGHT) ||
	    (bc == (UB)BARCODE_UTURN) || (bc == (UB)BARCODE_STRAIGHT)) {
		vc_jct_cmd = bc;
		vc_jct_odo = vc_odometer();
		tm_printf((UB*)"VC: barcode %s: at the next junction\n", vc_bc_name(bc));
	} else {
		tm_printf((UB*)"VC: barcode with no command: carry on\n");
	}
}

/* Line and Barcode found a junction: the car is about 24 mm past its
   middle (the line sensors), still following */
LOCAL void vc_jct_reached(void)
{
	UB	bc = vc_jct_cmd;
	W	since = vc_odometer() - vc_jct_odo;

	if (vc_st != (UB)SYS_PATROLLING) {
		return;				/* not following: not now */
	}
	vc_jct_cmd = (UB)BARCODE_NONE;
	if (bc == (UB)BARCODE_NONE) {
		tm_printf((UB*)"VC: junction, no barcode before it: straight on\n");
		return;
	}
	if (since > (W)VC_JCT_FORGET_MM) {
		tm_printf((UB*)"VC: junction %d mm after the barcode (more than %d): %s dropped, "
			  "straight on\n", (INT)since, (INT)VC_JCT_FORGET_MM, vc_bc_name(bc));
		return;
	}
	if (bc == (UB)BARCODE_STRAIGHT) {
		tm_printf((UB*)"VC: junction: STRAIGHT on\n");
		return;
	}
	tm_printf((UB*)"VC: junction %d mm after the barcode: %s\n", (INT)since, vc_bc_name(bc));
	vc_jct_cmd = bc;			/* kept while it turns */
	vc_to_line(VC_LINE_CMD_SUSPEND);
	vc_set_state((UB)SYS_HANDLING_BARCODE, "junction");
	if (motion_move_forward((int32_t)VC_JCT_AHEAD_MM, (int16_t)VC_BYPASS_MM_S) != E_OK) {
		vc_go_fault(CAUSE_MOVE, "junction: move refused by Motion Control");
		return;
	}
	vc_ph = PH_JCT_FWD;
	vc_ph_ms = vc_now_ms();
}

/* HANDLING_BARCODE at a junction, every tick */
LOCAL void vc_jct_tick(UW now)
{
	UINT	f = vc_flag(flg_motion);
	BOOL	left = (vc_jct_cmd != (UB)BARCODE_RIGHT) ? TRUE : FALSE;	/* U-turn: left */
	ER	er = E_OK;

	(void)now;
	switch (vc_ph) {
	case PH_JCT_FWD:			/* the axle onto the middle */
		if ((f & MOTION_FLG_DONE) == 0U) {
			break;
		}
		if ((f & MOTION_FLG_FAULT) != 0U) {
			vc_leg_failed("junction: move failed");
			break;
		}
		{
			W deg = (vc_jct_cmd == (UB)BARCODE_UTURN) ? (W)VC_JCT_UTURN_BLIND_DEG
								  : (W)VC_JCT_BLIND_DEG;

			er = (left != FALSE) ? motion_turn_left((int32_t)deg) : motion_turn_right((int32_t)deg);
			tm_printf((UB*)"VC: junction: turn %s %d deg, then until the line\n",
				  (left != FALSE) ? "left" : "right", (INT)deg);
		}
		vc_ph = PH_JCT_BLIND;
		break;

	case PH_JCT_BLIND:			/* off the old line */
		if ((f & MOTION_FLG_DONE) == 0U) {
			break;
		}
		if ((f & MOTION_FLG_FAULT) != 0U) {
			vc_leg_failed("junction: turn failed");
			break;
		}
		er = (left != FALSE) ? motion_turn_left((int32_t)VC_JCT_SEEK_DEG)
				     : motion_turn_right((int32_t)VC_JCT_SEEK_DEG);
		vc_jct_line_n = 0U;
		vc_ph = PH_JCT_SEEK;
		break;

	case PH_JCT_SEEK:			/* on until the new line */
		vc_jct_line_n = (vc_on_line() != FALSE) ? (UB)(vc_jct_line_n + 1U) : 0U;
		if (vc_jct_line_n >= VC_LINE_TICKS) {
			(void)motion_stop();
			tm_printf((UB*)"VC: junction: on the new line\n");
			vc_ph = PH_JCT_STOP;
		} else if ((f & MOTION_FLG_DONE) != 0U) {
			/* turned all the way without seeing it: Line and Barcode
			   searches (the sweep) */
			tm_printf((UB*)"VC: junction: new line not seen, searching\n");
			vc_jct_cmd = (UB)BARCODE_NONE;
			vc_go_patrol(FALSE, "junction turn (line not seen)");
		} else {
			/* turning */
		}
		break;

	default:				/* PH_JCT_STOP */
		if ((f & MOTION_FLG_DONE) != 0U) {
			vc_jct_cmd = (UB)BARCODE_NONE;
			vc_go_patrol(FALSE, "junction turn done");
		}
		break;
	}
	if (er != E_OK) {
		vc_go_fault(CAUSE_MOVE, "junction: turn refused by Motion Control");
	}
}
#endif /* VC_BARCODE_ACT == 2 */

LOCAL void vc_on_nav(UW w)
{
	switch (vc_nav_type(w)) {
	case VC_NAV_BARCODE:
		vc_barcode = vc_nav_barcode(w);
#if VC_BARCODE_ACT == 2
		vc_jct_remember(vc_barcode);
#elif VC_BARCODE_ACT
		if (vc_st == (UB)SYS_PATROLLING) {
			vc_plan_barcode(vc_barcode, &vc_legs);
			if (vc_legs.n > 0U) {
				vc_to_line(VC_LINE_CMD_SUSPEND);
				vc_set_state((UB)SYS_HANDLING_BARCODE, "barcode");
				vc_print_legs("barcode turn");
				vc_legs_begin(PH_LEGS);
			}
		}
#else
		tm_printf((UB*)"VC: barcode command %d noted, not acted on (VC_BARCODE_ACT 0)\n",
			  (INT)vc_barcode);
#endif
		break;
	case VC_NAV_LINE_LOST:
		vc_line_state = (UB)LINE_SEARCH;
		if (vc_st == (UB)SYS_PATROLLING) {
			(void)motion_stop();
			vc_set_state((UB)SYS_LINE_RECOVERY, "line lost");
			vc_begin_search();
		}
		break;
	case VC_NAV_LINE_REACQUIRED:
		vc_line_state = (UB)LINE_FOLLOWING;
		if ((vc_st == (UB)SYS_LINE_RECOVERY) && (vc_ph == PH_SEARCH)) {
			vc_go_patrol(FALSE, "line found");
		}
		break;
	case VC_NAV_LINE_TIMEOUT:
		vc_line_state = (UB)LINE_FAULT;
		if (vc_st == (UB)SYS_LINE_RECOVERY) {
			vc_fault_bit(FAULT_LINE_LOST, TRUE);	/* shared fault mask */
			vc_go_fault(CAUSE_LINE, "line not found within 2 s");
		}
		break;
	case VC_NAV_SENSOR_FAULT:
		vc_line_state = (UB)LINE_FAULT;
		vc_go_fault(CAUSE_LINE, "IR sensor fault");
		break;
	case VC_NAV_JUNCTION:
#if VC_BARCODE_ACT == 2
		vc_jct_reached();
#else
		tm_printf((UB*)"VC: junction (barcode turns there: VC_BARCODE_ACT 2)\n");
#endif
		break;
	default:
		break;
	}
}

LOCAL void vc_poll_inputs(void)
{
	remote_cmd_t	c;
	UW		w;
	INT		k;

	for (k = 0; k < VC_Q_DEPTH; k++) {
		if ((mbf_remote_cmd_q <= 0) ||
		    (tk_rcv_mbf(mbf_remote_cmd_q, &c, TMO_POL) != (INT)sizeof(c))) {
			break;
		}
		vc_on_remote(c);
	}
	for (k = 0; k < VC_Q_DEPTH; k++) {
		if (tk_rcv_mbf(obst_q, &w, TMO_POL) != (INT)sizeof(w)) {
			break;
		}
		vc_on_obst(w);
	}
	for (k = 0; k < VC_Q_DEPTH; k++) {
		if (tk_rcv_mbf(imu_evt_q, &w, TMO_POL) != (INT)sizeof(w)) {
			break;
		}
		vc_on_imu(w);
	}
	for (k = 0; k < VC_Q_DEPTH; k++) {
		if (tk_rcv_mbf(nav_q, &w, TMO_POL) != (INT)sizeof(w)) {
			break;
		}
		vc_on_nav(w);
	}
}

/* =====================================================================
 *  The current state, once per tick
 * ===================================================================== */
/* Patrol: steered by Line and Barcode while it follows the line
   (left wheel speed - steer, right wheel speed + steer), else straight.
   Refreshed every tick (Motion Control stops after 300 ms without). */
LOCAL void vc_drive(int16_t speed)
{
	S3_STEER	s;
	W		l = speed;
	W		r = speed;

	if ((vc_s3(&s) != FALSE) && ((s.flags & S3_STEER_FOLLOWING) != 0U)) {
		l = (W)speed - (W)s.steer_mm_s;
		r = (W)speed + (W)s.steer_mm_s;
		l = (l < 0) ? 0 : l;		/* never reverse while following */
		r = (r < 0) ? 0 : r;
	}
	(void)motion_set_velocity((int16_t)l, (int16_t)r);
}

/* Line search sweep: pivot on the spot towards where the line was last
   seen for VC_SWEEP_FIRST_MS, then swing back and forth.  Line and
   Barcode reports LINE_REACQUIRED (or LINE_TIMEOUT after 2 s). */
LOCAL void vc_sweep(UW elapsed)
{
	W dir = vc_search_dir;

	if ((elapsed >= (UW)VC_SWEEP_FIRST_MS) &&
	    ((((elapsed - (UW)VC_SWEEP_FIRST_MS) / (UW)VC_SWEEP_MS) % 2U) == 0U)) {
		dir = -dir;
	}
	(void)motion_set_velocity((int16_t)(-dir * VC_SEARCH_MM_S),
				  (int16_t)(dir * VC_SEARCH_MM_S));
}

/* A leg failed: an obstacle stop (ESTOP) is handled as an obstacle,
   anything else (stall, time-out) is a fault */
LOCAL void vc_leg_failed(const char *what)
{
	if ((vc_flag(flg_estop_req) & ESTOP_OBSTACLE) != 0U) {
		vc_enter_avoid(0U);
	} else {
		vc_go_fault(CAUSE_MOVE, what);
	}
}

/* Standing still (IDLE_READY, PAUSED) an obstacle needs no action, but
   Obstacle Scan has latched DETECTED and stopped pinging, waiting for a
   START_SCAN that will not come -- so the distance in telemetry would
   freeze.  Re-arm it once a second while the car stands still. */
LOCAL UW vc_rearm_ms;

LOCAL void vc_rearm_s5_when_still(UW now)
{
	if (S5_STW_STATE(s5_status_word()) != S5_ST_DETECTED) {
		vc_rearm_ms = now;
	} else if ((now - vc_rearm_ms) >= (UW)VC_REARM_MS) {
		vc_to_s5(S5_CMD_STOP);			/* back to PATROL */
		vc_estop_clear(ESTOP_OBSTACLE);
		vc_rearm_ms = now;
	} else {
		/* wait */
	}
}

LOCAL void vc_run_state(UW now)
{
	UINT	estop = vc_flag(flg_estop_req);
	INT	r;

	/* An IMU stop (lifted / overturned, or the IMU failed) while moving
	   always ends the run, even if its event was lost; a missed
	   obstacle event is caught the same way. */
	if ((vc_moving_state() != FALSE) &&
	    ((estop & (ESTOP_IMU_LIFTED | ESTOP_IMU_FAULT)) != 0U)) {
		vc_go_fault(CAUSE_IMU, ((estop & ESTOP_IMU_LIFTED) != 0U) ? "lifted: run ended" : "IMU stop");
		return;
	}

	switch (vc_st) {
	case SYS_PATROLLING:
		if ((estop & ESTOP_OBSTACLE) != 0U) {
			vc_enter_avoid(0U);
		} else {
			vc_drive((vc_slow != FALSE) ? (int16_t)VC_SLOW_MM_S
						    : (int16_t)VC_PATROL_MM_S);
		}
		break;

	case SYS_HANDLE_HUMP:
		if ((estop & ESTOP_OBSTACLE) != 0U) {
			vc_enter_avoid(0U);
		} else if ((now - vc_ph_ms) > (UW)VC_HUMP_TIMEOUT_MS) {
			vc_go_patrol(FALSE, "hump time-out");
		} else {
			vc_drive((int16_t)VC_HUMP_MM_S);
		}
		break;

	case SYS_OBSTACLE_AVOIDANCE:
		switch (vc_ph) {
		case PH_SETTLE:
			if ((now - vc_ph_ms) >= (UW)VC_STOP_SETTLE_MS) {
				vc_byp_begin();		/* the car stands: scan first */
				vc_byp_do(now);
			}
			break;
		case PH_SCAN:
			if ((now - vc_ph_ms) > (UW)VC_SCAN_TIMEOUT_MS) {
				vc_go_fault(CAUSE_MOVE, "Obstacle Scan did not answer");
			}
			break;
		case PH_MOVE:
			vc_move_tick(now, estop);
			break;
		case PH_REST:
			if ((now - vc_ph_ms) >= (UW)VC_MOVE_REST_MS) {
				vc_move_ended(now, estop);
			}
			break;
		case PH_LOOK:
			if ((now - vc_ph_ms) > (UW)VC_LOOK_TIMEOUT_MS) {
				vc_go_fault(CAUSE_MOVE, "Obstacle Scan did not answer the look");
			}
			break;
		default:
			break;
		}
		break;

	case SYS_LINE_RECOVERY:
		if (vc_ph == PH_SEARCH) {
			if ((now - vc_ph_ms) > (UW)VC_SEARCH_TIMEOUT_MS) {
				vc_fault_bit(FAULT_LINE_LOST, TRUE);
				vc_go_fault(CAUSE_LINE, "line search time-out");
			} else {
				vc_sweep(now - vc_ph_ms);
			}
		}
		break;

	case SYS_HANDLING_BARCODE:
#if VC_BARCODE_ACT == 2
		if (vc_ph >= PH_JCT_FWD) {
			vc_jct_tick(now);
			break;
		}
#endif
		r = vc_legs_step();
		if (r > 0) {
			vc_go_patrol(FALSE, "barcode turn done");
		} else if (r < 0) {
			vc_leg_failed("barcode turn failed");
		} else {
			/* turning */
		}
		break;

	case SYS_IDLE_READY:
#if VC_AUTOSTART_S > 0
		if ((vc_autostarted == FALSE) &&
		    ((now - vc_boot_ms) >= ((UW)VC_AUTOSTART_S * 1000U))) {
			vc_autostarted = TRUE;
			vc_go_patrol(TRUE, "bench auto-start");
		}
#endif
		vc_rearm_s5_when_still(now);
		break;

	case SYS_PAUSED:
		vc_rearm_s5_when_still(now);
		break;

	default:			/* FAULT, STARTUP_ERROR: wait */
		break;
	}
}

/* =====================================================================
 *  Telemetry: one snapshot per second to the Telemetry task
 * ===================================================================== */
LOCAL const UB vc_obst_state_of[8] = {	/* S5_ST_xxx -> obst_state_t */
	(UB)OBST_PATROLLING, (UB)OBST_DETECTED, (UB)OBST_COARSE_SCAN,
	(UB)OBST_FINE_SCAN, (UB)OBST_PROFILE_PLAN, (UB)OBST_BYPASSING,
	(UB)OBST_BLOCKED, (UB)OBST_SENSOR_FAULT
};

__attribute__((noinline)) LOCAL void vc_post_telem(void)
{
	telem_snapshot_t	snap = {0};
	motion_status_t		ms = {0};	/* stays 0 if the read fails */
	UW			s5 = s5_status_word();
	UH			d = S5_STW_DIST(s5);
	S3_STEER		s3 = {0};
	BOOL			s3_ok = vc_s3(&s3);
	BOOL			following;

	(void)motion_get_status(&ms);
	following = (((vc_st == (UB)SYS_PATROLLING) || (vc_st == (UB)SYS_HANDLE_HUMP)) &&
		     (s3_ok != FALSE) && ((s3.flags & S3_STEER_FOLLOWING) != 0U)) ? TRUE : FALSE;

	snap.sys_state  = vc_st;
	snap.drive_mode = (following != FALSE) ? DRIVE_FOLLOW_LINE : DRIVE_OPEN_LOOP;
	tc_snap_set_speed(&snap, ms.left_speed_mm_s / 10, ms.right_speed_mm_s / 10);
	tc_snap_set_enc(&snap, ms.left_count, ms.right_count);
	tc_snap_set_dist(&snap, (ms.odometer_mm > 0) ? ((UW)ms.odometer_mm / 10U) : 0U);
	snap.line_state = (s3_ok != FALSE) ? s3.state : vc_line_state;
	snap.ir_left    = (s3_ok != FALSE) ? ((s3.ir_bits >> 0) & 1U) : 0U;
	snap.ir_center  = (s3_ok != FALSE) ? ((s3.ir_bits >> 1) & 1U) : 0U;
	snap.ir_barcode = (s3_ok != FALSE) ? ((s3.ir_bits >> 2) & 1U) : 0U;
	snap.barcode    = vc_barcode;
	tc_snap_set_hump(&snap, vc_hump_last, vc_hump_peak, vc_hump_count);
	snap.motion     = vc_motion_class;
	tc_snap_set_obst(&snap, ((d != S5_CM_NONE) && (d != S5_CM_OPEN)) ? true : false,
			 d, vc_obst_width, 0U /* bearing not sent by S5 */,
			 vc_clear_l, vc_clear_r);
	snap.obst_plan  = vc_obst_plan;
	snap.obst_state = ((vc_st == (UB)SYS_OBSTACLE_AVOIDANCE) && vc_byp_returning(&vc_byp))
			  ? (UB)OBST_REACQUIRE : vc_obst_state_of[S5_STW_STATE(s5) & 7U];
	snap.fault      = vc_flag(flg_sysfault) & 0xFFU;
	snap.estop      = vc_flag(flg_estop_req) & ESTOP_ALL;
	snap.lift_cause = vc_imu_rep.lift_cause;
	snap.imu_fault  = vc_imu_rep.imu_fault;
	{
		uint32_t tilt = vc_imu_tilt_word;	/* 20 Nov: one load */

		snap.pitch_x2 = VC_IMU_TILT_PITCH(tilt);
		snap.roll_x2  = VC_IMU_TILT_ROLL(tilt);
	}

	if ((mbf_telem_q <= 0) ||
	    (tk_snd_mbf(mbf_telem_q, &snap, (INT)sizeof(snap), TMO_POL) != E_OK)) {
		vc_telem_dropped++;		/* never block (FR24) */
	}
}

/* =====================================================================
 *  The task
 * ===================================================================== */
EXPORT void vc_task(INT stacd, void *exinf)
{
	UW next_telem;

	(void)stacd; (void)exinf;

#if VC_AUTOSTART_S > 0
	vc_boot_ms = vc_now_ms();
#endif
	tm_printf((UB*)"VC: Vehicle Controller up\n");
	(void)tk_dly_tsk(VC_INIT_WAIT_MS);	/* let every subsystem start */
	vc_startup_check();
#if VC_AUTOSTART_S > 0
	tm_printf((UB*)"VC: BENCH AUTO-START in %d s (VC_AUTOSTART_S)\n", VC_AUTOSTART_S);
#endif
	next_telem = vc_now_ms() + VC_TELEM_MS;

	for (;;) {
		UW now;

		(void)tk_dly_tsk(VC_TICK_MS);
		vc_poll_inputs();
		now = vc_now_ms();
		vc_run_state(now);
		if ((W)(now - next_telem) >= 0) {
			next_telem += VC_TELEM_MS;
			vc_post_telem();
		}
	}
}

#endif /* VEHICLE_CONTROLLER */
