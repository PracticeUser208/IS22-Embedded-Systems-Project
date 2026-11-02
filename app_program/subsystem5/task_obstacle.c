/*
 *	task_obstacle.c
 *	Subsystem 5 -- the obstacle scan task (statechart 6.5).
 *
 *	States (S5_ST_xxx in subsystem5.h):
 *	  PATROL    ping straight ahead every 60 ms while the car follows the
 *	            line.  Obstacle nearer than S5_SLOW_CM: tell the VC to
 *	            slow (OBSTACLE_DETECTED, flag 0).  Nearer than S5_STOP_CM
 *	            on S5_STOP_CONFIRM pings in a row: set ESTOP_OBSTACLE and
 *	            send OBSTACLE_DETECTED (flag 1).                  FR14, FR15
 *	  DETECTED  stopped in front of it; wait for START_SCAN from the VC
 *	  COARSE    30 deg steps over +-60 deg (5 angles), median of 3 pings,
 *	            limited to the servo's safe travel (subsystem5.h)
 *	  FINE      5 deg steps across the obstacle and its edges, 1 ping each
 *	  PLAN      width, room each side, pick the side: BYPASS_PLAN, or
 *	            NO_PATH if neither side has room, or PATH_CLEAR   FR16, FR17
 *	  BYPASS    the VC drives round.  On MOVE_DONE, look ahead: clear ->
 *	            BYPASS_COMPLETE; still in the way -> rescan         FR18
 *	            "Obstacle passed" (undefined in Week 6, review feedback)
 *	            = nothing nearer than S5_OBJECT_CM straight ahead.
 *	  BLOCKED   no room either side: the VC backs up and sends START_SCAN
 *	            (the Week 6 BLOCKED -> back up -> COARSE SCAN loop).
 *	            Retry limit (Week 6 review feedback: the loop had none;
 *	            post-Tribunal 1 gap check: test it): after S5_MAX_RETRIES
 *	            rescans the task sends NO_PATH with flag 1 (gave up) and
 *	            stays BLOCKED.  The budget is per obstacle and is shared
 *	            by both rescans: "still in the way after a bypass" and
 *	            "blocked, backed up"
 *	  FAULT     sensor not responding: ESTOP_OBSTACLE and SYSFAULT set,
 *	            SENSOR_FAULT sent.  Keeps test-pinging every 500 ms; leaves
 *	            only when the sensor works again AND the VC sends RESET
 *	STOP from the VC returns to PATROL from any state.  Line reacquisition
 *	after a bypass belongs to Line and Barcode (Week 6 fix), not here.
 *
 *	The task never drives the motors and never clears an ESTOP bit: it
 *	sets its own bit, and the VC clears it once the car is stopped.
 *
 *	Timing: the task blocks on scan_q with a timeout that runs out at the
 *	next ping, so it uses no CPU between pings (NFR6).  The only spin is
 *	the 12 us trigger pulse.  The longest run between two blocking calls
 *	is measured and kept in s5_run_us_max (bounded run-to-block time,
 *	asked for in Version C).
 *
 *	At the end of this file: the bench-test stand-in for the Vehicle
 *	Controller (was s5_stub_vc.c), built only with S5_STUB_VC 1.
 */

#include "subsystem5.h"

/* ---- Tunables (starting values; tune with A9, A11, FR15) ---- */
#define S5_PING_MS		60	/* patrol ping period (HC-SR04 >= 60) */
#define S5_SLOW_CM		50	/* tell the VC to slow below this     */
#define S5_STOP_CM		15	/* FR15: stop within 15 cm            */
#define S5_HYST_CM		5	/* slow -> clear needs this much more */
#define S5_STOP_CONFIRM		2	/* pings in a row before stopping     */
#define S5_MAX_RETRIES		2	/* rescans per obstacle; the limit is
					   from Week 6 feedback, the value 2
					   is ours (team to confirm)         */
#define S5_FAULT_PING_MS	500	/* test ping period while faulty      */
#define S5_FAIL_LIMIT		3	/* bad pings in a row = sensor fault  */
#define S5_LATE_LIMIT		5	/* late pings in a row = warning      */
#define S5_COARSE_PINGS		3	/* median of 3 at each coarse angle   */
#define S5_DEBUG		1	/* 1: print state changes and scans   */

/* ---- State, single writer (this task) ---- */
LOCAL UB	s5_state = S5_ST_PATROL;
LOCAL BOOL	s5_pinging = TRUE;
LOCAL UB	s5_faults;		/* S5_FLT_xxx */
LOCAL UB	s5_retries;		/* 0..3, 2 bits */
LOCAL UH	s5_last_cm;		/* 9 bits */
LOCAL UB	s5_cnt_fail;		/* consecutive NONE pings  */
LOCAL UB	s5_cnt_stop;		/* consecutive stop pings  */
LOCAL UB	s5_cnt_late;		/* consecutive late pings  */
LOCAL UB	s5_cnt_good;		/* consecutive good pings while faulty */
LOCAL BOOL	s5_slow_sent;
LOCAL UW	s5_next_ms;		/* when the next ping is due */
LOCAL S5_SCAN	s5_scan;		/* 50 bytes */
LOCAL S5_PLAN	s5_plan_out;
LOCAL volatile UW s5_status;		/* packed snapshot, one store */
EXPORT UW	s5_run_us_max;		/* longest run between blocks */
EXPORT UW	s5_dropped;		/* obst_q posts dropped (full)  */

LOCAL const char *const s5_state_names[8] = {
	"PATROL", "DETECTED", "COARSE", "FINE", "PLAN", "BYPASS", "BLOCKED", "FAULT"
};
LOCAL const char *const s5_event_names[8] = {
	"-", "OBSTACLE_DETECTED", "PATH_CLEAR", "BYPASS_PLAN",
	"NO_PATH", "BYPASS_COMPLETE", "SENSOR_FAULT", "FAULT_CLEARED"
};

EXPORT const char *s5_state_name(UB st) { return s5_state_names[st & 7U]; }
EXPORT const char *s5_event_name(UB ev) { return s5_event_names[ev & 7U]; }

EXPORT UW s5_status_word(void)
{
	return s5_status;
}

/* ---------------------------------------------------------------------
 *  Helpers
 * --------------------------------------------------------------------- */
LOCAL UW s5_now_ms(void)
{
	SYSTIM t;

	(void)tk_get_tim(&t);
	return t.lo;
}

/* Run-to-block bookkeeping: CPU time between two blocking calls */
LOCAL void s5_note_run(UW us)
{
	if (us > s5_run_us_max) {
		s5_run_us_max = us;
	}
}

LOCAL void s5_publish(void)
{
	W servo = servo_get_deg() + 90;

	s5_status = ((UW)s5_state & 0x7U) |
		    (((UW)(s5_pinging ? 1U : 0U)) << 3) |
		    ((UW)s5_faults << 4) |
		    ((UW)s5_clamp9(s5_last_cm) << 12) |
		    (((UW)servo & 0xFFU) << 21) |
		    (((UW)s5_retries & 0x3U) << 29);	/* one 32-bit store */
}

LOCAL void s5_set_state(UB st)
{
#if S5_DEBUG
	if (st != s5_state) {
		tm_printf((UB*)"S5: %s -> %s\n", s5_state_name(s5_state),
			  s5_state_name(st));
	}
#endif
	s5_state = st;
	s5_publish();
}

/* Post one packed event.  Never blocks (team rule): drop + count if full */
LOCAL void s5_send(UB type, UB side, UB flag, UH dist, UB width, UH clear)
{
	S5_OBST_MSG m;
	UW w;

	m.type = type;
	m.side = side;
	m.flag = flag;
	m.dist_cm = dist;
	m.width_cm = width;
	m.clear_cm = clear;
	w = s5_obst_pack(&m);
	if (tk_snd_mbf(obst_q, &w, (INT)sizeof(w), TMO_POL) != E_OK) {
		s5_dropped++;
		s5_faults |= S5_FLT_QFULL;
	}
#if S5_DEBUG
	tm_printf((UB*)"S5: -> obst_q %s (0x%08x)\n", s5_event_name(type), w);
#endif
}

LOCAL void s5_estop_set(void)
{
	(void)tk_set_flg(flg_estop_req, ESTOP_OBSTACLE);	/* atomic */
}

/* Book-keeping for every ping: fault counters.  Returns TRUE if the
   sensor just crossed into a stop-class fault. */
LOCAL BOOL s5_count(UB st)
{
	UB before = s5_faults & S5_FLT_STOP_MASK;

	switch (st) {
	case US_OK:
	case US_OPEN:
		s5_cnt_fail = 0U;
		break;
	case US_NONE:
		if (s5_cnt_fail < S5_FAIL_LIMIT) { s5_cnt_fail++; }
		if (s5_cnt_fail >= S5_FAIL_LIMIT) { s5_faults |= S5_FLT_NO_ECHO; }
		break;
	case US_STUCK:
		s5_faults |= S5_FLT_ECHO_STUCK;
		break;
	default:			/* US_BUSY: no information */
		break;
	}
	return ((before == 0U) && ((s5_faults & S5_FLT_STOP_MASK) != 0U)) ? TRUE : FALSE;
}

LOCAL void s5_aim(W deg, INT min_wait_ms);

LOCAL void s5_enter_fault(void)
{
	s5_estop_set();					/* cannot see: stop */
	(void)tk_set_flg(flg_sysfault, SYSFAULT_OBSTACLE);
	s5_cnt_good = 0U;
	s5_set_state(S5_ST_FAULT);
	s5_aim(0, 0);		/* look ahead: the recovery pings test the sensor
				   facing forward, not wherever a scan stopped */
	s5_send(S5_EVT_SENSOR_FAULT, S5_SIDE_NONE, 0U, s5_last_cm, s5_faults, 0U);
}

/* Move the servo and wait for it to get there (blocks: no CPU used) */
LOCAL void s5_aim(W deg, INT min_wait_ms)
{
	INT wait = servo_settle_ms(deg - servo_get_deg());

	servo_set_deg(deg);
	s5_publish();
	(void)tk_dly_tsk((RELTIM)((wait > min_wait_ms) ? wait : min_wait_ms));
}

/* n pings at the current angle; median of the usable ones.
   Returns S5_CM_NONE if none were usable. */
LOCAL UH s5_measure(INT n)
{
	UH	v[3];
	INT	k = 0;
	INT	i, j;

	if (n > 3) {
		n = 3;					/* v[] holds 3 */
	}
	for (i = 0; i < n; i++) {
		UH r = us_ping();
		UB st = US_STATUS(r);

		if (s5_count(st) != FALSE) {
			return S5_CM_NONE;		/* fault: caller checks */
		}
		if (st == US_OK) {
			v[k++] = US_CM(r);
		} else if (st == US_OPEN) {
			v[k++] = S5_CM_OPEN;
		} else {
			/* no usable reading */
		}
		if ((i + 1) < n) {
			(void)tk_dly_tsk(S5_PING_MS);	/* sensor needs 60 ms */
		}
	}
	if (k == 0) {
		return S5_CM_NONE;
	}
	for (i = 0; i < k; i++) {			/* sort <= 3 values */
		for (j = i + 1; j < k; j++) {
			if (v[j] < v[i]) { UH t = v[i]; v[i] = v[j]; v[j] = t; }
		}
	}
	return v[k / 2];
}

LOCAL BOOL s5_stop_requested(void)	/* poll scan_q between scan steps */
{
	UW w;

	if (tk_rcv_mbf(scan_q, &w, TMO_POL) == (INT)sizeof(w)) {
		UB c = s5_cmd_of(w);
		if ((c == S5_CMD_STOP) || (c == S5_CMD_RESET)) {
			return TRUE;
		}
#if S5_DEBUG
		tm_printf((UB*)"S5: cmd %d ignored during scan\n", c);
#endif
	}
	return FALSE;
}

LOCAL void s5_print_scan(void)
{
#if S5_DEBUG
	INT i;

	tm_printf((UB*)"S5 scan (deg:cm, +=left):");
	for (i = S5_SCAN_SLOTS - 1; i >= 0; i--) {
		UH cm = s5_scan.cm[i];
		if (cm == S5_CM_NONE) { continue; }
		if (cm == S5_CM_OPEN) {
			tm_printf((UB*)" %+d:open", s5_deg_of(i));
		} else {
			tm_printf((UB*)" %+d:%d", s5_deg_of(i), cm);
		}
	}
	tm_printf((UB*)"\n");
#endif
}

LOCAL void s5_go_patrol(void)
{
	s5_retries = 0U;
	s5_cnt_stop = 0U;
	s5_slow_sent = FALSE;
	s5_set_state(S5_ST_PATROL);
	s5_aim(0, 0);
	s5_next_ms = s5_now_ms() + S5_PING_MS;
}

/* ---------------------------------------------------------------------
 *  Coarse scan -> fine scan -> profile & plan
 * --------------------------------------------------------------------- */
LOCAL void s5_run_scan(void)
{
	W deg, from, to;
	UB flag;

	s5_scan_clear(&s5_scan);

	/* Coarse: the 30-degree angles inside the servo's safe travel, plus
	   the two limit angles themselves (Week 6: 5 angles over +-60 deg; the
	   board limits how far the sensor can turn, see subsystem5.h) */
	s5_set_state(S5_ST_COARSE);
	for (deg = S5_SCAN_MIN_DEG; deg <= S5_SCAN_MAX_DEG; deg += S5_SCAN_STEP_DEG) {
		if ((servo_deg_ok(deg) == FALSE) ||
		    (((deg % S5_COARSE_STEP_DEG) != 0) &&
		     (deg != SERVO_LIMIT_LEFT_DEG) && (deg != SERVO_LIMIT_RIGHT_DEG))) {
			continue;
		}
		s5_aim(deg, 0);
		s5_scan_put(&s5_scan, deg, s5_measure(S5_COARSE_PINGS));
		if ((s5_faults & S5_FLT_STOP_MASK) != 0U) { s5_enter_fault(); return; }
		if (s5_stop_requested() != FALSE) { s5_go_patrol(); return; }
	}

	if (s5_fine_range(&s5_scan, &from, &to) != FALSE) {
		s5_set_state(S5_ST_FINE);
		for (deg = from; deg <= to; deg += S5_SCAN_STEP_DEG) {
			if ((servo_deg_ok(deg) == FALSE) ||
			    (s5_scan_get(&s5_scan, deg) != S5_CM_NONE)) {
				continue;	/* out of reach, or a coarse angle */
			}
			s5_aim(deg, S5_PING_MS);	/* also keeps 60 ms between pings */
			s5_scan_put(&s5_scan, deg, s5_measure(1));
			if ((s5_faults & S5_FLT_STOP_MASK) != 0U) { s5_enter_fault(); return; }
			if (s5_stop_requested() != FALSE) { s5_go_patrol(); return; }
		}
	}
	servo_set_deg(0);			/* head back while we think */

	s5_set_state(S5_ST_PLAN);
	{
		UW t0 = gpio_irq_now_us();

		s5_plan(&s5_scan, &s5_plan_out);
		s5_note_run(gpio_irq_now_us() - t0);
	}
	s5_print_scan();
#if S5_DEBUG
	tm_printf((UB*)"S5 plan: result=%d side=%d edges %+d..%+d deg  dist=%d"
		  " width=%d clearL=%d clearR=%d\n",
		  s5_plan_out.result, s5_plan_out.side, s5_plan_out.right_deg,
		  s5_plan_out.left_deg, s5_plan_out.dist_cm, s5_plan_out.width_cm,
		  s5_plan_out.clear_left_cm, s5_plan_out.clear_right_cm);
#endif

	switch (s5_plan_out.result) {
	case S5_PLAN_CLEAR:
		s5_send(S5_EVT_PATH_CLEAR, S5_SIDE_NONE, 0U, s5_plan_out.dist_cm, 0U, 0U);
		s5_go_patrol();
		break;
	case S5_PLAN_BYPASS:
		s5_send(S5_EVT_BYPASS_PLAN, s5_plan_out.side, 0U,
			s5_plan_out.dist_cm, (UB)s5_plan_out.width_cm,
			(s5_plan_out.side == S5_SIDE_LEFT) ? s5_plan_out.clear_left_cm
							   : s5_plan_out.clear_right_cm);
		s5_set_state(S5_ST_BYPASS);
		break;
	default:	/* BLOCKED */
		flag = (s5_retries >= S5_MAX_RETRIES) ? 1U : 0U;
		s5_send(S5_EVT_NO_PATH, S5_SIDE_NONE, flag, s5_plan_out.dist_cm,
			(UB)s5_plan_out.width_cm,
			(s5_plan_out.clear_left_cm > s5_plan_out.clear_right_cm)
				? s5_plan_out.clear_left_cm : s5_plan_out.clear_right_cm);
		s5_set_state(S5_ST_BLOCKED);
		break;
	}
	s5_aim(0, 0);
}

/* ---------------------------------------------------------------------
 *  Events
 * --------------------------------------------------------------------- */
LOCAL void s5_patrol_ping(void)
{
	UW t_wake = gpio_irq_now_us();
	UW t_pre;
	UW t_back;
	UW now = s5_now_ms();
	UH r;
	UB st;

	/* timing: late by more than one period? (CPU starved) */
	if ((W)(now - s5_next_ms) > S5_PING_MS) {
		if (s5_cnt_late < S5_LATE_LIMIT) { s5_cnt_late++; }
		if (s5_cnt_late >= S5_LATE_LIMIT) { s5_faults |= S5_FLT_LATE; }
		s5_next_ms = now;			/* resynchronise */
	} else {
		s5_cnt_late = 0U;
		s5_faults &= (UB)~S5_FLT_LATE;
	}
	s5_next_ms += S5_PING_MS;

	t_pre = gpio_irq_now_us() - t_wake;
	r = us_ping();				/* blocks on the echo flag */
	t_back = gpio_irq_now_us();
	st = US_STATUS(r);
	if (s5_count(st) != FALSE) {
		s5_enter_fault();
		return;
	}

	if (st == US_OK) {
		UH cm = US_CM(r);

		s5_last_cm = cm;
		if (cm <= (UH)S5_STOP_CM) {
			if (s5_cnt_stop < S5_STOP_CONFIRM) { s5_cnt_stop++; }
			if (s5_cnt_stop >= S5_STOP_CONFIRM) {
				s5_estop_set();			/* fast path */
				s5_send(S5_EVT_OBSTACLE, S5_SIDE_NONE, 1U, cm, 0U, 0U);
				s5_slow_sent = TRUE;
				s5_cnt_stop = 0U;
				s5_set_state(S5_ST_DETECTED);
			}
		} else {
			s5_cnt_stop = 0U;
			if ((cm <= (UH)S5_SLOW_CM) && (s5_slow_sent == FALSE)) {
				s5_send(S5_EVT_OBSTACLE, S5_SIDE_NONE, 0U, cm, 0U, 0U);
				s5_slow_sent = TRUE;
			} else if ((cm > (UH)(S5_SLOW_CM + S5_HYST_CM)) &&
				   (s5_slow_sent != FALSE)) {
				s5_send(S5_EVT_PATH_CLEAR, S5_SIDE_NONE, 0U, cm, 0U, 0U);
				s5_slow_sent = FALSE;
			} else {
				/* nothing new to report */
			}
		}
	} else if (st == US_OPEN) {
		s5_last_cm = S5_CM_OPEN;
		s5_cnt_stop = 0U;
		if (s5_slow_sent != FALSE) {
			s5_send(S5_EVT_PATH_CLEAR, S5_SIDE_NONE, 0U, S5_CM_OPEN, 0U, 0U);
			s5_slow_sent = FALSE;
		}
	} else {
		/* NONE / BUSY: no new reading, keep the last one */
	}
	s5_publish();
	/* before the wait + trigger pulse + after the wait */
	s5_note_run(t_pre + 12U + (gpio_irq_now_us() - t_back));
}

LOCAL void s5_fault_ping(void)
{
	UH r = us_ping();
	UB st = US_STATUS(r);

	s5_next_ms = s5_now_ms() + S5_FAULT_PING_MS;
	if ((st == US_OK) || (st == US_OPEN)) {
		s5_last_cm = (st == US_OK) ? US_CM(r) : S5_CM_OPEN;
		if (s5_cnt_good < S5_FAIL_LIMIT) { s5_cnt_good++; }
	} else {
		s5_cnt_good = 0U;
	}
	s5_publish();
}

LOCAL void s5_after_move(void)	/* MOVE_DONE while bypassing */
{
	UH cm;

	s5_aim(0, 0);
	cm = s5_measure(S5_COARSE_PINGS);
	if ((s5_faults & S5_FLT_STOP_MASK) != 0U) { s5_enter_fault(); return; }

	if ((cm == S5_CM_OPEN) || ((cm != S5_CM_NONE) && (cm > (UH)S5_OBJECT_CM))) {
		s5_last_cm = cm;
		s5_send(S5_EVT_BYPASS_DONE, S5_SIDE_NONE, 0U, cm, 0U, 0U);
		s5_go_patrol();			/* Line and Barcode now searches */
		return;
	}
	/* still in the way: rescan, within the retry limit */
	s5_last_cm = cm;
	if ((cm != S5_CM_NONE) && (cm <= (UH)S5_STOP_CM)) {
		s5_estop_set();
	}
	if (s5_retries >= S5_MAX_RETRIES) {
		s5_send(S5_EVT_NO_PATH, S5_SIDE_NONE, 1U, cm, 0U, 0U);
		s5_set_state(S5_ST_BLOCKED);
		return;
	}
	s5_retries++;
	s5_run_scan();
}

LOCAL void s5_handle_cmd(UW w)
{
	UB cmd = s5_cmd_of(w);

#if S5_DEBUG
	tm_printf((UB*)"S5: <- scan_q cmd %d in %s\n", cmd, s5_state_name(s5_state));
#endif
	switch (cmd) {
	case S5_CMD_STOP:
		if (s5_state != S5_ST_FAULT) {
			s5_go_patrol();
		}
		break;
	case S5_CMD_START_SCAN:
		if ((s5_state == S5_ST_DETECTED) || (s5_state == S5_ST_PATROL)) {
			s5_retries = 0U;
			s5_run_scan();
		} else if (s5_state == S5_ST_BLOCKED) {
			if (s5_retries >= S5_MAX_RETRIES) {
				s5_send(S5_EVT_NO_PATH, S5_SIDE_NONE, 1U, s5_last_cm, 0U, 0U);
			} else {
				s5_retries++;
				s5_run_scan();
			}
		} else {
			/* scanning, bypassing or faulty: ignore */
		}
		break;
	case S5_CMD_MOVE_DONE:
		if (s5_state == S5_ST_BYPASS) {
			s5_after_move();
		}
		break;
	case S5_CMD_RESET:
		if (s5_state == S5_ST_FAULT) {
			/* leave only if the sensor works again (statechart 6.5);
			   timer / interrupt faults cannot recover at run time */
			if ((s5_cnt_good >= S5_FAIL_LIMIT) &&
			    ((s5_faults & (S5_FLT_TIMER | S5_FLT_IRQ)) == 0U)) {
				s5_faults = 0U;
				s5_cnt_fail = 0U;
				(void)tk_clr_flg(flg_sysfault, ~SYSFAULT_OBSTACLE);
				s5_send(S5_EVT_FAULT_CLEARED, S5_SIDE_NONE, 0U, s5_last_cm, 0U, 0U);
				s5_go_patrol();
			} else {
				s5_send(S5_EVT_SENSOR_FAULT, S5_SIDE_NONE, 0U, s5_last_cm,
					s5_faults, 0U);
			}
		} else {
			s5_faults &= (UB)~(S5_FLT_LATE | S5_FLT_QFULL);	/* warnings */
			s5_go_patrol();
		}
		break;
	case S5_CMD_PATROL_OFF:
		s5_pinging = FALSE;
		s5_publish();
		break;
	case S5_CMD_PATROL_ON:
		s5_pinging = TRUE;
		s5_next_ms = s5_now_ms();
		s5_publish();
		break;
	default:
		break;
	}
}

/* ---------------------------------------------------------------------
 *  Init (called from subsystem5_init) and the task
 * --------------------------------------------------------------------- */
EXPORT ER s5_scan_task_init(void)
{
	ER	er;

	if (us_timer_check() == FALSE) {	/* times the echo */
		s5_faults |= S5_FLT_TIMER;	/* distances would be wrong */
	}
	er = us_init();
	if (er < E_OK) {
		s5_faults |= S5_FLT_IRQ;
	}
	(void)servo_init();
	s5_last_cm = S5_CM_NONE;
	s5_publish();
	return er;
}

EXPORT void s5_scan_task(INT stacd, void *exinf)
{
	(void)stacd; (void)exinf;

	tm_printf((UB*)"S5: obstacle scan task up (TRIG GP%d, ECHO GP%d, servo GP%d)\n",
		  US_TRIG_PIN, US_ECHO_PIN, SERVO_PIN);

	if ((s5_faults & S5_FLT_STOP_MASK) != 0U) {
		s5_enter_fault();		/* timer or echo interrupt missing */
	} else {
		s5_go_patrol();
		/* first ping only after 0.5 s: the servo has just jumped to
		   centre at power-up and the sensor needs a moment */
		s5_next_ms = s5_now_ms() + 500U;
	}

	for (;;) {
		UW	w = 0U;
		TMO	tmo = TMO_FEVR;
		INT	n;

		if (((s5_state == S5_ST_PATROL) && (s5_pinging != FALSE)) ||
		    (s5_state == S5_ST_FAULT)) {
			W left = (W)(s5_next_ms - s5_now_ms());
			tmo = (left > 0) ? (TMO)left : 0;
		}

		n = tk_rcv_mbf(scan_q, &w, tmo);	/* block */
		if (n == (INT)sizeof(w)) {
			s5_handle_cmd(w);
		} else if (n == E_TMOUT) {
			if (s5_state == S5_ST_PATROL) {
				s5_patrol_ping();
			} else if (s5_state == S5_ST_FAULT) {
				s5_fault_ping();
			} else {
				/* waiting for the VC */
			}
		} else {
			(void)tk_dly_tsk(10);		/* unexpected error: back off */
		}
	}
}

/* =====================================================================
 *  Bench-test stand-in VC (was s5_stub_vc.c)
 * =====================================================================
 *	BENCH-TEST STAND-IN for the Vehicle Controller.  Built only when
 *	S5_STUB_VC is 1 (subsystem5.h).  Turn it off once the real VC
 *	receives obst_q and sends scan_q, or the two will fight.
 *
 *	It plays the VC's side of the Obstacle Scan interface with no car:
 *	  OBSTACLE_DETECTED (stop)  -> "car stopped", then START_SCAN
 *	  BYPASS_PLAN               -> clears ESTOP_OBSTACLE (the VC's job),
 *	                               pretends to drive round for 3 s,
 *	                               then MOVE_DONE
 *	  NO_PATH                   -> pretends to back up, then START_SCAN;
 *	                               flag 1 (gave up): stops and waits
 *	  PATH_CLEAR / BYPASS_COMPLETE / FAULT_CLEARED -> clears ESTOP_OBSTACLE
 *	  SENSOR_FAULT              -> sends RESET every 3 s until cleared
 *	Every second with nothing happening it prints one status line.
 *
 *	Bench test: hold a box ~10 cm in front of the sensor.  You should see
 *	OBSTACLE_DETECTED, a coarse + fine scan, a plan, then the stub
 *	"driving round"; take the box away before MOVE_DONE and you get
 *	BYPASS_COMPLETE.  Unplug the echo wire to see SENSOR_FAULT.
 */

#if S5_STUB_VC

#define STUB_STATUS_MS		1000
#define STUB_BYPASS_MS		3000
#define STUB_BACKUP_MS		2000
#define STUB_RESET_MS		3000

LOCAL void stub_cmd(UB cmd)
{
	UW w = s5_cmd_pack(cmd, 0U);

	(void)tk_snd_mbf(scan_q, &w, (INT)sizeof(w), TMO_POL);
}

LOCAL void stub_clear_estop(void)
{
	(void)tk_clr_flg(flg_estop_req, ~ESTOP_OBSTACLE);
}

LOCAL UINT stub_flag(ID flgid)
{
	T_RFLG r;

	return (tk_ref_flg(flgid, &r) == E_OK) ? r.flgptn : 0U;
}

EXPORT void s5_stub_vc_task(INT stacd, void *exinf)
{
	BOOL	faulty = FALSE;

	(void)stacd; (void)exinf;
	tm_printf((UB*)"S5 stub VC: up (bench-test stand-in for the Vehicle Controller)\n");

	for (;;) {
		UW	w = 0U;
		INT	n = tk_rcv_mbf(obst_q, &w, faulty ? STUB_RESET_MS : STUB_STATUS_MS);

		if (n == E_TMOUT) {
			UW st = s5_status_word();
			UH cm = S5_STW_DIST(st);

			if (faulty != FALSE) {
				tm_printf((UB*)"VC(stub): sending RESET\n");
				stub_cmd(S5_CMD_RESET);
			}
			if (cm == S5_CM_OPEN) {
				tm_printf((UB*)"S5 [%s] dist=open servo=%+d faults=0x%02x"
					  " estop=0x%x sysfault=0x%x run_max=%dus\n",
					  s5_state_name(S5_STW_STATE(st)), S5_STW_SERVO(st),
					  S5_STW_FAULTS(st), stub_flag(flg_estop_req),
					  stub_flag(flg_sysfault), s5_run_us_max);
			} else {
				tm_printf((UB*)"S5 [%s] dist=%dcm (echo %dus) servo=%+d faults=0x%02x"
					  " estop=0x%x sysfault=0x%x run_max=%dus\n",
					  s5_state_name(S5_STW_STATE(st)), cm, us_last_width_us,
					  S5_STW_SERVO(st),
					  S5_STW_FAULTS(st), stub_flag(flg_estop_req),
					  stub_flag(flg_sysfault), s5_run_us_max);
			}
			continue;
		}
		if (n != (INT)sizeof(w)) {
			(void)tk_dly_tsk(100);
			continue;
		}

		{
			S5_OBST_MSG m;

			s5_obst_unpack(w, &m);
			tm_printf((UB*)"VC(stub): got %s side=%d flag=%d dist=%d width=%d clear=%d\n",
				  s5_event_name(m.type), m.side, m.flag, m.dist_cm,
				  m.width_cm, m.clear_cm);

			switch (m.type) {
			case S5_EVT_OBSTACLE:
				if (m.flag != 0U) {
					tm_printf((UB*)"VC(stub): car stopped (estop=0x%x), start scan\n",
						  stub_flag(flg_estop_req));
					(void)tk_dly_tsk(500);
					stub_cmd(S5_CMD_START_SCAN);
				} else {
					tm_printf((UB*)"VC(stub): slowing to crawl speed\n");
				}
				break;
			case S5_EVT_BYPASS_PLAN:
				tm_printf((UB*)"VC(stub): going round on the %s, clearing ESTOP,"
					  " driving (simulated %d ms)...\n",
					  (m.side == S5_SIDE_LEFT) ? "LEFT" : "RIGHT", STUB_BYPASS_MS);
				stub_clear_estop();
				(void)tk_dly_tsk(STUB_BYPASS_MS);
				tm_printf((UB*)"VC(stub): manoeuvre done, MOVE_DONE\n");
				stub_cmd(S5_CMD_MOVE_DONE);
				break;
			case S5_EVT_NO_PATH:
				if (m.flag != 0U) {
					tm_printf((UB*)"VC(stub): retry limit reached, the real VC"
						  " would go to FAULT.  Restarting the demo in 5 s\n");
					(void)tk_dly_tsk(5000);
					stub_cmd(S5_CMD_STOP);
					stub_clear_estop();
				} else {
					tm_printf((UB*)"VC(stub): backing up (simulated %d ms), rescan\n",
						  STUB_BACKUP_MS);
					(void)tk_dly_tsk(STUB_BACKUP_MS);
					stub_cmd(S5_CMD_START_SCAN);
				}
				break;
			case S5_EVT_SENSOR_FAULT:
				faulty = TRUE;
				break;
			case S5_EVT_FAULT_CLEARED:
				faulty = FALSE;
				stub_clear_estop();
				break;
			case S5_EVT_PATH_CLEAR:
			case S5_EVT_BYPASS_DONE:
				stub_clear_estop();
				break;
			default:
				break;
			}
		}
	}
}

#endif /* S5_STUB_VC */
