/*
 *	task_line.c
 *	Subsystem 3 -- Line and Barcode task (Week 6 statechart 6.3).
 *
 *	Every LF_PERIOD_MS (10 ms, one kernel tick):
 *	  1. takes the Vehicle Controller's commands from line_cmd_q
 *	     (SUSPEND, FOLLOW, SEARCH, RESET);
 *	  2. reads the three IR sensors (drv_ir.c) and works out the line's
 *	     position and the steering (line_follow.c, using Buddy 3's s3_ir
 *	     and s3_line);
 *	  3. drives Buddy 3's state machine (s3_state.c) and forwards its
 *	     events to the Vehicle Controller on nav_q:
 *	       FOLLOW --line gone LF_LOST_MS--> SEARCH          LINE_LOST
 *	       SEARCH --line seen-->            FOLLOW          LINE_REACQUIRED
 *	       SEARCH --2 s, FR19-->            FAULT           LINE_TIMEOUT
 *	       any    --ADC fails-->            FAULT           SENSOR_FAULT
 *	  4. publishes the steering in the steer_mtx mailbox (s3_steer_put);
 *	  5. WITH S3_BARCODES 1 (line_cfg.h, on since 2 Nov):
 *	     reads barcodes on the side sensor (LF_CH_SIDE, GP26) while
 *	     following:
 *	       FOLLOW --first bar--> CAPTURE (bar/space widths, bc_reader.c
 *	       with Buddy 3's s3_pulse) --white for the quiet time--> DECODE
 *	       (Code 39, s3_code39.c) --> FOLLOW, and sends BARCODE (left /
 *	       right / straight / U-turn: the letters in barcode_map.h, set
 *	       on the day) to the VC on nav_q.  The line is followed as usual
 *	       while a barcode is read.  The VC turns on it with
 *	       VC_BARCODE_ACT 1 (at once) or 2 (at the next junction: the
 *	       crossing tape under the side sensor, line_cfg.h LF_JCT_xxx,
 *	       is sent as JUNCTION on nav_q, 15 Nov).
 *	     Only while following (after START): picking the car up made the
 *	     side sensor "see" bars (3 Nov).  With LF_BC_READ_STANDING 1 it
 *	     also reads while the car stands (pushed over a barcode by hand),
 *	     never acting on those.
 *	     A reading that fails is not reported straight away: the side
 *	     sensor is also recorded from the first bar-like sample until
 *	     2.5 s after the last one (bc_rec_watch), and that recording is
 *	     read again with the barcode's own lightest/darkest values
 *	     (bc_rec_decode_auto; faint printed bars, 3 Nov).
 *	     Every read (and every failed one with 3+ bars and spaces) goes to
 *	     the dashboard on MQTT is22/bc with its widths and the side
 *	     sensor's levels (s1_publish_barcode), and to the console.
 *
 *	Hand-over with the Vehicle Controller:
 *	  - It starts SUSPENDED and stays there until the VC sends FOLLOW
 *	    (START) or SEARCH (after a bypass); SUSPEND stops it again.
 *	  - On FOLLOW / SEARCH it resumes searching (Buddy 3's policy); if
 *	    the line is under the sensors it is "found" at once, otherwise it
 *	    tells the VC LINE_LOST so the VC sweeps the car to find it.
 *	  - Not calibrated yet: it also says LINE_LOST; the VC's sweep moves
 *	    the sensors across the line, which calibrates them, and the line
 *	    is then found.
 *	  - RESET forgets the calibration (Week 6: FAULT --reset--> INIT,
 *	    recalibrate) and goes back to SUSPENDED.
 *	Barcodes (S3_BARCODES 1): Code 39, the symbology of the course's
 *	barcodes (BARCODE.pptx, and the printed 164.5 mm "K").  Widths are
 *	timed with the 1 MHz timer, so they are only as fine as the sampling
 *	(10 ms): barcodes of about 7 cm and more at 60 mm/s; the 164.5 mm
 *	"K" at 10-40 mm/s (bc_check.c).
 */

#include "subsystem3.h"
#if S3_BARCODES
#include "subsystem3.h"
#include "subsystem3.h"
#include "subsystem3.h"
#include "subsystem3.h"
#endif
#include "../vehicle_ctrl.h"
#if S3_BARCODES && LF_BC_BY_DISTANCE
#include "../subsystem2/subsystem2.h"	/* motion_get_status: wheel edges */
#endif
#if S3_BARCODES || S3_SIDE_TRACE
#include "../subsystem1/subsystem1.h"	/* s1_publish_barcode, s1_publish_ir_trace */
#endif

#if !TEMPLATE_DEMO

#define S3_FAIL_LIMIT	5		/* ADC failures in a row = sensor fault */
#define S3_PRINT_MS	2000U		/* console line while waiting */

/* Owned by the Vehicle Controller; 0 if the firmware is built without it */
__attribute__((weak)) ID nav_q = 0;

LOCAL lf_t		lf;
LOCAL s3_state_t	st;
LOCAL BOOL		reported_lost;	/* LINE_LOST sent, REACQUIRED not yet */
LOCAL BOOL		check_search;	/* just resumed: tell the VC if no line */
LOCAL BOOL		fault_sent;
LOCAL UB		fails;
LOCAL UB		cal_told;	/* bit per sensor: "calibrated" printed */

#if S3_BARCODES
/* Barcode reading (single writer: this task) */
LOCAL bc_reader_t	bcr;
LOCAL BOOL		bc_standing;	/* started while the car stood */
LOCAL bc_rec_t		bcrec;		/* raw recording: second look (14 KB) */
LOCAL bc_reader_t	bc_scratch;	/* the second look's reader       */
#define bc_rec_active_now()	((bcrec.active) ? TRUE : FALSE)
#endif

LOCAL const char *const s3_names[8] = {
	"INIT", "FOLLOW", "CAPTURE", "DECODE", "SUSPENDED", "SEARCH", "FAULT", "?"
};
LOCAL const char *const s3_nav_names[8] = {
	"-", "BARCODE", "LINE_LOST", "LINE_REACQUIRED", "LINE_TIMEOUT",
	"SENSOR_FAULT", "JUNCTION", "?"
};
#if S3_BARCODES
LOCAL const char *const s3_bc_names[BARCODE_COUNT] = {
	"NONE", "LEFT", "RIGHT", "STRAIGHT", "U-TURN", "INVALID (no command)"
};
#endif

LOCAL UW s3_now_ms(void)
{
	SYSTIM t;

	(void)tk_get_tim(&t);
	return t.lo;
}

/* Buddy 3's state machine takes microseconds (modulo 2^32) */
LOCAL uint32_t s3_us(UW ms)
{
	return (uint32_t)(ms * 1000U);
}

#if S3_BARCODES
/* Barcode widths: the 1 MHz hardware timer (the kernel's clock moves in
   10 ms steps) */
LOCAL uint32_t s3_now_us(void)
{
	return (uint32_t)in_w(TIMER_TIMERAWL);
}
#endif

LOCAL void s3_nav_code(UB type, UB barcode)
{
	UW w = vc_nav_pack(type, barcode);

	if (nav_q > 0) {
		(void)tk_snd_mbf(nav_q, &w, (INT)sizeof(w), TMO_POL);	/* never block */
	}
	tm_printf((UB*)"S3: -> nav_q %s\n", s3_nav_names[type & 7U]);
}

LOCAL void s3_nav(UB type)
{
	s3_nav_code(type, 0U);
}

/* Run Buddy 3's state machine and forward its events to the VC */
LOCAL void s3_step(s3_state_input_t in, UW now)
{
	UB before = s3_state_get(&st);
	UB evt    = s3_state_step(&st, in, s3_us(now));
	UB after  = s3_state_get(&st);

	if (after != before) {
		tm_printf((UB*)"S3: %s -> %s\n", s3_names[before & 7U], s3_names[after & 7U]);
	}
	if ((evt & S3_STATE_EVT_LOST) != 0U) {
		s3_nav(VC_NAV_LINE_LOST);
		reported_lost = TRUE;
	}
	if ((evt & S3_STATE_EVT_REACQUIRED) != 0U) {
		s3_nav(VC_NAV_LINE_REACQUIRED);
		reported_lost = FALSE;
	}
	if ((evt & S3_STATE_EVT_TIMEOUT) != 0U) {
		s3_nav(VC_NAV_LINE_TIMEOUT);
	}
	if ((evt & S3_STATE_EVT_FAULT) != 0U) {
		s3_nav(VC_NAV_SENSOR_FAULT);
	}
}

LOCAL void s3_on_cmd(UW w, UW now)
{
	UB cmd = (UB)((w >> 29) & 0x7U);

	switch (cmd) {
	case VC_LINE_CMD_SUSPEND:
		s3_step(S3_STATE_SUSPEND, now);
		reported_lost = FALSE;
		check_search  = FALSE;
		break;
	case VC_LINE_CMD_FOLLOW:
	case VC_LINE_CMD_SEARCH:
		if (cmd == VC_LINE_CMD_FOLLOW) {
			lf_assume_on_line(&lf, now);	/* START / RESUME: on the line */
		}
		if (s3_state_get(&st) == S3_STATE_SUSPENDED) {
			s3_step(S3_STATE_RESUME_SEARCH, now);
			check_search = TRUE;
		} else if (s3_state_get(&st) == S3_STATE_FAULT) {
			s3_nav(VC_NAV_SENSOR_FAULT);	/* needs RESET first */
		} else {
			/* already following or searching */
		}
		break;
	case VC_LINE_CMD_RESET:
		s3_step(S3_STATE_RESET, now);
		s3_step(S3_STATE_SUSPEND, now);
		lf_init(&lf);
		reported_lost = FALSE;
		check_search  = FALSE;
		fault_sent    = FALSE;
		fails         = 0U;
		cal_told      = 0U;
		tm_printf((UB*)"S3: RESET: calibration cleared (both sensors): "
			  "slide the car across the line\n");
		break;
	default:
		break;
	}
}

LOCAL void s3_publish(const lf_out_t *p_o, UW now)
{
	S3_STEER	s;
	BOOL		follow = (s3_state_can_track(&st) &&	/* FOLLOW, CAPTURE */
				  p_o->calibrated) ? TRUE : FALSE;

	s.steer_mm_s = (follow != FALSE) ? (H)p_o->steer_mm_s : 0;
	s.search_dir = (B)p_o->search_dir;
	s.state      = s3_state_get(&st);
	s.ir_bits    = lf_ir_by_pin(p_o->ir_bits);	/* GP26, GP27, GP28 */
	s.flags      = (UB)((p_o->present ? S3_STEER_PRESENT : 0U) |
			    (p_o->calibrated ? S3_STEER_CALIBRATED : 0U) |
			    ((follow != FALSE) ? S3_STEER_FOLLOWING : 0U));
	s.stamp_ms   = now;
	s3_steer_put(&s);
}

#if S3_BARCODES
/* ---- Barcodes (bc_reader.h, barcode_map.h) ---- */
LOCAL UB s3_bc_command(char c)
{
	if (bc_letter_in(BARCODE_LEFT_LETTERS, c)) {
		return (UB)BARCODE_LEFT;
	}
	if (bc_letter_in(BARCODE_RIGHT_LETTERS, c)) {
		return (UB)BARCODE_RIGHT;
	}
	if (bc_letter_in(BARCODE_STRAIGHT_LETTERS, c)) {
		return (UB)BARCODE_STRAIGHT;
	}
	if (bc_letter_in(BARCODE_UTURN_LETTERS, c)) {
		return (UB)BARCODE_UTURN;
	}
	return (UB)BARCODE_INVALID;	/* in no line of barcode_map.h */
}

/* Fewer bars + spaces than this: a single mark (the line, a crossing),
   not worth reporting */
#define S3_BC_REPORT_MIN	3U

/* The live reading that failed with the most bars + spaces while the
   recording ran: reported if the second look cannot read it either */
LOCAL uint32_t	bc_fail_w[S3_PULSE_CAPACITY];
LOCAL UB	bc_fail_n;
LOCAL BOOL	bc_fail_abandoned;
LOCAL BOOL	bc_handled;		/* read live: no second look       */
LOCAL BOOL	bc_rec_standing;	/* the recording started standing  */

LOCAL const char *const s3_bc_how[BC_REC_HOW_MAX + 1U] = {
	"-", "the side sensor's calibration", "the barcode's own levels, 50 %",
	"the barcode's own levels, 35 %", "the barcode's own levels, 65 %",
	"the barcode's own levels, 25 %", "the barcode's own levels, 75 %"
};

/* The widths on the console in ms, bar first (to check a misread) */
LOCAL void s3_bc_print_widths(const uint32_t *p_w, uint8_t count)
{
	UB i;

	tm_printf((UB*)"S3: barcode widths ms (bar, space, ...):");
	for (i = 0U; i < count; i++) {
		tm_printf((UB*)" %d", (INT)((p_w[i] + 500U) / 1000U));
	}
	tm_printf((UB*)"\n");
}

/* One read (or failed read) to the console and the dashboard (is22/bc),
   with the side sensor's levels: how dark the bars got against the tape */
LOCAL void s3_bc_report(UB code, UB flags, const char *p_text,
			const uint32_t *p_w, uint8_t count)
{
	uint16_t lv[4] = { 0U, 0U, 0U, 0U };

	if (bcrec.mask_ok && (bcrec.dropouts > 0U)) {
		flags = (UB)(flags | TF_BC_DROPOUTS);
	}
	(void)bc_rec_levels(&bcrec, &lv[0], &lv[1]);	/* paper, bars */
	if (lf.cal_ok[LF_BARCODE]) {
		lv[2] = lf.cal[LF_BARCODE].white;
		lv[3] = lf.cal[LF_BARCODE].black;
	}
	if (p_text[0] != '\0') {
		if ((flags & TF_BC_STANDING) != 0U) {
			tm_printf((UB*)"S3: barcode *%s* = %s (car standing: not acted on)\n",
				  p_text, s3_bc_names[code]);
		} else {
			tm_printf((UB*)"S3: barcode *%s* -> %s\n", p_text, s3_bc_names[code]);
		}
	} else {
		tm_printf((UB*)"S3: barcode not read (%d bars and spaces%s)\n", (INT)count,
			  ((flags & TF_BC_ABANDONED) != 0U) ? ", a bar or space too long" : "");
	}
	tm_printf((UB*)"S3: side sensor: paper %d, bars %d; calibrated white %d, black %d\n",
		  (INT)lv[0], (INT)lv[1], (INT)lv[2], (INT)lv[3]);
	if ((flags & TF_BC_DROPOUTS) != 0U) {
		tm_printf((UB*)"S3: the side sensor dropped out %d times during it: check its wires\n",
			  (INT)bcrec.dropouts);
	}
	s3_bc_print_widths(p_w, count);
	s1_publish_barcode(code, flags, p_text, p_w, (UB)count, lv);
}

/* The live reading is over (bc_reader).  Read: report it now (and tell
   the VC while following).  Not read: keep it; the second look at the
   whole recording decides when the recording ends. */
LOCAL void s3_bc_live_done(UW now, BOOL over)
{
	char	text[C39_MAX_TEXT + 1U];
	uint8_t	count = bcr.pulse.count;
	uint8_t	n = 0U;
	UB	flags = (bc_standing != FALSE) ? (UB)TF_BC_STANDING : 0U;
	UB	i;

	text[0] = '\0';
	if (bc_standing == FALSE) {
		s3_step(S3_STATE_BARCODE_END, now);		/* -> DECODE */
	}
	if (over != FALSE) {
		n = bc_reader_decode(&bcr, text, (uint8_t)sizeof(text), &count);
	}
	if (n > 0U) {
		UB code = s3_bc_command(text[0]);

		bc_handled = TRUE;
		s3_bc_report(code, flags, text, bcr.pulse.widths_us, count);
		if (bc_standing == FALSE) {
			s3_nav_code(VC_NAV_BARCODE, code);
		}
	} else if ((count >= S3_BC_REPORT_MIN) && (count >= bc_fail_n)) {
		for (i = 0U; (i < count) && (i < S3_PULSE_CAPACITY); i++) {
			bc_fail_w[i] = bcr.pulse.widths_us[i];
		}
		bc_fail_n = (count < S3_PULSE_CAPACITY) ? count : (UB)S3_PULSE_CAPACITY;
		bc_fail_abandoned = (over != FALSE) ? FALSE : TRUE;
		if (!bc_rec_active_now()) {			/* no recording: say so now */
			s3_bc_report((UB)BARCODE_NONE,
				     (UB)(flags | (bc_fail_abandoned ? TF_BC_ABANDONED : 0U)),
				     "", bc_fail_w, bc_fail_n);
			bc_fail_n = 0U;
		}
	} else {
		/* a single mark (a crossing line): nothing to say */
	}
	if (bc_standing == FALSE) {
		s3_step(S3_STATE_DECODE_DONE, now);		/* -> FOLLOW */
	}
}

/* The recording has ended (bc_rec_watch): if the live reading did not
   read it, read the whole recording again (bc_rec_decode_auto) */
LOCAL void s3_bc_second_look(void)
{
	char	text[C39_MAX_TEXT + 1U];
	uint8_t	count = 0U;
	uint8_t	how = BC_REC_HOW_NONE;
	uint8_t	n;
	UB	flags = (bc_rec_standing != FALSE) ? (UB)TF_BC_STANDING : 0U;

	text[0] = '\0';
	if (bc_handled != FALSE) {
		return;
	}
	(void)bc_rec_find_dropouts(&bcrec, (uint16_t)LF_BC_DEAD_RAW, (uint16_t)LF_BC_DEAD_JUMP);
	n = bc_rec_decode_auto(&bcrec, lf.cal_ok[LF_BARCODE],
			       lf.cal[LF_BARCODE].white, lf.cal[LF_BARCODE].black,
			       LF_BC_BLACK_PCT, (LF_BLACK_IS_HIGH != 0) ? true : false,
			       LF_BC_MIN_OWN_SPAN, LF_BC_HYST_DIV, &bc_scratch,
			       text, (uint8_t)sizeof(text), &count, &how);
	if (n > 0U) {
		UB code = s3_bc_command(text[0]);

		tm_printf((UB*)"S3: barcode read on the second look, with %s\n",
			  s3_bc_how[(how <= BC_REC_HOW_MAX) ? how : 0U]);
		s3_bc_report(code, (UB)(flags | TF_BC_REREAD), text,
			     bc_scratch.pulse.widths_us, count);
		if ((bc_rec_standing == FALSE) && (s3_state_get(&st) == S3_STATE_FOLLOW)) {
			s3_nav_code(VC_NAV_BARCODE, code);
		}
	} else if (bc_fail_n >= S3_BC_REPORT_MIN) {
		text[0] = '\0';
		s3_bc_report((UB)BARCODE_NONE,
			     (UB)(flags | (bc_fail_abandoned ? TF_BC_ABANDONED : 0U)),
			     text, bc_fail_w, bc_fail_n);
	} else {
		/* a mark (the line under the side sensor): nothing to say */
	}
	bc_fail_n = 0U;
}

/* The barcode clock (line_cfg.h LF_BC_BY_DISTANCE, 4 Nov): what the
   barcode reader and the recording use instead of time.  While the car
   drives (or is pushed: its wheel turned in the last LF_BC_DIST_HOLD_MS)
   it advances with the distance the side sensor's wheel travels, at
   LF_BC_DIST_MM_S worth of microseconds per mm, so a thin bar is as long
   as a thin bar whatever the speed: the line following's weaving swung
   the left wheel between 0 and 30 mm/s over the barcode (4 Nov log).
   Standing still (a barcode slid under the car) it is real time. */
LOCAL UW	bc_clk_us;
LOCAL uint32_t	bc_clk_real_us;		/* real time at the last sample  */
#if LF_BC_BY_DISTANCE
LOCAL uint32_t	bc_clk_edges;		/* wheel edges at the last sample */
LOCAL BOOL	bc_clk_have;
LOCAL UW	bc_clk_moved_ms;	/* when the wheel last turned    */
#endif

#if S3_JUNCTIONS
LOCAL lf_jct_t	jct;			/* junction finder (line_follow.c)   */
#endif

LOCAL UW s3_bc_clock(UW now, BOOL following)
{
	uint32_t	real = s3_now_us();
	uint32_t	d_real = real - bc_clk_real_us;
#if LF_BC_BY_DISTANCE
	motion_status_t	ms;
	uint32_t	d_edges = 0U;

	if (motion_get_status(&ms) == E_OK) {
		uint32_t edges = (LF_BC_DIST_WHEEL == 1) ? ms.right_count :
				 (LF_BC_DIST_WHEEL == 2) ? ((ms.left_count / 2U) + (ms.right_count / 2U)) :
				 ms.left_count;

		if (bc_clk_have != FALSE) {
			d_edges = edges - bc_clk_edges;
		}
		bc_clk_edges = edges;
		bc_clk_have = TRUE;
	}
	if (d_edges > 0U) {
		bc_clk_moved_ms = now;
	}
#endif
	bc_clk_real_us = real;
#if LF_BC_BY_DISTANCE
	if ((following != FALSE) || ((now - bc_clk_moved_ms) < LF_BC_DIST_HOLD_MS)) {
		/* um travelled x (1000 / LF_BC_DIST_MM_S) us per um */
		bc_clk_us += (UW)((d_edges * LF_BC_UM_PER_EDGE * 1000U) / LF_BC_DIST_MM_S);
		return bc_clk_us;
	}
#else
	(void)now;
	(void)following;
#endif
	bc_clk_us += (UW)d_real;
	return bc_clk_us;
}

/* Every sample, after the line logic */
LOCAL void s3_barcode(UW now, BOOL dark)
{
	UB	state = s3_state_get(&st);
	BOOL	standing = (state == S3_STATE_SUSPENDED) ? TRUE : FALSE;
	BOOL	following = ((state == S3_STATE_FOLLOW) || (state == S3_STATE_CAPTURE) ||
			     (state == S3_STATE_DECODE)) ? TRUE : FALSE;
	BOOL	stand_ok = ((standing != FALSE) && (LF_BC_READ_STANDING != 0)) ? TRUE : FALSE;
	BOOL	allow = ((following != FALSE) || (stand_ok != FALSE)) ? TRUE : FALSE;
	uint32_t t_us = s3_bc_clock(now, following);
	uint16_t raw = lf.filt[LF_BARCODE];
	bool	act;
	UB	r;

#if S3_JUNCTIONS
	/* A junction (the crossing tape under the side sensor, line_cfg.h
	   LF_JCT_xxx): tell the VC, which makes the barcode's turn there */
	if (following == FALSE) {
		lf_jct_reset(&jct, t_us, (dark != FALSE) ? true : false);
	} else if (lf_jct_step(&jct, t_us, (dark != FALSE) ? true : false,
			       lf.dark[LF_LEFT] || lf.dark[LF_CENTRE],
			       LF_JCT_US(LF_JCT_RUN_MM), LF_JCT_US(LF_JCT_MAX_MM),
			       LF_JCT_US(LF_JCT_GAP_MM))) {
		tm_printf((UB*)"S3: junction (the crossing tape under the side sensor)\n");
		s3_nav(VC_NAV_JUNCTION);
	} else {
		/* nothing */
	}
#endif

	/* The car started, stopped or lost the line in the middle of one:
	   the state machine has already left CAPTURE; forget it */
	if (bc_reader_active(&bcr) &&
	    (((bc_standing != FALSE) && (standing == FALSE)) ||
	     ((bc_standing == FALSE) && (state != S3_STATE_CAPTURE)))) {
		bc_reader_cancel(&bcr);
	}

	/* The recording for the second look (bc_rec_watch): from the first
	   bar-like sample (LF_BC_ACT_PCT % of the side sensor's span darker
	   than its white; not calibrated yet: what the live reader calls
	   black) to LF_BC_REC_TAIL_MS after the last one.  Started standing
	   and now moving, or the other way round: forget it. */
	if (bcrec.active && (bc_rec_standing != standing)) {
		allow = FALSE;
	}
	if (lf.cal_ok[LF_BARCODE]) {
		int32_t d = (int32_t)raw - (int32_t)lf.cal[LF_BARCODE].white;
		int32_t span = (int32_t)lf.cal[LF_BARCODE].black - (int32_t)lf.cal[LF_BARCODE].white;

		if (span < 0) {
			d = -d;
			span = -span;
		}
		span = (span * (int32_t)LF_BC_ACT_PCT) / 100;	/* the margin over white */
		if (span > (int32_t)LF_BC_ACT_MAX_RAW) {
			span = (int32_t)LF_BC_ACT_MAX_RAW;	/* black may be the lifted car's */
		}
		act = (d > span) ? true : false;
	} else {
		act = (dark != FALSE) ? true : false;
	}
	switch (bc_rec_watch(&bcrec, t_us, raw, act, (allow != FALSE) ? true : false,
			     (uint32_t)LF_BC_REC_TAIL_MS * 1000U)) {
	case BC_REC_EV_STARTED:
		bc_handled = FALSE;
		bc_fail_n = 0U;
		bc_rec_standing = standing;
		break;
	case BC_REC_EV_ENDED:
		s3_bc_second_look();
		break;
	case BC_REC_EV_CANCELLED:
		/* stopped (or lost the line) before the recording's tail was
		   over: still read what was recorded (4 Nov: STOP 4 s after
		   the barcode threw a good recording away) */
		s3_bc_second_look();
		break;
	default:
		break;
	}

	r = bc_reader_sample(&bcr, t_us, (dark != FALSE),
			     (state == S3_STATE_FOLLOW) || (stand_ok != FALSE));
	switch (r) {
	case BC_RD_START:				/* the first bar */
		bc_standing = standing;
		if (standing == FALSE) {
			s3_step(S3_STATE_BARCODE_START, now);	/* -> CAPTURE */
		}
		break;
	case BC_RD_DONE:
		s3_bc_live_done(now, TRUE);
		break;
	case BC_RD_ABORT:				/* not a barcode */
		s3_bc_live_done(now, FALSE);
		break;
	default:					/* reading, or nothing */
		break;
	}
}
#endif /* S3_BARCODES */

/* Print once when the left or centre sensor has its own calibration
   (white and black seen at least LF_MIN_SPAN apart).  Until then the
   left sensor borrows the centre's (line_follow.c). */
LOCAL void s3_cal_report(void)
{
	static const UB idx[2] = { LF_LEFT, LF_CENTRE };
	static const char *const nm[2] = { "centre", "right" };
	UB k;

	for (k = 0U; k < 2U; k++) {
		UB i = idx[k];
		UB bit = (UB)(1U << i);

		if (lf.cal_ok[i] && ((cal_told & bit) == 0U)) {
			tm_printf((UB*)"S3: %s sensor calibrated: white %d black %d\n",
				  nm[k], (INT)lf.cal[i].white, (INT)lf.cal[i].black);
			cal_told |= bit;
		}
	}
}

#if S3_SIDE_TRACE
/* The side sensor's raw value at every sample, TF_IR_TRACE_N per message
   to the dashboard (s1_publish_ir_trace) */
LOCAL UB	tr_buf[TF_IR_TRACE_N];
LOCAL UB	tr_n;
LOCAL UB	tr_seq;

LOCAL void s3_trace_put(void)
{
	UW	v = lf.filt[LF_BARCODE];

	tr_buf[tr_n] = (UB)((v > 4095U) ? 255U : (v >> 4));
	tr_n++;
	if (tr_n >= TF_IR_TRACE_N) {
		UB state = s3_state_get(&st);
		UB ls = (state == S3_STATE_SUSPENDED) ? 0U :
			(((state == S3_STATE_FOLLOW) || (state == S3_STATE_CAPTURE) ||
			  (state == S3_STATE_DECODE)) ? 1U : 2U);
		UB w = lf.cal_ok[LF_BARCODE] ? (UB)(lf.cal[LF_BARCODE].white >> 4) : 0U;
		UB b = lf.cal_ok[LF_BARCODE] ? (UB)(lf.cal[LF_BARCODE].black >> 4) : 0U;

		s1_publish_ir_trace(ls, tr_seq, w, b, tr_buf);
		tr_seq++;
		tr_n = 0U;
	}
}
#endif

LOCAL void s3_sample(UW now, UH raw[IR_COUNT], lf_out_t *p_o)
{
	ER er = ir_read(raw);

	if (er < E_OK) {
		p_o->steer_mm_s = 0;
		p_o->present = false;
		p_o->calibrated = false;
		p_o->ir_bits = 0U;
		if (fails < 255U) {
			fails++;
		}
		if ((fails >= S3_FAIL_LIMIT) && (fault_sent == FALSE)) {
			tm_printf((UB*)"S3: IR sensors: ADC read failed (%d)\n", (INT)er);
			s3_step(S3_STATE_SENSOR_FAULT, now);
			fault_sent = TRUE;
		}
		return;
	}
	fails = 0U;
	lf_update(&lf, raw, now, p_o);
#if S3_SIDE_TRACE
	s3_trace_put();
#endif
	s3_cal_report();

	switch (s3_state_get(&st)) {
	case S3_STATE_INIT:			/* waiting for a calibration */
		if (p_o->calibrated) {
			s3_step(S3_STATE_CALIBRATED, now);
		} else if (check_search != FALSE) {
			s3_nav(VC_NAV_LINE_LOST);	/* VC sweeps: calibrates */
			reported_lost = TRUE;
			check_search  = FALSE;
		} else {
			/* keep waiting */
		}
		break;

	case S3_STATE_FOLLOW:
	case S3_STATE_CAPTURE:			/* reading a barcode: still following
						   (S3_BARCODES 1 only) */
		check_search = FALSE;
		if (p_o->present) {
			if (reported_lost != FALSE) {
				s3_nav(VC_NAV_LINE_REACQUIRED);
				reported_lost = FALSE;
			}
		} else if (lf_lost(&lf, now)) {
			s3_step(S3_STATE_LINE_LOST, now);	/* drops a capture */
		} else {
			/* short gap: keep steering towards it */
		}
		break;

	case S3_STATE_SEARCH:
		s3_step(S3_STATE_POLL, now);	/* FR19: 2 s time-out */
		if (s3_state_get(&st) == S3_STATE_SEARCH) {
			if (p_o->present) {
				s3_step(S3_STATE_LINE_FOUND, now);
			} else if (check_search != FALSE) {
				s3_nav(VC_NAV_LINE_LOST);	/* VC: sweep */
				reported_lost = TRUE;
			} else {
				/* still searching */
			}
		}
		check_search = FALSE;
		break;

	default:			/* SUSPENDED, FAULT; DECODE lasts one sample */
		break;
	}
#if S3_BARCODES
	s3_barcode(now, ((p_o->ir_bits & LF_IR_BARCODE) != 0U) ? TRUE : FALSE);
#endif
}

EXPORT void line_task(INT stacd, void *exinf)
{
	UH		raw[IR_COUNT] = {0U, 0U, 0U};
	lf_out_t	o = {0, -1, false, false, 0U};
	UW		next_print;
	UW		w;

	(void)stacd; (void)exinf;

	lf_init(&lf);
#if S3_BARCODES
	bc_reader_init(&bcr);
	bc_rec_init(&bcrec);
#endif
	s3_state_init(&st);
	(void)s3_state_step(&st, S3_STATE_SUSPEND, 0U);	/* wait for the VC */

	tm_printf((UB*)"S3: line task up (side GP%d, centre GP%d, right GP%d; barcodes %s), every %d ms\n",
		  (INT)(26U + LF_CH_SIDE), (INT)(26U + LF_CH_PAIR_LEFT),
		  (INT)(26U + LF_CH_PAIR_RIGHT), (S3_BARCODES != 0) ? "on" : "off",
		  (INT)LF_PERIOD_MS);
	if (s3_ir_status < E_OK) {
		tm_printf((UB*)"S3: ADC driver did not open (%d)\n", (INT)s3_ir_status);
	}
	next_print = s3_now_ms() + S3_PRINT_MS;

	for (;;) {
		UW now = s3_now_ms();

		while (tk_rcv_mbf(line_cmd_q, &w, TMO_POL) == (INT)sizeof(w)) {
			s3_on_cmd(w, now);
		}
		s3_sample(now, raw, &o);
		s3_publish(&o, now);

		/* While the car waits: raw values on the console, to check the
		   wiring and the calibration ("7 Read Pico console") */
		if (((INT)(now - next_print) >= 0) &&
		    ((s3_state_get(&st) == S3_STATE_SUSPENDED) ||
		     (s3_state_get(&st) == S3_STATE_INIT))) {
			/* seen = lightest-darkest since power-up / RESET:
			   the calibration's white and black */
			{
				UB pins = lf_ir_by_pin(o.ir_bits);

				tm_printf((UB*)"S3: raw side=%d centre=%d right=%d  black side%d centre%d right%d  "
					  "seen centre %d-%d right %d-%d  %s\n",
					  (INT)raw[LF_CH_SIDE], (INT)raw[LF_CH_PAIR_LEFT], (INT)raw[LF_CH_PAIR_RIGHT],
					  (INT)((pins >> LF_CH_SIDE) & 1U), (INT)((pins >> LF_CH_PAIR_LEFT) & 1U),
					  (INT)((pins >> LF_CH_PAIR_RIGHT) & 1U),
					  (INT)lf.lo[LF_LEFT], (INT)lf.hi[LF_LEFT],
					  (INT)lf.lo[LF_CENTRE], (INT)lf.hi[LF_CENTRE],
					  !o.calibrated ? "NOT calibrated: slide the car across the line" :
					  !lf.cal_ok[LF_LEFT] ? "right calibrated, centre not yet: slide across the line" :
					  "both calibrated");
			}
			next_print = now + S3_PRINT_MS;
		}

		(void)tk_dly_tsk(LF_PERIOD_MS);
	}
}

#endif /* !TEMPLATE_DEMO */

/* ======== from task_consumer.c ======== */
/*
 *	task_consumer.c
 *	Subsystem 3 -- the consumer task.
 *
 *	APIs shown: tk_rcv_mbx, tk_rel_mpf, tk_sig_sem, tk_loc_mtx,
 *	            tk_unl_mtx, tk_snd_mbf, tk_set_flg
 */

#include "subsystem3.h"

#if TEMPLATE_DEMO	/* template example, off: see subsystem3.h */
#include "../subsystem2/subsystem2.h"	/* IF-01/02: SAMPLE_MSG, pool, slots */
#include "../subsystem4/subsystem4.h"	/* IF-03: mbf_summary, flg_events */

EXPORT void consumer_task(INT stacd, void *exinf)
{
	SAMPLE_MSG	*msg;
	UB		text[S4_SUMMARY_MAX];	/* tm_sprintf adds a '\0' */
	INT		len;
	UW		count;
	ER		er;

	(void)stacd; (void)exinf;

	while (1) {
		/* 1. Wait for the next sample (TMO_FEVR = wait forever) */
		er = tk_rcv_mbx(mbx_samples, (T_MSG **)&msg, TMO_FEVR);
		if (er < E_OK) {
			CHECK(er, "tk_rcv_mbx");
			continue;
		}

		/* 2. Update the shared statistics.  Subsystem 4 reads them
		      too, so lock the mutex around every access (IF-04). */
		tk_loc_mtx(mtx_stats, TMO_FEVR);
		if (app_stats.received == 0 || msg->value < app_stats.min) {
			app_stats.min = msg->value;
		}
		if (app_stats.received == 0 || msg->value > app_stats.max) {
			app_stats.max = msg->value;
		}
		app_stats.sum += msg->value;
		count = ++app_stats.received;
		tk_unl_mtx(mtx_stats);

		/* 3. Every S3_BATCH_SIZE samples, send a text line to subsystem
		      4 through its message buffer (the text is copied), then
		      set FLG_BATCH to wake it up (IF-03). */
		if (count % S3_BATCH_SIZE == 0) {
			/* tm_sprintf: like sprintf, returns the length.
			   The longest line here is ~35 bytes < S4_SUMMARY_MAX. */
			len = tm_sprintf(text, (const UB*)"n=%u last=%d from core %d",
					 count, msg->value, msg->prc - 1);

			/* TMO_POL: if the buffer is full, drop the line rather
			   than block the data path. */
			er = tk_snd_mbf(mbf_summary, text, len, TMO_POL);
			if (er == E_OK) {
				tk_set_flg(flg_events, FLG_BATCH);
			}
		}

		/* 4. Done with the sample (IF-02): return its memory block to
		      the pool, then the slot so the producer may go on. */
		tk_rel_mpf(mpf_samples, msg);
		tk_sig_sem(sem_slots, 1);
	}
}

#endif /* TEMPLATE_DEMO */
