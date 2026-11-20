/*
 *	vc_test.c
 *	On-target tests for the Vehicle Controller.  Built only with:
 *	make TEST=1.  Test IDs: VC-U01, ...  (testing/vehicle_ctrl/TEST_LOG.md)
 *
 *	These check the parts that need no hardware: manoeuvre planning, the
 *	bypass logic (vc_bypass.c: a few fixed cases here; the full check is
 *	the PC simulation, vc_bypass_check.c) and the provisional message
 *	formats.  The state machine itself is tested on the car (manual
 *	tests in TEST_LOG.md).
 */

#include "vehicle_ctrl.h"


#include "subsystem5/subsystem5.h"
#include "test_report.h"

#if APP_TEST && VEHICLE_CONTROLLER

/* One step of the bypass logic: feed the event, compare the answer */
LOCAL BOOL t_byp(vc_byp_t *p_b, UB ev, UW a, UW b, UW c, UW d,
		 UB kind, UW amount, UB flags)
{
	vc_byp_act_t act;

	vc_byp_event(p_b, ev, a, b, c, d, &act);
	return ((act.kind == kind) && ((UW)act.amount == amount) && (act.flags == flags)) ? TRUE : FALSE;
}

EXPORT void vc_test(void)
{
	VC_LEGS	l;
	W	step;

	TEST_INIT();

	/* VC-U01: bypass on the left = left 90, side-step, right 90, pass;
	   side-step = width + half car + margin when there is room */
	step = vc_plan_bypass(S5_SIDE_LEFT, 15U, 20U, 400U, &l);
	TEST_CHECK("VC-U01",
		   (l.n == 4U) &&
		   (l.leg[0].kind == VC_LEG_LEFT) && (l.leg[0].amount == 90U) &&
		   (l.leg[1].kind == VC_LEG_FWD) &&
		   (l.leg[1].amount == (UH)(200 + (VC_CAR_WIDTH_MM / 2) + VC_BYPASS_MARGIN_MM)) &&
		   (l.leg[2].kind == VC_LEG_RIGHT) &&
		   (l.leg[3].kind == VC_LEG_FWD) &&
		   (l.leg[3].amount == (UH)(150 + VC_OBST_DEPTH_MM + VC_CAR_LENGTH_MM)) &&
		   (step == (W)l.leg[1].amount),
		   "bypass LEFT: legs and distances");

	/* VC-U02: bypass on the right mirrors it, offset is negative */
	step = vc_plan_bypass(S5_SIDE_RIGHT, 15U, 20U, 400U, &l);
	TEST_CHECK("VC-U02",
		   (l.n == 4U) && (l.leg[0].kind == VC_LEG_RIGHT) &&
		   (l.leg[2].kind == VC_LEG_LEFT) && (step < 0),
		   "bypass RIGHT: mirrored, offset negative");

	/* VC-U03: tight side: side-step limited to the room there is */
	step = vc_plan_bypass(S5_SIDE_LEFT, 15U, 20U, 25U, &l);
	TEST_CHECK("VC-U03",
		   (step == (W)(100 + 250 - (VC_CAR_WIDTH_MM / 2))),
		   "bypass side-step limited by the measured room");

	/* VC-U04: no side -> no legs */
	step = vc_plan_bypass(S5_SIDE_NONE, 15U, 20U, 400U, &l);
	TEST_CHECK("VC-U04", (l.n == 0U) && (step == 0), "no side: no bypass");

	/* VC-U05: return from a left offset = right 90, across, left 90 */
	vc_plan_return(300, &l);
	TEST_CHECK("VC-U05",
		   (l.n == 3U) && (l.leg[0].kind == VC_LEG_RIGHT) &&
		   (l.leg[1].kind == VC_LEG_FWD) && (l.leg[1].amount == 300U) &&
		   (l.leg[2].kind == VC_LEG_LEFT),
		   "return to the path from a left offset");

	/* VC-U06: barcodes: U-turn = two left turns (FR9), straight = none */
	vc_plan_barcode((UB)BARCODE_UTURN, &l);
	{
		UB n_u = l.n;
		BOOL u_ok = (n_u == 2U) && (l.leg[0].kind == VC_LEG_LEFT) &&
			    (l.leg[1].kind == VC_LEG_LEFT);

		vc_plan_barcode((UB)BARCODE_STRAIGHT, &l);
		TEST_CHECK("VC-U06", u_ok && (l.n == 0U),
			   "barcode U-turn = 2 x left 90, straight = no turn");
	}

	/* VC-U07: IMU (Buddy 4's format) / line message round trips */
	{
		UW wi = VC_IMU_MSG_PACK(VC_IMU_EVT_HUMP, 127U, 0xBEEFU);
		UW wn = vc_nav_pack(VC_NAV_BARCODE, (UB)BARCODE_RIGHT);

		TEST_CHECK("VC-U07",
			   (VC_IMU_MSG_TYPE(wi) == VC_IMU_EVT_HUMP) &&
			   (VC_IMU_MSG_PAYLOAD(wi) == 127U) &&
			   (VC_IMU_MSG_TS16(wi) == 0xBEEFU) &&
			   (vc_nav_type(wn) == VC_NAV_BARCODE) &&
			   (vc_nav_barcode(wn) == (UB)BARCODE_RIGHT),
			   "imu_evt_q / nav_q pack and unpack");
	}

	/* VC-U09 (16 Nov, 17 Nov): going round an obstacle.  Plan LEFT (15 cm
	   away, 20 cm wide): side-step half its width + half the car + the
	   margin, check the side (standing): still beside -> the first pass,
	   and far enough to be past where it was seen; check: clear -> turn
	   VC_BYP_IN_DEG in.  Obstacle Scan stops the turn after 20 deg -> back by
	   those 20, further along the lane (17 Nov: no longer VC_JOG_MM
	   further out), check, on, turn in, look: free -> to the line at
	   that angle (stopped by the line sensors), wheels onto it, line up,
	   done. */
	{
		static const vc_byp_cfg_t cfg = VC_BYP_CFG_INIT;
		vc_byp_t b;
		vc_byp_act_t act;
		UW	side_mm = (UW)((200 / 2) + (VC_CAR_WIDTH_MM / 2) + VC_BYPASS_MARGIN_MM);
		UB	watch = VC_BYP_F_GUARD;
		BOOL	ok;
		INT	i;

		vc_byp_start(&b, &cfg);
		ok =  t_byp(&b, VC_BYP_EV_PLAN, VC_BYP_LEFT, 15U, 20U, 400U, VC_BYP_ACT_LEFT, 90U, 0U);
		ok = (t_byp(&b, VC_BYP_EV_ARRIVED, 0U, 0U, 0U, 0U, VC_BYP_ACT_FWD, side_mm, watch) != FALSE) && ok;
		ok = (t_byp(&b, VC_BYP_EV_ARRIVED, side_mm, 0U, 0U, 0U, VC_BYP_ACT_RIGHT, 90U, 0U) != FALSE) && ok;
		ok = (t_byp(&b, VC_BYP_EV_ARRIVED, 0U, 0U, 0U, 0U, VC_BYP_ACT_LOOK, VC_BYP_RIGHT, VC_BYP_F_SIDE) != FALSE) && ok;
		ok = (vc_byp_side(&b) == 1) && ok;
		/* 12 cm sideways, reaching 20 cm ahead: beside it -> the first
		   pass at least, watched, not WIDE */
		vc_byp_event(&b, VC_BYP_EV_LOOK, 12U, 20U, 0U, 0U, &act);
		ok = ((act.kind == VC_BYP_ACT_FWD) && (act.flags == watch) &&
		      (act.amount >= (UH)VC_PASS_FIRST_MM)) ? ok : FALSE;
		ok = (t_byp(&b, VC_BYP_EV_ARRIVED, (UW)act.amount, 0U, 0U, 0U,
			    VC_BYP_ACT_LOOK, VC_BYP_RIGHT, VC_BYP_F_SIDE) != FALSE) && ok;
		/* nothing there any more, and past it: turn in, watched */
		ok = (t_byp(&b, VC_BYP_EV_LOOK, VC_BYP_CM_OPEN, 0U, 0U, 0U, VC_BYP_ACT_RIGHT, (UW)VC_BYP_IN_DEG, watch) != FALSE) && ok;
		/* the turn is stopped after 20 deg: back by 20, then further
		   along the lane (not out), watched, and check again */
		ok = (t_byp(&b, VC_BYP_EV_STOPPED, 20U, VC_BYP_RAY_INNER, 18U, 0U, VC_BYP_ACT_LEFT, 20U, 0U) != FALSE) && ok;
		vc_byp_event(&b, VC_BYP_EV_ARRIVED, 0U, 0U, 0U, 0U, &act);
		ok = ((act.kind == VC_BYP_ACT_FWD) && (act.flags == watch) &&
		      (act.amount >= (UH)VC_PASS_STEP_MM)) ? ok : FALSE;
		ok = (t_byp(&b, VC_BYP_EV_ARRIVED, (UW)act.amount, 0U, 0U, 0U,
			    VC_BYP_ACT_LOOK, VC_BYP_RIGHT, VC_BYP_F_SIDE) != FALSE) && ok;
		/* clear: on past where the turn saw it (if needed), turn in */
		vc_byp_event(&b, VC_BYP_EV_LOOK, VC_BYP_CM_OPEN, 0U, 0U, 0U, &act);
		for (i = 0; (i < 3) && (act.kind == VC_BYP_ACT_FWD); i++) {
			ok = (act.flags == watch) ? ok : FALSE;
			vc_byp_event(&b, VC_BYP_EV_ARRIVED, (UW)act.amount, 0U, 0U, 0U, &act);
		}
		ok = ((act.kind == VC_BYP_ACT_RIGHT) && (act.amount == (UW)VC_BYP_IN_DEG) && (act.flags == watch)) ? ok : FALSE;
		ok = (t_byp(&b, VC_BYP_EV_ARRIVED, 0U, 0U, 0U, 0U, VC_BYP_ACT_LOOK, VC_BYP_RIGHT, 0U) != FALSE) && ok;
		vc_byp_event(&b, VC_BYP_EV_LOOK, VC_BYP_CM_OPEN, 0U, 0U, 0U, &act);
		ok = ((act.kind == VC_BYP_ACT_FWD) &&
		      (act.flags == (UB)(VC_BYP_F_GUARD | VC_BYP_F_LINE))) ? ok : FALSE;
		ok = (vc_byp_returning(&b) != false) && ok;
		ok = (t_byp(&b, VC_BYP_EV_LINE, 400U, 0U, 0U, 0U,
			    VC_BYP_ACT_FWD, VC_IR_AHEAD_MM, watch) != FALSE) && ok;
		ok = (t_byp(&b, VC_BYP_EV_ARRIVED, VC_IR_AHEAD_MM, 0U, 0U, 0U, VC_BYP_ACT_LEFT, (UW)VC_BYP_IN_DEG, 0U) != FALSE) && ok;
		ok = (t_byp(&b, VC_BYP_EV_ARRIVED, 0U, 0U, 0U, 0U, VC_BYP_ACT_DONE, 0U, 0U) != FALSE) && ok;
		TEST_CHECK("VC-U09", (ok != FALSE) && (b.line_seen != false) && (b.fail == VC_BYP_FAIL_NONE),
			   "bypass: side check, pass, turn stopped -> further along, diagonally back to the line");
	}

	/* VC-U10 (12 Nov, 16 Nov): stopped while driving along the obstacle.
	   Something ahead in the lane -> a new scan, and the next side-step
	   is on the SAME side whatever the scan says, 45 deg out (not 90).
	   Only the front corner on the obstacle's side too close -> out at
	   45 deg.  An obstacle that does not end -> gives up. */
	{
		static const vc_byp_cfg_t cfg = VC_BYP_CFG_INIT;
		vc_byp_t b;
		vc_byp_act_t act;
		/* 10 cm wide: taken as VC_OBST_MIN_WIDTH_MM; half of it + half
		   the car + the margin, at least VC_SIDE_STEP_MM */
		UW	step_mm = (UW)((VC_OBST_MIN_WIDTH_MM / 2) + (VC_CAR_WIDTH_MM / 2) + VC_BYPASS_MARGIN_MM);
		BOOL	ok;
		INT	i;

		if (step_mm < (UW)VC_SIDE_STEP_MM) {
			step_mm = (UW)VC_SIDE_STEP_MM;
		}
		vc_byp_start(&b, &cfg);
		ok =  t_byp(&b, VC_BYP_EV_PLAN, VC_BYP_RIGHT, 15U, 10U, 400U, VC_BYP_ACT_RIGHT, 90U, 0U);
		ok = (t_byp(&b, VC_BYP_EV_ARRIVED, 0U, 0U, 0U, 0U, VC_BYP_ACT_FWD,
			    step_mm, VC_BYP_F_GUARD) != FALSE) && ok;
		vc_byp_event(&b, VC_BYP_EV_ARRIVED, step_mm, 0U, 0U, 0U, &act);	/* turn along */
		vc_byp_event(&b, VC_BYP_EV_ARRIVED, 0U, 0U, 0U, 0U, &act);	/* check */
		vc_byp_event(&b, VC_BYP_EV_LOOK, VC_BYP_CM_OPEN, 0U, 0U, 0U, &act);	/* pass */
		ok = ((act.kind == VC_BYP_ACT_FWD) && (act.flags == VC_BYP_F_GUARD) &&
		      (act.amount >= (UH)VC_PASS_FIRST_MM)) ? ok : FALSE;
		ok = (t_byp(&b, VC_BYP_EV_STOPPED, 200U, VC_BYP_RAY_AHEAD, 14U, 0U,
			    VC_BYP_ACT_SCAN, 0U, 0U) != FALSE) && ok;
		ok = (t_byp(&b, VC_BYP_EV_PLAN, VC_BYP_LEFT, 14U, 10U, 400U,
			    VC_BYP_ACT_RIGHT, 45U, 0U) != FALSE) && ok;	/* still the right */

		vc_byp_start(&b, &cfg);			/* a new obstacle */
		vc_byp_event(&b, VC_BYP_EV_PLAN, VC_BYP_RIGHT, 15U, 10U, 400U, &act);
		vc_byp_event(&b, VC_BYP_EV_ARRIVED, 0U, 0U, 0U, 0U, &act);
		vc_byp_event(&b, VC_BYP_EV_ARRIVED, (UW)act.amount, 0U, 0U, 0U, &act);
		vc_byp_event(&b, VC_BYP_EV_ARRIVED, 0U, 0U, 0U, 0U, &act);	/* check */
		vc_byp_event(&b, VC_BYP_EV_LOOK, VC_BYP_CM_OPEN, 0U, 0U, 0U, &act);	/* pass */
		ok = (t_byp(&b, VC_BYP_EV_STOPPED, 100U, VC_BYP_RAY_INNER, 10U, 0U,
			    VC_BYP_ACT_RIGHT, 45U, 0U) != FALSE) && ok;
		vc_byp_event(&b, VC_BYP_EV_ARRIVED, 0U, 0U, 0U, 0U, &act);
		ok = ((act.kind == VC_BYP_ACT_FWD) && (act.flags == VC_BYP_F_GUARD) &&
		      (act.amount >= (UH)(((UW)VC_JOG_MM * 1414U) / 1000U))) ? ok : FALSE;

		/* always beside it: check, go on, check, ... until it gives up */
		vc_byp_start(&b, &cfg);
		vc_byp_event(&b, VC_BYP_EV_PLAN, VC_BYP_LEFT, 15U, 10U, 400U, &act);
		for (i = 0; (i < 200) && (act.kind != VC_BYP_ACT_FAIL) && (act.kind != VC_BYP_ACT_DONE); i++) {
			if (act.kind == VC_BYP_ACT_LOOK) {
				vc_byp_event(&b, VC_BYP_EV_LOOK, 12U, 20U, 0U, 0U, &act);
			} else {
				vc_byp_event(&b, VC_BYP_EV_ARRIVED,
					     ((act.kind == VC_BYP_ACT_FWD) || (act.kind == VC_BYP_ACT_BACK))
						? (UW)act.amount : 0U, 0U, 0U, 0U, &act);
			}
		}
		TEST_CHECK("VC-U10",
			   (ok != FALSE) && (act.kind == VC_BYP_ACT_FAIL) &&
			   (b.fail == VC_BYP_FAIL_TOO_LONG) && (b.x_mm <= (W)VC_MAX_PASS_MM),
			   "bypass: stop in the lane -> rescan / move out; endless obstacle -> gives up");
	}

	/* VC-U08 (live): the VC reached IDLE_READY (or a fault it explains) */
	TEST_CHECK("VC-U08",
		   (vc_state() == (UB)SYS_IDLE_READY) || (vc_state() == (UB)SYS_FAULT_ESTOP),
		   "Vehicle Controller started and is waiting for START");

	TEST_SUMMARY("vehicle_ctrl");
}

#endif /* APP_TEST && VEHICLE_CONTROLLER */
