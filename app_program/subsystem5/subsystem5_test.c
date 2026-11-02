/*
 *	subsystem5_test.c
 *	On-target tests for subsystem 5.  Built only with: make TEST=1
 *	Record results in testing/subsystem5/TEST_LOG.md.  IDs: S5-U01, ...
 *
 *	The tests do NOT drive the sensor or the servo themselves: those
 *	belong to the scan task (TEAM.md section 3).  The live checks read
 *	the task's status word instead.
 */

#include "subsystem5.h"
#include "../test_report.h"

#if APP_TEST

/* ---- S5-U01: obst_q pack/unpack round trip at the range limits ---- */
LOCAL BOOL t_obst_roundtrip(UB type, UB side, UB flag, UH dist, UB width,
			    UH clear, UH want_dist, UH want_clear)
{
	S5_OBST_MSG a = { type, side, flag, dist, width, clear };
	S5_OBST_MSG b;

	s5_obst_unpack(s5_obst_pack(&a), &b);
	return ((b.type == type) && (b.side == side) && (b.flag == flag) &&
		(b.dist_cm == want_dist) && (b.width_cm == width) &&
		(b.clear_cm == want_clear)) ? TRUE : FALSE;
}

/* ---- S5-U07 helper: a second task hammering another bit ---- */
#define RACE_LOOPS	2000
LOCAL ID	race_flg;
LOCAL ID	race_done;

LOCAL void race_task(INT stacd, void *exinf)
{
	INT i;

	(void)stacd; (void)exinf;
	for (i = 0; i < RACE_LOOPS; i++) {
		(void)tk_set_flg(race_flg, ESTOP_IMU_IMPACT);
		(void)tk_clr_flg(race_flg, ~ESTOP_IMU_IMPACT);
	}
	(void)tk_set_flg(race_flg, ESTOP_IMU_IMPACT);	/* leave it set */
	(void)tk_set_flg(race_done, 1U);
	tk_ext_tsk();
}

/* Build a scan from a list of (deg, cm) pairs */
LOCAL void t_scan(S5_SCAN *s, const W *deg_cm, INT n)
{
	INT i;

	s5_scan_clear(s);
	for (i = 0; i < n; i++) {
		s5_scan_put(s, deg_cm[2 * i], (UH)deg_cm[(2 * i) + 1]);
	}
}

EXPORT void subsystem5_test(void)
{
	S5_SCAN	s;
	S5_PLAN	p;
	TEST_INIT();

	/* S5-U01 pack/unpack: min, max, over-range (clamped to 511) */
	TEST_CHECK("S5-U01",
		   t_obst_roundtrip(S5_EVT_BYPASS_PLAN, S5_SIDE_RIGHT, 1U, 0U, 0U, 0U, 0U, 0U) &&
		   t_obst_roundtrip(S5_EVT_FAULT_CLEARED, S5_SIDE_LEFT, 1U, 511U, 255U, 511U, 511U, 511U) &&
		   t_obst_roundtrip(S5_EVT_OBSTACLE, S5_SIDE_NONE, 0U, 900U, 7U, 1000U, 511U, 511U) &&
		   t_obst_roundtrip(S5_EVT_SENSOR_FAULT, S5_SIDE_NONE, 0U, 14U, S5_FLT_NO_ECHO, 3U, 14U, 3U),
		   "obst_q pack/unpack round trip at 0, max (511) and over-range");

	/* S5-U02 scan_q command round trip */
	TEST_CHECK("S5-U02",
		   (s5_cmd_of(s5_cmd_pack(S5_CMD_START_SCAN, 0U)) == S5_CMD_START_SCAN) &&
		   (s5_cmd_of(s5_cmd_pack(S5_CMD_PATROL_ON, 255U)) == S5_CMD_PATROL_ON) &&
		   (s5_arg_of(s5_cmd_pack(S5_CMD_PATROL_ON, 255U)) == 255U),
		   "scan_q command pack/unpack");

	/* S5-U03 planner: 20 cm wide box, face 15 cm ahead (d = 15 / cos a out
	   to +-34 deg, corners read 18 cm out to +-40 deg because of the beam),
	   wall 30 cm to the right -> width within 20 %, go LEFT */
	{
		static const W box[] = {
			-60, 35, -55, 37, -50, 39, -45, 42,	/* wall on the right */
			-40, 18, -35, 18, -30, 17, -25, 17, -20, 16, -15, 16,
			-10, 15, -5, 15, 0, 15, 5, 15, 10, 15, 15, 16,
			20, 16, 25, 17, 30, 17, 35, 18, 40, 18,	/* the box */
			45, 511, 50, 511, 55, 511, 60, 511,	/* open on the left */
		};
		t_scan(&s, box, (INT)(sizeof(box) / sizeof(box[0]) / 2U));
		s5_plan(&s, &p);
		tm_printf((UB*)"[TEST] S5-U03 info: width=%d cm (true 20), clearL=%d clearR=%d\n",
			  p.width_cm, p.clear_left_cm, p.clear_right_cm);
		TEST_CHECK("S5-U03",
			   (p.result == S5_PLAN_BYPASS) && (p.side == S5_SIDE_LEFT) &&
			   (p.width_cm >= 16U) && (p.width_cm <= 24U) && (p.dist_cm == 15U),
			   "plan: 20 cm box -> width within 20 %, more room on the left");
	}

	/* S5-U04 planner: walls close on both sides -> BLOCKED */
	{
		static const W boxed[] = {
			-60, 22, -45, 20, -30, 40, -15, 15, 0, 15, 15, 15,
			30, 40, 45, 20, 60, 22,
		};
		t_scan(&s, boxed, (INT)(sizeof(boxed) / sizeof(boxed[0]) / 2U));
		s5_plan(&s, &p);
		TEST_CHECK("S5-U04", (p.result == S5_PLAN_BLOCKED) && (p.side == S5_SIDE_NONE),
			   "plan: no room either side -> NO_PATH");
	}

	/* S5-U05 planner: nothing within range -> CLEAR */
	{
		static const W open_scan[] = { -60, 511, -30, 120, 0, 90, 30, 511, 60, 200 };
		t_scan(&s, open_scan, (INT)(sizeof(open_scan) / sizeof(open_scan[0]) / 2U));
		s5_plan(&s, &p);
		TEST_CHECK("S5-U05", p.result == S5_PLAN_CLEAR, "plan: nothing near -> PATH_CLEAR");
	}

	/* S5-U19 planner with the servo's safe travel (+30 left .. -45 right):
	   slots past the limit are never measured, so that side's room is
	   unknown and must count as none.  (a) wall on the right -> NO_PATH;
	   (b) right side open -> go RIGHT. */
	{
		static const W limited_wall[] = {
			-45, 42, -40, 18, -35, 18, -30, 17, -25, 17, -20, 16, -15, 16,
			-10, 15, -5, 15, 0, 15, 5, 15, 10, 15, 15, 16, 20, 16, 25, 17, 30, 17,
		};
		static const W limited_open[] = {
			-45, 511, -40, 18, -35, 18, -30, 17, -25, 17, -20, 16, -15, 16,
			-10, 15, -5, 15, 0, 15, 5, 15, 10, 15, 15, 16, 20, 16, 25, 17, 30, 17,
		};
		S5_PLAN p2;

		t_scan(&s, limited_wall, (INT)(sizeof(limited_wall) / sizeof(limited_wall[0]) / 2U));
		s5_plan(&s, &p);
		t_scan(&s, limited_open, (INT)(sizeof(limited_open) / sizeof(limited_open[0]) / 2U));
		s5_plan(&s, &p2);
		TEST_CHECK("S5-U19",
			   (p.result == S5_PLAN_BLOCKED) && (p.clear_left_cm == 0U) &&
			   (p2.result == S5_PLAN_BYPASS) && (p2.side == S5_SIDE_RIGHT),
			   "plan: side past the servo limit counts as no room");
	}

	/* S5-U06 storage budget (Tribunal 1: ~240 -> 60 bytes) */
	tm_printf((UB*)"[TEST] S5-U06 info: scan table %d B, plan %d B, obst msg 4 B, cmd 4 B\n",
		  (INT)sizeof(S5_SCAN), (INT)sizeof(S5_PLAN));
	TEST_CHECK("S5-U06", (sizeof(S5_SCAN) + sizeof(S5_PLAN)) <= 64U,
		   "scan data fits the 60-byte budget (+ plan)");

	/* S5-U07 race stress: two tasks on two cores updating different bits
	   of one flag 2000 times each; no update may be lost (risk R3) */
	{
		T_CFLG cf = {0};
		UINT ptn = 0U;
		INT i;
		ID tid;

		cf.flgatr = TA_TFIFO | TA_WMUL;
		race_flg  = tk_cre_flg(&cf);
		race_done = tk_cre_flg(&cf);
		tid = app_start_task(race_task, S5_PRI_SCAN, 0, "s5_race");
		for (i = 0; i < RACE_LOOPS; i++) {
			(void)tk_set_flg(race_flg, ESTOP_OBSTACLE);
			(void)tk_clr_flg(race_flg, ~ESTOP_OBSTACLE);
		}
		(void)tk_set_flg(race_flg, ESTOP_OBSTACLE);
		(void)tk_wai_flg(race_done, 1U, TWF_ORW, &ptn, 5000);
		{
			T_RFLG r;
			(void)tk_ref_flg(race_flg, &r);
			TEST_CHECK("S5-U07", (tid > 0) &&
				   (r.flgptn == (ESTOP_OBSTACLE | ESTOP_IMU_IMPACT)),
				   "ESTOP bits: 2x2000 concurrent set/clear, no lost update");
		}
		(void)tk_del_flg(race_flg);
		(void)tk_del_flg(race_done);
	}

	/* S5-U08 microsecond timer runs (echo timing depends on it) */
	{
		UW t0 = gpio_irq_now_us();
		(void)tk_dly_tsk(50);
		UW dt = gpio_irq_now_us() - t0;
		TEST_CHECK("S5-U08", (dt >= 45000U) && (dt <= 70000U),
			   "1 MHz hardware timer: 50 ms delay reads 45-70 ms");
	}

	/* S5-U09 live: sensor answering (needs the HC-SR04+ connected).
	   Waits 1 s for the scan task, then reads its status word. */
	{
		UW st;
		(void)tk_dly_tsk(1000);
		st = s5_status_word();
		tm_printf((UB*)"[TEST] S5-U09 info: state=%s dist=%d faults=0x%02x\n",
			  s5_state_name(S5_STW_STATE(st)), S5_STW_DIST(st), S5_STW_FAULTS(st));
		TEST_CHECK("S5-U09",
			   ((S5_STW_FAULTS(st) & S5_FLT_STOP_MASK) == 0U) &&
			   (S5_STW_DIST(st) != S5_CM_NONE),
			   "ultrasonic answers: no sensor fault, a distance or 'open'");
	}

	TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */
