/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2 (Motion Control).
 *	Built only with: make TEST=1.  Record results in
 *	testing/subsystem2/TEST_LOG.md.
 *
 *	The tests run at power-up, so by default NONE of them moves the
 *	car: they test the maths, the argument checks and the command
 *	path.  Wheel tests need the car on a stand with wheels off the
 *	ground: build with S2_BENCH_TEST set to 1 below.
 */

#include "motion_private.h"
#include "motion_config.h"
#include "motion_math.h"
#include "../test_report.h"

#if APP_TEST

#define S2_BENCH_TEST	0	/* 1 = also spin the wheels (car on a stand!) */

static bool near(float a, float b, float tol)
{
	float d = a - b;
	return (d < tol) && (d > -tol);
}

static void test_math(INT *p_pass, INT *p_fail)
{
	TEST_INIT();
	motion_pid_t	pid;
	float		v;
	float		prev;
	uint32_t	k;
	bool		b_ok;

	/* S2-U01: 16-bit encoder counter wrap */
	TEST_CHECK("S2-U01",
		   (motion_count_delta(3u, 65534u) == 5u) &&
		   (motion_count_delta(100u, 90u) == 10u) &&
		   (motion_count_delta(0u, 0u) == 0u),
		   "encoder delta correct across 0xFFFF->0 wrap");

	/* S2-U02: counts <-> mm round trip, one revolution = pi * D */
	TEST_CHECK("S2-U02",
		   (motion_mm_to_counts(motion_counts_to_mm(100)) == 100) &&
		   (motion_mm_to_counts(motion_counts_to_mm(-37)) == -37) &&
		   near(motion_counts_to_mm((int32_t)ENC_COUNTS_PER_REV),
			MOTION_PI * WHEEL_DIAMETER_MM, 0.5f),
		   "counts/mm conversion and one wheel revolution");

	/* S2-U03: pivot arc, 360 deg = pi * track, 90 deg = a quarter */
	TEST_CHECK("S2-U03",
		   near(motion_turn_arc_mm(360.0f), MOTION_PI * TRACK_WIDTH_MM, 0.5f) &&
		   near(motion_turn_arc_mm(90.0f) * 4.0f, motion_turn_arc_mm(360.0f), 0.5f),
		   "turn arc length");

	/* S2-U04: PID output limits and anti-windup */
	pid_init(&pid, 0.01f, 0.5f, 0.0f, 0.3f, -1.0f, 1.0f);
	for (k = 0u; k < 500u; k++)
	{
		v = pid_update(&pid, 1000.0f, 0.0f, 0.02f);
	}
	b_ok = near(v, 1.0f, 0.001f) && near(pid.integral, 0.3f, 0.001f);
	pid_reset(&pid);
	v = pid_update(&pid, -1000.0f, 0.0f, 0.02f);
	TEST_CHECK("S2-U04", b_ok && (v < 0.0f) && (v >= -1.0f),
		   "PID clamps output and integral (anti-windup)");

	/* S2-U05: profile ramps, caps at cruise, slows near target */
	prev = 0.0f;
	b_ok = true;
	for (k = 0u; k < 100u; k++)
	{
		v = motion_profile_speed(prev, 200.0f, 1000.0f, 0.02f);
		if ((v > 200.0f) || (v < MOTION_MIN_SPEED_MM_S) ||
		    ((v - prev) > ((MOTION_ACCEL_MM_S2 * 0.02f) + MOTION_MIN_SPEED_MM_S)))
		{
			b_ok = false;
		}
		prev = v;
	}
	TEST_CHECK("S2-U05",
		   b_ok && near(prev, 200.0f, 0.01f) &&
		   (motion_profile_speed(200.0f, 200.0f, 5.0f, 0.02f) < 100.0f),
		   "speed profile: ramp, cruise cap, slow-down");

	*p_pass += test_pass_;
	*p_fail += test_fail_;
}

static void test_api(INT *p_pass, INT *p_fail)
{
	TEST_INIT();
	motion_status_t	st;

	/* S2-U06: bad arguments are rejected and nothing is sent */
	TEST_CHECK("S2-U06",
		   (motion_move_forward(0, 0) == E_PAR) &&
		   (motion_move_forward(-10, 0) == E_PAR) &&
		   (motion_move_forward(MOTION_MAX_DIST_MM + 1, 0) == E_PAR) &&
		   (motion_move_backward(100, MOTION_MAX_SPEED_MM_S + 1) == E_PAR) &&
		   (motion_turn_left(0) == E_PAR) &&
		   (motion_turn_right(MOTION_MAX_ANGLE_DEG + 1) == E_PAR) &&
		   (motion_set_velocity(MOTION_MAX_SPEED_MM_S + 1, 0) == E_PAR) &&
		   (motion_get_status(NULL) == E_PAR),
		   "API rejects out-of-range arguments");

	/* S2-U07: idle at power-up, status readable */
	TEST_CHECK("S2-U07",
		   (motion_get_status(&st) == E_OK) &&
		   (st.state == MOTION_STATE_IDLE) &&
		   (st.left_duty_permille == 0) && (st.right_duty_permille == 0),
		   "idle with motors off at power-up");

	/* S2-U08: command path API -> mbf -> motion_task -> flag works */
	TEST_CHECK("S2-U08",
		   (motion_stop() == E_OK) &&
		   (motion_wait_done(200) == E_OK),
		   "stop command reaches motion_task and DONE is signalled");

#if S2_BENCH_TEST
	{
		uint32_t	before;
		uint32_t	k;

		/* S2-U09: wheels turn and encoders count (car on stand) */
		(void)motion_get_status(&st);
		before = st.left_count + st.right_count;
		for (k = 0u; k < 25u; k++)	/* 1 s, refreshed every 40 ms */
		{
			(void)motion_set_velocity(150, 150);
			tk_dly_tsk(40);
		}
		(void)motion_get_status(&st);
		TEST_CHECK("S2-U09",
			   (st.left_count + st.right_count) > (before + 4u),
			   "both wheels turn and the encoders count");

		/* S2-U10: velocity watchdog stops a silent caller */
		tk_dly_tsk(MOTION_VEL_TIMEOUT_MS + 100);
		(void)motion_get_status(&st);
		TEST_CHECK("S2-U10", st.state == MOTION_STATE_IDLE,
			   "velocity watchdog stops the car");
	}
#endif

	*p_pass += test_pass_;
	*p_fail += test_fail_;
}

EXPORT void subsystem2_test(void)
{
	INT pass = 0;
	INT fail = 0;

	/* Give motion_task time to initialise its hardware first */
	tk_dly_tsk(100);

	test_math(&pass, &fail);
	test_api(&pass, &fail);

	tm_printf((UB *)"[TEST] subsystem2 SUMMARY %d passed, %d failed\n",
		  pass, fail);
}

#endif /* APP_TEST */
