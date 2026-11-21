/*
 *	subsystem4_test.c
 *	On-target tests for subsystem 4.  Built only with: make TEST=1
 *	Record unit results in testing/subsystem4/TEST_LOG.md and the
 *	INT- result in testing/integration/TEST_LOG.md.
 *
 *	These check the connection of Buddy 4's IMU code to the car (the
 *	kernel port).  Buddy 4's own logic tests run on a PC with his fake
 *	port (not in the repository yet).
 */

#include "subsystem4.h"
#include "../test_report.h"

#if APP_TEST

#if !TEMPLATE_DEMO
#include "imu.h"
#include "subsystem4.h"

LOCAL const char *const s4_state_names[4] = { "INIT", "CALIBRATE", "MONITORING", "FAULT" };
#endif

EXPORT void subsystem4_test(void)
{
#if !TEMPLATE_DEMO
	imu_stats_t	s0;
	imu_stats_t	s1;
	imu_state_t	st;
	uint32_t	a;
	uint32_t	b;
	UW		dpolls;
	INT		k;
	fault_t	f;
	bool	blocking;
	int	i;

	TEST_INIT();

	/* S4-P01 (live): the 10 ms poll runs: about 100 polls in 1 s */
	s0 = imu_get_stats();
	(void)tk_dly_tsk(1000);
	s1 = imu_get_stats();
	dpolls = (UW)(uint16_t)(s1.polls - s0.polls);
	TEST_CHECK("S4-P01", (dpolls >= 90U) && (dpolls <= 110U),
		   "IMU polled every 10 ms (cyclic handler)");
	tm_printf((UB*)"[TEST] S4-P01 polls in 1 s: %d\n", (INT)dpolls);

	/* S4-P02 (live): calibrated and monitoring within 3 s, or a reported
	   fault (no IMU connected) -- never stuck in INIT/CALIBRATE */
	for (k = 0; k < 30; k++) {
		st = imu_get_state();
		if ((st == IMU_ST_MONITORING) || (st == IMU_ST_FAULT)) {
			break;
		}
		(void)tk_dly_tsk(100);
	}
	st = imu_get_state();
	TEST_CHECK("S4-P02", (st == IMU_ST_MONITORING) || (st == IMU_ST_FAULT),
		   "IMU leaves INIT/CALIBRATE (MONITORING, or FAULT if not connected)");
	tm_printf((UB*)"[TEST] S4-P02 IMU state %s, fault bits 0x%02x\n",
		  s4_state_names[(UINT)st & 3U], (INT)imu_get_fault_mask());

	/* S4-P03 (live): Motion Control's snapshot is fresh (its sequence
	   moves every control tick, 20 ms) */
	a = s4_mc_snapshot();
	(void)tk_dly_tsk(100);
	b = s4_mc_snapshot();
	TEST_CHECK("S4-P03", (a >> 26) != (b >> 26),
		   "Motion Control speed/distance snapshot updates");

	/* S4-U03 (IMU): repeated I2C NACKs raise a blocking NACK fault */
	fault_init(&f);
	blocking = FALSE;
	for (i = 0; i < NACK_POLLS_TO_FAULT; i++) {
		blocking = fault_on_read(&f, RD_NACK, NULL);
	}
	TEST_CHECK("S4-U03", blocking && (f.mask & IMU_FAULT_NACK),
		   "NACK polls raise IMU_FAULT_NACK");

	/* INT-04 (IMU): the IMU task is running, so it has left INIT */
	TEST_CHECK("INT-04", imu_get_state() != IMU_ST_INIT,
		   "IMU task started and left INIT");

	TEST_SUMMARY("subsystem4");
#else
	T_RMBF	rmbf;
	UW	before;

	TEST_INIT();

	/* S4-U01: the message buffer accepts the documented line length */
	TEST_CHECK("S4-U01",
		   tk_ref_mbf(mbf_summary, &rmbf) == E_OK &&
		   rmbf.maxmsz == S4_SUMMARY_MAX,
		   "message buffer max line = S4_SUMMARY_MAX");

	/* INT-02 (integration, IF-03): summary lines from subsystem 3 reach
	   the monitor.  One line per 5 samples = one per 1.5 s. */
	before = s4_lines_received;
	tk_dly_tsk(3500);
	TEST_CHECK("INT-02", s4_lines_received >= before + 2,
		   "summary lines flow from subsystem 3 to subsystem 4");

	TEST_SUMMARY("subsystem4");
#endif /* TEMPLATE_DEMO */
}

#endif /* APP_TEST */
