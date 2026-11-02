/*
 *	subsystem4_test.c
 *	On-target tests for subsystem 4.  Built only with: make TEST=1
 *	Record unit results in testing/subsystem4/TEST_LOG.md and the
 *	INT- result in testing/integration/TEST_LOG.md.
 */

#include "subsystem4.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem4_test(void)
{
	T_RMBF	rmbf;
	UW	before;
	fault_t	f;
	bool	blocking;
	int	i;

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

	/* S4-U02 (IMU): 2-of-3 voting -- one vote alone must not confirm an impact */
	TEST_CHECK("S4-U02",
		   !vote_confirmed(VOTE_ACCEL) &&
		   vote_confirmed(VOTE_ACCEL | VOTE_DECEL),
		   "impact needs 2 distinct votes");

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
}

#endif /* APP_TEST */