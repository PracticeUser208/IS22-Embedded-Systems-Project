/*
 *	task_monitor.c
 *	Subsystem 4 -- the monitor task and its alarm handler.
 *
 *	APIs shown: tk_sta_alm, tk_stp_alm, alarm handler, tk_wai_flg,
 *	            tk_rcv_mbf, tk_loc_mtx/tk_unl_mtx, tk_ref_sem, tk_ref_mpf,
 *	            tk_ref_tsk, tk_sus_tsk, tk_rsm_tsk, tk_chg_pri,
 *	            tk_get_tid, tk_get_tim
 */

#include "subsystem4.h"

#if TEMPLATE_DEMO	/* template example, off: see subsystem4.h */
#include "../subsystem1/subsystem1.h"	/* IF-05: tid_led */
#include "../subsystem2/subsystem2.h"	/* sem_slots, mpf_samples (read only) */
#include "../subsystem3/subsystem3.h"	/* IF-04: app_stats, mtx_stats */

EXPORT UW	s4_lines_received;

/*
 * Alarm handler: one-shot, interrupt context (same rules as a cyclic
 * handler: keep it short, never wait).
 */
EXPORT void monitor_alarm_handler(void *exinf)
{
	(void)exinf;
	tk_set_flg(flg_events, FLG_ALARM);
}

LOCAL const char *state_name(UINT tskstat)
{
	switch (tskstat) {
	case TTS_RUN:	return "RUN";
	case TTS_RDY:	return "READY";
	case TTS_WAI:	return "WAIT";
	case TTS_SUS:	return "SUSPEND";
	case TTS_WAS:	return "WAIT+SUSPEND";
	case TTS_DMT:	return "DORMANT";
	default:	return "?";
	}
}

EXPORT void monitor_task(INT stacd, void *exinf)
{
	UINT		flgptn;
	UB		text[S4_SUMMARY_MAX + 1];
	INT		len;
	APP_STATS	snap;
	T_RSEM		rsem;
	T_RMPF		rmpf;
	T_RTSK		rtsk;
	SYSTIM		now;
	UW		reports = 0;
	BOOL		led_paused = FALSE;
	ER		er;

	(void)stacd; (void)exinf;

	tm_printf((UB*)"monitor: I am task %d\n", tk_get_tid());

	while (1) {
		/* 1. Arm the one-shot alarm, then wait for EITHER bit.
		      TWF_ORW = wake on any of the bits, TWF_CLR = clear them
		      on wake-up.  The extra timeout is just a safety net. */
		tk_sta_alm(alm_monitor, S4_ALARM_MS);
		er = tk_wai_flg(flg_events, FLG_ALARM | FLG_BATCH,
				TWF_ORW | TWF_CLR, &flgptn, S4_ALARM_MS * 2);
		tk_stp_alm(alm_monitor);	/* cancel it if a batch came first */

		if (er == E_TMOUT) {
			tm_printf((UB*)"monitor: nothing at all happened?\n");
			continue;
		}
		if (flgptn & FLG_ALARM) {
			tm_printf((UB*)"monitor: alarm -- no batch for %d ms\n",
				  S4_ALARM_MS);
		}

		/* 2. Drain the message buffer without waiting (TMO_POL).
		      tk_rcv_mbf returns the message length, or an error. */
		while ((len = tk_rcv_mbf(mbf_summary, text, TMO_POL)) > 0) {
			text[len] = '\0';
			s4_lines_received++;
			tm_printf((UB*)"monitor: batch  %s\n", text);
		}

		/* 3. Copy the stats under subsystem 3's mutex, print outside
		      it: hold a mutex for as short a time as possible. */
		tk_loc_mtx(mtx_stats, TMO_FEVR);
		snap = app_stats;
		tk_unl_mtx(mtx_stats);

		tk_get_tim(&now);
		tk_ref_sem(sem_slots, &rsem);
		tk_ref_mpf(mpf_samples, &rmpf);
		tm_printf((UB*)"monitor: t=%u ms  samples=%u  min=%d max=%d avg=%d"
			  "  free slots=%d  free blocks=%d\n",
			  now.lo, snap.received, snap.min, snap.max,
			  snap.received ? (INT)(snap.sum / (W)snap.received) : 0,
			  rsem.semcnt, (INT)rmpf.frbcnt);

		/* 4. Every few reports, pause or resume subsystem 1's LED task
		      (IF-05) and look at it with tk_ref_tsk. */
		if (++reports % S4_LED_TOGGLE_REPORTS == 0) {
			if (led_paused) {
				tk_rsm_tsk(tid_led);
				tk_chg_pri(tid_led, S1_PRI_LED);	/* back to normal */
			} else {
				tk_sus_tsk(tid_led);
				/* Changing the priority of a task: */
				tk_chg_pri(tid_led, S1_PRI_LED + 1);
			}
			led_paused = !led_paused;

			tk_ref_tsk(tid_led, &rtsk);
			tm_printf((UB*)"monitor: LED task %s, state=%s, pri=%d\n",
				  led_paused ? "paused" : "resumed",
				  state_name(rtsk.tskstat), rtsk.tskpri);
		}
	}
}

#endif /* TEMPLATE_DEMO */

/* ======== from imu_mc_snap.c ======== */
/*
 *	imu_mc_snap.c
 *	Subsystem 4 -- Motion Control's speed / distance for the IMU (see
 *	imu_mc_snap.h).  Reads Motion Control's public status with
 *	motion_get_status(); Motion Control stays the single writer.
 */

#include "../subsystem2/subsystem2.h"

#define SNAP_SPEED_MAX		1023
#define BRAKE_MARGIN_MM_S	20	/* target this much below the speed */

LOCAL uint32_t	snap_last;
LOCAL bool	snap_braking;

LOCAL W abs_w(W v)
{
	return (v < 0) ? -v : v;
}

EXPORT uint32_t s4_mc_snapshot(void)
{
	motion_status_t	ms;
	T_RFLG		rflg;
	W		speed;
	W		target;
	bool		estop = false;

	if (motion_get_status(&ms) != E_OK) {
		return snap_last;		/* sequence stops: data goes stale */
	}

	speed  = (abs_w(ms.left_speed_mm_s) + abs_w(ms.right_speed_mm_s)) / 2;
	target = (abs_w(ms.left_target_mm_s) + abs_w(ms.right_target_mm_s)) / 2;
	if (speed > SNAP_SPEED_MAX) {
		speed = SNAP_SPEED_MAX;
	}
	if ((flg_estop_req > 0) && (tk_ref_flg(flg_estop_req, &rflg) == E_OK)) {
		estop = (rflg.flgptn != 0U);	/* Motion Control brakes on any bit */
	}

	snap_braking = estop ||
		       ((ms.state == MOTION_STATE_IDLE) && (speed > 0)) ||
		       (target < (speed - BRAKE_MARGIN_MM_S));
	snap_last = S4_SNAP_PACK((uint32_t)speed, (uint32_t)ms.odometer_mm, ms.seq);
	return snap_last;
}

EXPORT bool s4_mc_braking(void)
{
	return snap_braking;
}
