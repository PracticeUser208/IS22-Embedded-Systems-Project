/*
 *	subsystem2.c
 *	Subsystem 2 -- Motion Control: creates its kernel objects and
 *	starts its task.
 *
 *	Objects (record them in TEAM.md sections 5 and 6):
 *	  mbf_motion_cmd     message buffer  commands in (any task -> S2)
 *	  flg_motion         event flag      DONE / FAULT out (S2 -> any)
 *	  mtx_motion_status  mutex           guards g_motion_status
 *	  cyc_motion         cyclic handler  20 ms control tick
 *	  motion_task        task            owns motors + encoders
 */

#include "motion_private.h"

EXPORT ID		mbf_motion_cmd;
EXPORT ID		flg_motion;
EXPORT ID		mtx_motion_status;
EXPORT ID		cyc_motion;
EXPORT ID		tid_motion;
EXPORT motion_status_t	g_motion_status;

/* Each message in a message buffer also stores a small size header,
   rounded up to 8 bytes; leave room for it. */
#define MBF_MSG_OVERHEAD	8u

EXPORT ER subsystem2_init(void)
{
	T_CMBF	cmbf = {0};
	T_CFLG	cflg = {0};
	T_CMTX	cmtx = {0};
	T_CCYC	ccyc = {0};

	/* Message buffer: copies each command, so callers can pass a
	   local variable.  Holds MOTION_CMD_QUEUE_LEN commands. */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = (INT)(MOTION_CMD_QUEUE_LEN *
			    (sizeof(motion_cmd_t) + MBF_MSG_OVERHEAD));
	cmbf.maxmsz = (INT)sizeof(motion_cmd_t);
	mbf_motion_cmd = tk_cre_mbf(&cmbf);
	if (mbf_motion_cmd < E_OK)
	{
		return mbf_motion_cmd;
	}

	/* Event flag: several tasks may wait for DONE at once (TA_WMUL).
	   Starts with DONE set: at power-up nothing is in progress. */
	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = MOTION_FLG_DONE;
	flg_motion = tk_cre_flg(&cflg);
	if (flg_motion < E_OK)
	{
		return flg_motion;
	}

	/* Mutex with priority inheritance for the status snapshot */
	cmtx.mtxatr = TA_INHERIT;
	mtx_motion_status = tk_cre_mtx(&cmtx);
	if (mtx_motion_status < E_OK)
	{
		return mtx_motion_status;
	}

	/* Cyclic handler: wakes motion_task every MOTION_TICK_MS, so the
	   PID sample time is fixed however long one loop takes.  Created
	   stopped; started in subsystem2_start() once the task exists. */
	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)motion_cyclic_handler;
	ccyc.cyctim = MOTION_TICK_MS;
	ccyc.cycphs = MOTION_TICK_MS;
	cyc_motion = tk_cre_cyc(&ccyc);
	return (cyc_motion < E_OK) ? cyc_motion : E_OK;
}

EXPORT ER subsystem2_start(void)
{
	tid_motion = app_start_task(motion_task, S2_PRI_MOTION, 0,
				    "motion_task");
	if (tid_motion < E_OK)
	{
		return tid_motion;
	}
	return tk_sta_cyc(cyc_motion);
}
