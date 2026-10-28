/*
 *	subsystem1.c
 *	Subsystem 1 -- WiFi / MQTT / telemetry: creates its kernel objects
 *	and starts its tasks.  Called from usermain() (app_main.c).
 */

#include "subsystem1.h"
#include "comm_bridge.h"

EXPORT ID	tid_led;		/* IF-05 */
EXPORT ID	cyc_led;
EXPORT ID	mbf_telem_q;		/* IF-06 */
EXPORT ID	mbf_remote_cmd_q;	/* IF-07 */
EXPORT ID	mbf_s1_tx;		/* Private: frames queued for the MQTT client */
/* Milliseconds since power-on (operating time, never set by anyone) */
EXPORT UW s1_now_ms(void)
{
	SYSTIM	t;

	(void)tk_get_otm(&t);
	return (UW)t.lo;
}

/* Create one message buffer for `depth` messages of `msgsz` bytes */
LOCAL ID make_mbf(UW depth, UW msgsz)
{
	T_CMBF	cmbf = {0};

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = (SZ)S1_MBF_SIZE(depth, msgsz);
	cmbf.maxmsz = (INT)msgsz;
	return tk_cre_mbf(&cmbf);
}

EXPORT ER subsystem1_init(void)
{
	T_CCYC	ccyc = {0};

	mbf_telem_q = make_mbf(S1_TELEM_Q_DEPTH, sizeof(telem_snapshot_t));
	if (mbf_telem_q < E_OK) {
		return mbf_telem_q;
	}

	mbf_remote_cmd_q = make_mbf(S1_CMD_Q_DEPTH, sizeof(remote_cmd_t));
	if (mbf_remote_cmd_q < E_OK) {
		return mbf_remote_cmd_q;
	}
	
	
	mbf_s1_tx = make_mbf(S1_TX_Q_DEPTH, S1_TX_MSG_MAX);
	if (mbf_s1_tx < E_OK) {
		return mbf_s1_tx;
	}

	/* LED example (template): cyclic handler wakes led_task */
	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)led_cyclic_handler;
	ccyc.cyctim = S1_TICK_MS;
	ccyc.cycphs = S1_TICK_MS;
	cyc_led = tk_cre_cyc(&ccyc);
	return (cyc_led < E_OK) ? cyc_led : E_OK;
}

EXPORT ER subsystem1_start(void)
{
	ID	tid;

	tid_led = app_start_task(led_task, S1_PRI_LED, 0, "led_task");
	if (tid_led < E_OK) {
		return tid_led;
	}

	tid = app_start_task(comm_task, S1_PRI_COMM, 0, "comm_task");
	if (tid < E_OK) {
		return tid;
	}

#if S1_SIM_VC
	tid = app_start_task(sim_vc_task, S1_PRI_SIM_VC, 0, "sim_vc_task");
	if (tid < E_OK) {
		return tid;
	}
#endif

	return tk_sta_cyc(cyc_led);
}