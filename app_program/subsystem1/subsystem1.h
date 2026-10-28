/*
 *	subsystem1.h
 *	Subsystem 1 -- WiFi / MQTT communication, command and telemetry.
 *	Buddy 1, student ID 2500510.
 *
 *	This header is subsystem 1's PUBLIC interface: only what other
 *	subsystems are allowed to use.  Other subsystems include it as
 *	"../subsystem1/subsystem1.h".  Data formats: telem_format.h.
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"
#include "telem_format.h"

/* ---- Build switch ---------------------------------------------------
 * 1 = run a SIMULATED Vehicle Controller inside subsystem 1, so the
 *     telemetry and command path can be shown before integration.
 * 0 = the real Vehicle Controller posts to telem_q and reads remote_cmd_q.
 */
#ifndef S1_SIM_VC
#define S1_SIM_VC		1
#endif

/* ---- Priorities: subsystem 1 band 12-13 (TEAM.md section 4) -------- */
#define S1_PRI_LED		12	/* template LED example, kept for IF-05 */
#define S1_PRI_SIM_VC		12	/* simulated VC (only if S1_SIM_VC)     */
#define S1_PRI_COMM		13	/* Telemetry / Communications task      */

#define S1_LED_PIN		19	/* GP19: LED + 330R resistor to GND */
#define S1_TICK_MS		250	/* LED toggles every tick */

/* ---- Timing -------------------------------------------------------- */
#define S1_TELEM_PERIOD_MS	1000	/* FR21: VC posts >= 1 snapshot/s */
#define S1_HB_PERIOD_MS		5000	/* FR23: heartbeat every 5 s      */

/* ---- Queue depths -------------------------------------------------- */
#define S1_TELEM_Q_DEPTH	4	/* snapshots; poster drops if full */
#define S1_CMD_Q_DEPTH		4	/* decoded remote commands         */
#define S1_TX_Q_DEPTH		4	/* frames waiting for the MQTT client */

/* Message-buffer size in bytes.  The kernel stores a 4-byte header with
   every message and rounds the message up to 4 bytes (messagebuf.c). */
#define S1_MBF_SIZE(depth, msgsz) \
	((depth) * (4u + ((((UW)(msgsz)) + 3u) & ~3u)))

/* IF-06  VC -> S1: telemetry snapshots (telem_snapshot_t, 16 B).
 *   Owned by S1 (the receiver).  ONE writer only: the Vehicle Controller.
 *   Post:  tk_snd_mbf(mbf_telem_q, &snap, sizeof snap, TMO_POL)
 *          never blocks; E_TMOUT means full, so drop and count it.     */
IMPORT ID	mbf_telem_q;

/* IF-07  S1 -> VC: decoded remote commands (remote_cmd_t, 1 B).
 *   Created by S1 for now; moves to the VC owner once its folder exists.
 *   Read:  tk_rcv_mbf(mbf_remote_cmd_q, &cmd, tmout)                    */
IMPORT ID	mbf_remote_cmd_q;

/* Private: frames queued for the MQTT client (comm_bridge.h) */
IMPORT ID	mbf_s1_tx;

/* IF-05 (template example): the LED task's ID, so S4 may pause/resume it */
IMPORT ID	tid_led;

/* ---- Private to subsystem 1, shared between its own .c files ------- */
IMPORT ID	cyc_led;
IMPORT void	led_task(INT stacd, void *exinf);
IMPORT void	led_cyclic_handler(void *exinf);
IMPORT void	comm_task(INT stacd, void *exinf);
IMPORT UW	s1_now_ms(void);
#if S1_SIM_VC
IMPORT void	sim_vc_task(INT stacd, void *exinf);
#endif

#endif /* SUBSYSTEM1_H */