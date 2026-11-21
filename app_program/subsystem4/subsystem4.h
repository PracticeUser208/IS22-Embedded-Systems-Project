/*
 *	subsystem4.h
 *	Subsystem 4 -- IMU and Terrain (Buddy 4).  Student ID: 2503710.
 *
 *	Public interface of subsystem 4.  Other subsystems include it as
 *	"../subsystem4/subsystem4.h".
 *
 *	Buddy 4's IMU code (imu.c, imu_logic.c, imu_cfg.h, app_types.h,
 *	app_fields.h; unchanged) runs in imu_task.  It reaches the hardware
 *	and the kernel only through imu_port.h, implemented for this kernel
 *	in imu_port_pico_utk.c:
 *	  I2C1 on GP2/GP3 (drv_i2c1.c)           LSM303D accelerometer, 0x1D
 *	  10 ms poll from a cyclic handler, 1 us timestamps (Tribunal 1)
 *	  imu_evt_q -> Vehicle Controller        hump / motion / lifted / fault
 *	  imu_q     <- Vehicle Controller        RESET (recalibrate)
 *	  flg_estop_req                          sets ESTOP_IMU_LIFTED / _FAULT
 *	  flg_sysfault                           sets / clears FAULT_IMU
 *	  Motion Control status (imu_mc_snap.c)  speed, distance for humps
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"

/* ---------------------------------------------------------------------
 *  Template example switch.  The template's example chain (producer in
 *  subsystem 2 -> consumer in subsystem 3 -> monitor in subsystem 4)
 *  stopped working when Motion Control replaced the producer, so it is
 *  switched off.  0 = the IMU runs here.  The example code stays in
 *  task_monitor.c, inside #if TEMPLATE_DEMO, for reference.
 * --------------------------------------------------------------------- */
#ifndef TEMPLATE_DEMO
#define TEMPLATE_DEMO		0
#endif

/* ---------------------------------------------------------------------
 *  IMU
 * --------------------------------------------------------------------- */
/* Priority band 20-21 (TEAM.md section 4).  Tribunal 1 change log:
   "Obstacle Scan is given a higher priority than IMU", so the bands of
   subsystems 4 and 5 were swapped (Obstacle Scan is now 18-19). */
#define S4_PRI_IMU		20
#define S4_POLL_MS		10	/* = IMU_POLL_PERIOD_US (imu_cfg.h) */

/* IF-15: VC -> IMU, 4-byte words: the command (IMU_CMD_RESET = 0).
   Owned here. */
IMPORT ID	imu_q;

/* Private to subsystem 4 */
IMPORT ID	tid_imu;
IMPORT ID	cyc_imu;
IMPORT void	imu_task(INT stacd, void *exinf);
IMPORT void	imu_cyclic_handler(void *exinf);

#if TEMPLATE_DEMO
/* ---------------------------------------------------------------------
 *  Template example (off)
 * --------------------------------------------------------------------- */
#define S4_PRI_MONITOR		18	/* priority band 18-19 (TEAM.md) */
#define S4_ALARM_MS		2000	/* report anyway if nothing for 2 s */
#define S4_LED_TOGGLE_REPORTS	6	/* pause/resume the LED every N reports */

/* IF-03: send one text line (at most S4_SUMMARY_MAX bytes, no '\0'
   needed) to mbf_summary, then set FLG_BATCH in flg_events. */
#define S4_SUMMARY_MAX		48
#define FLG_BATCH		0x0002	/* set by subsystem 3 */
#define FLG_ALARM		0x0001	/* private: set by our alarm handler */

IMPORT ID	mbf_summary;
IMPORT ID	flg_events;

IMPORT ID	alm_monitor;
IMPORT UW	s4_lines_received;	/* for the tests */
IMPORT void	monitor_task(INT stacd, void *exinf);
IMPORT void	monitor_alarm_handler(void *exinf);
#endif /* TEMPLATE_DEMO */

/* ======== from imu_mc_snap.h ======== */
/*
 *	imu_mc_snap.h
 *	Subsystem 4 -- Motion Control's speed / distance for the IMU
 *	(Tribunal 1: the IMU needs "read-only timestamped speed and distance
 *	from Motion Control" for hump height = pitch x encoder distance).
 *
 *	Kernel-free on purpose: imu_port_pico_utk.c includes it next to
 *	Buddy 4's app_types.h, and subsystem2.h cannot sit next to that
 *	header (both define motion_state_t).  The word layout is Buddy 4's
 *	MC_SNAP_PACK (app_types.h); imu_port_pico_utk.c checks they match.
 */

#ifndef IMU_MC_SNAP_H
#define IMU_MC_SNAP_H

#include <stdint.h>
#include <stdbool.h>

/* [9:0] speed mm/s   [25:10] distance mm (wraps)   [31:26] sequence */
#define S4_SNAP_PACK(speed, dist, seq) \
	((uint32_t)(((uint32_t)(speed) & 0x3FFu) | (((uint32_t)(dist) & 0xFFFFu) << 10) | \
		    (((uint32_t)(seq) & 0x3Fu) << 26)))

/* Read Motion Control's status once (bounded wait) and return the
   snapshot word.  If the status cannot be read, the previous word is
   returned unchanged, so its sequence stops and the IMU marks the wheel
   data stale (SNAP_STALE_POLLS). */
uint32_t s4_mc_snapshot(void);

/* From the same read: Motion Control is bringing the car to a stop
   (stop / brake / an ESTOP bit set / slowing towards a lower target), so
   the IMU must not count the deceleration as an impact vote.  (Not used
   since 16 Nov: impact voting was replaced by lifted / overturned.) */
bool s4_mc_braking(void);

#endif /* IMU_MC_SNAP_H */

/* ======== from drv_i2c1.h ======== */
/*
 *	drv_i2c1.h
 *	Subsystem 4 -- I2C1 master for the IMU (Week 6 pin map):
 *	  GP2 = I2C1 SDA, GP3 = I2C1 SCL  (Robo Pico Grove 2 / Maker port)
 *	Owned only by the IMU task (Week 6: "Owned only by IMU Task").
 *
 *	Why a driver here: the kernel's I2C driver is switched off in the
 *	shared config (DEVCNF_USE_IIC 0), its I2C1 pins are GP6/GP7, and
 *	turning it on would also start I2C0 on pins the car uses.  The
 *	README says drivers belong in the subsystem's folder, so this is a
 *	small register-level driver that behaves like the pico-sdk blocking
 *	calls Buddy 4's bring-up port uses (same timeouts, same bus recovery).
 *	Every wait is bounded by I2C1_TIMEOUT_US.
 */

#ifndef DRV_I2C1_H
#define DRV_I2C1_H

#include "../app.h"

#define I2C1_SDA_PIN		2U
#define I2C1_SCL_PIN		3U
#define I2C1_HZ			400000U	/* as Buddy 4's bring-up port */
#define I2C1_TIMEOUT_US		2000U	/* per transfer, as the bring-up */

#define I2C1_OK			0
#define I2C1_NACK		1	/* no ACK, or the transfer aborted */
#define I2C1_TIMEOUT		2	/* bus held / no progress in time  */

IMPORT void	i2c1_init(void);	/* pins + controller, 400 kHz master */
IMPORT INT	i2c1_write(UB addr, const UB *p_src, INT len, BOOL nostop);
IMPORT INT	i2c1_read(UB addr, UB *p_dst, INT len, BOOL nostop);
IMPORT void	i2c1_recover(void);	/* clock out a stuck bus, re-init */
IMPORT UW	i2c1_now_us(void);	/* 1 MHz hardware timer, low word */

#endif /* DRV_I2C1_H */


#endif /* SUBSYSTEM4_H */
