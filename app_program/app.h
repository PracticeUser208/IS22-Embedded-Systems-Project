/*
 *	app.h
 *	Definitions shared by the whole team.  Changed only by team agreement.
 *
 *	Anything that belongs to ONE subsystem goes in that subsystem's own
 *	header instead (app_program/subsystemN/subsystemN.h).
 */

#ifndef APP_H
#define APP_H

#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include <tk/tkernel.h>		/* micro T-Kernel API (tk_xxx_yyy) */
#include <tm/tmonitor.h>	/* T-Monitor API: tm_printf() console output */
#include <bsp/libbsp.h>		/* Board helpers: gpio_set_pin/val, pwm_... */

/* ---------------------------------------------------------------------
 *  Pins: free for projects are GP0..GP22 and GP26..GP28.  GP0/GP1 are
 *  taken by UART0 when built with "make CRASH_REPORT=1" (see README).
 *  Never use GP23/24/25/29: they are wired to the Pico W radio.
 *  Claim every pin in TEAM.md section 2 before using it.
 * --------------------------------------------------------------------- */

/* ---------------------------------------------------------------------
 *  Task priorities: 1 is the highest; the kernel's USB console (3) and
 *  WiFi (4) tasks sit above every application task.  Each subsystem
 *  has its own band (TEAM.md section 4), defined in its own header.
 * --------------------------------------------------------------------- */
#define TASK_STACK_SIZE		2048	/* bytes; tm_printf needs ~1 KB */

/* ---------------------------------------------------------------------
 *  Team-wide stop and fault flags.  Bit values: shared_bits.h.
 *    flg_estop_req  ESTOP_REQ.  Created by Motion Control (subsystem 2).
 *                   IMU and Obstacle Scan set their own bit; Motion
 *                   Control brakes while any bit is set; only the
 *                   Vehicle Controller clears bits.
 *    flg_sysfault   shared fault mask.  Each subsystem sets and clears
 *                   only its own FAULT_xxx bit.  Until a Vehicle
 *                   Controller creates it, subsystem 5 makes a stand-in.
 *  Always use tk_set_flg / tk_clr_flg / tk_ref_flg: the kernel makes
 *  each update atomic on both cores (risk R3).
 * --------------------------------------------------------------------- */
#include "shared_bits.h"

IMPORT ID flg_estop_req;
IMPORT ID flg_sysfault;

/* Create and start a task.  prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or
   0 for "any core".  Defined in app_main.c. */
IMPORT ID app_start_task(FP entry, PRI pri, UW prc, const char *name);

/* ---------------------------------------------------------------------
 *  Every subsystem provides these.  usermain() calls ALL the _init()
 *  functions first and only then ALL the _start() functions, so every
 *  kernel object exists before any task can use it.
 *    subsystemN_init()   create this subsystem's kernel objects
 *    subsystemN_start()  create and start this subsystem's tasks
 *    subsystemN_test()   on-target tests, only in test builds (make TEST=1)
 * --------------------------------------------------------------------- */
IMPORT ER subsystem1_init(void);
IMPORT ER subsystem2_init(void);
IMPORT ER subsystem3_init(void);
IMPORT ER subsystem4_init(void);
IMPORT ER subsystem5_init(void);

IMPORT ER subsystem1_start(void);
IMPORT ER subsystem2_start(void);
IMPORT ER subsystem3_start(void);
IMPORT ER subsystem4_start(void);
IMPORT ER subsystem5_start(void);

/* ---------------------------------------------------------------------
 *  Vehicle Controller (app_program/vehicle_ctrl/, owned by the whole
 *  team).  1 = the real VC runs the system statechart and the two
 *  bench stand-ins are switched off: Telemetry's simulated VC
 *  (S1_SIM_VC) and Obstacle Scan's stub VC (S5_STUB_VC).
 *  0 = no VC; each subsystem's stand-in runs instead.
 * --------------------------------------------------------------------- */
#ifndef VEHICLE_CONTROLLER
#define VEHICLE_CONTROLLER	1
#endif
#if VEHICLE_CONTROLLER
#define S1_SIM_VC		0
#define S5_STUB_VC		0
IMPORT ER vc_init(void);	/* called BEFORE the subsystems' _init() */
IMPORT ER vc_start(void);	/* called AFTER the subsystems' _start() */
#endif

#if APP_TEST
IMPORT void subsystem1_test(void);
IMPORT void subsystem2_test(void);
IMPORT void subsystem3_test(void);
IMPORT void subsystem4_test(void);
IMPORT void subsystem5_test(void);
#if VEHICLE_CONTROLLER
IMPORT void vc_test(void);
#endif
#endif

/* Print a readable message if a kernel call failed (negative error code) */
#define CHECK(er, what)							\
	do {								\
		INT check_er_ = (INT)(er);	/* evaluate once */	\
		if (check_er_ < E_OK) {					\
			tm_printf((UB*)"ERROR %s: %d\n", (what), check_er_); \
		}							\
	} while (0)

#endif /* APP_H */
