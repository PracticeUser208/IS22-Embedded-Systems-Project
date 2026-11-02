/*
 *	subsystem5.h
 *	Subsystem 5 -- Obstacle Scan (Buddy 5, "Spatial Optimizer").
 *	Student ID: 2501748.
 *
 *	Public interface: what other subsystems (mainly the Vehicle
 *	Controller) may use.  Everything else stays LOCAL in the .c files.
 *
 *	Hardware (Week 6 pin map, TEAM.md section 2):
 *	  GP4   US_TRIG     HC-SR04+ trigger    Grove 3
 *	  GP5   US_ECHO     HC-SR04+ echo       Grove 3, edge interrupt
 *	  GP12  SERVO_SCAN  scan servo, 50 Hz   servo port S1, PWM slice 6A
 *	The sensor is the HC-SR04+ (3.3 V part): powered from the Grove 3V3
 *	pin and wired straight to GP5, NO voltage divider (the divider in the
 *	Week 6 pin map was for a 5 V HC-SR04).
 *
 *	Interfaces (TEAM.md section 5):
 *	  scan_q  Vehicle Controller -> Obstacle Scan   owned by S5
 *	  obst_q  Obstacle Scan -> Vehicle Controller   owned by the VC
 *	          (S5 creates a stand-in until the VC exists)
 *	  ESTOP_REQ bit ESTOP_OBSTACLE                   set by S5 only
 *	  SYSFAULT bit SYSFAULT_OBSTACLE                 set/cleared by S5
 *	Both queues carry ONE packed 32-bit word per message (Tribunal 1
 *	curveball, "Minimum-Bit Data Representation").  Always use the
 *	pack/unpack functions below; never build the bits by hand.
 *
 *	FOUR FILES (same layout as subsystem 4, 2 Nov):
 *	  subsystem5.h       this file: interface, message formats, pins,
 *	                     tunables of the drivers and the planner
 *	                     (was also s5_shared.h, gpio_irq.h,
 *	                     drv_ultrasonic.h, drv_servo.h, obstacle_plan.h)
 *	  subsystem5.c       kernel objects, init / start, the drivers and the
 *	                     planner (was also gpio_irq.c, drv_ultrasonic.c,
 *	                     drv_servo.c, obstacle_plan.c)
 *	  task_obstacle.c    the obstacle scan task (statechart 6.5) and the
 *	                     bench-test stand-in VC (was s5_stub_vc.c)
 *	  subsystem5_test.c  on-target tests (make TEST=1)
 *	Other subsystems include only this header.  Motion Control's wheel
 *	encoders use the GPIO interrupt dispatcher below (gpio_irq_attach).
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"

/* ---------------------------------------------------------------------
 *  Team-wide stop and fault flags (was s5_shared.h)
 *	ESTOP_OBSTACLE     set when an obstacle is within 15 cm or the
 *	                   ultrasonic sensor fails.  Never cleared here:
 *	                   only the Vehicle Controller clears ESTOP bits.
 *	SYSFAULT_OBSTACLE  (= FAULT_NO_ECHO) set on a sensor fault and
 *	                   cleared by subsystem 5 once the sensor works
 *	                   again and the VC sends RESET.
 *  The values live in the team's app_program/shared_bits.h.  Where that
 *  file is not in the tree yet, the same values are defined here, so
 *  this subsystem builds on its own.  flg_estop_req is created by
 *  Motion Control (subsystem 2); until an owner exists,
 *  subsystem5_init() creates stand-ins for both flags.
 * --------------------------------------------------------------------- */
#ifndef SHARED_BITS_H
#if defined(__has_include)
#if __has_include("../shared_bits.h")
#include "../shared_bits.h"
#endif
#endif
#endif
#ifndef SHARED_BITS_H
#define ESTOP_IMU_IMPACT	(1u << 0)	/* IMU (Buddy 4): collision      */
#define ESTOP_IMU_FAULT		(1u << 1)	/* IMU (Buddy 4): IMU not working */
#define ESTOP_OBSTACLE		(1u << 2)	/* Obstacle Scan (Buddy 5)       */
#define FAULT_NO_ECHO		(1u << 3)	/* Obstacle Scan (Buddy 5)       */
#define SYSFAULT_OBSTACLE	FAULT_NO_ECHO
#endif
IMPORT ID	flg_estop_req;		/* ESTOP_REQ, Motion Control's   */
IMPORT ID	flg_sysfault;		/* shared fault mask             */

/* ---------------------------------------------------------------------
 *  Task priorities.  TEAM.md band for S5 is 18-19.
 *  Tribunal 1 (Version B change log, Teams 37 min): Obstacle Scan runs
 *  at a HIGHER priority than the IMU task, so the bands of S5 and the
 *  IMU's subsystem were swapped when the IMU went in (S4 is now 20-21).
 * --------------------------------------------------------------------- */
#define S5_PRI_SCAN		18	/* obstacle scan task (above the IMU, 20) */
#define S5_PRI_STUB_VC		19	/* bench-test stand-in for the VC */
#define S5_SCAN_CORE		0	/* 0 = any core */

/* ---------------------------------------------------------------------
 *  Kernel objects (message buffers, fixed 4-byte messages)
 * --------------------------------------------------------------------- */
IMPORT ID	scan_q;		/* VC -> S5 commands  (created by S5)     */
IMPORT ID	obst_q;		/* S5 -> VC events    (VC's; stand-in)    */
#define S5_Q_DEPTH		8	/* messages each queue holds */

/* Posting rule (team): never block on a post.  Use TMO_POL; if the
   queue is full the message is dropped and counted. */

/* ---------------------------------------------------------------------
 *  Statechart 6.5 states.  8 states -> 3 bits.
 * --------------------------------------------------------------------- */
#define S5_ST_PATROL		0U	/* ping ahead every 60 ms          */
#define S5_ST_DETECTED		1U	/* stopped in front of an obstacle */
#define S5_ST_COARSE		2U	/* coarse scan, -60..+60 in 30 deg */
#define S5_ST_FINE		3U	/* fine scan, 5 deg around edges   */
#define S5_ST_PLAN		4U	/* profile & plan                  */
#define S5_ST_BYPASS		5U	/* VC is driving around it         */
#define S5_ST_BLOCKED		6U	/* no room either side (fault)     */
#define S5_ST_FAULT		7U	/* sensor fault                    */

/* ---------------------------------------------------------------------
 *  obst_q events (S5 -> VC).  7 types -> 3 bits.
 * --------------------------------------------------------------------- */
#define S5_EVT_OBSTACLE		1U	/* OBSTACLE_DETECTED: flag 0 = slow,
					   flag 1 = stopped (ESTOP bit set) */
#define S5_EVT_PATH_CLEAR	2U	/* nothing in the way any more      */
#define S5_EVT_BYPASS_PLAN	3U	/* side, width, clearance, distance */
#define S5_EVT_NO_PATH		4U	/* blocked; flag 1 = retry limit hit */
#define S5_EVT_BYPASS_DONE	5U	/* BYPASS_COMPLETE: path ahead clear */
#define S5_EVT_SENSOR_FAULT	6U	/* width field carries fault bits   */
#define S5_EVT_FAULT_CLEARED	7U	/* sensor works again after RESET   */

#define S5_SIDE_NONE		0U
#define S5_SIDE_LEFT		1U
#define S5_SIDE_RIGHT		2U

/* ---------------------------------------------------------------------
 *  scan_q commands (VC -> S5).  6 commands -> 3 bits.
 * --------------------------------------------------------------------- */
#define S5_CMD_STOP		1U	/* abort anything, back to PATROL  */
#define S5_CMD_START_SCAN	2U	/* car is stopped: scan and plan   */
#define S5_CMD_MOVE_DONE	3U	/* bypass manoeuvre finished       */
#define S5_CMD_RESET		4U	/* clear a sensor fault            */
#define S5_CMD_PATROL_OFF	5U	/* stop pinging (car idle, paused) */
#define S5_CMD_PATROL_ON	6U	/* resume pinging (line following) */

/* ---------------------------------------------------------------------
 *  Obstacle Scan fault bits (S5 only; 8 bits).  Single writer: the
 *  scan task.  The echo ISR never touches this word.
 * --------------------------------------------------------------------- */
#define S5_FLT_NO_ECHO		0x01U	/* 3 pings in a row, no echo at all  */
#define S5_FLT_ECHO_STUCK	0x02U	/* echo pin held high > 250 ms       */
#define S5_FLT_TIMER		0x04U	/* echo timer not at 1 MHz           */
#define S5_FLT_IRQ		0x08U	/* echo interrupt could not be set up */
#define S5_FLT_LATE		0x10U	/* warning: pings 5x late in a row   */
#define S5_FLT_QFULL		0x20U	/* warning: obst_q full, event lost  */
#define S5_FLT_STOP_MASK	(S5_FLT_NO_ECHO | S5_FLT_ECHO_STUCK | \
				 S5_FLT_TIMER | S5_FLT_IRQ)
					/* these stop the car (SENSOR FAULT) */

/* ---------------------------------------------------------------------
 *  Field limits.  Distances: 1 cm resolution, 0..511 in 9 bits
 *  (HC-SR04 range 2..400 cm).  0 = no reading, 511 = nothing in range.
 * --------------------------------------------------------------------- */
#define S5_CM_NONE		0U
#define S5_CM_OPEN		511U
#define S5_CM_MAX9		511U
#define S5_WIDTH_MAX8		255U

/* ---------------------------------------------------------------------
 *  obst_q word layout (32 bits, all used)
 *    31..29 type    3   S5_EVT_xxx
 *    28..27 side    2   S5_SIDE_xxx
 *    26     flag    1   OBSTACLE: 1 = stop; NO_PATH: 1 = gave up
 *    25..17 dist    9   cm to the obstacle (0..511)
 *    16..9  width   8   cm (0..255); SENSOR_FAULT: S5_FLT_xxx bits
 *    8..0   clear   9   cm of room on the chosen side (0..511)
 * --------------------------------------------------------------------- */
typedef struct {
	UB	type;
	UB	side;
	UB	flag;
	UH	dist_cm;
	UB	width_cm;
	UH	clear_cm;
} S5_OBST_MSG;

Inline UH s5_clamp9(UW v)
{
	return (UH)((v > S5_CM_MAX9) ? S5_CM_MAX9 : v);
}

Inline UW s5_obst_pack(const S5_OBST_MSG *m)
{
	return (((UW)m->type & 0x7U) << 29) |
	       (((UW)m->side & 0x3U) << 27) |
	       (((UW)m->flag & 0x1U) << 26) |
	       ((UW)s5_clamp9(m->dist_cm) << 17) |
	       ((UW)m->width_cm << 9) |
	       (UW)s5_clamp9(m->clear_cm);
}

Inline void s5_obst_unpack(UW w, S5_OBST_MSG *m)
{
	m->type     = (UB)((w >> 29) & 0x7U);
	m->side     = (UB)((w >> 27) & 0x3U);
	m->flag     = (UB)((w >> 26) & 0x1U);
	m->dist_cm  = (UH)((w >> 17) & 0x1FFU);
	m->width_cm = (UB)((w >> 9) & 0xFFU);
	m->clear_cm = (UH)(w & 0x1FFU);
}

/* ---------------------------------------------------------------------
 *  scan_q word layout
 *    31..29 cmd     3   S5_CMD_xxx
 *    28..8  -      21   reserved, send 0
 *    7..0   arg     8   optional (e.g. bypass leg number)
 * --------------------------------------------------------------------- */
Inline UW s5_cmd_pack(UB cmd, UB arg)
{
	return (((UW)cmd & 0x7U) << 29) | (UW)arg;
}

Inline UB s5_cmd_of(UW w)  { return (UB)((w >> 29) & 0x7U); }
Inline UB s5_arg_of(UW w)  { return (UB)(w & 0xFFU); }

/* ---------------------------------------------------------------------
 *  Status snapshot for the VC / telemetry (FR21: obstacle scan, distance).
 *  One aligned 32-bit word, written only by the scan task with a single
 *  store, so any task on either core reads a consistent value with no
 *  lock.
 *    2..0   state      3   S5_ST_xxx
 *    3      pinging    1   1 = patrol pings on
 *    11..4  faults     8   S5_FLT_xxx
 *    20..12 dist       9   last forward distance, cm (0 none, 511 open)
 *    28..21 servo      8   servo angle + 90 (0..180), + = left
 *    30..29 retries    2   rescans used for this obstacle
 * --------------------------------------------------------------------- */
IMPORT UW s5_status_word(void);

#define S5_STW_STATE(w)		((UB)((w) & 0x7U))
#define S5_STW_PINGING(w)	((UB)(((w) >> 3) & 0x1U))
#define S5_STW_FAULTS(w)	((UB)(((w) >> 4) & 0xFFU))
#define S5_STW_DIST(w)		((UH)(((w) >> 12) & 0x1FFU))
#define S5_STW_SERVO(w)		((H)((H)(((w) >> 21) & 0xFFU) - 90))
#define S5_STW_RETRIES(w)	((UB)(((w) >> 29) & 0x3U))

/* Measurements (read-only for others) */
IMPORT UW s5_run_us_max;	/* longest CPU run between two blocks, us */
IMPORT UW s5_dropped;		/* obst_q events dropped because it was full */

/* 1 = also start a stand-in Vehicle Controller (task_obstacle.c) so this
   subsystem can be run and watched on its own.  Set 0 once the real
   Vehicle Controller handles obst_q and scan_q. */
#ifndef S5_STUB_VC
#define S5_STUB_VC		1
#endif

/* Short names for printing */
IMPORT const char *s5_state_name(UB st);
IMPORT const char *s5_event_name(UB ev);

/* Tasks (started by subsystem5_start) */
IMPORT void s5_scan_task(INT stacd, void *exinf);
IMPORT void s5_stub_vc_task(INT stacd, void *exinf);
IMPORT ER   s5_scan_task_init(void);

/* =====================================================================
 *  GPIO interrupt dispatcher (was gpio_irq.h)
 * =====================================================================
 *	Shared GPIO edge-interrupt dispatcher (IO_IRQ_BANK0).
 *
 *	WHY THIS EXISTS
 *	All 30 GPIO pins share ONE interrupt line on the RP2040
 *	(IO_IRQ_BANK0, NVIC 13), and tk_def_int() keeps only ONE handler
 *	per interrupt number: a second tk_def_int(13, ...) silently replaces
 *	the first.  So the echo pin (Obstacle Scan, GP5) and the wheel
 *	encoders (Motion Control, GP16/GP17) cannot each install their own
 *	handler.  This module installs the one handler and calls a per-pin
 *	callback instead.  (Week 6 note: "the SDK supports only one GPIO
 *	callback" -- this is the shared dispatcher that note asked for.)
 *
 *	OWNERSHIP
 *	Kept in subsystem 5 until the team agrees where shared code lives.
 *	Other subsystems include "../subsystem5/subsystem5.h" and call
 *	gpio_irq_attach(); if the team moves it to app_program/, only the
 *	include path changes.
 *
 *	RULES FOR CALLBACKS
 *	A callback runs in interrupt context: keep it to a few lines, never
 *	wait, never call tm_printf.  Signalling a task (tk_set_flg,
 *	tk_wup_tsk) is the normal thing to do.  The microsecond timestamp of
 *	the interrupt is passed in, taken once at handler entry.
 */

/* Edge selection, same bit values as the IO_BANK0 INTE/INTS nibble */
#define GIRQ_EDGE_FALL		0x4U
#define GIRQ_EDGE_RISE		0x8U

#define GIRQ_MAX_PINS		8U	/* callbacks the table can hold */
#define GIRQ_INTNO		13U	/* IO_IRQ_BANK0 */
#define GIRQ_INTPRI		2	/* same level as the kernel's drivers */

/* pin = GPIO number, events = GIRQ_EDGE_xxx bits that fired,
   now_us = free-running microsecond timer at handler entry */
typedef void (*GIRQ_FN)(UINT pin, UW events, UW now_us);

/* Register a callback for edges on one pin and enable them.
   Task context only (it may call tk_def_int).  E_PAR on a bad pin or
   edge mask, E_LIMIT if the table is full, E_OBJ if the pin is taken. */
IMPORT ER gpio_irq_attach(UINT pin, UW edges, GIRQ_FN fn);

/* Free-running 1 MHz timer (RP2040 TIMER, TIMERAWL).  Wraps every
   ~71 minutes; always subtract two readings as UW. */
IMPORT UW gpio_irq_now_us(void);

/* =====================================================================
 *  HC-SR04+ ultrasonic driver (was drv_ultrasonic.h)
 * =====================================================================
 *	HC-SR04+ ultrasonic driver.  Called ONLY from the obstacle scan task,
 *	which owns the sensor (TEAM.md section 3).
 *
 *	Wiring: TRIG -> GP4, ECHO -> GP5 directly (HC-SR04+ runs on 3.3 V,
 *	so the echo is already 3.3 V), VCC -> 3V3, GND -> GND (Grove 3).
 *
 *	How a ping works:
 *	  1. the task holds TRIG high for >= 10 us
 *	  2. the sensor raises ECHO for the round-trip time of the sound
 *	  3. the GPIO interrupt timestamps both ECHO edges with the 1 MHz
 *	     hardware timer and wakes the task through an event flag
 *	  4. the task waits on that flag WITH A TIMEOUT, so a missing echo
 *	     can never hang it (the LAB 3 sample code's busy-wait could)
 *	The kernel tick is 10 ms, far too coarse for this, which is why the
 *	edges are timed with the hardware timer, checked at start-up to run
 *	at 1 MHz (us_timer_check).  Tested on the car 25 Oct: 32 cm falling
 *	smoothly to 15 cm as a box approached.
 */

#define US_TRIG_PIN		4U
#define US_ECHO_PIN		5U

/* Result of one ping, packed in 16 bits:
 *   15..13 status  3   US_xxx below
 *   8..0   cm      9   distance, valid when status == US_OK */
#define US_OK		0U	/* echo measured, cm valid                  */
#define US_OPEN		1U	/* echo started but nothing within ~4 m     */
#define US_NONE		2U	/* no usable echo (none at all, or a glitch) */
#define US_BUSY		3U	/* echo still high from the last ping: skip */
#define US_STUCK	4U	/* echo held high for over US_STUCK_MS      */

#define US_MAKE(st, cm)	((UH)((((UH)(st)) << 13) | ((UH)(cm) & 0x1FFU)))
#define US_STATUS(r)	((UB)(((r) >> 13) & 0x7U))
#define US_CM(r)	((UH)((r) & 0x1FFU))

#define US_ECHO_TMO_MS	30	/* ~5 m round trip; longer = nothing there */
#define US_STUCK_MS	250	/* echo high this long = sensor fault       */

/* Set up the pins, the echo interrupt and the event flag.
   Task context (call from subsystem5_init).  Returns E_OK, or the error
   code if the interrupt could not be attached. */
IMPORT ER us_init(void);

/* Check the 1 MHz TIMER that times the echo, and start its tick if it is
   stopped or off.  Decides from the watchdog TICK register (running, 12
   cycles); also measures and prints the rate.  Returns TRUE if the timer
   is good.  Call BEFORE us_ping (us_ping refuses to run on a bad timer). */
IMPORT BOOL us_timer_check(void);

/* Raw width of the last echo in microseconds (for printing) */
IMPORT UW us_last_width_us;

/* One measurement.  Blocks the caller for at most ~US_ECHO_TMO_MS + one
   tick (it waits on an event flag, never spins on the pin). */
IMPORT UH us_ping(void);

/* =====================================================================
 *  Scan servo (was drv_servo.h)
 * =====================================================================
 *	Scan servo on servo port S1 (GP12 = PWM slice 6, channel A), 50 Hz.
 *	Called ONLY from the obstacle scan task, which owns PWM slice 6.
 *
 *	Angles are degrees from straight ahead, + = left (anticlockwise seen
 *	from above).  Use the us_servo_bringup test firmware to find:
 *	  SERVO_CENTER_US  the pulse that points the sensor straight ahead
 *	  SERVO_DIR        +1 if a bigger pulse turns it left, else -1
 */

#define SERVO_PIN		12U
#define SERVO_CENTER_US		1500	/* checked 25 Oct: 0 deg faces ahead */
#define SERVO_DIR		(+1)	/* checked 25 Oct: + turns it left   */
#define SERVO_US_PER_90DEG	1000	/* typical hobby servo            */

/* Safe travel, MEASURED ON THE CAR (25 Oct, us_servo_bringup v2/v3):
   the sensor hits the Robo Pico board and is knocked off its mount on the
   way to +45 (left) and to -60 (right).  Past these limits the servo would
   also stall against the board (gear wear, current spike, possible
   brown-out: A10).  This is a hardware limit of the current mounting, so
   the Week 6 coarse scan (5 angles over +-60) becomes 4 angles:
   -45, -30, 0, +30.  Re-measure (test S5-U20) if the mounting changes.
   Multiples of 5 only (the scan uses a 5-degree grid). */
#define SERVO_LIMIT_LEFT_DEG	30
#define SERVO_LIMIT_RIGHT_DEG	(-45)

#if ((SERVO_LIMIT_LEFT_DEG % 5) != 0) || ((SERVO_LIMIT_RIGHT_DEG % 5) != 0)
#error "servo limits must be multiples of 5 degrees"
#endif
#define SERVO_MIN_US		600
#define SERVO_MAX_US		2400

IMPORT ER  servo_init(void);		/* PWM out of reset, 50 Hz, centred */
IMPORT void servo_set_deg(W deg);	/* clamps to the safe travel above */
IMPORT BOOL servo_deg_ok(W deg);	/* TRUE if deg is inside the limits */
IMPORT W   servo_get_deg(void);

/* Time to wait after a move of 'delta' degrees before pinging.  Rated
   speed is ~0.1 s per 60 deg; this allows double that plus 40 ms. */
IMPORT INT servo_settle_ms(W delta_deg);

/* =====================================================================
 *  Scan table and the profile & plan step (was obstacle_plan.h)
 * =====================================================================
 *	Scan storage and the profile & plan step (FR16, FR17).
 *	Pure logic: no hardware, no kernel calls, so the on-target tests can
 *	feed it made-up scans.
 *
 *	Scan storage (Tribunal 1 curveball): one 9-bit distance per 5 deg
 *	slot from -60 to +60 deg.  The angle is not stored: it follows from
 *	the slot number (angle = -60 + 5 * slot).  25 slots x 2 bytes = 50
 *	bytes, against 25 x (4-byte float distance + 4-byte float angle)
 *	= 200 bytes before.  Coarse and fine samples share the one table.
 */

#define S5_SCAN_MIN_DEG		(-60)
#define S5_SCAN_MAX_DEG		60
#define S5_SCAN_STEP_DEG	5
#define S5_SCAN_SLOTS		25	/* (60 - -60) / 5 + 1 */
#define S5_COARSE_STEP_DEG	30	/* coarse: -60 -30 0 30 60 (5 angles) */

/* Tunables.  Starting values; tune them with tests A9 / FR16 / FR17. */
#define S5_AHEAD_DEG		30	/* obstacle must be within +-30 deg  */
#define S5_OBJECT_CM		35	/* nearer than this = in the way    */
#define S5_JUMP_CM		10	/* bigger step between slots = edge  */
#define S5_BEAM_CORR_DEG	7	/* half beam width: edges read wide  */
#define S5_DEPTH_CM		40	/* side objects deeper than obstacle
					   + this do not block the bypass    */
#define S5_MIN_CLEAR_CM		25	/* car needs this much room to pass  */
#define S5_CLEAR_OPEN_CM	400	/* "nothing there" on a side         */

/* Plan results */
#define S5_PLAN_CLEAR		0U	/* nothing in the way                */
#define S5_PLAN_BYPASS		1U	/* go round on 'side'                */
#define S5_PLAN_BLOCKED		2U	/* no side has room                  */

typedef struct {
	UH	cm[S5_SCAN_SLOTS];	/* 9-bit values: 0 none, 511 open */
} S5_SCAN;

typedef struct {
	UB	result;			/* S5_PLAN_xxx                    */
	UB	side;			/* S5_SIDE_xxx                    */
	B	left_deg;		/* obstacle's left edge (+ = left) */
	B	right_deg;		/* obstacle's right edge          */
	UH	dist_cm;		/* nearest point of the obstacle  */
	UH	width_cm;
	UH	clear_left_cm;
	UH	clear_right_cm;
} S5_PLAN;

IMPORT void s5_scan_clear(S5_SCAN *s);
IMPORT INT  s5_slot_of(W deg);			/* -1 if not a slot */
IMPORT W    s5_deg_of(INT slot);
IMPORT void s5_scan_put(S5_SCAN *s, W deg, UH cm);
IMPORT UH   s5_scan_get(const S5_SCAN *s, W deg);

/* Fine-scan range from the coarse samples: the 30 deg either side of the
   measured angles that hit something nearer than S5_OBJECT_CM (the caller
   skips angles past the servo's safe travel).  Returns
   FALSE if no coarse angle hit anything (the path is clear). */
IMPORT BOOL s5_fine_range(const S5_SCAN *s, W *from_deg, W *to_deg);

IMPORT void s5_plan(const S5_SCAN *s, S5_PLAN *p);

/* sin(deg) * 4096, deg in -90..90 */
IMPORT W s5_sin_q12(W deg);

#endif /* SUBSYSTEM5_H */
