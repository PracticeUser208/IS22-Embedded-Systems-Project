/*
 *	vehicle_ctrl.h
 *	Vehicle Controller (VC) -- the system-level statechart (Week 6
 *	report, section 5).  Owned by the whole team ("All Buddy" in the
 *	Tribunal 1 module table); first version written for the team so the
 *	car can run end to end.
 *
 *	What it does
 *	------------
 *	It owns no hardware.  It listens to every subsystem, decides what
 *	the car does next, and tells the subsystems:
 *	  in:  remote_cmd_q  (Telemetry, S1)   START STOP PAUSE RESUME RESET
 *	       obst_q        (Obstacle Scan)   obstacle / bypass plan / no path
 *	       imu_evt_q     (IMU, S4)         hump / motion / lifted / fault
 *	       nav_q         (Line + Barcode)  barcode / line lost / reacquired
 *	       flg_motion    (Motion Control)  move finished / failed
 *	  out: motion_xxx()  (Motion Control)  drive, turn, stop
 *	       scan_q        (Obstacle Scan)   start scan, watch / look during
 *	                                       a bypass, reset
 *	       imu_q         (IMU)             reset
 *	       line_cmd_q    (Line + Barcode)  suspend / resume search
 *	       telem_q       (Telemetry)       one snapshot per second
 *	       flg_estop_req                   clears stop bits (only the VC
 *	                                       does), flg_sysfault FAULT_NO_PATH
 *
 *	States (sys_state_t, subsystem1/telem_format.h):
 *	  INITIALISING -> IDLE_READY (or STARTUP_ERROR)
 *	  IDLE_READY  --START--> PATROLLING
 *	  PATROLLING  --PAUSE--> PAUSED --RESUME--> PATROLLING
 *	  PATROLLING  --obstacle stop--> OBSTACLE_AVOIDANCE --> LINE_RECOVERY
 *	              --> PATROLLING
 *	              (OBSTACLE_AVOIDANCE, 12 Nov: the car feels its way
 *	              round, one move at a time: vc_bypass.h)
 *	  PATROLLING  --barcode--> HANDLING_BARCODE --> PATROLLING
 *	              (only with VC_BARCODE_ACT 1; 0 = read and report only)
 *	  PATROLLING  --climbing hump--> HANDLE_HUMP --> PATROLLING
 *	  PATROLLING  --line lost--> LINE_RECOVERY --> PATROLLING
 *	  any         --lifted / sensor fault / no path--> FAULT_ESTOP
 *	  any         --STOP--> IDLE_READY     (not from FAULT: needs RESET)
 *	  any         --RESET--> clears what can be cleared, IDLE_READY
 *	Commands are only accepted in the states where they make sense
 *	(Week 6 feedback: "remote control is ... not state-gated").
 *
 *	Line and Barcode (S3) runs: while patrolling the VC reads its
 *	steering (s3_steer_get(), steer_mtx) every tick and drives the left
 *	wheel at speed - steer, the right at speed + steer.  When the line
 *	is lost it sweeps the car left and right until S3 finds it again.
 *	The IMU (S4) runs Buddy 4's code: its imu_evt_q / imu_q formats are
 *	his (vc_imu_msg.h).  The nav_q / line_cmd_q formats below are still
 *	PROVISIONAL, marked TBC: Buddy 3 changes them here if his differ.
 */

#ifndef VEHICLE_CTRL_H
#define VEHICLE_CTRL_H

#include "app.h"
#include "subsystem1/telem_format.h"
#include "vc_imu_msg.h"

/* ---------------------------------------------------------------------
 *  Task (TEAM.md section 4: VC band 10-11, above the subsystems; it
 *  only coordinates, so it runs briefly every VC_TICK_MS)
 * --------------------------------------------------------------------- */
#define VC_PRI			11
#define VC_TICK_MS		20	/* event poll + steering refresh:
					   same rate as Motion Control's loop */
#define VC_TELEM_MS		1000	/* FR21: one snapshot per second   */

/* ---------------------------------------------------------------------
 *  Speeds (mm/s) and bypass geometry (mm).  STARTING VALUES: measure
 *  the car and tune.  Motion Control limits speeds to 400 mm/s.
 * --------------------------------------------------------------------- */
#define VC_PATROL_MM_S		25	/* normal patrol (150 -> 80 -> 60 -> 20:
					   slow tests, 2 Nov) */
#define VC_SLOW_MM_S		25	/* obstacle ahead (< 50 cm); not below
					   patrol while that is 20 */
#define VC_HUMP_MM_S		25	/* while climbing a hump */
#define VC_BYPASS_MM_S		50	/* straight legs of a bypass */
#define VC_CAR_WIDTH_MM		160	/* TODO measure, wheel to wheel      */
#define VC_CAR_LENGTH_MM	200	/* TODO measure, front to back       */
#define VC_BYPASS_MARGIN_MM	50	/* room between the car's side and the
					   obstacle.  Side-step from the line
					   = half the width the scan measured
					   (at least VC_OBST_MIN_WIDTH_MM)
					   + half the car + this (17 Nov; was
					   the whole width + 100: 22-25 cm out
					   for a 12-15 cm obstacle, now 19.5-
					   20.5).  Below about 65 the car's
					   front corner can touch when it turns
					   in while still beside the obstacle
					   (deeper than VC_OBST_MIN_DEPTH_MM
					   and not seen): 70 if that happens  */
#define VC_SIDE_STEP_MM		150	/* ... and at least this (17 Nov: was
					   200; the minimum width above now
					   covers the scan measuring a 13 cm
					   obstacle as 2-9 cm)                */
#define VC_OBST_MIN_WIDTH_MM	130	/* an obstacle is taken to be at least
					   this wide when working out how
					   close the car is beside it        */
#define VC_OBST_MIN_DEPTH_MM	120	/* ... and this long along the line
					   (its back cannot be seen from
					   beside it): the car turns in only
					   when it is past that.  17 Nov: was
					   100; the course's obstacles are up
					   to 15 cm deep, and at 100 the car
					   turned in beside one that deep.
					   19 Nov: 120, the obstacle is 7-10 cm
					   deep                               */
#define VC_PASS_FIRST_MM	200	/* along the lane (at least), then
					   check whether the obstacle is still
					   beside (19 Nov: was 300)           */
#define VC_SIDE_NEAR_MM		100	/* the check: something sideways up to
					   the line + this = still beside     */
#define VC_US_AHEAD_MM		120	/* TODO measure: the ultrasonic is this
					   far ahead of the wheels' axle      */
#define VC_OBST_DEPTH_MM	250	/* FIRST GUESS of the obstacle's depth
					   (12 Nov: no longer assumed.  The car
					   drives this far past its front, then
					   looks; if it is longer it goes on in
					   steps of VC_PASS_STEP_MM)         */
#define VC_BACKUP_MM		150	/* back up this far when blocked     */

/* Going round an obstacle one move at a time (12 Nov, vc_bypass.h) */
#define VC_PASS_STEP_MM		50	/* still beside it, or in the way when
					   turning back: this much further,
					   then check again (16 Nov: was 150;
					   19 Nov: was 100)                   */
#define VC_JOG_MM		50	/* too close to its side while driving
					   along the lane: out by this (16 Nov:
					   was 100).  17 Nov: no longer used
					   when a turn in is stopped; the car
					   then only goes further along       */
#define VC_IR_AHEAD_MM		107	/* the line sensors are this far ahead
					   of the wheels' axle (15 Nov, measured:
					   10.5-11 cm to the LEDs)            */
#define VC_LOOK_MARGIN_MM	30	/* the way to the line counts as free
					   if nothing is seen up to the line
					   + this                             */
#define VC_CROSS_EXTRA_MM	300	/* crossing back (at VC_BYP_IN_DEG,
					   vc_bypass.h; at 70 deg: 1.06 x the
					   sideways distance): gives up looking
					   for the line after 1.25 x that + this */
#define VC_MAX_SIDE_MM		700	/* never further from the line        */
#define VC_MAX_PASS_MM		2000	/* an obstacle longer than this: FAULT
					   "no way round"                     */
#define VC_MAX_PLANS		4	/* full scans for one obstacle        */
#define VC_BACKOFF_MAX		2	/* blocked both sides, backed away and
					   blocked again this often: FAULT    */
#define VC_MOVE_REST_MS		400	/* the car settles after each move;
					   also longer than Obstacle Scan needs
					   to stop watching (one ping + its
					   confirmation, < 300 ms), so a stop
					   from the watch always arrives before
					   the next move starts               */
#define VC_TURN_FIX_MAX_DEG	20	/* a turn makes up for the turns and
					   drift before it by at most this    */
/* The values above as a vc_byp_cfg_t (vc_bypass.h) */
#define VC_BYP_CFG_INIT	{					\
	.car_width_mm    = VC_CAR_WIDTH_MM,			\
	.car_length_mm   = VC_CAR_LENGTH_MM,			\
	.side_step_mm    = VC_SIDE_STEP_MM,			\
	.margin_mm       = VC_BYPASS_MARGIN_MM,			\
	.min_width_mm    = VC_OBST_MIN_WIDTH_MM,		\
	.first_pass_mm   = VC_PASS_FIRST_MM,			\
	.pass_step_mm    = VC_PASS_STEP_MM,			\
	.jog_mm          = VC_JOG_MM,				\
	.sensor_ahead_mm = VC_IR_AHEAD_MM,			\
	.us_ahead_mm     = VC_US_AHEAD_MM,			\
	.min_depth_mm    = VC_OBST_MIN_DEPTH_MM,		\
	.side_near_mm    = VC_SIDE_NEAR_MM,			\
	.look_margin_mm  = VC_LOOK_MARGIN_MM,			\
	.cross_extra_mm  = VC_CROSS_EXTRA_MM,			\
	.backup_mm       = VC_BACKUP_MM,			\
	.max_side_mm     = VC_MAX_SIDE_MM,			\
	.max_pass_mm     = VC_MAX_PASS_MM,			\
	.max_plans       = VC_MAX_PLANS				\
}
#define VC_STOP_SETTLE_MS	300	/* car settles before a scan starts  */
#define VC_REARM_MS		1000	/* standing still: re-arm Obstacle Scan */
#define VC_SCAN_TIMEOUT_MS	8000	/* no answer from Obstacle Scan      */
#define VC_HUMP_TIMEOUT_MS	4000	/* leave HANDLE_HUMP anyway          */
#define VC_LINE_TIMEOUT_MS	4000	/* line not found: = LF_SEARCH_MS (FR19
					   was 2 s; 4 s since 2 Nov)          */

/* Line following (S3 steering) and the line search sweep */
#define VC_STEER_STALE_MS	100	/* older steering is not used        */
/* Line search: spin on the spot, the SAME angle to both sides:
     first swing   +A  (towards where the line was last seen), then
     back          -A  (through the start, as far the other way),
     then towards +A again, until the line is found or LF_SEARCH_MS
     (line_cfg.h, 4 s) runs out.
   Spin rate = 2 x VC_SEARCH_MM_S / TRACK_WIDTH_MM rad/s: at 50 mm/s
   about 48 deg/s, so 950 ms = about 45 deg each side, and the whole
   +45 / -45 sweep takes 2.85 s.  First swing = A / rate, swing back =
   2 x the first swing (keep that ratio or one side gets less).
   2 Nov: the sweep was +31 / -15 deg in 2 s; after a missed LEFT turn
   with the first swing to the right, the left side was never reached. */
#define VC_SEARCH_MM_S		50	/* each wheel, pivoting on the spot  */
#define VC_SWEEP_FIRST_MS	950	/* first swing: about 45 deg         */
#define VC_SWEEP_MS		1900	/* each swing after: 2 x the first   */

/* Barcodes (read by Line and Barcode, S3_BARCODES in line_cfg.h):
   0 = read only: the command is remembered and shown (telemetry bc=,
       dashboard) and the car keeps following the line.
   1 = turn as soon as one is read while patrolling: LEFT / RIGHT one
       90 deg turn, U-TURN two lefts, STRAIGHT carries on (vc_plan.c).
   2 = (15 Nov, the course: a barcode before each "+" junction) remember
       the command and carry it out at the NEXT junction (Line and
       Barcode reports it, line_cfg.h LF_JCT_xxx): drive the axle onto
       the junction's middle, turn left / right (U-TURN: round to the
       left) without looking for VC_JCT_BLIND_DEG, then on until the
       middle line sensors find the new line, and follow it.  STRAIGHT:
       straight on through it.  No junction within VC_JCT_FORGET_MM:
       the command is dropped.  A junction with no command: straight
       on. */
#ifndef VC_BARCODE_ACT
#define VC_BARCODE_ACT		2
#endif
#define VC_JCT_AHEAD_MM		73	/* junction reported -> axle onto its
					   middle: it comes 24 mm after the
					   side sensor met the tape's middle,
					   the axle is VC_IR_AHEAD_MM behind
					   (107 - 24 = 83).  17 Nov: 73, the
					   car turns 10 mm sooner (as if the
					   junction were reported 5 mm after
					   the tape, not LF_JCT_GAP_MM = 15;
					   that gap itself stays, it is what
					   tells a junction from a barcode).
					   If the side sensor sits further
					   forward than the line sensors, add
					   the difference                     */
#define VC_JCT_BLIND_DEG	50	/* LEFT / RIGHT: turn this far first,
					   off the line ahead (it ends 5 deg
					   either side of it)                 */
#define VC_JCT_UTURN_BLIND_DEG	140	/* U-TURN: ... past the side arm at
					   90 deg                             */
#define VC_JCT_SEEK_DEG		100	/* then on, at most this far, until the
					   line sensors see the new line (90 /
					   180 deg, more if the axle is off the
					   middle)                            */
#define VC_JCT_FORGET_MM	10000	/* (23 Nov: was 1500) a barcode's command is dropped if no
					   junction comes within this         */

/* Bench option: 0 = wait for an MQTT START (Week 6 design).  N > 0 =
   start patrolling by itself N seconds after power-up, for testing
   without WiFi.  ONLY with the car on a stand or on the floor. */
#ifndef VC_AUTOSTART_S
#define VC_AUTOSTART_S		0
#endif

/* ---------------------------------------------------------------------
 *  Kernel objects the VC owns (the receiver owns the queue)
 *	obst_q       S5 -> VC   (subsystem5.h layout, 4-byte words)
 *	imu_evt_q    S4 -> VC   (layout below, 4-byte words)
 *	nav_q        S3 -> VC   (layout below, 4-byte words)
 *	flg_sysfault shared fault mask (app.h / shared_bits.h)
 *  Owned by others, used here: mbf_remote_cmd_q, mbf_telem_q (S1),
 *  scan_q (S5), flg_motion (S2), flg_estop_req (S2), and, once they
 *  exist, imu_q (S4) and line_cmd_q (S3).
 * --------------------------------------------------------------------- */
IMPORT ID	obst_q;
IMPORT ID	imu_evt_q;
IMPORT ID	nav_q;
IMPORT ID	imu_q;		/* 0 until the IMU creates it      */
IMPORT ID	line_cmd_q;	/* 0 until Line and Barcode does   */
#define VC_Q_DEPTH		8	/* 4-byte messages per queue */

/* ---------------------------------------------------------------------
 *  imu_evt_q (IMU -> VC) and imu_q (VC -> IMU): Buddy 4's formats, see
 *  vc_imu_msg.h (VC_IMU_EVT_xxx, VC_IMU_MSG_TYPE/PAYLOAD, VC_IMU_CMD_RESET).
 *  The IMU sets its own ESTOP bit BEFORE posting LIFTED / SENSOR_FAULT.
 *  Humps are reported when they are over, so the VC records height and
 *  peak (Tribunal 1: peak tracking moved to the VC); it does not enter
 *  HANDLE_HUMP, which needs a "hump started" event the IMU does not send.
 * --------------------------------------------------------------------- */

/* ---------------------------------------------------------------------
 *  nav_q word (Line and Barcode -> VC).  PROVISIONAL, TBC with Buddy 3.
 *    31..29 type     3   VC_NAV_xxx
 *    2..0   barcode  3   barcode_t (telem_format.h), for VC_NAV_BARCODE
 * --------------------------------------------------------------------- */
#define VC_NAV_BARCODE		1U	/* barcode decoded                    */
#define VC_NAV_LINE_LOST	2U	/* searching for the line             */
#define VC_NAV_LINE_REACQUIRED	3U	/* back on the line                   */
#define VC_NAV_LINE_TIMEOUT	4U	/* FR19: not found within 2 s         */
#define VC_NAV_SENSOR_FAULT	5U	/* IR sensors not working             */
#define VC_NAV_JUNCTION		6U	/* 15 Nov: a crossing ("+") under the
					   side sensor (line_cfg.h LF_JCT_xxx):
					   the car is about 24 mm past its
					   middle, its axle VC_IR_AHEAD_MM
					   further back                       */

Inline UW vc_nav_pack(UB type, UB barcode)
{
	return (((UW)type & 0x7U) << 29) | ((UW)barcode & 0x7U);
}
Inline UB vc_nav_type(UW w)    { return (UB)((w >> 29) & 0x7U); }
Inline UB vc_nav_barcode(UW w) { return (UB)(w & 0x7U); }

/* line_cmd_q word (VC -> Line and Barcode), PROVISIONAL: 31..29 command */
#define VC_LINE_CMD_SUSPEND	1U	/* stop steering: the VC drives       */
#define VC_LINE_CMD_SEARCH	2U	/* search for the line (after bypass) */
#define VC_LINE_CMD_FOLLOW	3U	/* follow the line                    */
#define VC_LINE_CMD_RESET	4U

/* ---------------------------------------------------------------------
 *  Status for printing and tests
 * --------------------------------------------------------------------- */
IMPORT UB		vc_state(void);			/* sys_state_t */
IMPORT const char	*vc_state_name(UB st);
IMPORT UW		vc_telem_dropped;		/* telem_q full */

/* Called from app_main.c (declared in app.h too) */
IMPORT ER	vc_init(void);
IMPORT ER	vc_start(void);
IMPORT void	vc_task(INT stacd, void *exinf);

/* ======== from vc_plan.h ======== */
/*
 *	vc_plan.h
 *	Vehicle Controller -- manoeuvre planning.  Pure functions (no kernel
 *	calls, no globals), so they are unit-tested in vc_test.c.
 *
 *	A manoeuvre is a short list of "legs"; the VC task hands them to
 *	Motion Control one at a time and waits for each to finish.
 *
 *	12 Nov: the VC uses only vc_plan_barcode() now.  Going round an
 *	obstacle is decided move by move in vc_bypass.c; vc_plan_bypass(),
 *	vc_plan_return() and vc_plan_backup() (the fixed route) are kept
 *	with their tests (VC-U01..05) but no longer called.
 */

#ifndef VC_PLAN_H
#define VC_PLAN_H

#include "app.h"

#define VC_LEG_FWD		1U	/* amount = mm   */
#define VC_LEG_BACK		2U	/* amount = mm   */
#define VC_LEG_LEFT		3U	/* amount = deg  */
#define VC_LEG_RIGHT		4U	/* amount = deg  */

#define VC_MAX_LEGS		6

typedef struct {
	UB	kind;
	UH	amount;
} VC_LEG;

typedef struct {
	VC_LEG	leg[VC_MAX_LEGS];
	UB	n;
} VC_LEGS;

/* Bypass on 'side' (S5_SIDE_LEFT / S5_SIDE_RIGHT) of an obstacle 'dist_cm'
 * ahead and 'width_cm' wide, with 'clear_cm' of room on that side:
 *   turn 90 toward the side, side-step, turn back, drive past.
 * The car then faces its original direction, shifted sideways.
 * Returns the sideways shift in mm, + = left (the caller keeps the
 * total so Line Recovery can undo it).  Returns 0 and no legs if
 * 'side' is not left or right. */
IMPORT W vc_plan_bypass(UB side, UH dist_cm, UH width_cm, UH clear_cm,
			VC_LEGS *p_legs);

/* Back onto the original path from a sideways offset (+ = car is left
   of it): turn toward it, drive across, turn back.  No legs if 0. */
IMPORT void vc_plan_return(W offset_mm, VC_LEGS *p_legs);

/* Back up when there is no way round (Obstacle Scan rescans after) */
IMPORT void vc_plan_backup(VC_LEGS *p_legs);

/* Barcode command (barcode_t): LEFT/RIGHT = one 90 deg turn, UTURN = two
   left turns (FR9), STRAIGHT / NONE / INVALID = no legs. */
IMPORT void vc_plan_barcode(UB barcode, VC_LEGS *p_legs);

#endif /* VC_PLAN_H */

/* ======== from vc_bypass.h ======== */
/*
 *	vc_bypass.h
 *	Vehicle Controller -- going round an obstacle one move at a time
 *	(12 Nov; replaces the fixed 4-move route of vc_plan.c).
 *
 *	Before: the route was fixed once the scan was done (turn, side-step,
 *	turn back, drive past an obstacle ASSUMED 250 mm deep, then cut back
 *	in by dead reckoning).  A longer obstacle, or one with a part
 *	sticking out further back, was driven into.
 *
 *	Now the car feels its way round and decides after every move
 *	(16 Nov):
 *
 *	  1. scan (Obstacle Scan): which side, how wide
 *	  2. turn 90 deg to that side, side-step, turn back      "the lane"
 *	     out by half the obstacle's width (as the scan measured it, at
 *	     least min_width_mm) + half the car + margin_mm (17 Nov), and
 *	     at least side_step_mm
 *	  3. CHECK, standing: Obstacle Scan points its servo towards the
 *	     obstacle's side (the car does not turn) and says how far
 *	     sideways the nearest thing there is, and how far ahead the
 *	     furthest one near the car is.  Where it was seen is
 *	     remembered.
 *	  4. along the lane: first_pass_mm (30 cm) the first time, then
 *	     pass_step_mm, or on until the car is past where the obstacle
 *	     was seen (or must be: its front + min_depth_mm) if that is
 *	     further; CHECK again
 *	       still beside it -> on again
 *	  5. clear and past it: turn VC_BYP_IN_DEG (70 deg since 17 Nov, was
 *	     45) towards the line (Obstacle Scan
 *	     watches that side and stops the turn if the obstacle is still
 *	     close) and LOOK along the way to the line (standing; the servo
 *	     then reaches almost square onto the obstacle's side, where a
 *	     flat face echoes -- from the lane only its corners do)
 *	       not free: turn back, further on along the lane, check again
 *	       (17 Nov: no longer a side-step further out as well)
 *	  6. free: drive diagonally until the LINE SENSORS see the line (not
 *	     by distance), put the wheels on it, turn the same angle back to
 *	     the old direction.  Line and Barcode then takes over (line search,
 *	     a small sweep).
 *	  So at least: out, along, in (right, left, left -- or the other
 *	  way round), and lining up on the line.
 *
 *	Obstacle Scan watches during every forward move (ahead and to both
 *	front corners).  If it stops the car:
 *	  in the lane (4)      -> something ahead: scan again and side-step
 *	                          further out (a part of the obstacle sticks
 *	                          out, or there is a second obstacle);
 *	                          only close to the corner on the obstacle's
 *	                          side: move out (jog_mm, or more if it
 *	                          was seen nearer).  These side-steps from
 *	                          the lane go 45 deg out and diagonally, not
 *	                          90 deg (16 Nov).
 *	  side-stepping from the line -> go back and try the other side
 *	                          (once)
 *	  on the way back (5)  -> back into the lane, further along, check
 *	                          again
 *	It gives up (FAIL -> the VC goes to FAULT, "no way round") when the
 *	car would get too far from the line, the obstacle does not end
 *	within max_pass_mm, or after max_plans scans.
 *
 *	Pure logic: stdint / stdbool only, no kernel, no hardware, no
 *	globals.  The VC task (task_vc.c) feeds it events and carries out
 *	the action it answers with.  Tested on a PC with a simulated car
 *	(vc_bypass_check.c) and on the car (vc_test.c).
 */

#ifndef VC_BYPASS_H
#define VC_BYPASS_H

#include <stdbool.h>
#include <stdint.h>

/* Turning in towards the line and crossing back to it: this many deg
   from the lane (17 Nov: 70, was 45.  At 45 the car came back onto the
   line about 27 cm past a 15 cm deep obstacle, at 70 about 18 cm: the
   steeper it crosses, the less it moves on meanwhile).  Not much above
   70: the car lines up on the line by turning on the spot, and its
   tail (11 cm from the axle) needs that room behind the obstacle.
   Set all three together (sin and cos x 1000); for a test e.g.
   -DVC_BYP_IN_DEG=45 -DVC_BYP_IN_SIN_X1000=707 -DVC_BYP_IN_COS_X1000=707 */
#ifndef VC_BYP_IN_DEG
#define VC_BYP_IN_DEG		70
#define VC_BYP_IN_SIN_X1000	940
#define VC_BYP_IN_COS_X1000	342
#endif

/* Sides: the same values as S5_SIDE_xxx (subsystem5.h) */
#define VC_BYP_LEFT		1u
#define VC_BYP_RIGHT		2u

/* ---- What the VC task has to do next (vc_byp_act_t.kind).  1..4 are
   the same values as VC_LEG_xxx (vc_plan.h). ---- */
#define VC_BYP_ACT_WAIT		0u	/* the scan is running: wait for it  */
#define VC_BYP_ACT_FWD		1u	/* amount = mm                       */
#define VC_BYP_ACT_BACK		2u	/* amount = mm                       */
#define VC_BYP_ACT_LEFT		3u	/* amount = deg                      */
#define VC_BYP_ACT_RIGHT	4u	/* amount = deg                      */
#define VC_BYP_ACT_LOOK		5u	/* standing; amount = VC_BYP_LEFT /
					   _RIGHT, the side to look at.  How
					   far is it free ahead (the car's
					   width)?  With VC_BYP_F_SIDE (the
					   check in the lane): how far sideways
					   is the nearest thing on that side?
					   Answer: VC_BYP_EV_LOOK             */
#define VC_BYP_ACT_SCAN		6u	/* a full scan (START_SCAN)          */
#define VC_BYP_ACT_DONE		7u	/* round it: search for the line     */
#define VC_BYP_ACT_FAIL		8u	/* no way round (vc_byp_t.fail)      */

/* vc_byp_act_t.flags, for FWD (and GUARD for LEFT / RIGHT) */
#define VC_BYP_F_GUARD		0x01u	/* Obstacle Scan watches meanwhile:
					   FWD: ahead and the car's own width;
					   LEFT / RIGHT (15 Nov, the turn
					   towards the line): the side it
					   turns to, and stops the turn if
					   something is that close            */
#define VC_BYP_F_LINE		0x02u	/* stop when the line sensors see the
					   line                              */
#define VC_BYP_F_WIDE		0x04u	/* with GUARD, along the lane: also
					   keep room on the obstacle's side
					   (not used since 16 Nov)            */
#define VC_BYP_F_SIDE		0x08u	/* LOOK: the side check (step 4)     */

/* ---- What happened (vc_byp_event) ---- */
#define VC_BYP_EV_PLAN		1u	/* scan: go round.  a = side, b = cm to
					   it, c = its width cm, d = room cm  */
#define VC_BYP_EV_CLEAR		2u	/* scan: nothing in the way          */
#define VC_BYP_EV_NO_PATH	3u	/* scan: no room on either side      */
#define VC_BYP_EV_ARRIVED	4u	/* the move finished.  a = mm driven */
#define VC_BYP_EV_STOPPED	5u	/* Obstacle Scan stopped the move.
					   a = mm driven (a turn: deg turned),
					   b = VC_BYP_RAY_xxx, c = cm away it
					   was seen (0 = not known)           */
#define VC_BYP_EV_LINE		6u	/* the line sensors stopped the move.
					   a = mm driven                     */
#define VC_BYP_EV_LOOK		7u	/* a = cm free straight ahead (0 = no
					   reading, VC_BYP_CM_OPEN = nothing).
					   The side check: a = cm sideways to
					   the nearest thing on that side
					   (0 / OPEN the same), b = cm ahead of
					   the ultrasonic the furthest thing
					   seen on that side is               */
#define VC_BYP_CM_OPEN		511u

/* What Obstacle Scan saw when it stopped a move (it watches straight
   ahead and towards both front corners) */
#define VC_BYP_RAY_AHEAD	0u	/* straight ahead                    */
#define VC_BYP_RAY_INNER	1u	/* at the corner on the obstacle's side */
#define VC_BYP_RAY_OUTER	2u	/* at the other corner               */

/* Why it gave up (vc_byp_t.fail) */
#define VC_BYP_FAIL_NONE	0u
#define VC_BYP_FAIL_NO_ROOM	1u	/* blocked on both sides, or too far
					   from the line                     */
#define VC_BYP_FAIL_TOO_LONG	2u	/* the obstacle does not end         */
#define VC_BYP_FAIL_PLANS	3u	/* stopped and scanned too often     */
#define VC_BYP_FAIL_LOGIC	4u	/* an event that cannot happen here  */

/* The car and the limits (task_vc.c fills it from vehicle_ctrl.h) */
typedef struct {
	uint16_t	car_width_mm;
	uint16_t	car_length_mm;
	uint16_t	side_step_mm;	/* out from the line at least this    */
	uint16_t	margin_mm;	/* room between the car's side and the
					   obstacle in the lane               */
	uint16_t	min_width_mm;	/* an obstacle is taken to be at least
					   this wide (for how close it is)    */
	uint16_t	first_pass_mm;	/* along the lane before the 2nd check */
	uint16_t	pass_step_mm;	/* still beside it: this much further */
	uint16_t	jog_mm;		/* too close to its side while driving
					   along the lane: out by this        */
	uint16_t	sensor_ahead_mm; /* line sensors ahead of the wheels  */
	uint16_t	us_ahead_mm;	/* the ultrasonic ahead of the wheels */
	uint16_t	min_depth_mm;	/* an obstacle is taken to be at least
					   this deep (its back cannot be seen
					   from beside it)                    */
	uint16_t	side_near_mm;	/* the check: something sideways up
					   to the line + this = still beside  */
	uint16_t	look_margin_mm;	/* the look: free up to the line + this
					   = the way is free                  */
	uint16_t	cross_extra_mm;	/* crossing back: at most 1.25 x the
					   way to the line + this             */
	uint16_t	backup_mm;	/* no room either side: back up, rescan */
	uint16_t	max_side_mm;	/* furthest from the line             */
	uint16_t	max_pass_mm;	/* furthest along the obstacle        */
	uint8_t		max_plans;	/* scans for one obstacle             */
} vc_byp_cfg_t;

typedef struct {
	uint8_t		kind;		/* VC_BYP_ACT_xxx   */
	uint8_t		flags;		/* VC_BYP_F_xxx     */
	uint16_t	amount;		/* mm, deg or side  */
} vc_byp_act_t;

typedef struct {
	vc_byp_cfg_t	cfg;
	uint8_t		ph;		/* where in the bypass (vc_bypass.c)  */
	int8_t		side;		/* +1 left, -1 right, 0 not chosen    */
	int32_t		x_mm;		/* along the old path since the stop  */
	int32_t		y_mm;		/* sideways from it, + = left         */
	int32_t		step_mm;	/* this side-step                     */
	int32_t		going_mm;	/* this move along the lane           */
	int32_t		far_x_mm;	/* the obstacle reaches at least this
					   far along (x)                      */
	int32_t		gap_mm;		/* from the middle of the car's path
					   in the lane to the obstacle's side  */
	uint8_t		out_deg;	/* this side-step: 90 or 45 deg out   */
	bool		turn_after;	/* this move along the lane is the last:
					   then turn in                      */
	bool		first_due;	/* the first pass is still to come   */
	bool		rescan;		/* back parallel, then scan again    */
	uint8_t		plans;		/* scans answered with PLAN           */
	uint8_t		jogs;		/* short side-steps out               */
	uint8_t		fail;		/* VC_BYP_FAIL_xxx                    */
	bool		tried_other;	/* the other side was tried already   */
	bool		line_seen;	/* the line sensors found the line    */
} vc_byp_t;

/* A new obstacle: the car has stopped in front of it and the scan is
   starting.  p_cfg is copied. */
void vc_byp_start(vc_byp_t *p_b, const vc_byp_cfg_t *p_cfg);

/* Feed one event (VC_BYP_EV_xxx with its values, 0 where unused);
   *p_act is what to do next. */
void vc_byp_event(vc_byp_t *p_b, uint8_t ev, uint32_t a, uint32_t b,
		  uint32_t c, uint32_t d, vc_byp_act_t *p_act);

/* The scan saw no room, the car backed away, and now the obstacle is out
   of the scan's range: it is still there (the VC counts these) */
bool vc_byp_backed_off(const vc_byp_t *p_b);

/* The car is on its way back to the line (telemetry: REACQUIRE) */
bool vc_byp_returning(const vc_byp_t *p_b);

/* The side it is going round on: +1 left, -1 right, 0 not chosen yet */
int8_t vc_byp_side(const vc_byp_t *p_b);

/* Short name of where it is, for the console */
const char *vc_byp_phase_name(const vc_byp_t *p_b);

#endif /* VC_BYPASS_H */


#endif /* VEHICLE_CTRL_H */
