/*
 *	subsystem3.h
 *	Subsystem 3 -- Line and Barcode (Buddy 3).
 *
 *	Public interface of subsystem 3.  Other subsystems include it as
 *	"../subsystem3/subsystem3.h".
 *
 *	Buddy 3's core logic (s3_ir, s3_line, s3_pulse, s3_state; see
 *	README.md) runs inside line_task (task_line.c), which samples the IR
 *	sensors every LF_PERIOD_MS and:
 *	  - publishes the latest steering in a mutex-protected mailbox
 *	    (steer_mtx, Week 6 design); the Vehicle Controller reads it with
 *	    s3_steer_get() and drives the wheels at speed -/+ steer;
 *	  - reports LINE_LOST / LINE_REACQUIRED / LINE_TIMEOUT / SENSOR_FAULT
 *	    to the Vehicle Controller on nav_q (vehicle_ctrl.h);
 *	  - takes SUSPEND / SEARCH / FOLLOW / RESET from the Vehicle
 *	    Controller on line_cmd_q (owned here);
 *	  - with S3_BARCODES 1 (line_cfg.h; OFF for now): reads Code 39
 *	    barcodes on IR_BARCODE (GP28) while following (bc_reader.c,
 *	    s3_code39.c) and sends BARCODE to the Vehicle Controller on
 *	    nav_q; letters -> commands in barcode_map.h.
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"

/* ---------------------------------------------------------------------
 *  Template example switch.  The template's example chain (producer in
 *  subsystem 2 -> consumer in subsystem 3 -> monitor in subsystem 4)
 *  stopped working when Motion Control replaced the producer, so it is
 *  switched off.  0 = Line and Barcode runs here.  The example code
 *  stays in task_consumer.c, inside #if TEMPLATE_DEMO, for reference.
 * --------------------------------------------------------------------- */
#ifndef TEMPLATE_DEMO
#define TEMPLATE_DEMO		0
#endif

/* ---------------------------------------------------------------------
 *  Line and Barcode
 * --------------------------------------------------------------------- */
#define S3_PRI_LINE		16	/* priority band 16-17 (TEAM.md) */

/* IF-17: VC -> Line and Barcode, 4-byte words (vehicle_ctrl.h:
   VC_LINE_CMD_xxx in bits 31..29).  Owned here. */
IMPORT ID	line_cmd_q;

/* IF-19: latest steering, written by line_task, read by the Vehicle
   Controller.  Read it ONLY through s3_steer_get(). */
IMPORT ID	steer_mtx;

typedef struct {
	H	steer_mm_s;	/* + = turn left, - = turn right               */
	B	search_dir;	/* +1 = look for the line to the left, -1 right */
	UB	state;		/* Buddy 3's state = line_state_t (telem_format.h):
				   0 INIT 1 FOLLOWING 2 BARCODE_SCAN 3 DECODE
				   4 SUSPENDED 5 SEARCH 6 FAULT                 */
	UB	ir_bits;	/* 1 = black: bit 0 left, 1 centre, 2 barcode   */
	UB	flags;		/* S3_STEER_xxx                                 */
	UW	stamp_ms;	/* when line_task wrote it                      */
} S3_STEER;

#define S3_STEER_PRESENT	0x01U	/* line under / next to the sensors */
#define S3_STEER_CALIBRATED	0x02U	/* centre sensor has seen black and white */
#define S3_STEER_FOLLOWING	0x04U	/* steering is valid: use it          */

/* Copy of the latest steering (bounded wait on steer_mtx).  E_OK, or an
   error and *p_steer unchanged. */
IMPORT ER	s3_steer_get(S3_STEER *p_steer);

/* Private to subsystem 3 */
IMPORT void	line_task(INT stacd, void *exinf);
IMPORT void	s3_steer_put(const S3_STEER *p_steer);
IMPORT ER	s3_ir_status;	/* ir_init() result, for the start-up check */

#if TEMPLATE_DEMO
/* ---------------------------------------------------------------------
 *  Template example (off)
 * --------------------------------------------------------------------- */
#define S3_PRI_CONSUMER		16	/* priority band 16-17 (TEAM.md) */
#define S3_BATCH_SIZE		5	/* report to subsystem 4 every N samples */

/* IF-01: send SAMPLE_MSG (subsystem2.h) here with tk_snd_mbx */
IMPORT ID	mbx_samples;

/* IF-04: statistics.  Read them ONLY while holding mtx_stats:
 *	tk_loc_mtx(mtx_stats, TMO_FEVR);  copy = app_stats;  tk_unl_mtx(mtx_stats);
 */
typedef struct {
	UW	received;	/* samples received so far */
	INT	min;
	INT	max;
	W	sum;
} APP_STATS;

IMPORT ID		mtx_stats;
IMPORT APP_STATS	app_stats;

IMPORT void	consumer_task(INT stacd, void *exinf);
#endif /* TEMPLATE_DEMO */

/* ======== from line_cfg.h ======== */
/*
 *	line_cfg.h
 *	Subsystem 3 (Line and Barcode) -- pins and tuning for line following.
 *
 *	Pins: Week 6 pin map (TEAM.md section 2).  MH-series IR modules
 *	(TCRT5000), A0 output, powered from 3.3 V so A0 never exceeds the
 *	ADC's 3.3 V limit.  On the car (2 Nov photo), from behind, left to
 *	right:
 *	  GP26  ADC0  side sensor, out on the left, a little further back:
 *	              barcodes (and T-junctions later); never steers
 *	  GP27  ADC1  centre: left one of the two line sensors
 *	  GP28  ADC2  right:  right one of the two line sensors
 *	(The Week 6 map named them IR_LEFT / IR_CENTER / IR_BARCODE with the
 *	barcode on the right; the car reads barcodes on the LEFT.  The jobs
 *	are set by LF_CH_xxx below.)
 *
 *	How the car follows the line (two sensors, Week 6 design):
 *	  The CENTRE sensor rides the line's LEFT EDGE: half on the line it
 *	  reads half dark.  Darker = the car has drifted right (more of the
 *	  line under it): steer left.  Lighter = drifted left: steer right.
 *	  Steering is proportional to how far it is from half dark.
 *	  The LEFT sensor normally sees white; if it sees black the car has
 *	  drifted far right (or a sharp left bend): steer left hard.
 *	  Steering is in mm/s: the Vehicle Controller drives the left wheel
 *	  at speed - steer and the right wheel at speed + steer.
 *
 *	Calibration (Week 6 INIT state) is automatic: each sensor remembers
 *	the lightest and darkest value it has seen since power-up (or RESET);
 *	once those are far enough apart (LF_MIN_SPAN) they are its white and
 *	black.  Moving the sensors across the line once is enough: slide the
 *	car sideways over the line by hand, or let the car's search sweep do
 *	it after START.
 *
 *	All values are STARTING VALUES: tune them on the track and record
 *	the changes in testing/subsystem3/TEST_LOG.md.
 */

#ifndef LINE_CFG_H
#define LINE_CFG_H

/* ---- Sensors ---- */
#define LF_PIN_LEFT		26u	/* ADC channel 0 */
#define LF_PIN_CENTRE		27u	/* ADC channel 1 */
#define LF_PIN_BARCODE		28u	/* ADC channel 2 */

/* Which sensor is on which pin.  Looking from BEHIND the car (the
   ultrasonic sensor at the front), left to right:
     side sensor   out on the left: barcodes (and T-junctions later);
                   never used for steering
     centre        the left one of the two line sensors in the middle
     right         the right one of the two
   Channels: 0 = GP26, 1 = GP27, 2 = GP28.  2 Nov, from the car's own
   readings: GP26 always white (the side sensor), GP27 and GP28 take
   turns on the line.  Telemetry ir= and the dashboard show them in
   pin order: side, centre, right. */
#define LF_CH_SIDE		0u	/* GP26 */
#define LF_CH_PAIR_LEFT		1u	/* GP27: centre */
#define LF_CH_PAIR_RIGHT	2u	/* GP28: right  */
#if (LF_CH_SIDE == LF_CH_PAIR_LEFT) || (LF_CH_SIDE == LF_CH_PAIR_RIGHT) || \
    (LF_CH_PAIR_LEFT == LF_CH_PAIR_RIGHT) || (LF_CH_SIDE > 2u) || \
    (LF_CH_PAIR_LEFT > 2u) || (LF_CH_PAIR_RIGHT > 2u)
#error "LF_CH_SIDE / LF_CH_PAIR_LEFT / LF_CH_PAIR_RIGHT: use 0, 1 and 2, each once"
#endif

/* MH / TCRT5000 module: the phototransistor pulls A0 low when it sees
   reflected IR (white), so A0 is HIGH over black.  If the ir= bits in
   telemetry show 1 over WHITE, set this to 0. */
#define LF_BLACK_IS_HIGH	1

/* ---- Timing ---- */
#define LF_PERIOD_MS		10u	/* sample period: one kernel tick (Week 6
					   asked ~2 ms; the 10 ms tick cannot do
					   that, see subsystem3/README.md gate 5) */
#define LF_SEARCH_MS		4000u	/* then this long to find it again (the
					   car's search sweep, vehicle_ctrl.h)
					   before LINE_TIMEOUT: the car stops in
					   FAULT.  FR19 said 2 s; 4 s since 2 Nov
					   so the sweep can cover both sides.
					   Keep VC_LINE_TIMEOUT_MS the same      */
#define LF_LOST_MS		600u	/* no line for this long = LINE_LOST.
					   Bridges gaps of 1 cm (NFR10): 1 cm
					   takes 0.5 s at the 20 mm/s patrol
					   (was 300 at 150 mm/s)              */

/* ---- Calibration ---- */
#define LF_ADC_MAX		4095u
#define LF_MIN_SPAN		400u	/* raw counts between white and black
					   before a sensor counts as calibrated */
#define LF_HYST_DIV		8u	/* hysteresis = span / 8 */
/* Black or white (the ir= bits, which sensor "sees the line"): black
   above LF_BLACK_PCT % of the way from the sensor's white to its black,
   with LF_CLS_HYST_DIV hysteresis.  Was 50 % with span/8 hysteresis
   (black only above ~62 %): a sensor half on the line, or a weaker one,
   was missed (2 Nov).  The steering still uses the full 0..1000 scale. */
#define LF_BLACK_PCT		35u
#define LF_CLS_HYST_DIV		16u

/* ---- Line position and steering ---- */
#define LF_EDGE_TARGET		500u	/* centre sensor: 0 white .. 1000 black;
					   500 = half on the line's edge       */
#define LF_NEAR_LEVEL		150u	/* centre this dark = line right next to it */
#define LF_KP_PER_1000		30	/* steer mm/s per 1000 darkness units
					   beyond the dead band: 30 -> at most
					   +-12 mm/s on the edge sensor alone.
					   Lower = smoother, higher = tighter on
					   bends                               */
#define LF_EDGE_DEADBAND	100u	/* centre within 400..600 dark (on the
					   edge): steer 0, drive straight.
					   0 = no dead band                    */
#define LF_STEER_HARD		21	/* mm/s: the most the car ever steers
					   (left sensor on the line, line gone
					   off to the left).  Kept BELOW the
					   patrol speed (25): no wheel ever
					   stops while following -- wheels at
					   4 and 46 mm/s at most.  2 Nov: the
					   timed hard turns that stopped a wheel
					   were removed (jerky).  4 Nov: 15 ->
					   21 for tight bends (see
					   LF_C_KP_PER_1000)                   */

/* ---- How to follow the line (LF_MODE) ----
   LF_MODE_CENTRE (2 Nov, default): keep the line under the two middle
   sensors (centre and right; never the side sensor).  Steering =
   centre darkness - right darkness (0..1000 each):
     centre darker -> the line is moving left  -> steer left
     right darker  -> the line is moving right -> steer right
     equal         -> straight (both on the line, or a crossing)
     both white    -> off the line: turn at once towards the side that
                      saw it last (LF_STEER_HARD); still nothing after
                      LF_LOST_MS -> LINE_LOST, the car's search sweep
                      (LF_STRADDLE_MS > 0: drive straight that long
                      first -- only for sensors further apart than the
                      line, where both white can mean "line between")
   Same for left and right bends.
   LF_MODE_EDGE (before 2 Nov): the right sensor rides the line's left
   edge (LF_KP_PER_1000, LF_EDGE_DEADBAND).  It lost left bends: once
   the right sensor crossed the whole line it steered the wrong way. */
#define LF_MODE_EDGE		0
#define LF_MODE_CENTRE		1
#define LF_MODE			1
/* 4 Nov, tight bends (the course's R110 hairpin, R150 S-bend): the car
   lost the line and fell back on the search sweep every few seconds on
   them.  To follow a bend of radius R at speed v the wheels must differ
   by v x 120 mm / R: at 25 mm/s on R110 that is a steer of 14 mm/s, and
   more, because the sensors are well ahead of the wheels and the car has
   to cut inside the bend to keep them on the line.  With 12 here the
   steering reached at most 11 mm/s before the line left the sensors.
   Simulated (that hairpin and S-bend, sensors 60-110 mm ahead of the
   wheels): 22 with LF_STEER_HARD 21 lost the line 0-4 times on the
   hairpin instead of 6-19; straights unchanged (the dead band).  The
   dashboard's speed preset keeps these ratios (0.9 x and 0.85 x the
   speed).  If it wobbles on straights, step back to 18. */
#define LF_C_KP_PER_1000	22	/* centre mode: steer mm/s per 1000 of
					   (left - right) darkness beyond the dead
					   band: 20 -> up to 18 mm/s.  About 0.5 x
					   the patrol speed.  Higher = tighter
					   bends, lower = smoother             */
#define LF_C_DEADBAND		100u	/* centre mode: a difference below this
					   (0..1000) = straight on             */
#define LF_STRADDLE_MS		0u	/* centre mode: both white -> drive
					   straight this long before turning to
					   find the line.  0 = turn at once (the
					   centre and right sensors sit next to
					   each other, so both white = off the
					   line).  2 Nov 22:30 it was "never",
					   and the car drove past the line      */
#define LF_C_SIDE_LEVEL		150	/* centre mode: a difference this big
					   shows which side the line is on (for
					   turning back and the search)        */

/* ---- Crossings (junction detection, Write-up: Buddy 3 tasks) ----
   The course crosses itself.  At a crossing the crossing tape reaches
   the left sensor while the centre sensor is still riding the line's
   edge (or both go dark together).  Drifting right never looks like
   that: the centre sensor goes fully black, then white, long before the
   left one reaches the line.  So: the left sensor going dark within
   LF_CROSS_WINDOW_MS of the centre being on the edge is a crossing (or a
   mark beside the line, such as a barcode's bars): the car drives
   straight on until both sensors are clear again for LF_CROSS_CLEAR_MS,
   for at most LF_CROSS_MAX_MS.  Then normal steering.
   A crossing met at a shallow angle from the right (centre sensor first)
   can still pull the car left: check on the mat. */
#define LF_DARK_LEVEL		850u	/* centre this dark = fully on black */
#define LF_CROSS_WINDOW_MS	150u	/* kept at 10 mm/s: a longer window
					   would take more right drifts for
					   crossings (line_check LF-14)      */
#define LF_CROSS_CLEAR_MS	40u
#define LF_CROSS_MAX_MS		1500u	/* tape 18 mm at 25 mm/s = 0.7 s
					   (2500 at 10 mm/s)                */

/* ---- Barcodes: Code 39 on the side sensor (LF_CH_SIDE, GP26) ----
   0 = no barcode reading: the side sensor only shows in telemetry, and
       none of the barcode code is in the firmware.
   1 = read barcodes (task_line.c, bc_reader.c, s3_code39.c), while
       following the line and while standing (push the car over one by
       hand: read and shown, never acted on).  Each read goes to the
       dashboard (MQTT is22/bc: letter, command, widths).  Which letter
       means which command: barcode_map.h (the dashboard's barcode
       table).  Whether the car turns on one: VC_BARCODE_ACT in
       vehicle_ctrl.h (2 = at the next junction, LF_JCT_xxx below).
   The reading itself: capture starts on the first bar, ends after a
   white run longer than 2 x the widest and 5 x the narrowest bar/space
   so far (Code 39's quiet zone is 10 narrow; its widest element 3
   narrow), clamped to LF_BC_QUIET_MIN/MAX_US.
   The course's 164.5 mm "K" (narrow 3.5 mm, wide 10.5 mm): at 20 mm/s
   a narrow bar lasts 175 ms (17 samples), a wide one 525 ms, and the
   end is seen 1.05 s (21 mm) after the last bar.  The limits below
   still read it at 10 mm/s (wide 1.05 s; bc_check BC-08). */
#ifndef S3_BARCODES
#define S3_BARCODES		1
#endif
/* When barcodes are read (3 Nov): only while the car follows the line
   (after START).  1 = also while it stands (IDLE / STOP), pushed over a
   barcode by hand.  0 by default: picking the car up to carry it back
   made the side sensor "see" bars that were not there. */
/* 1 = send the side sensor's raw reading at every sample (10 ms) to the
   dashboard (MQTT is22/ir, 2 messages a second), to see exactly what it
   sees over a barcode (3 Nov debugging).  0 = off. */
#ifndef S3_SIDE_TRACE
#define S3_SIDE_TRACE		1
#endif

#ifndef LF_BC_READ_STANDING
#define LF_BC_READ_STANDING	1
#endif
#define LF_BC_GAP_US		100000u	/* samples further apart: give up    */
#define LF_BC_RUN_MAX_US	3000000u /* one bar/space longer: not a code  */
#define LF_BC_QUIET_MIN_US	150000u
#define LF_BC_QUIET_MAX_US	2500000u
/* Black/white for the SIDE sensor only, from its own lightest/darkest
   (usually the black tape).  3 Nov car log: the printed bars read much
   lighter than the tape, so at 50 % only the wide bars just showed and
   the narrow ones never did.  35 % sees bars about half as dark as the
   tape; the exact widths do not matter much any more, because a failed
   reading is read again from the recording with the barcode's OWN
   lightest and darkest values (bc_rec.c).  Simulated (bc_check BC-10):
   reads the K with bars 45-100 % as dark as the tape, 4 mm blur,
   10-40 mm/s.  The line sensors use LF_BLACK_PCT.
   4 Nov: 20 % (more sensitive than the line sensors' 35 %), asked for
   while the side sensor is made physically less sensitive to black.
   Dashboard: Tuning, Barcodes, "Side sensor black at" (20-80). */
#ifndef LF_BC_BLACK_PCT
#define LF_BC_BLACK_PCT		20u
#endif
#define LF_BC_HYST_DIV		32u	/* hysteresis = span / 32 (+-3 %)   */
/* Recording window for the second look (bc_rec_watch): starts when the
   side sensor reads LF_BC_ACT_PCT % of its white..black span darker than
   white -- but never needs more than LF_BC_ACT_MAX_RAW counts, because
   lifting the car makes every sensor read "black" (no reflection) and
   that empty-air reading can become the calibrated black (3 Nov) --
   and ends LF_BC_REC_TAIL_MS after the last such sample. */
#define LF_BC_ACT_PCT		15u
#define LF_BC_ACT_MAX_RAW	250u
#define LF_BC_REC_TAIL_MS	2500u
#define LF_BC_MIN_OWN_SPAN	100u	/* raw counts between the barcode's own
					   lightest and darkest, at least     */

/* Side sensor drop-outs (4 Nov car log): a loose wire made the side
   sensor read a flat ~150 (as if very white), falling to it and coming
   back within one or two samples, even in the middle of a black bar.
   A real edge under the sensor takes 5-10 samples at barcode speed.
   Samples below LF_BC_DEAD_RAW that are entered and left that way (a
   step of at least LF_BC_DEAD_JUMP, 4 x any step next to it) are not
   used when a recording is read (bc_rec_find_dropouts).  0 = off.
   4 Nov 10:34: the cause was found -- the centre line sensor's DO wire
   shared GP26 (Grove GP26/GP27) and pulled it low over white.  With that
   wire off the side sensor's real white is ~240, where the "drop-out"
   level was, so this is off (450 would treat sharp white gaps as
   drop-outs). */
#define LF_BC_DEAD_RAW		0u
#define LF_BC_DEAD_JUMP		400u

/* Bar widths by DISTANCE, not time (task_line.c s3_bc_clock, 4 Nov).
   Code 39 is read from how long each bar and space lasts, which only
   works if the side sensor crosses the barcode at an even speed.  It
   does not: the line following weaves, and the 4 Nov log shows the left
   wheel (the side sensor's side) going between 0 and 30 mm/s over the
   barcode, so a thin bar could last longer than a thick one.  With 1,
   while the car drives (or is pushed by hand) the reader's clock is the
   distance that wheel travels, LF_BC_DIST_MM_S worth of time per mm, so
   every limit above that is in ms still means the same at that speed
   (the quiet zone, gaps, LF_BC_REC_TAIL_MS = 62 mm at 25 mm/s).  A
   barcode slid under the standing car is timed as before.
   LF_BC_DIST_WHEEL: 0 left wheel (the side sensor is on the left, just
   outside that wheel), 1 right, 2 both averaged.
   LF_BC_UM_PER_EDGE: wheel travel per encoder edge, um: pi x 65 mm /
   628 edges (subsystem2/motion_config.h); a few % off only rescales the
   limits.  LF_BC_DIST_HOLD_MS: standing, the wheel turned this recently =
   being pushed. */
#ifndef LF_BC_BY_DISTANCE
#define LF_BC_BY_DISTANCE	1
#endif
#define LF_BC_DIST_WHEEL	0
#define LF_BC_DIST_MM_S		25u
#define LF_BC_UM_PER_EDGE	325u
#define LF_BC_DIST_HOLD_MS	300u

/* ---- Junctions (15 Nov): where a barcode's turn is made ----
   The course's junctions are full crosses ("+"): the crossing tape
   (18 mm wide) reaches out past the side sensor on the left.  So a
   junction = ONE dark run of LF_JCT_RUN_MM .. LF_JCT_MAX_MM under the
   side sensor, with at least LF_JCT_GAP_MM of white before AND after it,
   while the middle pair is on the line.  Reported LF_JCT_GAP_MM after
   the tape, so the car is then about 24 mm past its middle.
   A barcode never looks like that: its widest bar is 10.5 mm (9 mm on
   the course's 141 mm ones) and inside it no white is longer than
   10.5 mm, so even two bars run together across a narrow space (a
   blurry sensor) have a short white on one side (line_check LF-25,
   LF-26: every letter, both ways).
   -> nav_q VC_NAV_JUNCTION, once per crossing.  The VC makes the turn
   the last barcode asked for there (VC_BARCODE_ACT 2, vehicle_ctrl.h).
   Measured by the barcode clock (wheel travel, LF_BC_BY_DISTANCE). */
#ifndef S3_JUNCTIONS
#define S3_JUNCTIONS		1
#endif
#define LF_JCT_RUN_MM		14u	/* dark at least this long ...        */
#define LF_JCT_MAX_MM		40u	/* ... and at most this: the crossing
					   tape (longer: running along a line) */
#define LF_JCT_GAP_MM		15u	/* white before and after it          */
#define LF_JCT_US(mm)		((uint32_t)(((mm) * 1000000u) / LF_BC_DIST_MM_S))

#endif /* LINE_CFG_H */

/* ======== from barcode_map.h ======== */
/*
 *	barcode_map.h
 *	WHICH BARCODE MEANS WHAT -- set on the day.
 *
 *	Easiest: the dashboard's "Barcodes" table (pick an action for each
 *	letter, Save, then Build + flash).  It rewrites the four lines below.
 *
 *	The car reads Code 39 barcodes: letters A-Z, digits 0-9 and - . $ / + %
 *	on the side sensor (GP26, out on the left).
 *
 *	By hand: put each barcode's letter between the quotes of the command
 *	it stands for.  Several letters for one command: put them all, e.g.
 *	"AF".  Nothing for a command: leave the quotes empty, "".
 *	A letter must not be in two lines.  A letter in no line: the car
 *	reports INVALID (no command) and carries on.
 *
 *	Reading is on (S3_BARCODES 1 in line_cfg.h).  Whether the car TURNS
 *	on a barcode: VC_BARCODE_ACT in vehicle_ctrl.h (0 = read only).
 *
 *	Which letter is a barcode?  Put the barcode beside the line, under
 *	the side sensor's path, and let the car drive over it after START:
 *	the dashboard shows the letter (the console prints, for example,
 *	    S3: barcode *A* -> LEFT ).
 *	Barcodes are only read while the car follows the line; to read them
 *	by pushing the standing car over them, set LF_BC_READ_STANDING 1 in
 *	line_cfg.h (dashboard: Tuning, Barcodes, "Read when pushed by hand").
 */

#ifndef BARCODE_MAP_H
#define BARCODE_MAP_H

#define BARCODE_LEFT_LETTERS		"W"
#define BARCODE_RIGHT_LETTERS		"X"
#define BARCODE_STRAIGHT_LETTERS	"Y"
#define BARCODE_UTURN_LETTERS		"Z"

#endif /* BARCODE_MAP_H */

/* ======== from s3_ir.h ======== */
/* Buddy 3: hardware-independent, caller-calibrated IR classification. */
#ifndef S3_IR_H
#define S3_IR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint16_t white;
    uint16_t black;
    uint16_t hysteresis;
} s3_ir_cal_t;

/* References are measured on known surfaces. No electrical polarity assumed.
 * Hysteresis is in raw input units; min_span must be measured/selected by caller.
 * On failure the destination is not changed. */
bool s3_ir_calibrate(s3_ir_cal_t * p_cal, uint16_t white, uint16_t black,
                     uint16_t min_span, uint16_t hysteresis);

/* Returns false for invalid calibration/pointers. b_black is in/out state;
 * b_known=false initializes using the midpoint, without historical hysteresis. */
bool s3_ir_classify(s3_ir_cal_t const * p_cal, uint16_t raw,
                    bool b_known, bool * p_black);

/* Normalize measured reflectance: white=0, black=1000, clamped.
 * This is reflectance, NOT steering units or a motor command. */
bool s3_ir_normalize(s3_ir_cal_t const * p_cal, uint16_t raw,
                     uint16_t * p_blackness);

#endif /* S3_IR_H */
/* End of file. */

/* ======== from s3_line.h ======== */
/* Configured two-sensor interpretation; no assumed sensor spacing or steering units. */
#ifndef S3_LINE_H
#define S3_LINE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    /* Index: bit 0 left dark, bit 1 centre dark. Each bit in the masks
     * describes one index. Caller must establish these mappings on the track.
     * Steering values are opaque caller-defined units, NOT PWM values. */
    int16_t steering[4];
    uint8_t present_mask;
    uint8_t steering_valid_mask;
} s3_line_config_t;

typedef struct
{
    int16_t steering;
    /* bits 0..1 sensor pattern; bit 2 line present; bit 3 steering valid */
    uint8_t flags;
} s3_line_result_t;

bool s3_line_evaluate(s3_line_config_t const * p_config, bool b_left_dark,
                      bool b_centre_dark, s3_line_result_t * p_result);
bool s3_line_is_present(s3_line_result_t const * p_result);
bool s3_line_has_steering(s3_line_result_t const * p_result);

#endif /* S3_LINE_H */
/* End of file. */

/* ======== from s3_pulse.h ======== */
/* Bounded pulse capture. Framing and symbol definitions belong to configuration. */
#ifndef S3_PULSE_H
#define S3_PULSE_H

#include <stdbool.h>
#include <stdint.h>

/* Software storage limit, not an assertion about the barcode format. */
#define S3_PULSE_CAPACITY 64u

typedef struct
{
    uint32_t widths_us[S3_PULSE_CAPACITY];
    uint32_t last_edge_us;
    uint32_t last_sample_us;
    uint32_t max_gap_us;
    uint32_t max_width_us;
    uint8_t count;
    /* bit 0 active, bit 1 invalid, bit 2 current dark, bit 3 first dark */
    uint8_t flags;
} s3_pulse_t;

/* Call only on an externally established frame-start edge. Samples and edges
 * are monotonic modulo uint32_t, with gaps/widths below 2^31 microseconds. */
bool s3_pulse_begin(s3_pulse_t * p_capture, uint32_t now_us, bool b_dark,
                    uint32_t max_gap_us, uint32_t max_width_us);
bool s3_pulse_sample(s3_pulse_t * p_capture, uint32_t now_us, bool b_dark);
/* Finish at an established frame boundary; appends final run. Does not infer
 * a quiet zone from the lateral 20 mm track spacing. */
bool s3_pulse_finish(s3_pulse_t * p_capture, uint32_t now_us);

/* Compare a COMPLETE frame with a caller-supplied run pattern (including any
 * start/stop elements). Each expected run is an inclusive duration interval.
 * No encoding, command mapping, reverse policy or speed is assumed. */
bool s3_pulse_matches(s3_pulse_t const * p_capture, bool b_first_dark,
                      uint32_t const * p_min_us, uint32_t const * p_max_us,
                      uint8_t count);

#endif /* S3_PULSE_H */
/* End of file. */

/* ======== from s3_state.h ======== */
/* Pure seven-state transition engine. Values below are internal, NOT queue ABI. */
#ifndef S3_STATE_H
#define S3_STATE_H

#include <stdbool.h>
#include <stdint.h>

#define S3_STATE_INIT 0u
#define S3_STATE_FOLLOW 1u
#define S3_STATE_CAPTURE 2u
#define S3_STATE_DECODE 3u
#define S3_STATE_SUSPENDED 4u
#define S3_STATE_SEARCH 5u
#define S3_STATE_FAULT 6u
#include "subsystem3.h"   /* LF_SEARCH_MS: how long SEARCH may take */
#define S3_STATE_SEARCH_US ((uint32_t)LF_SEARCH_MS * UINT32_C(1000))

typedef enum
{
    S3_STATE_CALIBRATED,
    S3_STATE_BARCODE_START,
    S3_STATE_BARCODE_END,
    S3_STATE_DECODE_DONE,
    S3_STATE_LINE_LOST,
    S3_STATE_LINE_FOUND,
    S3_STATE_SUSPEND,
    S3_STATE_RESUME_SEARCH,
    S3_STATE_RESET,
    S3_STATE_SENSOR_FAULT,
    S3_STATE_POLL
} s3_state_input_t;

#define S3_STATE_EVT_NONE 0u
#define S3_STATE_EVT_LOST 1u
#define S3_STATE_EVT_REACQUIRED 2u
#define S3_STATE_EVT_TIMEOUT 4u
#define S3_STATE_EVT_FAULT 8u

typedef struct
{
    uint32_t search_start_us;
    /* bits 0..2 state; bit 3 calibrated; remaining bits reserved */
    uint8_t packed;
} s3_state_t;

void s3_state_init(s3_state_t * p_state);
uint8_t s3_state_get(s3_state_t const * p_state);
/* Returns internal event bits for the caller to translate to agreed nav_q.
 * Timeout wins at exactly 2 s, except RESET/SUSPEND/fault commands.
 * This is a proposed policy, documented for team review, not physical motion.
 * Call at least once per 2^31 us; timestamps are acquisition/monotonic time. */
uint8_t s3_state_step(s3_state_t * p_state, s3_state_input_t input,
                      uint32_t now_us);
bool s3_state_can_track(s3_state_t const * p_state);

#endif /* S3_STATE_H */
/* End of file. */

/* ======== from s3_code39.h ======== */
/*
 *	s3_code39.h
 *	Subsystem 3 (Line and Barcode) -- Code 39 barcode decoding from the
 *	bar and space widths that s3_pulse captures.
 *
 *	Pure logic (stdint/stdbool only): tested on a PC and on the car
 *	(bc_check.c).
 *
 *	Code 39: every character is 9 elements (bar space bar ... bar), 3 of
 *	them wide; one narrow space separates characters; a symbol starts and
 *	ends with '*'.  The course's barcodes (BARCODE.pptx: A, F, Z, W, X, Y)
 *	are one letter each, "*A*" = 29 elements; this decoder reads them all
 *	(checked against the slides' images).
 *
 *	Widths can be in any unit; the car gives microseconds at roughly
 *	constant speed.  Each character is decoded on its own (its 3 widest
 *	elements are the wide ones), so a slow change of speed across the
 *	barcode does not matter.  The car can cross a barcode either way, so
 *	the reversed order is tried too.
 */

#ifndef S3_CODE39_H
#define S3_CODE39_H

#include <stdbool.h>
#include <stdint.h>

#define C39_MAX_TEXT		4u	/* characters between the '*'s       */
#define C39_MIN_ELEMENTS	29u	/* "*x*": 3 x 9 + 2 gaps             */
#define C39_MAX_ELEMENTS	(((C39_MAX_TEXT + 2u) * 10u) - 1u)	/* 59 */

/* p_widths: count bar/space widths, starting AND ending with a bar (the
   white run after the barcode is not included).
   Writes the characters between the '*'s to p_text (NUL-terminated,
   cap >= 2) and returns how many there are, or 0 if the widths are not
   a valid Code 39 symbol in either direction. */
uint8_t c39_decode(const uint32_t *p_widths, uint8_t count, char *p_text,
		   uint8_t cap);

/* The Code 39 table (the tests use it to draw barcodes, bc_check.c):
   entry idx (0 .. C39_TABLE_SIZE - 1) is the
   character *p_c with its 9 elements in *p_pat, first element = most
   significant bit, 1 = wide.  false past the end of the table. */
#define C39_TABLE_SIZE		44u
#define C39_STAR_PATTERN	0x094u	/* '*' */
bool c39_table(uint8_t idx, char *p_c, uint16_t *p_pat);

#endif /* S3_CODE39_H */

/* ======== from line_follow.h ======== */
/*
 *	line_follow.h
 *	Subsystem 3 (Line and Barcode) -- line following logic: automatic
 *	calibration, black/white per sensor, line present / lost, steering.
 *
 *	Pure logic: no kernel calls, no hardware (stdint/stdbool only), so it
 *	is tested on a PC as well as on the car (line_check.c).  Built on
 *	Buddy 3's modules: s3_ir (calibration, classification with
 *	hysteresis, darkness 0..1000) and s3_line (two-sensor pattern table).
 *	See line_cfg.h for how the steering works and the tuning values.
 */

#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdbool.h>
#include <stdint.h>

#define LF_SENSORS		3u
/* Internal sensor numbers (the pins: line_cfg.h LF_CH_xxx) */
#define LF_LEFT			0u	/* middle pair, left one ("centre") */
#define LF_CENTRE		1u	/* middle pair, right one ("right") */
#define LF_BARCODE		2u	/* side sensor: barcodes            */

/* lf_out_t.ir_bits: 1 = that sensor sees black, by INTERNAL index
   (LF_LEFT = the centre sensor, LF_CENTRE = the right one, LF_BARCODE =
   the side sensor; pins in line_cfg.h).  lf_ir_by_pin() gives them in
   pin order (bit 0 GP26, 1 GP27, 2 GP28) for telemetry. */
#define LF_IR_LEFT		0x1u
#define LF_IR_CENTRE		0x2u
#define LF_IR_BARCODE		0x4u

typedef struct {
	uint16_t	lo[LF_SENSORS];		/* lightest/darkest raw seen:  */
	uint16_t	hi[LF_SENSORS];		/* the automatic calibration   */
	s3_ir_cal_t	cal[LF_SENSORS];	/* white..black: level 0..1000 */
	s3_ir_cal_t	cls[LF_SENSORS];	/* black/white decision (LF_BLACK_PCT) */
	bool		cal_ok[LF_SENSORS];
	bool		dark[LF_SENSORS];	/* with hysteresis (s3_ir)      */
	bool		dark_known[LF_SENSORS];
	uint16_t	level[LF_SENSORS];	/* 0 white .. 1000 black        */
	uint16_t	filt[LF_SENSORS];	/* raw, lightly smoothed        */
	bool		have_sample;
	int8_t		last_side;		/* +1 line was left, -1 right   */
	bool		seen;			/* line seen at least once      */
	uint32_t	last_seen_ms;
	s3_line_config_t line_cfg;
	uint8_t		mode;			/* LF_MODE_xxx (lf_init: LF_MODE) */
	bool		contact;		/* centre mode: a sensor touched */
	uint32_t	contact_ms;		/* the line at this time        */
	/* crossings (line_cfg.h LF_CROSS_xxx) */
	bool		edge_seen;		/* centre was on the edge ...   */
	uint32_t	edge_ms;		/* ... at this time             */
	bool		left_was_dark;
	bool		cross;			/* holding through a crossing   */
	uint32_t	cross_ms;		/* since when                   */
	bool		clear;			/* both sensors clear ...       */
	uint32_t	clear_ms;		/* ... since when               */
} lf_t;

typedef struct {
	int16_t		steer_mm_s;	/* + = turn left, - = turn right    */
	int8_t		search_dir;	/* +1 search left, -1 search right  */
	bool		present;	/* line under / next to the sensors */
	bool		calibrated;	/* centre sensor calibrated         */
	uint8_t		ir_bits;	/* LF_IR_xxx                        */
} lf_out_t;

uint8_t lf_ir_by_pin(uint8_t ir_bits);

/* Start again: forget the calibration (power-up, RESET) */
void lf_init(lf_t *p_lf);

/* One sample: raw ADC counts (0..4095) of the three sensors, by ADC
   channel (raw[0] GP26, raw[1] GP27, raw[2] GP28) */
void lf_update(lf_t *p_lf, const uint16_t raw[LF_SENSORS], uint32_t now_ms,
	       lf_out_t *p_out);

/* The car has just been put on the line (START / RESUME): in centre
   mode, both sensors white then means "line between them" */
void lf_assume_on_line(lf_t *p_lf, uint32_t now_ms);

/* No line seen for LF_LOST_MS (or never seen since lf_init) */
bool lf_lost(const lf_t *p_lf, uint32_t now_ms);

/* Junctions (line_cfg.h LF_JCT_xxx): one dark run of run_us..max_us
   under the side sensor with at least gap_us of white before and after
   it, the middle pair on the line during it.  t_us: the barcode clock
   (wheel travel as time).  lf_jct_step returns true once per junction,
   gap_us after the run. */
typedef struct {
	uint32_t	white_us;	/* the side sensor went white then */
	uint32_t	dark_us;	/* ... dark then                    */
	bool		was_dark;
	bool		armed;		/* enough white before this dark run */
	bool		on_line;	/* the middle pair on the line in it */
	bool		candidate;	/* the run fits: wait for the white  */
} lf_jct_t;

void lf_jct_reset(lf_jct_t *p_j, uint32_t t_us, bool side_dark);
bool lf_jct_step(lf_jct_t *p_j, uint32_t t_us, bool side_dark, bool on_line,
		 uint32_t run_us, uint32_t max_us, uint32_t gap_us);

#endif /* LINE_FOLLOW_H */

/* ======== from bc_reader.h ======== */
/*
 *	bc_reader.h
 *	Subsystem 3 (Line and Barcode) -- reading a barcode off the side
 *	sensor, one sample at a time: when it starts, its bar and space widths
 *	(Buddy 3's s3_pulse), when it is over, and what it says (s3_code39).
 *
 *	Pure logic (stdint/stdbool only): tested on a PC and on the car with
 *	simulated drives over the course's barcodes (bc_check.c).
 *
 *	A barcode starts at the first bar and is over after a white run longer
 *	than 2 x the widest and 5 x the narrowest bar/space so far (Code 39's
 *	quiet zone is 10 narrow wide, its widest element 3 narrow), clamped to
 *	LF_BC_QUIET_MIN/MAX_US (line_cfg.h).  "5 x the narrowest" matters at
 *	the very start: '*' begins narrow bar, WIDE space.
 */

#ifndef BC_READER_H
#define BC_READER_H

#include <stdbool.h>
#include <stdint.h>

#define BC_RD_NONE	0u	/* not reading                            */
#define BC_RD_START	1u	/* first bar seen: reading started         */
#define BC_RD_BUSY	2u	/* reading                                 */
#define BC_RD_DONE	3u	/* over: decode it now (bc_reader_decode)  */
#define BC_RD_ABORT	4u	/* not a barcode: samples too far apart, a
				   bar or space too long, or too many       */

typedef struct {
	s3_pulse_t	pulse;		/* widths, us                   */
	uint32_t	min_us;		/* narrowest and widest finished */
	uint32_t	max_us;		/* element so far                */
	uint8_t		seen;		/* elements folded into min/max  */
	bool		active;
	bool		was_dark;	/* last sample                   */
} bc_reader_t;

void bc_reader_init(bc_reader_t *p_rd);

/* One sample of the barcode sensor (dark = sees black), timestamped in
   microseconds (modulo 2^32).  allow_start: a new barcode may start now
   (the car is following the line, or standing). */
uint8_t bc_reader_sample(bc_reader_t *p_rd, uint32_t now_us, bool dark,
			 bool allow_start);

/* Stop reading without a result (the car stopped following, ...) */
void bc_reader_cancel(bc_reader_t *p_rd);

bool bc_reader_active(const bc_reader_t *p_rd);

/* After BC_RD_DONE: the text between the '*'s, its length (0 = not a
   Code 39 barcode).  *p_count = how many bars and spaces were read. */
uint8_t bc_reader_decode(const bc_reader_t *p_rd, char *p_text, uint8_t cap,
			 uint8_t *p_count);

/* Is letter c in p_letters (barcode_map.h)?  Not case sensitive. */
bool bc_letter_in(const char *p_letters, char c);

#endif /* BC_READER_H */

/* ======== from bc_rec.h ======== */
/*
 *	bc_rec.h
 *	Subsystem 3 (Line and Barcode) -- a second look at a barcode that was
 *	not read: the side sensor's raw values are recorded while a barcode
 *	is captured (plus a little before it), and if the live reading fails
 *	they are classified again with the sensor's calibration as it is at
 *	the END of the barcode, then read again (bc_reader + s3_code39).
 *
 *	Why: the side sensor learns its white and black from what it has
 *	seen.  If the first black it ever sees is the barcode's first,
 *	narrow, bar, its "black" is only that bar's (blurred, grey) value
 *	until the first wide bar, so the first character comes out wrong.
 *	After the barcode its black is the wide bars' black: the recording,
 *	classified again, reads.  (2 Nov simulation: a 5.5-6.5 mm blurry
 *	sensor spot reads the printed K only this way.)
 *
 *	Faint bars (3 Nov, car log): the side sensor saw the printed bars
 *	much lighter than the black tape it calibrated on, so against the
 *	tape's black only the wide bars just showed, and the narrow ones
 *	never did.  bc_rec_decode_auto() therefore also tries the
 *	recording's OWN lightest and darkest values (the paper and the
 *	barcode's bars) as white and black.
 *
 *	Pure logic (stdint/stdbool only): tested on a PC (bc_check.c).
 */

#ifndef BC_REC_H
#define BC_REC_H

#include <stdbool.h>
#include <stdint.h>

#define BC_REC_MAX	3072u	/* samples: 30 s at 10 ms (164.5 mm at 10 mm/s
				   is 16.5 s, plus 2.56 s before and 2.5 s
				   after it)                                 */
#define BC_REC_PRE	256u	/* samples kept from before the first bar the
				   live reader saw (2.56 s: 51 mm at 20 mm/s).
				   Faint bars: the first ones it sees can be
				   the first WIDE ones, 21 mm into a K
				   (3 Nov car log)                          */

typedef struct {
	uint16_t	raw[BC_REC_MAX];	/* side sensor, raw ADC           */
	uint16_t	ms[BC_REC_MAX];		/* time since t0_us, ms           */
	uint16_t	pre_raw[BC_REC_PRE];	/* ring: the latest samples       */
	uint32_t	pre_us[BC_REC_PRE];
	uint32_t	t0_us;
	uint32_t	last_act_us;		/* bc_rec_watch: last bar-like sample */
	uint16_t	n;
	uint16_t	pre_head;		/* next slot in the ring          */
	uint16_t	pre_n;
	uint16_t	dropouts;		/* bc_rec_find_dropouts: how many */
	uint8_t		dead[(BC_REC_MAX + 7u) / 8u];	/* 1 = a drop-out sample */
	bool		mask_ok;		/* dead[] is for this recording   */
	bool		active;
	bool		overflow;
} bc_rec_t;

void bc_rec_init(bc_rec_t *p_rec);

/* Every sample of the side sensor (raw ADC), timestamped in us */
void bc_rec_sample(bc_rec_t *p_rec, uint32_t now_us, uint16_t raw);

/* The live reader saw the first bar: keep recording from the samples
   just before it.  Call after bc_rec_sample for the same sample. */
void bc_rec_start(bc_rec_t *p_rec);

/* The live reading is over (read, failed or abandoned) */
void bc_rec_stop(bc_rec_t *p_rec);

/* Read the recording again: black above pct % of white..black, with
   span / hyst_div hysteresis (black_is_high: black reads higher).
   p_rd is scratch (its widths are the ones read).  Returns the number of
   characters (0 = still not a barcode), text in p_text, bars + spaces in
   *p_count. */
uint8_t bc_rec_decode(const bc_rec_t *p_rec, uint16_t white, uint16_t black,
		      uint32_t pct, uint32_t hyst_div, bc_reader_t *p_rd,
		      char *p_text, uint8_t cap, uint8_t *p_count);

/* The recording window on its own, not tied to the live reader (faint
   bars cut the live reading into pieces, 3 Nov): call for every sample
   INSTEAD of bc_rec_sample.  act = the sample looks bar-like (darker
   than white by a margin); allow = a barcode may be read now.  Starts
   recording on the first bar-like sample (keeping the BC_REC_PRE before
   it), ends tail_us after the last one.  Returns BC_REC_EV_xxx. */
#define BC_REC_EV_NONE		0u
#define BC_REC_EV_STARTED	1u
#define BC_REC_EV_ENDED		2u	/* read it now: bc_rec_decode_auto */
#define BC_REC_EV_CANCELLED	3u	/* allow went away: forget it       */
uint8_t bc_rec_watch(bc_rec_t *p_rec, uint32_t now_us, uint16_t raw, bool act,
		     bool allow, uint32_t tail_us);

/* Lightest and darkest raw value in the recording (drop-outs left out).
   false if empty. */
bool bc_rec_levels(const bc_rec_t *p_rec, uint16_t *p_lo, uint16_t *p_hi);

/* Mark the side sensor's drop-outs in the recording (line_cfg.h
   LF_BC_DEAD_RAW): runs of samples below dead_raw that start with a
   sudden fall and end with a sudden rise (at least jump, and 4 x the
   step next to it, within 1-2 samples).  The readers then skip them.
   Returns how many runs (also in p_rec->dropouts).  dead_raw 0 = none. */
uint16_t bc_rec_find_dropouts(bc_rec_t *p_rec, uint16_t dead_raw, uint16_t jump);

/* Sample i is a drop-out (bc_rec_find_dropouts) */
bool bc_rec_is_dead(const bc_rec_t *p_rec, uint32_t i);

/* How bc_rec_decode_auto read it */
#define BC_REC_HOW_NONE		0u
#define BC_REC_HOW_CAL		1u	/* the sensor's calibration       */
#define BC_REC_HOW_OWN50	2u	/* the barcode's own levels, 50 % */
#define BC_REC_HOW_OWN35	3u	/* ... 35 %                       */
#define BC_REC_HOW_OWN65	4u	/* ... 65 %                       */
#define BC_REC_HOW_OWN25	5u	/* ... 25 %                       */
#define BC_REC_HOW_OWN75	6u	/* ... 75 %                       */
#define BC_REC_HOW_MAX		BC_REC_HOW_OWN75
#define BC_REC_OWN_TRIES	5u	/* 50, 35, 65, 25, 75 %           */

/* Second look, trying in turn: the sensor's calibration (cal_ok) at
   cal_pct, then the recording's own lightest/darkest at 50, 35, 65,
   25 and 75 % (if they are at least min_span apart; 25 / 75 % added
   8 Nov from the recorded passes).  Code 39's checks (start/stop characters, element count,
   wide/narrow ratio) reject a wrong try.  Drop-outs are skipped if
   bc_rec_find_dropouts was called first.  *p_how = BC_REC_HOW_xxx. */
uint8_t bc_rec_decode_auto(const bc_rec_t *p_rec, bool cal_ok,
			   uint16_t cal_white, uint16_t cal_black,
			   uint32_t cal_pct, bool black_is_high,
			   uint32_t min_span, uint32_t hyst_div,
			   bc_reader_t *p_rd, char *p_text, uint8_t cap,
			   uint8_t *p_count, uint8_t *p_how);

#endif /* BC_REC_H */

/* ======== from drv_ir.h ======== */
/*
 *	drv_ir.h
 *	Subsystem 3 -- IR line / barcode sensors on the ADC (GP26-GP28).
 *	Owned by the Line and Barcode task only (TEAM.md section 3: ADC).
 */

#ifndef DRV_IR_H
#define DRV_IR_H


#define IR_COUNT	3	/* ADC channels 0..2 = GP26, GP27, GP28 */

/* Make GP26-GP28 analog inputs and open the kernel's ADC driver ("adca") */
IMPORT ER ir_init(void);

/* Read the three sensors, raw 12-bit (0..4095).  Blocks only for the
   conversions (a few microseconds each; the driver gives up after 1 s). */
IMPORT ER ir_read(UH raw[IR_COUNT]);

#endif /* DRV_IR_H */

/* ======== from s3_check.h ======== */
#ifndef S3_CHECK_H
#define S3_CHECK_H

#include <stdint.h>

/* Pure regression checks; returns number of failed checks. No peripherals. */
uint32_t s3_check_run(void);

#endif /* S3_CHECK_H */
/* End of file. */

/* ======== from line_check.h ======== */
/*
 *	line_check.h
 *	Subsystem 3 -- regression checks for line_follow.c (no hardware).
 */

#ifndef LINE_CHECK_H
#define LINE_CHECK_H

#include <stdint.h>

/* Returns the number of failed checks (0 = all passed) */
uint32_t lf_check_run(void);

#endif /* LINE_CHECK_H */

/* ======== from bc_check.h ======== */
/*
 *	bc_check.h
 *	Subsystem 3 -- Code 39 decoder regression checks (bc_check.c).
 */

#ifndef BC_CHECK_H
#define BC_CHECK_H

#include <stdint.h>

/* Runs every check; returns the number of failures (0 = all passed) */
uint32_t bc_check_run(void);

#endif /* BC_CHECK_H */


#endif /* SUBSYSTEM3_H */
