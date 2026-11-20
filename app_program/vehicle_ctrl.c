/*
 *	vehicle_ctrl.c
 *	Vehicle Controller -- creates the kernel objects it owns and starts
 *	its task.  The state machine is in task_vc.c.
 *
 *	vc_init() runs BEFORE the subsystems' _init() (app_main.c), so the
 *	queues the VC owns exist when subsystem 5 looks for obst_q and
 *	flg_sysfault, and it does not make its stand-ins.
 *
 *	Objects (TEAM.md sections 5 and 6):
 *	  obst_q        message buffer  S5 -> VC, 4-byte packed words
 *	  imu_evt_q     message buffer  S4 -> VC, 4-byte packed words
 *	  nav_q         message buffer  S3 -> VC, 4-byte packed words
 *	  flg_sysfault  event flag      shared fault mask (shared_bits.h)
 *	  vc_task       task
 */

#include "vehicle_ctrl.h"

#if VEHICLE_CONTROLLER

#define VC_MBF_BYTES	(VC_Q_DEPTH * 8)	/* 4-byte message + header */

EXPORT ID	obst_q;
EXPORT ID	imu_evt_q;
EXPORT ID	nav_q;
EXPORT ID	flg_sysfault;

/* Owned by the IMU and by Line and Barcode.  0 until they exist: their
   own "EXPORT ID imu_q;" replaces these at link time. */
__attribute__((weak)) ID imu_q = 0;
__attribute__((weak)) ID line_cmd_q = 0;

LOCAL ID vc_cre_mbf(void)
{
	T_CMBF cmbf = {0};

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = VC_MBF_BYTES;
	cmbf.maxmsz = (INT)sizeof(UW);
	return tk_cre_mbf(&cmbf);
}

EXPORT ER vc_init(void)
{
	T_CFLG cflg = {0};

	obst_q = vc_cre_mbf();
	if (obst_q < E_OK) {
		return obst_q;
	}
	imu_evt_q = vc_cre_mbf();
	if (imu_evt_q < E_OK) {
		return imu_evt_q;
	}
	nav_q = vc_cre_mbf();
	if (nav_q < E_OK) {
		return nav_q;
	}

	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0U;
	flg_sysfault = tk_cre_flg(&cflg);
	return (flg_sysfault < E_OK) ? flg_sysfault : E_OK;
}

EXPORT ER vc_start(void)
{
	ID tid = app_start_task(vc_task, VC_PRI, 0, "vc_task");

	return (tid < E_OK) ? tid : E_OK;
}

#endif /* VEHICLE_CONTROLLER */

/* ======== from vc_plan.c ======== */
/*
 *	vc_plan.c
 *	Vehicle Controller -- manoeuvre planning (see vc_plan.h).
 *
 *	Bypass geometry (car frame, mm; the obstacle's exact position is not
 *	sent by Obstacle Scan, only its width, distance and the room on the
 *	chosen side, so this plans for the worst case):
 *
 *	   side-step = width + half the car + margin
 *	               (enough even if the obstacle's far edge sits on the
 *	               car's centre line)
 *	   but no more than the room there is: half the obstacle + the room
 *	   beyond its edge - half the car (assuming it is centred)
 *	   pass      = distance to it + its assumed depth + the car's length
 */

#include "vehicle_ctrl.h"
#include "subsystem5/subsystem5.h"

#if VEHICLE_CONTROLLER

#define TURN_DEG		90U
#define MIN_SIDE_STEP_MM	100
#define MOTION_MAX_LEG_MM	5000	/* Motion Control's MOTION_MAX_DIST_MM */

LOCAL void legs_clear(VC_LEGS *p)
{
	p->n = 0U;
}

LOCAL void legs_add(VC_LEGS *p, UB kind, W amount)
{
	if ((p->n < (UB)VC_MAX_LEGS) && (amount > 0)) {
		if (amount > MOTION_MAX_LEG_MM) {
			amount = MOTION_MAX_LEG_MM;
		}
		p->leg[p->n].kind   = kind;
		p->leg[p->n].amount = (UH)amount;
		p->n++;
	}
}

EXPORT W vc_plan_bypass(UB side, UH dist_cm, UH width_cm, UH clear_cm,
			VC_LEGS *p_legs)
{
	W width_mm = (W)width_cm * 10;
	W need     = width_mm + (VC_CAR_WIDTH_MM / 2) + VC_BYPASS_MARGIN_MM;
	W room     = (width_mm / 2) + ((W)clear_cm * 10) - (VC_CAR_WIDTH_MM / 2);
	W step;
	W pass;
	UB out_turn;
	UB back_turn;

	legs_clear(p_legs);
	if (side == S5_SIDE_LEFT) {
		out_turn  = VC_LEG_LEFT;
		back_turn = VC_LEG_RIGHT;
	} else if (side == S5_SIDE_RIGHT) {
		out_turn  = VC_LEG_RIGHT;
		back_turn = VC_LEG_LEFT;
	} else {
		return 0;
	}

	step = need;
	if (room < step) {
		step = room;		/* tight: use what there is */
	}
	if (step < MIN_SIDE_STEP_MM) {
		step = MIN_SIDE_STEP_MM;
	}
	pass = ((W)dist_cm * 10) + VC_OBST_DEPTH_MM + VC_CAR_LENGTH_MM;

	legs_add(p_legs, out_turn, (W)TURN_DEG);
	legs_add(p_legs, VC_LEG_FWD, step);
	legs_add(p_legs, back_turn, (W)TURN_DEG);
	legs_add(p_legs, VC_LEG_FWD, pass);

	return (side == S5_SIDE_LEFT) ? step : -step;
}

EXPORT void vc_plan_return(W offset_mm, VC_LEGS *p_legs)
{
	legs_clear(p_legs);
	if (offset_mm > 0) {		/* car is left of the path: go right */
		legs_add(p_legs, VC_LEG_RIGHT, (W)TURN_DEG);
		legs_add(p_legs, VC_LEG_FWD, offset_mm);
		legs_add(p_legs, VC_LEG_LEFT, (W)TURN_DEG);
	} else if (offset_mm < 0) {
		legs_add(p_legs, VC_LEG_LEFT, (W)TURN_DEG);
		legs_add(p_legs, VC_LEG_FWD, -offset_mm);
		legs_add(p_legs, VC_LEG_RIGHT, (W)TURN_DEG);
	} else {
		/* on the path already */
	}
}

EXPORT void vc_plan_backup(VC_LEGS *p_legs)
{
	legs_clear(p_legs);
	legs_add(p_legs, VC_LEG_BACK, VC_BACKUP_MM);
}

EXPORT void vc_plan_barcode(UB barcode, VC_LEGS *p_legs)
{
	legs_clear(p_legs);
	switch (barcode) {
	case BARCODE_LEFT:
		legs_add(p_legs, VC_LEG_LEFT, (W)TURN_DEG);
		break;
	case BARCODE_RIGHT:
		legs_add(p_legs, VC_LEG_RIGHT, (W)TURN_DEG);
		break;
	case BARCODE_UTURN:		/* FR9: two chained left turns */
		legs_add(p_legs, VC_LEG_LEFT, (W)TURN_DEG);
		legs_add(p_legs, VC_LEG_LEFT, (W)TURN_DEG);
		break;
	default:			/* STRAIGHT, NONE, INVALID: carry on */
		break;
	}
}

#endif /* VEHICLE_CONTROLLER */

/* ======== from vc_bypass.c ======== */
/*
 *	vc_bypass.c
 *	Vehicle Controller -- going round an obstacle one move at a time
 *	(see vc_bypass.h for how it works).
 *
 *	The car's position is kept by dead reckoning from the distance each
 *	move really drove (the wheel encoders, given with every event):
 *	  x  along the old path since the car stopped for the obstacle
 *	  y  sideways from the old path, + = left
 *	The direction follows from the phase: 90 deg out and back for the
 *	first side-step (180 to try the other side), 45 deg for later ones,
 *	and VC_BYP_IN_DEG towards the line while crossing back to it.  It only has to be
 *	roughly right: the obstacle is found again by looking, and the line
 *	by the line sensors.
 */

#include <stddef.h>

/* Phases */
#define PH_SCAN		0u	/* standing; waiting for the scan's answer   */
#define PH_BACKUP	1u	/* no room either side: backing up           */
#define PH_OUT_TURN	2u	/* turning towards the bypass side           */
#define PH_OUT		3u	/* side-stepping (straight or diagonally)    */
#define PH_OUT_RETURN	4u	/* side-step blocked: backing to its start   */
#define PH_OUT_FLIP	5u	/* ... turning round to the other side       */
#define PH_OUT_BACK	6u	/* turning back to the old direction         */
#define PH_CHECK	7u	/* standing: is the obstacle still beside?   */
#define PH_PASS		8u	/* driving along the lane                    */
#define PH_IN_TURN	9u	/* turning IN_DEG towards the line           */
#define PH_LOOK		10u	/* turned in: is the way to the line free?   */
#define PH_UNLOOK	11u	/* turned in too soon: back along the lane   */
#define PH_CROSS	12u	/* driving to the line at IN_DEG             */
#define PH_CROSS_BACK	13u	/* stopped on the way: backing into the lane */
#define PH_CENTRE	14u	/* line seen: putting the wheels on it       */
#define PH_ALIGN	15u	/* turning back to the old direction         */
#define PH_END		16u
#define PH_COUNT	17u

#define MIN_STEP_MM	100
#undef TURN_DEG
#define TURN_DEG	90	/* the first side-step, from the line     */
#define DIAG_DEG	45	/* later side-steps (16 Nov: were 90 too)  */
/* Towards the line: VC_BYP_IN_DEG (vc_bypass.h; 17 Nov: 70, was 45) */
#define IN_DEG		VC_BYP_IN_DEG
#define IN_SIN_X1000	VC_BYP_IN_SIN_X1000	/* sin IN_DEG x 1000 */
#define IN_COS_X1000	VC_BYP_IN_COS_X1000	/* cos IN_DEG x 1000 */
#define SIN45_X1000	707
#define SQRT2_X1000	1414
#define GUARD_SIN_X100	50	/* Obstacle Scan's corner beams: 30 deg
				   (S5_GUARD_DEG, task_obstacle.c) */
#define GUARD_COS_X100	87
#define TURN_LOOK_DEG	30	/* turning in, Obstacle Scan pings at the
				   servo's limit on that side: 30 deg left,
				   45 right (SERVO_LIMIT_xxx_DEG); the
				   smaller one, so where it saw something
				   comes out rather too far ahead */
#define JOG_EXTRA_MM	20
#ifndef IN_EXTRA_MM
#define IN_EXTRA_MM	20	/* past the point where turning in is safe:
				   for drift and the car turning a few
				   deg more or less than it thinks
				   (19 Nov: was 50) */
#endif
/* The scan's width can come out short by this much on each side (its
   edges are pulled in by S5_BEAM_CORR_DEG for a beam 7 deg wide each
   side, on a 5 deg grid: up to about 5 deg too far in at 20 cm) */
#ifndef SCAN_EDGE_MM
#define SCAN_EDGE_MM	20
#endif

static const char *const ph_names[PH_COUNT] = {
	"scan", "back up", "turn out", "side-step", "side-step back",
	"turn round", "turn along", "check the side", "pass", "turn in",
	"look", "turn along", "cross to the line", "back to the lane",
	"onto the line", "line up", "done"
};

static int32_t iabs(int32_t v)
{
	return (v < 0) ? -v : v;
}

static int32_t imax(int32_t a, int32_t b)
{
	return (a > b) ? a : b;
}

/* cos(deg) x 1000, deg 0..90 (else clamped), from a 10 deg table */
static int32_t cos_x1000(int32_t deg)
{
	static const int16_t tab[10] = { 1000, 985, 940, 866, 766, 643, 500, 342, 174, 0 };
	int32_t i;

	if (deg <= 0) {
		return 1000;
	}
	if (deg >= 90) {
		return 0;
	}
	i = deg / 10;
	return (int32_t)tab[i] + ((((int32_t)tab[i + 1] - (int32_t)tab[i]) * (deg % 10)) / 10);
}

static int32_t sin_x1000(int32_t deg)
{
	return cos_x1000(90 - deg);
}

static void act_set(vc_byp_act_t *p_act, uint8_t kind, int32_t amount, uint8_t flags)
{
	p_act->kind   = kind;
	p_act->flags  = flags;
	p_act->amount = (uint16_t)((amount < 0) ? 0 : ((amount > 0xFFFF) ? 0xFFFF : amount));
}

static void fail(vc_byp_t *p_b, uint8_t why, vc_byp_act_t *p_act)
{
	p_b->fail = why;
	p_b->ph   = PH_END;
	act_set(p_act, VC_BYP_ACT_FAIL, (int32_t)why, 0u);
}

/* Turn away from the old path (towards the bypass side) / back to it */
static uint8_t turn_out(const vc_byp_t *p_b)
{
	return (p_b->side > 0) ? VC_BYP_ACT_LEFT : VC_BYP_ACT_RIGHT;
}

static uint8_t turn_in(const vc_byp_t *p_b)
{
	return (p_b->side > 0) ? VC_BYP_ACT_RIGHT : VC_BYP_ACT_LEFT;
}

/* The obstacle's side, as VC_BYP_LEFT / _RIGHT */
static int32_t inner_side(const vc_byp_t *p_b)
{
	return (p_b->side > 0) ? (int32_t)VC_BYP_RIGHT : (int32_t)VC_BYP_LEFT;
}

/* Dead reckoning on the diagonal: mm driven towards the line (dir +1) or
   backed away from it (dir -1) */
static void diag(vc_byp_t *p_b, int32_t mm, int32_t dir)
{
	p_b->x_mm += dir * ((mm * IN_COS_X1000) / 1000);
	p_b->y_mm -= dir * (int32_t)p_b->side * ((mm * IN_SIN_X1000) / 1000);
}

/* The obstacle reaches at least to x (along the old path) */
static void reaches(vc_byp_t *p_b, int32_t x)
{
	if (x > p_b->far_x_mm) {
		p_b->far_x_mm = x;
	}
}

/* Something on the obstacle's side, lat_mm sideways from the middle of
   the car's path in the lane */
static void beside_at(vc_byp_t *p_b, int32_t lat_mm)
{
	if (lat_mm < p_b->gap_mm) {
		p_b->gap_mm = lat_mm;
	}
}

/* Obstacle Scan saw something cm away, with the car turned car_deg
   towards the line and its beam ray_deg further round from the car's
   heading: it reaches at least that far along, and is at most that far
   sideways from the lane */
static void seen_at(vc_byp_t *p_b, int32_t car_deg, int32_t ray_deg, uint32_t cm)
{
	int32_t us = (int32_t)p_b->cfg.us_ahead_mm;
	int32_t x = p_b->x_mm + ((us * cos_x1000(car_deg)) / 1000);
	int32_t lat = (us * sin_x1000(car_deg)) / 1000;

	if (cm > 0u) {
		x += ((int32_t)cm * cos_x1000(car_deg + ray_deg)) / 100;
		lat += ((int32_t)cm * sin_x1000(car_deg + ray_deg)) / 100;
		beside_at(p_b, lat);
	}
	reaches(p_b, x);
}

/* How much further along the lane before turning IN_DEG in is safe.
   Turning on the spot and then driving diagonally, the car's side
   nearest the obstacle crosses the line along the obstacle's side
   (gap_mm from the middle of the car's path) at
       x + (gap - half the car / cos) / tan
   so the obstacle has to end before that: x >= far_x - that.  (Turning
   on the spot, its front corner swings out to 1.4 x half its width at
   x + 28 mm: never further than this.) */
static int32_t still_to_go(const vc_byp_t *p_b)
{
	int32_t hw = (int32_t)p_b->cfg.car_width_mm / 2;
	int32_t reach = ((p_b->gap_mm - ((hw * 1000) / IN_COS_X1000)) * IN_COS_X1000) / IN_SIN_X1000;

	return p_b->far_x_mm - reach - p_b->x_mm;
}

/* Turn IN_DEG towards the line, watched */
static void go_in(vc_byp_t *p_b, vc_byp_act_t *p_act)
{
	p_b->turn_after = false;
	act_set(p_act, turn_in(p_b), IN_DEG, VC_BYP_F_GUARD);
	p_b->ph = PH_IN_TURN;
}

/* Standing in the lane: is the obstacle still beside? */
static void check(vc_byp_t *p_b, vc_byp_act_t *p_act)
{
	act_set(p_act, VC_BYP_ACT_LOOK, inner_side(p_b), VC_BYP_F_SIDE);
	p_b->ph = PH_CHECK;
}

/* Drive len mm further along the lane (watched); then check, or (then_in)
   turn in */
static void pass(vc_byp_t *p_b, int32_t len, bool then_in, vc_byp_act_t *p_act)
{
	int32_t left = (int32_t)p_b->cfg.max_pass_mm - p_b->x_mm;

	if (len > left) {
		len = left;
	}
	if (len <= 0) {
		fail(p_b, VC_BYP_FAIL_TOO_LONG, p_act);
		return;
	}
	p_b->going_mm = len;
	p_b->turn_after = then_in;
	p_b->first_due = false;
	/* 16 Nov: no WIDE (keeping extra room on the obstacle's side).  With
	   the side-step only 15 cm it would stop the car at the obstacle's
	   front corner every time; the turn in is watched and looked
	   along instead. */
	act_set(p_act, VC_BYP_ACT_FWD, len, VC_BYP_F_GUARD);
	p_b->ph = PH_PASS;
}

/* The next move along the lane after a check: at least the first pass
   (once) or a step, and on until the car is past where the obstacle
   was seen */
static void pass_on(vc_byp_t *p_b, vc_byp_act_t *p_act)
{
	const vc_byp_cfg_t *p_c = &p_b->cfg;
	int32_t len = p_b->first_due ? (int32_t)p_c->first_pass_mm : (int32_t)p_c->pass_step_mm;

	pass(p_b, imax(len, still_to_go(p_b) + IN_EXTRA_MM), false, p_act);
}

/* Side-step step mm outwards: from the line 90 deg out (then 90 back),
   later 45 deg out and diagonally (then 45 back) */
static void side_step(vc_byp_t *p_b, int32_t step, vc_byp_act_t *p_act)
{
	p_b->step_mm = step;
	p_b->out_deg = (p_b->y_mm == 0) ? (uint8_t)TURN_DEG : (uint8_t)DIAG_DEG;
	act_set(p_act, turn_out(p_b), (int32_t)p_b->out_deg, 0u);
	p_b->ph = PH_OUT_TURN;
}

/* The scan's plan: side-step on the bypass side.  The first plan chooses
   the side; later ones (stopped in the lane) keep it: the other side is
   where the obstacle is. */
static void on_plan(vc_byp_t *p_b, uint32_t side, uint32_t dist_cm, uint32_t width_cm,
		    uint32_t room_cm, vc_byp_act_t *p_act)
{
	const vc_byp_cfg_t *p_c = &p_b->cfg;
	int8_t	s = (side == VC_BYP_LEFT) ? (int8_t)1 : ((side == VC_BYP_RIGHT) ? (int8_t)-1 : (int8_t)0);
	int32_t	width = (int32_t)width_cm * 10;
	/* the obstacle taken as centred on the car's path (the car follows
	   the line, the obstacle stands on it): out by half its width + half
	   the car + the margin, and from the line at least side_step_mm.  If
	   it is off-centre towards the bypass side, the watch along the lane
	   stops the car near its corner and it moves out (jog). */
	/* the scan's width can come out much too small (2-9 cm for a 13 cm
	   obstacle, 16 Nov), so it is taken as at least min_width_mm */
	int32_t	half = imax(width, (int32_t)p_c->min_width_mm) / 2;
	/* 17 Nov: out by half its width + half the car + margin_mm, so
	   margin_mm is the room between the car's side and the obstacle
	   (16 Nov: its whole width + 10 cm, which was too far out) */
	int32_t	need = half + ((int32_t)p_c->car_width_mm / 2) + (int32_t)p_c->margin_mm;

	if (s == 0) {
		fail(p_b, VC_BYP_FAIL_LOGIC, p_act);
		return;
	}
	if (p_b->plans >= p_c->max_plans) {
		fail(p_b, VC_BYP_FAIL_PLANS, p_act);
		return;
	}
	p_b->plans++;
	if (p_b->side == 0) {
		p_b->side = s;
	}
	if (p_b->y_mm == 0) {
		need = imax(need, (int32_t)p_c->side_step_mm);
	}
	if (s == p_b->side) {
		/* the room was measured on this side: no further than that */
		int32_t room = (width / 2) + ((int32_t)room_cm * 10) - ((int32_t)p_c->car_width_mm / 2);

		if (room < need) {
			need = room;
		}
	}
	if (need < MIN_STEP_MM) {
		need = MIN_STEP_MM;
	}
	if ((iabs(p_b->y_mm) + need) > (int32_t)p_c->max_side_mm) {
		fail(p_b, VC_BYP_FAIL_NO_ROOM, p_act);
		return;
	}
	/* its front is dist_cm ahead of the ultrasonic; it is taken to be at
	   least min_depth_mm deep.  Beside it, the middle of the car's path
	   is need - half its width (- what the scan may have missed) from
	   its side; side-stepping diagonally, the car also moves on by
	   need. */
	reaches(p_b, p_b->x_mm + (int32_t)p_c->us_ahead_mm + ((int32_t)dist_cm * 10) +
		     (int32_t)p_c->min_depth_mm);
	/* gap_mm grows by the side-step when it is done */
	if (p_b->y_mm == 0) {
		p_b->gap_mm = -half - SCAN_EDGE_MM;
		p_b->first_due = true;
	} else {
		beside_at(p_b, -half - SCAN_EDGE_MM);
	}
	side_step(p_b, need, p_act);
}

void vc_byp_start(vc_byp_t *p_b, const vc_byp_cfg_t *p_cfg)
{
	if ((p_b == NULL) || (p_cfg == NULL)) {
		return;
	}
	p_b->cfg = *p_cfg;
	p_b->ph = PH_SCAN;
	p_b->side = 0;
	p_b->x_mm = 0;
	p_b->y_mm = 0;
	p_b->step_mm = 0;
	p_b->going_mm = 0;
	p_b->far_x_mm = 0;
	p_b->gap_mm = 0;
	p_b->out_deg = (uint8_t)TURN_DEG;
	p_b->turn_after = false;
	p_b->first_due = false;
	p_b->rescan = false;
	p_b->jogs = 0u;
	p_b->plans = 0u;
	p_b->fail = VC_BYP_FAIL_NONE;
	p_b->tried_other = false;
	p_b->line_seen = false;
}

void vc_byp_event(vc_byp_t *p_b, uint8_t ev, uint32_t a, uint32_t b,
		  uint32_t c, uint32_t d, vc_byp_act_t *p_act)
{
	const vc_byp_cfg_t *p_c;
	int32_t	mm = (int32_t)a;		/* driven, for the move events */
	bool	moved = (ev == VC_BYP_EV_ARRIVED) || (ev == VC_BYP_EV_STOPPED);

	if ((p_b == NULL) || (p_act == NULL)) {
		return;
	}
	p_c = &p_b->cfg;
	act_set(p_act, VC_BYP_ACT_WAIT, 0, 0u);

	switch (p_b->ph) {
	case PH_SCAN:
		if (ev == VC_BYP_EV_PLAN) {
			on_plan(p_b, a, b, c, d, p_act);
		} else if (ev == VC_BYP_EV_CLEAR) {
			if (p_b->side == 0) {		/* never left the line */
				p_b->ph = PH_END;
				act_set(p_act, VC_BYP_ACT_DONE, 0, 0u);
			} else {			/* it went away: carry on */
				check(p_b, p_act);
			}
		} else if (ev == VC_BYP_EV_NO_PATH) {
			act_set(p_act, VC_BYP_ACT_BACK, (int32_t)p_c->backup_mm, 0u);
			p_b->ph = PH_BACKUP;
		} else {
			fail(p_b, VC_BYP_FAIL_LOGIC, p_act);
		}
		break;

	case PH_BACKUP:
		if (moved) {
			p_b->x_mm -= mm;
			act_set(p_act, VC_BYP_ACT_SCAN, 0, 0u);
			p_b->ph = PH_SCAN;
		} else {
			fail(p_b, VC_BYP_FAIL_LOGIC, p_act);
		}
		break;

	case PH_OUT_TURN:
	case PH_OUT_FLIP:
		if (ev == VC_BYP_EV_ARRIVED) {
			int32_t len = p_b->step_mm;

			if (p_b->ph == PH_OUT_FLIP) {
				p_b->side = (int8_t)-p_b->side;
			}
			if (p_b->out_deg == (uint8_t)DIAG_DEG) {
				len = (len * SQRT2_X1000) / 1000;
			}
			act_set(p_act, VC_BYP_ACT_FWD, len, VC_BYP_F_GUARD);
			p_b->ph = PH_OUT;
		} else {
			fail(p_b, VC_BYP_FAIL_LOGIC, p_act);
		}
		break;

	case PH_OUT:
		if (moved) {
			bool from_line = (p_b->y_mm == 0);
			int32_t out = mm;

			if (p_b->out_deg == (uint8_t)DIAG_DEG) {
				out = (mm * SIN45_X1000) / 1000;
				p_b->x_mm += out;
			}
			p_b->y_mm += (int32_t)p_b->side * out;
			if (ev == VC_BYP_EV_ARRIVED) {
				p_b->gap_mm += out;
				act_set(p_act, turn_in(p_b), (int32_t)p_b->out_deg, 0u);
				p_b->ph = PH_OUT_BACK;
			} else if (from_line && !p_b->tried_other) {
				/* something in the way sideways, straight off the
				   line: go back and try the other side, once */
				p_b->tried_other = true;
				if (mm > 0) {
					act_set(p_act, VC_BYP_ACT_BACK, mm, 0u);
					p_b->ph = PH_OUT_RETURN;
				} else {
					act_set(p_act, VC_BYP_ACT_LEFT, 2 * TURN_DEG, 0u);
					p_b->ph = PH_OUT_FLIP;
				}
			} else if (!from_line && (p_b->out_deg == (uint8_t)DIAG_DEG)) {
				/* in the way diagonally: back parallel, scan again */
				p_b->gap_mm += out;
				p_b->rescan = true;
				act_set(p_act, turn_in(p_b), (int32_t)p_b->out_deg, 0u);
				p_b->ph = PH_OUT_BACK;
			} else {
				fail(p_b, VC_BYP_FAIL_NO_ROOM, p_act);
			}
		} else {
			fail(p_b, VC_BYP_FAIL_LOGIC, p_act);
		}
		break;

	case PH_OUT_RETURN:
		if (moved) {
			p_b->y_mm = 0;			/* back on the old path */
			act_set(p_act, VC_BYP_ACT_LEFT, 2 * TURN_DEG, 0u);
			p_b->ph = PH_OUT_FLIP;
		} else {
			fail(p_b, VC_BYP_FAIL_LOGIC, p_act);
		}
		break;

	case PH_OUT_BACK:
		if ((ev == VC_BYP_EV_ARRIVED) && p_b->rescan) {
			p_b->rescan = false;
			act_set(p_act, VC_BYP_ACT_SCAN, 0, 0u);
			p_b->ph = PH_SCAN;
		} else if (ev == VC_BYP_EV_ARRIVED) {
			/* in the lane: is it beside the car? */
			check(p_b, p_act);
		} else {
			fail(p_b, VC_BYP_FAIL_LOGIC, p_act);
		}
		break;

	case PH_CHECK:
		if (ev == VC_BYP_EV_LOOK) {
			/* a = how far sideways (cm) the nearest thing on that side
			   is, b = how far ahead of the ultrasonic the furthest
			   thing near the car on that side is (0 / OPEN: nothing).
			   Between the car and the line (or not far past it):
			   still beside it. */
			bool beside = (a != 0u) && (a != VC_BYP_CM_OPEN) &&
				      (((int32_t)a * 10) <= (iabs(p_b->y_mm) + (int32_t)p_c->side_near_mm));

			if (beside) {
				reaches(p_b, p_b->x_mm + (int32_t)p_c->us_ahead_mm + ((int32_t)b * 10));
				beside_at(p_b, (int32_t)a * 10);
				pass_on(p_b, p_act);
			} else if (p_b->first_due) {
				/* nothing seen from the start of the lane (it
				   cannot see what is level with the car): the
				   first pass anyway, then check again */
				pass_on(p_b, p_act);
			} else if (still_to_go(p_b) > 0) {
				/* nothing ahead on that side, but not yet past
				   where it was seen (or must be): on that far,
				   then in */
				pass(p_b, still_to_go(p_b) + IN_EXTRA_MM, true, p_act);
			} else {
				go_in(p_b, p_act);
			}
		} else {
			fail(p_b, VC_BYP_FAIL_LOGIC, p_act);
		}
		break;

	case PH_PASS:
		p_b->x_mm += mm;
		if ((ev == VC_BYP_EV_ARRIVED) && p_b->turn_after) {
			go_in(p_b, p_act);
		} else if (ev == VC_BYP_EV_ARRIVED) {
			check(p_b, p_act);
		} else if ((ev == VC_BYP_EV_STOPPED) && (b == VC_BYP_RAY_INNER)) {
			/* nothing ahead, but too close to pass on the obstacle's
			   side: a short side-step out (45 deg), then on */
			/* How far out: the corner beam looks GUARD_SIN_X100 / 100
			   sideways per unit of distance, so what it saw is that
			   far from the middle of the car's path.  Out until it is
			   half the car + the margin + JOG_EXTRA_MM away. */
			int32_t jog = (int32_t)p_c->jog_mm;

			if (c > 0u) {
				int32_t seen = ((int32_t)c * 10 * GUARD_SIN_X100) / 100;
				int32_t want = ((int32_t)p_c->car_width_mm / 2) + (int32_t)p_c->margin_mm +
					       JOG_EXTRA_MM - seen;

				if (want > jog) {
					jog = want;
				}
				reaches(p_b, p_b->x_mm + (int32_t)p_c->us_ahead_mm +
					     (((int32_t)c * 10 * GUARD_COS_X100) / 100));
				/* off the beam's middle it is nearer: half that */
				beside_at(p_b, seen / 2);
			}
			if ((p_b->jogs >= (uint8_t)(2u * p_c->max_plans)) ||
			    ((iabs(p_b->y_mm) + jog) > (int32_t)p_c->max_side_mm)) {
				fail(p_b, VC_BYP_FAIL_NO_ROOM, p_act);
			} else {
				p_b->jogs++;
				side_step(p_b, jog, p_act);
			}
		} else if (ev == VC_BYP_EV_STOPPED) {
			/* something in the lane: scan and side-step further out */
			act_set(p_act, VC_BYP_ACT_SCAN, 0, 0u);
			p_b->ph = PH_SCAN;
		} else {
			fail(p_b, VC_BYP_FAIL_LOGIC, p_act);
		}
		break;

	case PH_IN_TURN:
		if (ev == VC_BYP_EV_ARRIVED) {
			act_set(p_act, VC_BYP_ACT_LOOK, inner_side(p_b), 0u);
			p_b->ph = PH_LOOK;
		} else if (ev == VC_BYP_EV_STOPPED) {
			/* Obstacle Scan stopped the turn: the obstacle is still
			   close on that side.  Back along the lane by what it
			   turned, further on, check again. */
			seen_at(p_b, (int32_t)a, TURN_LOOK_DEG, c);
			if (a > 0u) {
				act_set(p_act, turn_out(p_b), (int32_t)a, 0u);
				p_b->ph = PH_UNLOOK;
			} else {
				pass_on(p_b, p_act);
			}
		} else {
			fail(p_b, VC_BYP_FAIL_LOGIC, p_act);
		}
		break;

	case PH_LOOK:
		if (ev == VC_BYP_EV_LOOK) {
			/* a = cm free ahead (the car's width).  Free all the way
			   to the line (sideways distance / sin) + the margin? */
			int32_t to_line = (iabs(p_b->y_mm) * 1000) / IN_SIN_X1000;
			bool	blocked = (a != 0u) && (a != VC_BYP_CM_OPEN) &&
					  (((int32_t)a * 10) < (to_line + (int32_t)p_c->look_margin_mm));

			if (blocked) {			/* back along the lane, further */
				seen_at(p_b, IN_DEG, 0, a);
				act_set(p_act, turn_out(p_b), IN_DEG, 0u);
				p_b->ph = PH_UNLOOK;
			} else {
				/* it stops at the line: up to 1.25 x the way + the
				   extra */
				act_set(p_act, VC_BYP_ACT_FWD,
					((to_line * 5) / 4) + (int32_t)p_c->cross_extra_mm,
					(uint8_t)(VC_BYP_F_GUARD | VC_BYP_F_LINE));
				p_b->ph = PH_CROSS;
			}
		} else {
			fail(p_b, VC_BYP_FAIL_LOGIC, p_act);
		}
		break;

	case PH_UNLOOK:
		if (ev == VC_BYP_EV_ARRIVED) {
			/* it was still there: further along the lane (past
			   where it was seen), then check again.  17 Nov: no
			   side-step further out here (16 Nov: jog_mm out).
			   Undoing the turn and then turning 45 deg out looked
			   like a 90 deg turn for a new obstacle, and every
			   time took the car further from the line. */
			pass_on(p_b, p_act);
		} else {
			fail(p_b, VC_BYP_FAIL_LOGIC, p_act);
		}
		break;

	case PH_CROSS:
		diag(p_b, mm, 1);
		if (ev == VC_BYP_EV_LINE) {
			p_b->line_seen = true;
			act_set(p_act, VC_BYP_ACT_FWD, (int32_t)p_c->sensor_ahead_mm, VC_BYP_F_GUARD);
			p_b->ph = PH_CENTRE;
		} else if (ev == VC_BYP_EV_ARRIVED) {
			/* no line where it should be: line up anyway, the line
			   search has the last word */
			act_set(p_act, turn_out(p_b), IN_DEG, 0u);
			p_b->ph = PH_ALIGN;
		} else if (ev == VC_BYP_EV_STOPPED) {
			/* still something there: back into the lane, further on */
			seen_at(p_b, IN_DEG,
				(b == VC_BYP_RAY_INNER) ? 30 : ((b == VC_BYP_RAY_OUTER) ? -30 : 0), c);
			if (mm > 0) {
				act_set(p_act, VC_BYP_ACT_BACK, mm, 0u);
				p_b->ph = PH_CROSS_BACK;
			} else {
				act_set(p_act, turn_out(p_b), IN_DEG, 0u);
				p_b->ph = PH_UNLOOK;
			}
		} else {
			fail(p_b, VC_BYP_FAIL_LOGIC, p_act);
		}
		break;

	case PH_CROSS_BACK:
		if (moved) {
			diag(p_b, mm, -1);
			act_set(p_act, turn_out(p_b), IN_DEG, 0u);
			p_b->ph = PH_UNLOOK;
		} else {
			fail(p_b, VC_BYP_FAIL_LOGIC, p_act);
		}
		break;

	case PH_CENTRE:
		if (moved) {
			diag(p_b, mm, 1);
			act_set(p_act, turn_out(p_b), IN_DEG, 0u);
			p_b->ph = PH_ALIGN;
		} else {
			fail(p_b, VC_BYP_FAIL_LOGIC, p_act);
		}
		break;

	case PH_ALIGN:
		if (ev == VC_BYP_EV_ARRIVED) {
			p_b->ph = PH_END;
			act_set(p_act, VC_BYP_ACT_DONE, 0, 0u);
		} else {
			fail(p_b, VC_BYP_FAIL_LOGIC, p_act);
		}
		break;

	default:		/* PH_END: finished or failed, nothing more */
		act_set(p_act, (p_b->fail != VC_BYP_FAIL_NONE) ? VC_BYP_ACT_FAIL : VC_BYP_ACT_DONE,
			(int32_t)p_b->fail, 0u);
		break;
	}
}

bool vc_byp_backed_off(const vc_byp_t *p_b)
{
	return (p_b != NULL) && (p_b->side == 0) && (p_b->x_mm < 0);
}

bool vc_byp_returning(const vc_byp_t *p_b)
{
	return (p_b != NULL) && (p_b->ph >= PH_CROSS) && (p_b->ph <= PH_ALIGN);
}

int8_t vc_byp_side(const vc_byp_t *p_b)
{
	return (p_b != NULL) ? p_b->side : (int8_t)0;
}

const char *vc_byp_phase_name(const vc_byp_t *p_b)
{
	return ((p_b != NULL) && (p_b->ph < PH_COUNT)) ? ph_names[p_b->ph] : "?";
}
