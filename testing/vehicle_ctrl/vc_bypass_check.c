/*
 *	vc_bypass_check.c
 *	Vehicle Controller -- PC simulation of the bypass (vc_bypass.c).
 *	No hardware.  Not part of the firmware: everything below is inside
 *	#if VC_BYP_HOST_TEST.
 *
 *	A simulated car (its body, the ultrasonic sensor on its servo, the
 *	line sensors, turns and distances that are a few % off) drives up
 *	to obstacles on a straight line and goes round them with
 *	vc_bypass.c deciding every move, as task_vc.c does on the car.
 *	Obstacle Scan is imitated: its scan and plan (the same steps and
 *	numbers as s5_plan, subsystem5.c), the watching while the car
 *	drives or turns towards the obstacle, and the look
 *	(task_obstacle.c).
 *
 *	The obstacles: boxes of different width, depth and position, long
 *	ones, ones with a part sticking out further back, a second obstacle
 *	in the way, walls beside the path.  Each with three kinds of echo
 *	(ideal; flat faces seen only nearly square on; the same with a
 *	narrow beam) and several seeds for the turn / distance errors.
 *
 *	Pass = the car never touches anything and ends on the line past the
 *	obstacle, or (where there is no way round) stops without touching.
 *
 *	From app_program/vehicle_ctrl:
 *	  gcc -std=c11 -Wall -Wextra -Werror -Wconversion -Wshadow -pedantic \
 *	      -DVC_BYP_HOST_TEST vc_bypass.c vc_bypass_check.c -lm \
 *	      -o /tmp/byp-tests && /tmp/byp-tests
 *	Add -v to see every move of every run; -s N runs only scenario N
 *	(from 0); -n N uses N seeds for each kind of echo (8 if not given).
 */

#include "vc_bypass.h"

#if defined(VC_BYP_HOST_TEST)

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- The car (mm), from the middle of the wheels' axle ---- */
#define CAR_FRONT	120.0		/* to the front edge (the ultrasonic) */
#define CAR_REAR	80.0
#define CAR_HALF_W	80.0
#define IR_AHEAD	107.0		/* line sensors (15 Nov: 105-110 mm)  */
#define IR_HALF_GAP	8.0		/* the two of them, left and right    */
#define LINE_HALF_W	9.0		/* 18 mm tape                         */

/* ---- Obstacle Scan, as on the car (subsystem5.h, task_obstacle.c) ---- */
#define S5_LIMIT_LEFT	30
#define S5_LIMIT_RIGHT	(-45)
#define S5_AHEAD_DEG	30
#define S5_OBJECT_CM	35
#define S5_JUMP_CM	10
#define S5_BEAM_CORR	7
#define S5_DEPTH_CM	15
#define S5_MIN_CLEAR	25
#define S5_CLEAR_OPEN	400
#define S5_STOP_CM	15
#define S5_GUARD_DEG	30
#define S5_GUARD_CM	17		/* at the corner away from the obstacle */
#ifndef S5_GUARD_IN_CM
#define S5_GUARD_IN_CM	25		/* at the corner on the obstacle's side */
#endif
#define S5_LOOK_HALF_CM	13
#ifndef S5_TURN_CM
#define S5_TURN_CM	20		/* turning towards the obstacle: stop */
#endif
#define S5_MAX_RETRIES	2
#define CM_NONE		0
#define CM_OPEN		511
#define SLOTS		25

#define DEG		(3.14159265358979 / 180.0)

typedef struct {
	double x0, x1, y0, y1;
} rect_t;

#define MAX_RECTS	6

typedef struct {
	const char	*name;
	rect_t		r[MAX_RECTS];
	int		n;
	bool		expect_fail;	/* no way round: must stop, not touch */
	double		past_x;		/* the car must end beyond this x     */
} scene_t;

typedef struct {
	double	beam;		/* half beam width, deg */
	double	spec_deg;	/* a flat face echoes only within this of
				   square on (corners always do) */
	const char *name;
} echo_t;

typedef struct {
	const scene_t	*sc;
	const echo_t	*echo;
	double		x, y, th;	/* th in rad, 0 = along +x, + = left */
	unsigned	seed;
	double		turn_err;	/* each turn is off by up to this share;
					   the wheel encoders see it */
	double		slip_err;	/* ... and by this, which they do not */
	double		dist_err;
	double		want_deg;	/* the direction the moves add up to */
	double		odo_deg;	/* the direction by the wheel encoders */
	int		ping_mm;	/* the car moves this far per ping */
	int		guard_i;
	unsigned	inner;		/* VC_BYP_LEFT / _RIGHT: the obstacle's side */
	bool		wide;		/* keep extra room on that side */
	bool		hit;		/* touched an obstacle */
	bool		verbose;
	double		driven_total;
	int		turns;
	double		turned_deg;	/* all turns added up               */
	double		max_out;	/* furthest the axle got from the line */
} sim_t;

static double frand(sim_t *s)		/* -1 .. +1 */
{
	s->seed = (s->seed * 1103515245u) + 12345u;
	return ((double)((s->seed >> 8) & 0xFFFFu) / 32767.5) - 1.0;
}

/* ---- Geometry ---- */
static bool in_rect(const rect_t *r, double x, double y)
{
	return (x > r->x0) && (x < r->x1) && (y > r->y0) && (y < r->y1);
}

/* Does the car's body touch an obstacle?  Points along its outline. */
static bool touching(const sim_t *s)
{
	double c = cos(s->th);
	double sn = sin(s->th);
	int i;
	int k;

	for (i = 0; i <= 20; i++) {
		double f = (double)i / 20.0;
		double px[4];
		double py[4];

		px[0] = -CAR_REAR + (f * (CAR_FRONT + CAR_REAR));	py[0] = CAR_HALF_W;
		px[1] = px[0];						py[1] = -CAR_HALF_W;
		px[2] = CAR_FRONT;	py[2] = -CAR_HALF_W + (f * 2.0 * CAR_HALF_W);
		px[3] = -CAR_REAR;	py[3] = py[2];
		for (k = 0; k < 4; k++) {
			double wx = s->x + (px[k] * c) - (py[k] * sn);
			double wy = s->y + (px[k] * sn) + (py[k] * c);
			int q;

			for (q = 0; q < s->sc->n; q++) {
				if (in_rect(&s->sc->r[q], wx, wy)) {
					return true;
				}
			}
		}
	}
	return false;
}

/* Ray from (ox, oy) along (dx, dy) into a box: distance, or -1.  *p_inc =
   angle between the ray and the face's normal, deg; *p_corner = the hit
   is within 12 mm of a corner. */
static double ray_rect(const rect_t *r, double ox, double oy, double dx, double dy,
		       double *p_inc, bool *p_corner)
{
	double tx0 = -1e18;
	double tx1 = 1e18;
	double ty0 = -1e18;
	double ty1 = 1e18;
	double t;
	double hx;
	double hy;
	bool x_face;

	if (fabs(dx) > 1e-9) {
		double a = (r->x0 - ox) / dx;
		double b = (r->x1 - ox) / dx;

		tx0 = (a < b) ? a : b;
		tx1 = (a < b) ? b : a;
	} else if ((ox <= r->x0) || (ox >= r->x1)) {
		return -1.0;
	} else {
		/* inside the slab */
	}
	if (fabs(dy) > 1e-9) {
		double a = (r->y0 - oy) / dy;
		double b = (r->y1 - oy) / dy;

		ty0 = (a < b) ? a : b;
		ty1 = (a < b) ? b : a;
	} else if ((oy <= r->y0) || (oy >= r->y1)) {
		return -1.0;
	} else {
		/* inside the slab */
	}
	t = (tx0 > ty0) ? tx0 : ty0;
	if ((t > ((tx1 < ty1) ? tx1 : ty1)) || (t <= 0.0)) {
		return -1.0;
	}
	x_face = (tx0 > ty0);
	*p_inc = acos(x_face ? fabs(dx) : fabs(dy)) / DEG;
	hx = ox + (t * dx);
	hy = oy + (t * dy);
	*p_corner = x_face ? ((fabs(hy - r->y0) < 12.0) || (fabs(hy - r->y1) < 12.0))
			   : ((fabs(hx - r->x0) < 12.0) || (fabs(hx - r->x1) < 12.0));
	return t;
}

/* One ultrasonic reading with the servo at servo_deg (+ = left), cm */
static int us_cm(const sim_t *s, double servo_deg)
{
	double ox = s->x + (CAR_FRONT * cos(s->th));
	double oy = s->y + (CAR_FRONT * sin(s->th));
	double best = 1e18;
	double off;

	for (off = -s->echo->beam; off <= (s->echo->beam + 0.01); off += 2.5) {
		double a = s->th + ((servo_deg + off) * DEG);
		int q;

		for (q = 0; q < s->sc->n; q++) {
			double inc = 0.0;
			bool corner = false;
			double t = ray_rect(&s->sc->r[q], ox, oy, cos(a), sin(a), &inc, &corner);

			if ((t > 0.0) && (t < best) &&
			    ((inc <= s->echo->spec_deg) || corner)) {
				best = t;
			}
		}
	}
	if (best > 4000.0) {
		return CM_OPEN;
	}
	return (best < 20.0) ? 2 : (int)((best / 10.0) + 0.5);
}

static bool ir_on_line(const sim_t *s)
{
	double c = cos(s->th);
	double sn = sin(s->th);
	int k;

	for (k = -1; k <= 1; k += 2) {
		double wy = s->y + (IR_AHEAD * sn) + ((double)k * IR_HALF_GAP * c);

		if (fabs(wy) <= LINE_HALF_W) {
			return true;
		}
	}
	return false;
}

/* ---- Obstacle Scan imitated ---- */
typedef struct {
	int result;		/* 0 clear, 1 bypass, 2 blocked */
	int side;
	int dist;
	int width;
	int clear_l;
	int clear_r;
} plan_t;

static int deg_of(int slot) { return -60 + (5 * slot); }
static int x_cm(int d, int deg) { return (int)((double)d * sin((double)deg * DEG)); }
static int y_cm(int d, int deg) { return (int)((double)d * cos((double)deg * DEG)); }
static bool is_hit(int cm) { return (cm != CM_NONE) && (cm != CM_OPEN) && (cm <= S5_OBJECT_CM); }

static int side_clear(const int *cm, int edge_slot, int dir, int edge_x, int max_depth)
{
	int i;
	int best = S5_CLEAR_OPEN;
	bool seen = false;

	for (i = edge_slot + dir; (i >= 0) && (i < SLOTS); i += dir) {
		int gap;

		if (cm[i] == CM_NONE) {
			continue;
		}
		seen = true;
		if ((cm[i] == CM_OPEN) || (y_cm(cm[i], deg_of(i)) > max_depth)) {
			continue;
		}
		gap = dir * (x_cm(cm[i], deg_of(i)) - edge_x);
		gap = (gap < 0) ? 0 : gap;
		best = (gap < best) ? gap : best;
	}
	return seen ? best : 0;
}

/* The scan (every 5 deg the servo can reach) and s5_plan's steps */
static void s5_scan_plan(const sim_t *s, plan_t *p)
{
	int cm[SLOTS];
	int i;
	int near_slot = -1;
	int left;
	int right;
	int prev;
	int a_left;
	int a_right;
	int width;

	for (i = 0; i < SLOTS; i++) {
		int deg = deg_of(i);

		cm[i] = ((deg > S5_LIMIT_LEFT) || (deg < S5_LIMIT_RIGHT)) ? CM_NONE
									  : us_cm(s, (double)deg);
	}
	memset(p, 0, sizeof *p);
	p->dist = CM_OPEN;
	p->clear_l = S5_CLEAR_OPEN;
	p->clear_r = S5_CLEAR_OPEN;
	for (i = 0; i < SLOTS; i++) {
		if ((abs(deg_of(i)) <= S5_AHEAD_DEG) && is_hit(cm[i])) {
			if ((near_slot < 0) || (cm[i] < cm[near_slot]) ||
			    ((cm[i] == cm[near_slot]) && (abs(deg_of(i)) < abs(deg_of(near_slot))))) {
				near_slot = i;
			}
		}
	}
	if (near_slot < 0) {
		return;
	}
	p->dist = cm[near_slot];
	left = near_slot;
	prev = cm[near_slot];
	for (i = near_slot + 1; i < SLOTS; i++) {
		if (cm[i] == CM_NONE) {
			continue;
		}
		if ((cm[i] == CM_OPEN) || (abs(cm[i] - prev) > S5_JUMP_CM)) {
			break;
		}
		left = i;
		prev = cm[i];
	}
	right = near_slot;
	prev = cm[near_slot];
	for (i = near_slot - 1; i >= 0; i--) {
		if (cm[i] == CM_NONE) {
			continue;
		}
		if ((cm[i] == CM_OPEN) || (abs(cm[i] - prev) > S5_JUMP_CM)) {
			break;
		}
		right = i;
		prev = cm[i];
	}
	a_left = deg_of(left) - S5_BEAM_CORR;
	a_right = deg_of(right) + S5_BEAM_CORR;
	if (a_left < a_right) {
		a_left = (a_left + a_right) / 2;
		a_right = a_left;
	}
	width = x_cm(cm[left], a_left) - x_cm(cm[right], a_right);
	width = (width < 1) ? 1 : ((width > 255) ? 255 : width);
	p->width = width;
	p->clear_l = side_clear(cm, left, +1, x_cm(cm[left], deg_of(left)), p->dist + S5_DEPTH_CM);
	p->clear_r = side_clear(cm, right, -1, x_cm(cm[right], deg_of(right)), p->dist + S5_DEPTH_CM);
	if ((p->clear_l < S5_MIN_CLEAR) && (p->clear_r < S5_MIN_CLEAR)) {
		p->result = 2;
	} else {
		p->result = 1;
		p->side = (p->clear_l >= p->clear_r) ? (int)VC_BYP_LEFT : (int)VC_BYP_RIGHT;
	}
}

/* The watch while the car drives: ahead, left corner, ahead, right corner.
   Returns -1, or the VC_BYP_RAY_xxx that stops the car. */
static int g_stop_cm;		/* where the watch saw what stopped the car */

static int s5_guard_stops(sim_t *s)
{
	static const int degs[4] = { 0, S5_GUARD_DEG, 0, -S5_GUARD_DEG };
	int deg = degs[s->guard_i & 3];
	int cm = us_cm(s, (double)deg);
	bool inner = ((deg > 0) && (s->inner == VC_BYP_LEFT)) ||
		     ((deg < 0) && (s->inner == VC_BYP_RIGHT));
	int lim = (deg == 0) ? S5_STOP_CM : ((inner && s->wide) ? S5_GUARD_IN_CM : S5_GUARD_CM);

	s->guard_i++;
	if ((cm == CM_OPEN) || (cm > lim)) {
		return -1;
	}
	g_stop_cm = cm;
	return (deg == 0) ? (int)VC_BYP_RAY_AHEAD
			  : (inner ? (int)VC_BYP_RAY_INNER : (int)VC_BYP_RAY_OUTER);
}

/* The look: how far is the car's own width free ahead?  cm */
static int s5_look(const sim_t *s, unsigned side)
{
	static const int left[3] = { 0, 15, 30 };
	static const int right[3] = { 0, -20, -40 };
	const int *degs = (side == VC_BYP_LEFT) ? left : right;
	int best = CM_OPEN;
	int i;

	for (i = 0; i < 3; i++) {
		int cm = us_cm(s, (double)degs[i]);

		if ((cm != CM_OPEN) && (abs(x_cm(cm, degs[i])) <= S5_LOOK_HALF_CM)) {
			int depth = y_cm(cm, degs[i]);

			best = (depth < best) ? depth : best;
		}
	}
	return best;
}

/* The side check (16 Nov, task_obstacle.c s5_side): the car stands in
   the lane; the servo looks towards the obstacle's side.  Of the
   readings within S5_SIDE_AHEAD_CM ahead: returns how far sideways the
   nearest one is (cm); *p_along = how far ahead the furthest one within
   S5_SIDE_NEAR_CM sideways is. */
#ifndef S5_SIDE_AHEAD_CM
#define S5_SIDE_AHEAD_CM	60
#endif
#define S5_SIDE_NEAR_CM		35
static int s5_side(const sim_t *s, unsigned side, int *p_along)
{
	static const int left[3] = { 30, 20, 10 };
	static const int right[3] = { -45, -30, -15 };
	const int *degs = (side == VC_BYP_LEFT) ? left : right;
	int best = CM_OPEN;
	int i;

	*p_along = 0;
	for (i = 0; i < 3; i++) {
		int cm = us_cm(s, (double)degs[i]);

		if (cm != CM_OPEN) {
			int across = abs(x_cm(cm, degs[i]));
			int along = y_cm(cm, degs[i]);

			if (along <= S5_SIDE_AHEAD_CM) {
				best = (across < best) ? across : best;
				if ((across <= S5_SIDE_NEAR_CM) && (along > *p_along)) {
					*p_along = along;
				}
			}
		}
	}
	return best;
}

/* ---- The car's moves ---- */
/* Returns VC_BYP_EV_ARRIVED / _STOPPED / _LINE; *p_mm = what the wheels
   measured.  dir +1 forward, -1 backward. */
static uint8_t drive(sim_t *s, int mm, int dir, uint8_t flags, int *p_mm, int *p_ray)
{
	double scale = 1.0 + (s->dist_err * frand(s));		/* true / measured */
	double drift = 1.5 * DEG * frand(s) / 300.0;		/* rad per mm, measured */
	double unseen = 0.3 * DEG * frand(s) / 300.0;		/* ... and not */
	double want = (double)mm * scale;
	double done = 0.0;
	uint8_t ev = VC_BYP_EV_ARRIVED;
	int n = 0;
	int line_n = 0;

	while (done < want) {
		s->x += (double)dir * cos(s->th);
		s->y += (double)dir * sin(s->th);
		s->th += drift + unseen;
		s->odo_deg += drift / DEG;
		done += 1.0;
		n++;
		if (fabs(s->y) > s->max_out) {
			s->max_out = fabs(s->y);
		}
		if (touching(s)) {
			s->hit = true;
			break;
		}
		if (((flags & VC_BYP_F_GUARD) != 0u) && ((n % s->ping_mm) == 0)) {
			*p_ray = s5_guard_stops(s);
			if (*p_ray >= 0) {
				ev = VC_BYP_EV_STOPPED;
				break;
			}
		}
		if (((flags & VC_BYP_F_LINE) != 0u) && ir_on_line(s)) {
			line_n++;
			if (line_n >= 2) {
				ev = VC_BYP_EV_LINE;
				break;
			}
		} else {
			line_n = 0;
		}
	}
	s->driven_total += done;
	*p_mm = (int)((done / scale) + 0.5);
	return ev;
}

/* A turn, as task_vc.c makes it: by as much as brings the direction the
   wheel encoders measure to where the moves so far add up to.
   guard: Obstacle Scan pings at the servo's limit on the side the car
   turns to (about every degree: 60 ms at 18 deg/s) and stops the turn
   if something is within S5_TURN_CM twice in a row.  Returns the degrees
   turned as the wheel encoders saw them, or -1 if it was not stopped. */
static int turn(sim_t *s, int deg, int dir, bool guard)	/* dir +1 left */
{
	double cmd;
	double measured;
	double real;
	double done = 0.0;
	double sgn;
	double want0 = s->want_deg;
	int near_n = 0;

	s->want_deg += (double)(dir * deg);
	cmd = s->want_deg - s->odo_deg;
	sgn = (cmd < 0.0) ? -1.0 : 1.0;
	measured = fabs(cmd) * (1.0 + (s->turn_err * frand(s)));
	real = measured * (1.0 + (s->slip_err * frand(s)));
	while (done < real) {
		s->th += sgn * DEG;
		done += 1.0;
		if (touching(s)) {
			s->hit = true;
			return -1;
		}
		if (guard) {
			int cm = us_cm(s, (dir > 0) ? (double)S5_LIMIT_LEFT : (double)S5_LIMIT_RIGHT);

			near_n = ((cm != CM_OPEN) && (cm <= S5_TURN_CM)) ? (near_n + 1) : 0;
			if (near_n >= 2) {
				double m = done * measured / real;

				g_stop_cm = cm;

				s->odo_deg += sgn * m;
				s->want_deg = want0 + (sgn * m);
				s->turns++;
				s->turned_deg += done;
				if (s->verbose) {
					printf("      turn stopped after %.0f deg (%d cm at the servo's limit)\n",
					       done, cm);
				}
				return (int)(m + 0.5);
			}
		}
	}
	s->odo_deg += sgn * measured;
	s->turns++;
	s->turned_deg += real;
	return -1;
}

/* ---- One run ---- */
#define RUN_OK		0
#define RUN_FAILED	1	/* the bypass gave up (FAIL / no path) */
#define RUN_HIT		2
#define RUN_LOST	3	/* finished, but not on the line past it */
#define RUN_STUCK	4

#ifndef MARGIN_MM
#define MARGIN_MM	50u		/* VC_BYPASS_MARGIN_MM */
#endif
#ifndef SIDE_STEP_MM
#define SIDE_STEP_MM	150u		/* VC_SIDE_STEP_MM */
#endif
#ifndef MIN_WIDTH_MM
#define MIN_WIDTH_MM	130u		/* VC_OBST_MIN_WIDTH_MM */
#endif
#ifndef FIRST_PASS_MM
#define FIRST_PASS_MM	200u		/* VC_PASS_FIRST_MM */
#endif
#ifndef MIN_DEPTH_MM
#define MIN_DEPTH_MM	120u		/* VC_OBST_MIN_DEPTH_MM */
#endif
#ifndef PASS_STEP_MM
#define PASS_STEP_MM	50u		/* VC_PASS_STEP_MM */
#endif
static const vc_byp_cfg_t g_cfg = {
	.car_width_mm = 160u, .car_length_mm = 200u, .side_step_mm = SIDE_STEP_MM,
	.margin_mm = MARGIN_MM, .min_width_mm = MIN_WIDTH_MM,
	.first_pass_mm = FIRST_PASS_MM, .pass_step_mm = PASS_STEP_MM, .jog_mm = 50u,
	.sensor_ahead_mm = 107u, .us_ahead_mm = 120u, .min_depth_mm = MIN_DEPTH_MM,
	.side_near_mm = 100u, .look_margin_mm = 30u, .cross_extra_mm = 300u, .backup_mm = 150u,
	.max_side_mm = 700u, .max_pass_mm = 2000u, .max_plans = 4u
};

/* One obstacle stop: from the scan to DONE or FAIL.  *p_backed = it only
   backed away (vc_byp_backed_off). */
static int bypass(sim_t *s, int *p_looks, bool *p_backed)
{
	vc_byp_t	b;
	vc_byp_act_t	act;
	int		mm = 0;
	int		ray = 0;
	int		retries = 0;
	int		steps = 0;
	uint8_t		ev;

	vc_byp_start(&b, &g_cfg);
	act.kind = VC_BYP_ACT_SCAN;
	act.flags = 0u;
	act.amount = 0u;

	for (;;) {
		if (++steps > 200) {
			return RUN_STUCK;
		}
		if (s->verbose) {
			printf("    [%s] act %u amount %u flags %u   car %.0f,%.0f %.0f deg (thinks %ld,%ld)\n",
			       vc_byp_phase_name(&b), (unsigned)act.kind, (unsigned)act.amount,
			       (unsigned)act.flags, s->x, s->y, s->th / DEG, (long)b.x_mm, (long)b.y_mm);
		}
		switch (act.kind) {
		case VC_BYP_ACT_SCAN:
		case VC_BYP_ACT_WAIT: {
			plan_t p;

			s5_scan_plan(s, &p);
			if (s->verbose) {
				printf("      scan: result %d side %d dist %d width %d clear L%d R%d\n",
				       p.result, p.side, p.dist, p.width, p.clear_l, p.clear_r);
			}
			if (p.result == 0) {
				vc_byp_event(&b, VC_BYP_EV_CLEAR, 0u, 0u, 0u, 0u, &act);
			} else if (p.result == 1) {
				retries = 0;
				vc_byp_event(&b, VC_BYP_EV_PLAN, (uint32_t)p.side, (uint32_t)p.dist,
					     (uint32_t)p.width,
					     (uint32_t)((p.side == (int)VC_BYP_LEFT) ? p.clear_l : p.clear_r),
					     &act);
			} else if (retries >= S5_MAX_RETRIES) {
				return RUN_FAILED;		/* NO_PATH, gave up */
			} else {
				retries++;
				vc_byp_event(&b, VC_BYP_EV_NO_PATH, 0u, 0u, 0u, 0u, &act);
			}
			break;
		}
		case VC_BYP_ACT_FWD:
			s->inner = (b.side > 0) ? VC_BYP_RIGHT : VC_BYP_LEFT;
			s->wide = (act.flags & VC_BYP_F_WIDE) != 0u;
			ev = drive(s, (int)act.amount, +1, act.flags, &mm, &ray);
			if (ev == VC_BYP_EV_STOPPED) {
				retries = 0;	/* DETECTED: the next scan starts afresh */
				if (s->verbose) {
					printf("      stopped by the watch (%s)\n",
					       (ray == (int)VC_BYP_RAY_AHEAD) ? "ahead" :
					       (ray == (int)VC_BYP_RAY_INNER) ? "corner on the obstacle's side"
									      : "the other corner");
				}
			}
			vc_byp_event(&b, ev, (uint32_t)mm,
				     (ev == VC_BYP_EV_STOPPED) ? (uint32_t)ray : 0u,
				     (ev == VC_BYP_EV_STOPPED) ? (uint32_t)g_stop_cm : 0u, 0u, &act);
			break;
		case VC_BYP_ACT_BACK:
			ev = drive(s, (int)act.amount, -1, 0u, &mm, &ray);
			vc_byp_event(&b, ev, (uint32_t)mm, 0u, 0u, 0u, &act);
			break;
		case VC_BYP_ACT_LEFT:
		case VC_BYP_ACT_RIGHT:
		{
			int got = turn(s, (int)act.amount, (act.kind == VC_BYP_ACT_LEFT) ? +1 : -1,
				       (act.flags & VC_BYP_F_GUARD) != 0u);

			if (got >= 0) {
				vc_byp_event(&b, VC_BYP_EV_STOPPED, (uint32_t)got, (uint32_t)VC_BYP_RAY_INNER,
					     (uint32_t)g_stop_cm, 0u, &act);
			} else {
				vc_byp_event(&b, VC_BYP_EV_ARRIVED, 0u, 0u, 0u, 0u, &act);
			}
		}
			break;
		case VC_BYP_ACT_LOOK: {
			int along = 0;
			int cm = ((act.flags & VC_BYP_F_SIDE) != 0u) ? s5_side(s, act.amount, &along)
								      : s5_look(s, act.amount);

			(*p_looks)++;
			if (s->verbose && ((act.flags & VC_BYP_F_SIDE) != 0u)) {
				printf("      side check: nearest %d cm sideways, furthest %d cm ahead\n", cm, along);
			} else if (s->verbose) {
				printf("      look: %d cm free\n", cm);
			}
			vc_byp_event(&b, VC_BYP_EV_LOOK, (uint32_t)cm, (uint32_t)along, 0u, 0u, &act);
			break;
		}
		case VC_BYP_ACT_DONE: {
			/* Line and Barcode's search: a sweep of +-45 deg */
			int a;
			bool found = false;

			for (a = -45; (a <= 45) && !found; a++) {
				sim_t t = *s;

				t.th += (double)a * DEG;
				found = ir_on_line(&t);
			}
			*p_backed = vc_byp_backed_off(&b);
			while (s->th > (180.0 * DEG)) {
				s->th -= 360.0 * DEG;
			}
			while (s->th < (-180.0 * DEG)) {
				s->th += 360.0 * DEG;
			}
			return (found && (fabs(s->th) < (70.0 * DEG))) ? RUN_OK : RUN_LOST;
		}
		default:		/* FAIL */
			return RUN_FAILED;
		}
		if (s->hit) {
			return RUN_HIT;
		}
	}
}

/* Patrol along the line up to an obstacle (stop within S5_STOP_CM), go
   round it, and again until the car is past everything on the line */
static int run(sim_t *s, int *p_looks)
{
	int n;
	int backed_n = 0;		/* as task_vc.c: VC_BACKOFF_MAX 2 */

	s->x = -700.0;
	s->y = 0.0;
	s->th = 0.0;
	for (n = 0; n < 6; n++) {
		int r;
		bool backed = false;

		while (us_cm(s, 0.0) > S5_STOP_CM) {
			if (s->x > s->sc->past_x) {
				return RUN_OK;		/* past it, following the line */
			}
			s->x += 1.0;			/* Line and Barcode keeps it on the line */
			s->y *= 0.98;
			s->th *= 0.98;
			if (touching(s)) {
				return RUN_HIT;
			}
		}
		r = bypass(s, p_looks, &backed);
		if (r != RUN_OK) {
			return r;
		}
		backed_n = backed ? (backed_n + 1) : 0;
		if (backed_n >= 2) {
			return RUN_FAILED;		/* still no way round */
		}
	}
	return RUN_STUCK;
}

/* ---- The obstacles (mm; the line is y = 0, the car comes from -x) ---- */
static scene_t g_scenes[64];
static int g_n_scenes;

static scene_t *scene_new(const char *name, bool expect_fail, double past_x)
{
	scene_t *p = &g_scenes[g_n_scenes++];

	p->name = name;
	p->n = 0;
	p->expect_fail = expect_fail;
	p->past_x = past_x;
	return p;
}

static void scene_box(scene_t *p, double x0, double x1, double y0, double y1)
{
	p->r[p->n].x0 = x0;
	p->r[p->n].x1 = x1;
	p->r[p->n].y0 = y0;
	p->r[p->n].y1 = y1;
	p->n++;
}

static char g_names[64][80];

static void scenes_make(void)
{
	static const double widths[4] = { 100.0, 130.0, 160.0, 240.0 };
	static const double depths[5] = { 100.0, 250.0, 450.0, 700.0, 1000.0 };
	static const double offs[3] = { -50.0, 0.0, 50.0 };
	int w;
	int d;
	int o;
	scene_t *p;

	for (w = 0; w < 4; w++) {
		for (d = 0; d < 5; d++) {
			o = (w + d) % 3;
			snprintf(g_names[g_n_scenes], sizeof g_names[0], "box %.0f wide, %.0f deep, %+.0f off the line",
				 widths[w], depths[d], offs[o]);
			p = scene_new(g_names[g_n_scenes], false, depths[d]);
			scene_box(p, 0.0, depths[d], offs[o] - (widths[w] / 2.0), offs[o] + (widths[w] / 2.0));
		}
	}
	/* The scan goes LEFT when it can see the obstacle's left edge (the
	   servo turns only 30 deg left): obstacles shifted to the right.
	   Otherwise it goes RIGHT. */
	p = scene_new("part sticking out LEFT 450 mm back (the bypass side)", false, 700.0);
	scene_box(p, 0.0, 700.0, -110.0, 20.0);
	scene_box(p, 450.0, 700.0, 20.0, 260.0);

	p = scene_new("part sticking out RIGHT 450 mm back (the bypass side)", false, 700.0);
	scene_box(p, 0.0, 700.0, -50.0, 50.0);
	scene_box(p, 450.0, 700.0, -280.0, -50.0);

	p = scene_new("part sticking out LEFT 200 mm back (seen by the first scan)", false, 500.0);
	scene_box(p, 0.0, 500.0, -110.0, 20.0);
	scene_box(p, 200.0, 500.0, 20.0, 230.0);

	p = scene_new("part sticking out RIGHT 200 mm back (seen by the first scan)", false, 500.0);
	scene_box(p, 0.0, 500.0, -50.0, 50.0);
	scene_box(p, 200.0, 500.0, -250.0, -50.0);

	p = scene_new("T shape: parts sticking out on BOTH sides 250 mm back", false, 500.0);
	scene_box(p, 0.0, 250.0, -50.0, 50.0);
	scene_box(p, 250.0, 500.0, -220.0, 220.0);

	p = scene_new("wider at the far end (3 steps)", false, 750.0);
	scene_box(p, 0.0, 250.0, -50.0, 50.0);
	scene_box(p, 250.0, 500.0, -120.0, 120.0);
	scene_box(p, 500.0, 750.0, -190.0, 190.0);

	p = scene_new("a second box in the lane, on the right", false, 200.0);
	scene_box(p, 0.0, 200.0, -80.0, 80.0);
	scene_box(p, 450.0, 550.0, -330.0, -200.0);

	p = scene_new("a second box in the lane, on the left", false, 200.0);
	scene_box(p, 0.0, 200.0, -110.0, 20.0);
	scene_box(p, 450.0, 550.0, 180.0, 330.0);

	p = scene_new("a wall 300 mm to the right: goes left", false, 250.0);
	scene_box(p, 0.0, 250.0, -60.0, 60.0);
	scene_box(p, -500.0, 1200.0, -320.0, -300.0);

	p = scene_new("a box beside the car that the scan cannot see (side-step blocked)", false, 200.0);
	scene_box(p, 0.0, 200.0, -80.0, 80.0);
	scene_box(p, -340.0, -200.0, -330.0, -200.0);		/* beside the car, on the right */

	p = scene_new("walls close on both sides: no way round", true, 0.0);
	scene_box(p, 0.0, 200.0, -100.0, 100.0);
	scene_box(p, -600.0, 900.0, 190.0, 210.0);
	scene_box(p, -600.0, 900.0, -210.0, -190.0);

	p = scene_new("a wall across everything: no way round", true, 0.0);
	scene_box(p, 0.0, 50.0, -1500.0, 1500.0);

	p = scene_new("an obstacle 3 m long: gives up", true, 0.0);
	scene_box(p, 0.0, 3000.0, -80.0, 80.0);
}

int main(int argc, char **argv)
{
	static const echo_t echoes[3] = {
		{ 15.0, 70.0, "ideal echo" },
		{ 15.0, 35.0, "flat faces only square on" },
		{ 7.5, 25.0, "the same, narrow beam" }
	};
	bool verbose = false;
	int only = -1;
	int seeds = 8;
	int sc;
	int e;
	int seed;
	int bad = 0;
	int runs = 0;
	int i;

	for (i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-v") == 0) {
			verbose = true;
		} else if ((strcmp(argv[i], "-s") == 0) && ((i + 1) < argc)) {
			only = atoi(argv[++i]);
		} else if ((strcmp(argv[i], "-n") == 0) && ((i + 1) < argc)) {
			seeds = atoi(argv[++i]);
		} else {
			/* ignore */
		}
	}
	scenes_make();
	for (sc = 0; sc < g_n_scenes; sc++) {
		int ok = 0;
		int gave_up = 0;
		int n = 0;
		int looks = 0;
		double mm = 0.0;
		int turns = 0;
		double deg = 0.0;
		double out_sum = 0.0;
		double out_max = 0.0;

		if ((only >= 0) && (sc != only)) {
			continue;
		}
		for (e = 0; e < 3; e++) {
			for (seed = 0; seed < seeds; seed++) {
				sim_t s;
				int r;
				int lvl = seed % 8;	/* how far off the car's moves are */

				memset(&s, 0, sizeof s);
				s.sc = &g_scenes[sc];
				s.echo = &echoes[e];
				s.seed = (unsigned)((sc * 977) + (e * 131) + (seed * 7919) + 1);
				/* seed 0: exact; 1-4: turns up to 3 % off (2.7 deg in
				   90) + 1.5 % the encoders do not see, distances
				   2 %; 5-7: 6 % + 2.5 %, 3 % */
				s.turn_err = (lvl == 0) ? 0.0 : ((lvl <= 4) ? 0.03 : 0.06);
				s.slip_err = (lvl == 0) ? 0.0 : ((lvl <= 4) ? 0.015 : 0.025);
				s.dist_err = (lvl == 0) ? 0.0 : ((lvl <= 4) ? 0.02 : 0.03);
				s.ping_mm = ((seed % 2) == 0) ? 5 : 10;
				s.verbose = verbose;
				if (verbose) {
					printf("  run: %s, %s, seed %d\n", g_scenes[sc].name, echoes[e].name, seed);
				}
				r = run(&s, &looks);
				n++;
				runs++;
				mm += s.driven_total;
				turns += s.turns;
				deg += s.turned_deg;
				if (!g_scenes[sc].expect_fail) {
					out_sum += s.max_out;
					out_max = (s.max_out > out_max) ? s.max_out : out_max;
				}
				if (r == RUN_OK) {
					ok++;
				} else if (r == RUN_FAILED) {
					gave_up++;
				} else {
					/* hit, lost or stuck: always wrong */
				}
				if ((r == RUN_HIT) || (r == RUN_LOST) || (r == RUN_STUCK) ||
				    (g_scenes[sc].expect_fail != (r == RUN_FAILED))) {
					bad++;
					printf("  WRONG: %s, %s, seed %d: %s (car at %.0f, %.0f, %.0f deg)\n",
					       g_scenes[sc].name, echoes[e].name, seed,
					       (r == RUN_HIT) ? "TOUCHED the obstacle" :
					       (r == RUN_LOST) ? "finished but not on the line past it" :
					       (r == RUN_STUCK) ? "never finished" :
					       (r == RUN_FAILED) ? "gave up" : "went round (should give up)",
					       s.x, s.y, s.th / DEG);
				}
			}
		}
		printf("BY-%02d %-66s %3d of %3d round, %3d gave up; %.1f looks, %.1f turns (%.0f deg), "
		       "%.0f mm a run, out %.0f mm (at most %.0f)\n",
		       sc + 1, g_scenes[sc].name, ok, n, gave_up, (double)looks / (double)n,
		       (double)turns / (double)n, deg / (double)n, mm / (double)n,
		       out_sum / (double)n, out_max);
	}
	printf("bypass simulation: %d runs, failures: %d\n", runs, bad);
	return (bad == 0) ? 0 : 1;
}

#endif /* VC_BYP_HOST_TEST */
