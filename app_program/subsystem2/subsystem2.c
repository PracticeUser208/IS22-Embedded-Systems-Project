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
 *	  flg_estop_req      event flag      team ESTOP_REQ (app.h): created
 *	                                     here because Motion Control owns
 *	                                     the motors; IMU / Obstacle Scan
 *	                                     set bits, the VC clears them
 */

#include "subsystem2.h"

EXPORT ID		mbf_motion_cmd;
EXPORT ID		flg_motion;
EXPORT ID		mtx_motion_status;
EXPORT ID		cyc_motion;
EXPORT ID		tid_motion;
EXPORT motion_status_t	g_motion_status;
EXPORT ID		flg_estop_req;	/* team flag, declared in app.h */

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

	/* ESTOP_REQ: one bit per stop source (shared_bits.h), all clear at
	   power-up.  Several tasks may wait on it (TA_WMUL). */
	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0u;
	flg_estop_req = tk_cre_flg(&cflg);
	if (flg_estop_req < E_OK)
	{
		return flg_estop_req;
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

/* ======== from motion_api.c ======== */
/*
 *	motion_api.c
 *	Subsystem 2 -- the public motion API (see subsystem2.h).
 *
 *	These functions run in the CALLER's task, not in motion_task.
 *	They never touch hardware: they check the arguments, pack a
 *	motion_cmd_t and send it to motion_task through mbf_motion_cmd.
 */


static ER motion_send(const motion_cmd_t *p_cmd)
{
	ER er;

	/* Clear DONE/FAULT BEFORE sending, so a motion_wait_done() that
	   follows cannot see the DONE left over from the previous move. */
	er = tk_clr_flg(flg_motion, ~(MOTION_FLG_DONE | MOTION_FLG_FAULT));
	if (er == E_OK)
	{
		er = tk_snd_mbf(mbf_motion_cmd, p_cmd, (INT)sizeof(*p_cmd),
				MOTION_SEND_TMO_MS);
	}
	return er;
}

static bool speed_ok(int16_t speed_mm_s)
{
	return (speed_mm_s >= 0) && (speed_mm_s <= MOTION_MAX_SPEED_MM_S);
}

static ER motion_move(int32_t distance_mm, int16_t speed_mm_s, int32_t sign)
{
	motion_cmd_t cmd = {0};

	if ((distance_mm <= 0) || (distance_mm > MOTION_MAX_DIST_MM) ||
	    !speed_ok(speed_mm_s))
	{
		return E_PAR;
	}
	cmd.type       = MOTION_CMD_MOVE;
	cmd.value      = sign * distance_mm;
	cmd.speed_mm_s = speed_mm_s;	/* 0 -> default speed */
	return motion_send(&cmd);
}

static ER motion_turn(int32_t angle_deg, int32_t sign)
{
	motion_cmd_t cmd = {0};

	if ((angle_deg <= 0) || (angle_deg > MOTION_MAX_ANGLE_DEG))
	{
		return E_PAR;
	}
	cmd.type  = MOTION_CMD_TURN;
	cmd.value = sign * angle_deg;
	return motion_send(&cmd);
}

EXPORT ER motion_move_forward(int32_t distance_mm, int16_t speed_mm_s)
{
	return motion_move(distance_mm, speed_mm_s, 1);
}

EXPORT ER motion_move_backward(int32_t distance_mm, int16_t speed_mm_s)
{
	return motion_move(distance_mm, speed_mm_s, -1);
}

EXPORT ER motion_turn_left(int32_t angle_deg)
{
	return motion_turn(angle_deg, 1);
}

EXPORT ER motion_turn_right(int32_t angle_deg)
{
	return motion_turn(angle_deg, -1);
}

EXPORT ER motion_stop(void)
{
	motion_cmd_t cmd = {0};

	cmd.type = MOTION_CMD_STOP;
	return motion_send(&cmd);
}

EXPORT ER motion_set_velocity(int16_t left_mm_s, int16_t right_mm_s)
{
	motion_cmd_t cmd = {0};

	if ((left_mm_s  > MOTION_MAX_SPEED_MM_S) ||
	    (left_mm_s  < -MOTION_MAX_SPEED_MM_S) ||
	    (right_mm_s > MOTION_MAX_SPEED_MM_S) ||
	    (right_mm_s < -MOTION_MAX_SPEED_MM_S))
	{
		return E_PAR;
	}
	cmd.type       = MOTION_CMD_VELOCITY;
	cmd.left_mm_s  = left_mm_s;
	cmd.right_mm_s = right_mm_s;
	return motion_send(&cmd);
}

EXPORT ER motion_wait_done(int32_t timeout_ms)
{
	UINT	ptn = 0u;
	ER	er;

	/* No TWF_CLR: DONE stays set, so several tasks can wait for it */
	er = tk_wai_flg(flg_motion, MOTION_FLG_DONE, TWF_ORW, &ptn,
			(TMO)timeout_ms);
	if ((er == E_OK) && ((ptn & MOTION_FLG_FAULT) != 0u))
	{
		er = E_IO;
	}
	return er;
}

EXPORT ER motion_get_status(motion_status_t *p_status)
{
	ER er;

	if (p_status == NULL)
	{
		return E_PAR;
	}
	er = tk_loc_mtx(mtx_motion_status, MOTION_SEND_TMO_MS);
	if (er == E_OK)
	{
		*p_status = g_motion_status;	/* struct copy */
		er = tk_unl_mtx(mtx_motion_status);
	}
	return er;
}

/* ======== from motion_math.c ======== */
/*
 *	motion_math.c
 *	Subsystem 2 -- hardware-free maths (see motion_math.h).
 *
 *	Float is used for readability while tuning.  The RP2040 has no FPU,
 *	so these are software float calls, but at 50 Hz for two wheels that
 *	is a few thousand operations per second -- negligible.  Constants
 *	use the 'f' suffix so nothing is silently promoted to (slower)
 *	double.
 *
 *	Encoder resolution note
 *	-----------------------
 *	With a 20-slot disc on a 65 mm wheel one edge is ~10 mm.  At
 *	200 mm/s that is only 0.4 edges per 20 ms tick, so speed is
 *	averaged over ENC_SPEED_WINDOW ticks.  If your encoder has few
 *	counts per revolution and the speed PID is too jumpy, measure the
 *	TIME BETWEEN EDGES instead (GPIO interrupt + timestamp); only
 *	drv_encoder.c and the speed calculation in task_motion.c change.
 */


float motion_clampf(float value, float lo, float hi)
{
	float result = value;

	if (result < lo)
	{
		result = lo;
	}
	else if (result > hi)
	{
		result = hi;
	}
	else
	{
		/* in range: unchanged */
	}
	return result;
}

void pid_init(motion_pid_t *p_pid, float kp, float ki, float kd,
	      float i_limit, float out_min, float out_max)
{
	p_pid->kp      = kp;
	p_pid->ki      = ki;
	p_pid->kd      = kd;
	p_pid->i_limit = i_limit;
	p_pid->out_min = out_min;
	p_pid->out_max = out_max;
	pid_reset(p_pid);
}

void pid_reset(motion_pid_t *p_pid)
{
	p_pid->integral  = 0.0f;
	p_pid->prev_meas = 0.0f;
	p_pid->b_primed  = false;
}

/*
 * One PID step.
 *  - Integral is stored already multiplied by ki, so changing ki while
 *    tuning does not make the output jump.
 *  - Integral is clamped to +/- i_limit (anti-windup), so a wheel that
 *    was held still (hump, wall) does not shoot off when released.
 *  - Derivative acts on the MEASUREMENT, not the error, so a step in
 *    the setpoint does not cause a spike ("derivative kick").
 */
float pid_update(motion_pid_t *p_pid, float setpoint, float measured, float dt_s)
{
	float error = setpoint - measured;
	float d_term = 0.0f;
	float out;

	p_pid->integral += p_pid->ki * error * dt_s;
	p_pid->integral  = motion_clampf(p_pid->integral,
					 -p_pid->i_limit, p_pid->i_limit);

	if (p_pid->b_primed && (dt_s > 0.0f))
	{
		d_term = -p_pid->kd * (measured - p_pid->prev_meas) / dt_s;
	}
	p_pid->prev_meas = measured;
	p_pid->b_primed  = true;

	out = (p_pid->kp * error) + p_pid->integral + d_term;
	return motion_clampf(out, p_pid->out_min, p_pid->out_max);
}

uint16_t motion_count_delta(uint16_t now, uint16_t prev)
{
	/* uint16_t arithmetic wraps modulo 65536 (well defined for
	   unsigned types), e.g. now = 3, prev = 65534 -> 5.  The cast is
	   needed because both operands are promoted to int first. */
	return (uint16_t)(now - prev);
}

float motion_counts_to_mm(int32_t counts)
{
	return (float)counts * MM_PER_COUNT;
}

int32_t motion_mm_to_counts(float mm)
{
	float counts = mm / MM_PER_COUNT;

	/* round half away from zero */
	return (int32_t)((counts >= 0.0f) ? (counts + 0.5f) : (counts - 0.5f));
}

float motion_turn_arc_mm(float angle_deg)
{
	/* Pivot on the spot: each wheel runs on a circle whose diameter
	   is the track width.  arc = pi * track * (angle / 360). */
	return (MOTION_PI * TRACK_WIDTH_MM * angle_deg) / 360.0f;
}

float motion_feedforward(float target_mm_s)
{
	float duty = 0.0f;

	if (target_mm_s > 0.0f)
	{
		duty = MOTOR_FF_DEADBAND + (MOTOR_FF_GAIN * target_mm_s);
	}
	else if (target_mm_s < 0.0f)
	{
		duty = -MOTOR_FF_DEADBAND + (MOTOR_FF_GAIN * target_mm_s);
	}
	else
	{
		/* zero speed: no feed-forward, let the motor stop */
	}
	return motion_clampf(duty, -1.0f, 1.0f);
}

float motion_profile_speed(float current_mm_s, float cruise_mm_s,
			   float remaining_mm, float dt_s)
{
	float speed = current_mm_s + (MOTION_ACCEL_MM_S2 * dt_s);
	float decel_cap;

	if (speed > cruise_mm_s)
	{
		speed = cruise_mm_s;
	}

	/* Slow down linearly as the target approaches */
	decel_cap = MOTION_MIN_SPEED_MM_S + (MOTION_DECEL_GAIN_PER_S * remaining_mm);
	if (speed > decel_cap)
	{
		speed = decel_cap;
	}

	if (speed < MOTION_MIN_SPEED_MM_S)
	{
		speed = MOTION_MIN_SPEED_MM_S;
	}
	return speed;
}

/* ======== from drv_motor.c ======== */
/*
 *	drv_motor.c
 *	Subsystem 2 -- DC motor driver.
 *
 *	Each motor has two driver inputs on the two channels (A, B) of one
 *	PWM slice.  Driving one input with PWM and holding the other low
 *	sets speed and direction; both high brakes; both low coasts.
 *
 *	Uses the board library (libbsp) for pin / wrap / compare setup and
 *	writes the slice's DIV register directly because libbsp has no
 *	call for it.
 *
 *	The kernel only takes the PWM block out of reset when its physical
 *	timer is enabled (USE_PTMR, off in config.h), so motor_init() does
 *	it.  Only ever CLEARS the reset bit, with the atomic clear alias, so
 *	it is harmless if the scan servo (subsystem 5) did it first.
 */

#include <bsp/libbsp.h>

#define PWM_SLICE_OF(pin)	(((pin) >> 1) & 0x07u)
#define PWM_SLICE_BASE(slice)	(PWM_BASE + (0x14u * (slice)))
#define PWM_DIV_INT_1		(1u << 4)	/* DIV = 1.0 (8.4 fixed) */
#define RESETS_ALIAS_CLR	0x3000u		/* atomic bit-clear alias */

/* Both pins of a motor must be the A and B channels of ONE slice */
#if (PWM_SLICE_OF(MOTOR_L_PIN_A) != PWM_SLICE_OF(MOTOR_L_PIN_B)) || \
    (PWM_SLICE_OF(MOTOR_R_PIN_A) != PWM_SLICE_OF(MOTOR_R_PIN_B))
#error "motor pins A/B must share one PWM slice"
#endif

typedef struct
{
	uint32_t	pin_a;
	uint32_t	pin_b;
	bool		b_invert;
} motor_hw_t;

static const motor_hw_t g_motor_hw[MOTOR_COUNT] =
{
	{ MOTOR_L_PIN_A, MOTOR_L_PIN_B, (MOTOR_L_INVERT != 0) },
	{ MOTOR_R_PIN_A, MOTOR_R_PIN_B, (MOTOR_R_INVERT != 0) },
};

static ER motor_pin_setup(uint32_t pin)
{
	ER er = pwm_set_pin(pin);

	if (er == E_OK)
	{
		er = pwm_set_wrap(pin, MOTOR_PWM_TOP);
	}
	if (er == E_OK)
	{
		er = pwm_set_cc(pin, 0u);	/* start with output low */
	}
	return er;
}

ER motor_init(void)
{
	ER		er = E_OK;
	uint32_t	idx;

	/* PWM block out of reset (see the top of this file) */
	if ((in_w(RESETS_RESET) & RESETS_RESET_PWM) != 0u)
	{
		out_w(RESETS_RESET + RESETS_ALIAS_CLR, RESETS_RESET_PWM);
		while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0u)
		{
			;	/* a few cycles */
		}
	}

	for (idx = 0u; (idx < (uint32_t)MOTOR_COUNT) && (er == E_OK); idx++)
	{
		const motor_hw_t *p_hw = &g_motor_hw[idx];

		out_w(PWM_SLICE_BASE(PWM_SLICE_OF(p_hw->pin_a)) + PWM_CHx_DIV,
		      PWM_DIV_INT_1);
		er = motor_pin_setup(p_hw->pin_a);
		if (er == E_OK)
		{
			er = motor_pin_setup(p_hw->pin_b);
		}
		if (er == E_OK)
		{
			er = pwm_set_enabled(p_hw->pin_a, TRUE);  /* whole slice */
		}
	}
	return er;
}

void motor_set_duty(motor_id_t motor, int16_t duty_permille)
{
	const motor_hw_t	*p_hw;
	int32_t			duty = duty_permille;
	uint32_t		cc;

	if ((uint32_t)motor >= (uint32_t)MOTOR_COUNT)
	{
		return;
	}
	p_hw = &g_motor_hw[motor];

	if (p_hw->b_invert)
	{
		duty = -duty;
	}
	if (duty > MOTOR_DUTY_MAX)
	{
		duty = MOTOR_DUTY_MAX;
	}
	else if (duty < -MOTOR_DUTY_MAX)
	{
		duty = -MOTOR_DUTY_MAX;
	}
	else
	{
		/* in range */
	}

	/* permille -> compare value.  Done in 32-bit unsigned:
	   1000 * 6250 = 6.25e6, no overflow. */
	cc = ((uint32_t)((duty < 0) ? -duty : duty) * (MOTOR_PWM_TOP + 1u))
	     / (uint32_t)MOTOR_DUTY_MAX;

	if (duty >= 0)
	{
		(void)pwm_set_cc(p_hw->pin_b, 0u);
		(void)pwm_set_cc(p_hw->pin_a, cc);
	}
	else
	{
		(void)pwm_set_cc(p_hw->pin_a, 0u);
		(void)pwm_set_cc(p_hw->pin_b, cc);
	}
}

void motor_brake(motor_id_t motor)
{
	const motor_hw_t *p_hw;

	if ((uint32_t)motor >= (uint32_t)MOTOR_COUNT)
	{
		return;
	}
	p_hw = &g_motor_hw[motor];

	/* compare > TOP keeps the output high for the whole period */
	(void)pwm_set_cc(p_hw->pin_a, MOTOR_PWM_TOP + 1u);
	(void)pwm_set_cc(p_hw->pin_b, MOTOR_PWM_TOP + 1u);
}

/* ======== from drv_encoder.c ======== */
/*
 *	drv_encoder.c
 *	Subsystem 2 -- wheel encoders.
 *
 *	Week 6 pin map: left encoder on GP16, right on GP17 (Grove 4),
 *	"pulses counted in an ISR".  Each rising edge raises a GPIO
 *	interrupt and adds 1 to that wheel's 16-bit count.  The API is the
 *	same as before (encoder_init, encoder_read_raw), so the motion task
 *	does not change.
 *
 *	Why not the PWM-slice counter any more?
 *	---------------------------------------
 *	A PWM slice can only count edges on its channel-B pin, which is
 *	always an ODD GPIO.  GP16 is even (slice 0, channel A), so the Week 6
 *	wiring cannot use it.  An interrupt works on any pin.
 *
 *	Why through the shared GPIO dispatcher (gpio_irq_attach)?
 *	---------------------------------------------------------
 *	All 30 GPIOs share ONE interrupt (IO_IRQ_BANK0), and tk_def_int()
 *	keeps one handler per interrupt, so the ultrasonic echo (GP5) and
 *	these encoders cannot each install their own: the second would
 *	silently replace the first.  The shared dispatcher installs the one
 *	handler and calls a callback per pin (Week 6 feedback: "route both
 *	through one GPIO callback dispatcher").  It lives in subsystem 5
 *	(subsystem5.c / subsystem5.h) until the team picks a place for
 *	shared code.
 *
 *	Load: about 40 edges/s per wheel at 400 mm/s (20 counts per turn,
 *	65 mm wheel), a few microseconds each.
 *
 *	Limits (unchanged): rising edges only, no direction (single-channel
 *	encoder) -- the motion task takes the direction from the sign of the
 *	duty it applied.
 */

#include "../subsystem5/subsystem5.h"	/* gpio_irq_attach (shared GPIO interrupt) */
#include <bsp/libbsp.h>

#if (ENC_L_PIN == ENC_R_PIN)
#error "the two encoders need two different pins"
#endif

static const uint32_t g_enc_pin[ENCODER_COUNT] = { ENC_L_PIN, ENC_R_PIN };

/* Written only by the interrupt (one 16-bit store), read by the motion
   task.  An aligned 16-bit read on the M0+ is a single access, so the
   task never sees half an update.  Wraps 0xFFFF -> 0 like the old
   hardware counter, which motion_count_delta() already handles. */
static volatile uint16_t g_enc_count[ENCODER_COUNT];

/* Interrupt context: count and return, nothing else */
static void encoder_edge_cb(UINT pin, UW events, UW now_us)
{
	(void)events;
	(void)now_us;

	if (pin == ENC_L_PIN)
	{
		g_enc_count[ENCODER_LEFT]++;
	}
	else
	{
		g_enc_count[ENCODER_RIGHT]++;
	}
}

/* Task context (motion_task) only: may call tk_def_int via the dispatcher */
ER encoder_init(void)
{
	ER		er = E_OK;
	uint32_t	idx;

	for (idx = 0u; (idx < (uint32_t)ENCODER_COUNT) && (er == E_OK); idx++)
	{
		uint32_t pin = g_enc_pin[idx];

		g_enc_count[idx] = 0u;
		(void)gpio_set_pin(pin, GPIO_MODE_IN);
		/* Pull-up for open-collector sensor outputs (LM393 type) */
		set_w(GPIO(pin), GPIO_PUE);
		clr_w(GPIO(pin), GPIO_PDE);

		er = gpio_irq_attach(pin, GIRQ_EDGE_RISE, encoder_edge_cb);
	}
	return er;
}

uint16_t encoder_read_raw(encoder_id_t enc)
{
	uint16_t count = 0u;

	if ((uint32_t)enc < (uint32_t)ENCODER_COUNT)
	{
		count = g_enc_count[enc];
	}
	return count;
}
