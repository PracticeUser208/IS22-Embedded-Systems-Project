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

#include "motion_math.h"
#include "motion_config.h"

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
