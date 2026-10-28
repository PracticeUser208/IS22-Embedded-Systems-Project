/*
 *	motion_math.h
 *	Subsystem 2 -- hardware-free maths: PID, unit conversion, profile.
 *
 *	Nothing in here touches a register or a kernel object, so every
 *	function can be unit-tested (subsystem2_test.c) without the car
 *	moving, and even compiled and tested on a PC.
 */

#ifndef MOTION_MATH_H
#define MOTION_MATH_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
	float	kp;
	float	ki;
	float	kd;
	float	i_limit;	/* |integral term| never exceeds this */
	float	out_min;
	float	out_max;
	float	integral;	/* state: accumulated ki * error * dt */
	float	prev_meas;	/* state: for derivative on measurement */
	bool	b_primed;	/* state: prev_meas is valid */
} motion_pid_t;

void	pid_init(motion_pid_t *p_pid, float kp, float ki, float kd,
		 float i_limit, float out_min, float out_max);
void	pid_reset(motion_pid_t *p_pid);
float	pid_update(motion_pid_t *p_pid, float setpoint, float measured, float dt_s);

float	motion_clampf(float value, float lo, float hi);

/* Edges counted since last read of a 16-bit hardware counter.
   Correct across the 0xFFFF -> 0 wrap (unsigned arithmetic). */
uint16_t motion_count_delta(uint16_t now, uint16_t prev);

float	motion_counts_to_mm(int32_t counts);
int32_t	motion_mm_to_counts(float mm);

/* Arc each wheel travels when the car pivots on the spot by angle_deg */
float	motion_turn_arc_mm(float angle_deg);

/* Feed-forward duty for a target speed (sign follows the speed) */
float	motion_feedforward(float target_mm_s);

/* Next profile speed: ramp up by at most accel*dt, and never faster
   than what still lets us slow down for remaining_mm. */
float	motion_profile_speed(float current_mm_s, float cruise_mm_s,
			     float remaining_mm, float dt_s);

#endif /* MOTION_MATH_H */
