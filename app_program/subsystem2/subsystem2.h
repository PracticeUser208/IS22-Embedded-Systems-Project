/*
 *	subsystem2.h
 *	Subsystem 2 -- Motion Control (motors + wheel encoders).
 *	Student ID: 2500261 (Buddy 2, group IS22).
 *
 *	PUBLIC interface of subsystem 2.  Other subsystems include it as
 *	"../subsystem2/subsystem2.h" and use ONLY what is declared here.
 *
 *	How other subsystems move the car
 *	---------------------------------
 *	The motors and encoders are owned by ONE task, motion_task
 *	(TEAM.md section 3).  Nobody else touches the PWM or encoder pins.
 *	Other tasks call the motion_xxx() functions below; each one packs a
 *	command into the message buffer mbf_motion_cmd and returns at once.
 *	motion_task picks it up on its next control tick (every 20 ms).
 *
 *	    motion_turn_left(90);                     -- start a 90 deg turn
 *	    er = motion_wait_done(3000);              -- block until finished
 *
 *	A new command always REPLACES the one in progress, so
 *	motion_stop() works at any time (e.g. obstacle detected).
 *
 *	Line following uses velocity mode instead of distance moves:
 *	    motion_set_velocity(left_mm_s, right_mm_s);   -- every cycle
 *	If no new velocity arrives within MOTION_VEL_TIMEOUT_MS the car
 *	stops by itself (protects against a crashed caller task).
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include <stdint.h>
#include <stdbool.h>
#include "../app.h"

/* ---------------------------------------------------------------------
 *  Task priority and timing (TEAM.md section 4: band 14-15)
 * --------------------------------------------------------------------- */
#define S2_PRI_MOTION		14	/* control loop: highest in our band */

#define MOTION_TICK_MS		20u	/* control period: 50 Hz */
#define MOTION_VEL_TIMEOUT_MS	300u	/* velocity-mode watchdog */

/* ---------------------------------------------------------------------
 *  Limits accepted by the API (requests outside these are rejected
 *  with E_PAR instead of being silently clipped).
 * --------------------------------------------------------------------- */
#define MOTION_MAX_SPEED_MM_S	400	/* fastest commanded wheel speed */
#define MOTION_MAX_DIST_MM	5000	/* longest single move */
#define MOTION_MAX_ANGLE_DEG	360	/* largest single turn */

/* ---------------------------------------------------------------------
 *  Event flag flg_motion: bits other tasks may wait on
 * --------------------------------------------------------------------- */
#define MOTION_FLG_DONE		(1u << 0)  /* last move/turn/stop finished */
#define MOTION_FLG_FAULT	(1u << 1)  /* last move timed out (stalled?) */

/* ---------------------------------------------------------------------
 *  Status, read with motion_get_status() (telemetry, navigation)
 * --------------------------------------------------------------------- */
typedef enum
{
	MOTION_STATE_IDLE = 0,	/* motors braked, nothing to do */
	MOTION_STATE_MOVE,	/* driving a set distance, fwd or back */
	MOTION_STATE_TURN,	/* pivoting on the spot by an angle */
	MOTION_STATE_VELOCITY	/* following wheel speeds from a caller */
} motion_state_t;

typedef struct
{
	motion_state_t	state;
	int32_t		left_mm;	/* wheel travel since power-up */
	int32_t		right_mm;
	int32_t		odometer_mm;	/* total |distance| driven */
	int16_t		left_speed_mm_s;	/* measured */
	int16_t		right_speed_mm_s;
	int16_t		left_target_mm_s;	/* PID setpoint */
	int16_t		right_target_mm_s;
	int16_t		left_duty_permille;	/* -1000..1000 */
	int16_t		right_duty_permille;
	uint32_t	left_count;	/* raw encoder edges */
	uint32_t	right_count;
	uint32_t	fault_count;	/* moves that timed out */
	uint32_t	seq;		/* +1 each time this is published (every
					   control tick): lets the IMU see the
					   data is fresh (Tribunal 1, hump height) */
} motion_status_t;

/* ---------------------------------------------------------------------
 *  The API.  Callable from any TASK (not from handlers/ISRs).
 *  All return E_OK, E_PAR (bad argument) or a kernel error code.
 *  distance in mm, angle in degrees, speed in mm/s (0 = default).
 * --------------------------------------------------------------------- */
IMPORT ER motion_move_forward(int32_t distance_mm, int16_t speed_mm_s);
IMPORT ER motion_move_backward(int32_t distance_mm, int16_t speed_mm_s);
IMPORT ER motion_turn_left(int32_t angle_deg);
IMPORT ER motion_turn_right(int32_t angle_deg);
IMPORT ER motion_stop(void);
IMPORT ER motion_set_velocity(int16_t left_mm_s, int16_t right_mm_s);

/* Wait until the current move/turn/stop is DONE.
   Returns E_OK, E_TMOUT, or E_IO if the move faulted (timed out). */
IMPORT ER motion_wait_done(int32_t timeout_ms);

IMPORT ER motion_get_status(motion_status_t *p_status);

/* ---------------------------------------------------------------------
 *  Kernel objects owned by subsystem 2 (created in subsystem2_init).
 *  Other subsystems should use the functions above, not these IDs.
 * --------------------------------------------------------------------- */
IMPORT ID	mbf_motion_cmd;	/* IF-xx: commands in */
IMPORT ID	flg_motion;	/* IF-xx: DONE / FAULT out */
IMPORT ID	mtx_motion_status;	/* guards the status copy */
IMPORT ID	cyc_motion;	/* 20 ms control tick */
IMPORT ID	tid_motion;

/* ======== from motion_config.h ======== */
/*
 *	motion_config.h
 *	Subsystem 2 -- pins, robot geometry and tuning constants.
 *
 *	EVERY value marked TODO must be checked against YOUR car before the
 *	numbers mean anything.  Calibration procedure: see the comments on
 *	each block and testing/subsystem2/TEST_LOG.md.
 *
 *	Private to subsystem 2: do not include from other subsystems.
 */

#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H

/* ---------------------------------------------------------------------
 *  Pins  (claim every one in TEAM.md section 2 first)
 *
 *  Motors: Robo Pico on-board driver, 2 inputs per motor.
 *    IN_A = PWM, IN_B = 0  -> forward      both 0 -> coast
 *    IN_A = 0,   IN_B = PWM -> reverse     both 1 -> brake
 *  TODO verify pins and truth table against the Robo Pico datasheet.
 *
 *  Encoders: Week 6 pin map, Grove 4 (GP16 left, GP17 right).  Each
 *  rising edge is counted by a GPIO interrupt (drv_encoder.c), through
 *  the team's shared GPIO interrupt dispatcher, as the Week 6 design
 *  says ("pulses counted in an ISR").
 *
 *  These pins follow the Week 6 pin map (TEAM.md section 2), so they
 *  match the car rather than a practice board.  The earlier GP3/GP5
 *  would clash with the IMU's I2C1 clock (GP3) and the ultrasonic
 *  echo (GP5).
 * --------------------------------------------------------------------- */
#define MOTOR_L_PIN_A		8u	/* M1A, slice 4 A */
#define MOTOR_L_PIN_B		9u	/* M1B, slice 4 B */
#define MOTOR_R_PIN_A		10u	/* M2A, slice 5 A */
#define MOTOR_R_PIN_B		11u	/* M2B, slice 5 B */

#define ENC_L_PIN		16u	/* Grove 4 */
#define ENC_R_PIN		17u	/* Grove 4 */

/* Set to 1 if that wheel turns backwards on a "forward" command
   (motors are mounted mirror-image, so one of them usually is). */
#define MOTOR_L_INVERT		0
#define MOTOR_R_INVERT		1	/* bench 29 Oct: both wheels forward */

/* ---------------------------------------------------------------------
 *  PWM: 125 MHz / 1 / (6249 + 1) = 20 kHz, above hearing range.
 * --------------------------------------------------------------------- */
#define MOTOR_PWM_TOP		6249u

/* ---------------------------------------------------------------------
 *  Geometry  (ENC_COUNTS_PER_REV measured; the rest TODO measure)
 *    ENC_COUNTS_PER_REV : edges counted per WHEEL revolution
 *                         (slots in the disc x gear ratio if the disc
 *                          is on the motor shaft).  Rising edges only.
 *    WHEEL_DIAMETER_MM  : measure over the tyre, or better: drive 10
 *                         revolutions and divide distance by 10*pi.
 *    TRACK_WIDTH_MM     : centre of left tyre to centre of right tyre.
 *                         Fine-tune it until a 360 deg turn is 360.
 * --------------------------------------------------------------------- */
/* Measured 29 Oct: right wheel turned 5 times by hand = 3138 edges
   (627.6 per turn).  Likely a 13-pulse encoder x 1:48 gearbox (624). */
#define ENC_COUNTS_PER_REV	628.0f
#define WHEEL_DIAMETER_MM	65.0f
#define TRACK_WIDTH_MM		120.0f

#define MOTION_PI		3.14159265f
#define MM_PER_COUNT		((MOTION_PI * WHEEL_DIAMETER_MM) / ENC_COUNTS_PER_REV)

/* Speed is averaged over this many ticks (window = N * 20 ms).
   Low-resolution encoders need a longer window; see the notes in
   motion_math.c. */
#define ENC_SPEED_WINDOW	5u

/* ---------------------------------------------------------------------
 *  Motion profile
 * --------------------------------------------------------------------- */
#define MOTION_DEFAULT_SPEED_MM_S	200.0f	/* moves */
#define MOTION_TURN_SPEED_MM_S		25.0f	/* each wheel, in turns (120 -> 80
						   -> 60 -> 20 -> 10 -> 20: slow
						   tests, 2 Nov; 90 deg ~5 s) */
#define MOTION_MIN_SPEED_MM_S		20.0f	/* creep speed near target (was 60,
						   then 20: not above the test speed) */
#define MOTION_ACCEL_MM_S2		400.0f	/* ramp-up limit */
#define MOTION_DECEL_GAIN_PER_S		3.0f	/* cap = min + gain * remaining */

/* A move gives up (FAULT) after: expected time * factor + margin */
#define MOTION_TIMEOUT_FACTOR		3
#define MOTION_TIMEOUT_MARGIN_MS	2000

/* ---------------------------------------------------------------------
 *  Speed PID, one per wheel.  Output is duty as a fraction -1.0..1.0.
 *  Tuning order (record each step in the PID tuning report):
 *    1. FF only (KP=KI=KD=0): find MOTOR_FF_* so the wheel roughly
 *       reaches the target speed on its own.
 *    2. Raise KP until it responds fast but does not oscillate.
 *    3. Add KI to remove the remaining steady-state error.
 *    4. KD usually stays 0 with noisy, low-resolution encoders.
 * --------------------------------------------------------------------- */
/* Speed control with or without the encoders (2 Nov).
   1 = speed PID on each wheel: each wheel is held at its target speed
       measured by its encoder (needs both encoders working).
   0 = NO encoder feedback: each wheel gets only the feed-forward power
       for its target speed (MOTOR_FF_xxx below), the same on both sides.
       Only for when an encoder is broken.  Measured 2 Nov with 0: a
       "20 mm/s" target ran at 150-220 mm/s (the FF values are guesses)
       and the left wheel turned 25 % more than the right at equal power.
   Set back to 1 on 2 Nov once the left encoder counted again. */
#define MOTION_SPEED_FEEDBACK	1

/* 2 Nov: were 0.25 + 0.0018 per mm/s (duty 0.29 for 20 mm/s) -- that
   duty ran the wheels at about 185 mm/s, so every start and line search
   shot off at 2-3 x the set speed before the PID caught up.  The line
   below passes through that measurement (0.30 -> about 185 mm/s); the
   PID adds what is missing.  Lower = gentler starts. */
#define MOTOR_FF_DEADBAND	0.15f	/* duty where the wheel starts to turn */
#define MOTOR_FF_GAIN		0.0008f	/* extra duty per mm/s */

#define SPEED_KP		0.0015f
#define SPEED_KI		0.0040f
#define SPEED_KD		0.0f
#define SPEED_I_LIMIT		0.30f	/* anti-windup: max integral duty */

/* Straight-line correction: mm/s of speed difference per mm that the
   left wheel is ahead of the right wheel. */
#define SYNC_KP			4.0f
#define SYNC_MAX_MM_S		60.0f

#endif /* MOTION_CONFIG_H */

/* ======== from motion_math.h ======== */
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

/* ======== from drv_motor.h ======== */
/*
 *	drv_motor.h
 *	Subsystem 2 -- DC motor driver (Robo Pico, 2 PWM inputs per motor).
 *
 *	Call ONLY from motion_task, the owner of the motor PWM slices.
 */

#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

typedef enum
{
	MOTOR_LEFT = 0,
	MOTOR_RIGHT,
	MOTOR_COUNT
} motor_id_t;

#define MOTOR_DUTY_MAX		1000	/* duty in permille: -1000..1000 */

ER	motor_init(void);

/* Signed duty: +forward, -reverse, 0 = coast.  Clipped to +/-1000. */
void	motor_set_duty(motor_id_t motor, int16_t duty_permille);

/* Short both motor terminals: stops quickly and holds position */
void	motor_brake(motor_id_t motor);

#endif /* DRV_MOTOR_H */

/* ======== from drv_encoder.h ======== */
/*
 *	drv_encoder.h
 *	Subsystem 2 -- wheel encoders (GP16 / GP17, Week 6 pin map), counted
 *	by a GPIO interrupt on each rising edge (drv_encoder.c).
 *
 *	Call ONLY from motion_task, the owner of the encoder pins.
 */

#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include <stdint.h>
#include <tk/tkernel.h>

typedef enum
{
	ENCODER_LEFT = 0,
	ENCODER_RIGHT,
	ENCODER_COUNT
} encoder_id_t;

ER		encoder_init(void);

/* Raw 16-bit edge counter.  It wraps 0xFFFF -> 0; use
   motion_count_delta() to get edges since the previous read. */
uint16_t	encoder_read_raw(encoder_id_t enc);

#endif /* DRV_ENCODER_H */

/* ======== from motion_private.h ======== */
/*
 *	motion_private.h
 *	Subsystem 2 -- definitions shared by the files of subsystem 2 only.
 */

#ifndef MOTION_PRIVATE_H
#define MOTION_PRIVATE_H


/* Command sent through mbf_motion_cmd (the message buffer COPIES it,
   so the caller's local variable can go out of scope immediately). */
typedef enum
{
	MOTION_CMD_STOP = 0,
	MOTION_CMD_MOVE,	/* value = signed distance mm (+fwd / -back) */
	MOTION_CMD_TURN,	/* value = signed angle deg (+left / -right) */
	MOTION_CMD_VELOCITY	/* left/right = wheel speeds mm/s */
} motion_cmd_type_t;

typedef struct
{
	motion_cmd_type_t	type;
	int32_t			value;
	int16_t			speed_mm_s;	/* cruise speed, MOVE only */
	int16_t			left_mm_s;	/* VELOCITY only */
	int16_t			right_mm_s;
} motion_cmd_t;

#define MOTION_CMD_QUEUE_LEN	4u	/* commands the buffer can hold */
#define MOTION_SEND_TMO_MS	50	/* caller waits at most this long */

/* Status snapshot: written by motion_task, read under the mutex */
IMPORT motion_status_t	g_motion_status;

IMPORT void	motion_task(INT stacd, void *exinf);
IMPORT void	motion_cyclic_handler(void *exinf);

#endif /* MOTION_PRIVATE_H */


#endif /* SUBSYSTEM2_H */
