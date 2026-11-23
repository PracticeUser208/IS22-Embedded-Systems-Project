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

#endif /* SUBSYSTEM2_H */
