/*
 *	motion_private.h
 *	Subsystem 2 -- definitions shared by the files of subsystem 2 only.
 */

#ifndef MOTION_PRIVATE_H
#define MOTION_PRIVATE_H

#include "subsystem2.h"

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
