/*
 *	motion_api.c
 *	Subsystem 2 -- the public motion API (see subsystem2.h).
 *
 *	These functions run in the CALLER's task, not in motion_task.
 *	They never touch hardware: they check the arguments, pack a
 *	motion_cmd_t and send it to motion_task through mbf_motion_cmd.
 */

#include "motion_private.h"

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
