/*
 *	task_motion.c
 *	Subsystem 2 -- motion_task: the 50 Hz control loop.
 *
 *	motion_task is the ONLY code that touches the motors and encoders.
 *	Every MOTION_TICK_MS the cyclic handler wakes it and it:
 *
 *	  1. reads both encoders -> wheel position (mm) and speed (mm/s)
 *	  2. takes any new commands from mbf_motion_cmd (newest wins)
 *	  3. runs the current state:
 *	       MOVE / TURN : speed profile (ramp up, slow near target)
 *	                     + left/right sync so the car runs straight
 *	                     + finish when the distance/arc is covered
 *	       VELOCITY    : use the caller's wheel speeds (line following)
 *	       IDLE        : brake
 *	  4. runs one speed PID per wheel: duty = feed-forward + PID
 *	  5. publishes a status snapshot for telemetry
 *
 *	Single-channel encoders cannot tell direction, so each wheel's
 *	edges are counted in the direction of its current SETPOINT, and
 *	the PID is never allowed to drive a wheel against its setpoint.
 */

#include "motion_private.h"
#include "motion_config.h"
#include "motion_math.h"
#include "drv_motor.h"
#include "drv_encoder.h"

#define TICK_S		((float)MOTION_TICK_MS / 1000.0f)
#define WHEELS		2u
#define WHEEL_L		0u
#define WHEEL_R		1u

typedef struct
{
	motion_pid_t	pid;
	uint16_t	prev_raw;		/* last hardware count */
	int16_t		window[ENC_SPEED_WINDOW];	/* signed edges/tick */
	int32_t		pos_counts;		/* signed position */
	uint32_t	total_counts;		/* raw edges, for telemetry */
	int32_t		dir;			/* +1 / -1: counting direction */
	int32_t		start_counts;		/* position when move began */
	float		speed_mm_s;		/* measured */
	float		target_mm_s;		/* PID setpoint */
	int16_t		duty_permille;		/* applied */
} wheel_t;

typedef struct
{
	motion_state_t	state;
	float		goal_mm;	/* MOVE: distance, TURN: arc per wheel */
	float		cruise_mm_s;
	float		profile_mm_s;	/* current ramped speed */
	int32_t		dir_l;		/* wheel directions for this move */
	int32_t		dir_r;
	uint32_t	deadline_ms;	/* MOVE/TURN fault time */
	uint32_t	last_vel_ms;	/* VELOCITY watchdog */
	float		vel_l_mm_s;
	float		vel_r_mm_s;
} motion_ctrl_t;

static wheel_t		g_wheel[WHEELS];
static motion_ctrl_t	g_ctrl;
static uint32_t		g_now_ms;	/* time since start, in ticks */
static uint32_t		g_window_idx;
static uint32_t		g_fault_count;
static int32_t		g_odometer_counts;

/* ------------------------------------------------------------------ */

EXPORT void motion_cyclic_handler(void *exinf)
{
	/* Interrupt context: keep it short, never wait */
	(void)exinf;
	(void)tk_wup_tsk(tid_motion);
}

/* 1. Encoders -> position and windowed speed */
static void read_encoders(void)
{
	static const encoder_id_t enc_of[WHEELS] = { ENCODER_LEFT, ENCODER_RIGHT };
	uint32_t w;
	uint32_t k;

	for (w = 0u; w < WHEELS; w++)
	{
		wheel_t		*p_w = &g_wheel[w];
		uint16_t	raw = encoder_read_raw(enc_of[w]);
		int32_t		edges = (int32_t)motion_count_delta(raw, p_w->prev_raw);
		int32_t		sum = 0;

		p_w->prev_raw      = raw;
		p_w->total_counts += (uint32_t)edges;
		p_w->pos_counts   += p_w->dir * edges;
		g_odometer_counts += edges;

		p_w->window[g_window_idx] = (int16_t)(p_w->dir * edges);
		for (k = 0u; k < ENC_SPEED_WINDOW; k++)
		{
			sum += p_w->window[k];
		}
		p_w->speed_mm_s = motion_counts_to_mm(sum) /
				  ((float)ENC_SPEED_WINDOW * TICK_S);
	}
	g_window_idx = (g_window_idx + 1u) % ENC_SPEED_WINDOW;
}

static void reset_wheel_control(void)
{
	uint32_t w;

	for (w = 0u; w < WHEELS; w++)
	{
		pid_reset(&g_wheel[w].pid);
		g_wheel[w].target_mm_s = 0.0f;
	}
}

/* End the current move.  b_fault = it timed out before arriving. */
static void finish(bool b_fault)
{
	motor_brake(MOTOR_LEFT);
	motor_brake(MOTOR_RIGHT);
	reset_wheel_control();
	g_ctrl.state        = MOTION_STATE_IDLE;
	g_ctrl.profile_mm_s = 0.0f;

	if (b_fault)
	{
		g_fault_count++;
		(void)tk_set_flg(flg_motion, MOTION_FLG_DONE | MOTION_FLG_FAULT);
	}
	else
	{
		(void)tk_set_flg(flg_motion, MOTION_FLG_DONE);
	}
}

static void begin_move(motion_state_t state, float goal_mm, float cruise_mm_s,
		       int32_t dir_l, int32_t dir_r)
{
	float expected_ms = (goal_mm / cruise_mm_s) * 1000.0f;

	g_ctrl.state        = state;
	g_ctrl.goal_mm      = goal_mm;
	g_ctrl.cruise_mm_s  = cruise_mm_s;
	g_ctrl.profile_mm_s = 0.0f;
	g_ctrl.dir_l        = dir_l;
	g_ctrl.dir_r        = dir_r;
	g_ctrl.deadline_ms  = g_now_ms +
			      ((uint32_t)expected_ms * MOTION_TIMEOUT_FACTOR) +
			      MOTION_TIMEOUT_MARGIN_MS;

	g_wheel[WHEEL_L].start_counts = g_wheel[WHEEL_L].pos_counts;
	g_wheel[WHEEL_R].start_counts = g_wheel[WHEEL_R].pos_counts;
	reset_wheel_control();
}

/* 2. Apply one command (a new command replaces the current one) */
static void apply_command(const motion_cmd_t *p_cmd)
{
	float	cruise;
	int32_t	sign = (p_cmd->value >= 0) ? 1 : -1;

	switch (p_cmd->type)
	{
		case MOTION_CMD_MOVE:
			cruise = (p_cmd->speed_mm_s > 0) ?
				 (float)p_cmd->speed_mm_s : MOTION_DEFAULT_SPEED_MM_S;
			begin_move(MOTION_STATE_MOVE, (float)(sign * p_cmd->value),
				   cruise, sign, sign);
			break;

		case MOTION_CMD_TURN:
			/* left turn (+): left wheel back, right wheel forward */
			begin_move(MOTION_STATE_TURN,
				   motion_turn_arc_mm((float)(sign * p_cmd->value)),
				   MOTION_TURN_SPEED_MM_S, -sign, sign);
			break;

		case MOTION_CMD_VELOCITY:
			if (g_ctrl.state != MOTION_STATE_VELOCITY)
			{
				reset_wheel_control();
			}
			g_ctrl.state       = MOTION_STATE_VELOCITY;
			g_ctrl.vel_l_mm_s  = (float)p_cmd->left_mm_s;
			g_ctrl.vel_r_mm_s  = (float)p_cmd->right_mm_s;
			g_ctrl.last_vel_ms = g_now_ms;
			break;

		case MOTION_CMD_STOP:
		default:
			finish(false);
			break;
	}
}

static void receive_commands(void)
{
	motion_cmd_t	cmd;
	INT		size;

	/* TMO_POL: never block the control loop.  Drain everything that
	   queued up since the last tick; the newest command wins. */
	size = tk_rcv_mbf(mbf_motion_cmd, &cmd, TMO_POL);
	while (size == (INT)sizeof(cmd))
	{
		apply_command(&cmd);
		size = tk_rcv_mbf(mbf_motion_cmd, &cmd, TMO_POL);
	}
}

/* 3. MOVE / TURN: profile + straight-line sync -> wheel setpoints */
static void run_move(void)
{
	float travel_l = motion_counts_to_mm(g_wheel[WHEEL_L].pos_counts - g_wheel[WHEEL_L].start_counts);
	float travel_r = motion_counts_to_mm(g_wheel[WHEEL_R].pos_counts - g_wheel[WHEEL_R].start_counts);
	float abs_l = (travel_l < 0.0f) ? -travel_l : travel_l;
	float abs_r = (travel_r < 0.0f) ? -travel_r : travel_r;
	float remaining = g_ctrl.goal_mm - ((abs_l + abs_r) * 0.5f);
	float sync;

	if (remaining <= 0.0f)
	{
		finish(false);
		return;
	}
	if ((int32_t)(g_now_ms - g_ctrl.deadline_ms) >= 0)
	{
		finish(true);		/* stalled, blocked, or lost encoder */
		return;
	}

	g_ctrl.profile_mm_s = motion_profile_speed(g_ctrl.profile_mm_s,
						   g_ctrl.cruise_mm_s,
						   remaining, TICK_S);

	/* Left ahead of right -> slow left, speed up right */
	sync = motion_clampf(SYNC_KP * (abs_l - abs_r),
			     -SYNC_MAX_MM_S, SYNC_MAX_MM_S);

	g_wheel[WHEEL_L].target_mm_s = (float)g_ctrl.dir_l * (g_ctrl.profile_mm_s - sync);
	g_wheel[WHEEL_R].target_mm_s = (float)g_ctrl.dir_r * (g_ctrl.profile_mm_s + sync);
}

static void run_velocity(void)
{
	if ((g_now_ms - g_ctrl.last_vel_ms) > MOTION_VEL_TIMEOUT_MS)
	{
		finish(false);		/* caller went quiet: stop safely */
		return;
	}
	g_wheel[WHEEL_L].target_mm_s = g_ctrl.vel_l_mm_s;
	g_wheel[WHEEL_R].target_mm_s = g_ctrl.vel_r_mm_s;
}

/* 4. Speed PID per wheel -> motor duty */
static void drive_wheels(void)
{
	static const motor_id_t motor_of[WHEELS] = { MOTOR_LEFT, MOTOR_RIGHT };
	uint32_t w;

	for (w = 0u; w < WHEELS; w++)
	{
		wheel_t	*p_w = &g_wheel[w];
		float	duty;

		if (p_w->target_mm_s == 0.0f)
		{
			pid_reset(&p_w->pid);
			p_w->duty_permille = 0;
			motor_brake(motor_of[w]);
			continue;
		}

		p_w->dir = (p_w->target_mm_s > 0.0f) ? 1 : -1;
		duty = motion_feedforward(p_w->target_mm_s) +
		       pid_update(&p_w->pid, p_w->target_mm_s,
				  p_w->speed_mm_s, TICK_S);

		/* Never drive against the setpoint: with no direction
		   sensing we could not count those edges correctly. */
		duty = (p_w->dir > 0) ? motion_clampf(duty, 0.0f, 1.0f)
				      : motion_clampf(duty, -1.0f, 0.0f);

		p_w->duty_permille = (int16_t)(duty * (float)MOTOR_DUTY_MAX);
		motor_set_duty(motor_of[w], p_w->duty_permille);
	}
}

/* 5. Status snapshot for other subsystems (telemetry) */
static void publish_status(void)
{
	/* TMO_POL: if a reader holds the lock right now, skip this tick
	   rather than delay the control loop. */
	if (tk_loc_mtx(mtx_motion_status, TMO_POL) != E_OK)
	{
		return;
	}
	g_motion_status.state               = g_ctrl.state;
	g_motion_status.left_mm             = (int32_t)motion_counts_to_mm(g_wheel[WHEEL_L].pos_counts);
	g_motion_status.right_mm            = (int32_t)motion_counts_to_mm(g_wheel[WHEEL_R].pos_counts);
	g_motion_status.odometer_mm         = (int32_t)(motion_counts_to_mm(g_odometer_counts) * 0.5f);
	g_motion_status.left_speed_mm_s     = (int16_t)g_wheel[WHEEL_L].speed_mm_s;
	g_motion_status.right_speed_mm_s    = (int16_t)g_wheel[WHEEL_R].speed_mm_s;
	g_motion_status.left_target_mm_s    = (int16_t)g_wheel[WHEEL_L].target_mm_s;
	g_motion_status.right_target_mm_s   = (int16_t)g_wheel[WHEEL_R].target_mm_s;
	g_motion_status.left_duty_permille  = g_wheel[WHEEL_L].duty_permille;
	g_motion_status.right_duty_permille = g_wheel[WHEEL_R].duty_permille;
	g_motion_status.left_count          = g_wheel[WHEEL_L].total_counts;
	g_motion_status.right_count         = g_wheel[WHEEL_R].total_counts;
	g_motion_status.fault_count         = g_fault_count;
	(void)tk_unl_mtx(mtx_motion_status);
}

static void motion_hw_init(void)
{
	uint32_t w;

	CHECK(motor_init(), "motor_init");
	CHECK(encoder_init(), "encoder_init");
	motor_brake(MOTOR_LEFT);
	motor_brake(MOTOR_RIGHT);

	g_wheel[WHEEL_L].prev_raw = encoder_read_raw(ENCODER_LEFT);
	g_wheel[WHEEL_R].prev_raw = encoder_read_raw(ENCODER_RIGHT);
	for (w = 0u; w < WHEELS; w++)
	{
		g_wheel[w].dir = 1;
		pid_init(&g_wheel[w].pid, SPEED_KP, SPEED_KI, SPEED_KD,
			 SPEED_I_LIMIT, -1.0f, 1.0f);
	}
	g_ctrl.state = MOTION_STATE_IDLE;
}

EXPORT void motion_task(INT stacd, void *exinf)
{
	(void)stacd;
	(void)exinf;

	/* Hardware is set up HERE, by the owning task, not in _init() */
	motion_hw_init();
	tm_printf((UB *)"[S2] motion_task ready, %d mm per encoder edge x100\n",
		  (INT)(MM_PER_COUNT * 100.0f));

	while (1)
	{
		(void)tk_slp_tsk(TMO_FEVR);	/* until the 20 ms tick */
		g_now_ms += MOTION_TICK_MS;

		read_encoders();
		receive_commands();

		switch (g_ctrl.state)
		{
			case MOTION_STATE_MOVE:
			case MOTION_STATE_TURN:
				run_move();
				break;
			case MOTION_STATE_VELOCITY:
				run_velocity();
				break;
			case MOTION_STATE_IDLE:
			default:
				g_wheel[WHEEL_L].target_mm_s = 0.0f;
				g_wheel[WHEEL_R].target_mm_s = 0.0f;
				break;
		}

		drive_wheels();
		publish_status();
	}
}
