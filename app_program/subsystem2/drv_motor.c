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
 */

#include "drv_motor.h"
#include "motion_config.h"
#include <bsp/libbsp.h>

#define PWM_SLICE_OF(pin)	(((pin) >> 1) & 0x07u)
#define PWM_SLICE_BASE(slice)	(PWM_BASE + (0x14u * (slice)))
#define PWM_DIV_INT_1		(1u << 4)	/* DIV = 1.0 (8.4 fixed) */

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
