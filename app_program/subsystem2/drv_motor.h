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
