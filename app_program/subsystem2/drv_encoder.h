/*
 *	drv_encoder.h
 *	Subsystem 2 -- wheel encoders, counted in hardware by PWM slices.
 *
 *	Call ONLY from motion_task, the owner of the encoder PWM slices.
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
