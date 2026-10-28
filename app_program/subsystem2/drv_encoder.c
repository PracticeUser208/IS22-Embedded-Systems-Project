/*
 *	drv_encoder.c
 *	Subsystem 2 -- wheel encoders.
 *
 *	Why a PWM slice and not a GPIO interrupt?
 *	-----------------------------------------
 *	An RP2040 PWM slice can run as a COUNTER: in DIVMODE "rising edge"
 *	its 16-bit counter advances once per rising edge on the slice's
 *	channel-B pin.  The hardware counts every edge with no interrupt
 *	and no CPU time, and the motion task just reads the counter every
 *	tick.  It also avoids the single shared GPIO interrupt vector
 *	(IO_IRQ_BANK0), which other subsystems (ultrasonic echo, barcode)
 *	may want.
 *
 *	Limits: rising edges only, and no direction (single-channel
 *	encoder) -- the motion task takes the direction from the sign of
 *	the duty it applied.  Cost: one whole PWM slice per encoder.
 */

#include "drv_encoder.h"
#include "motion_config.h"
#include <bsp/libbsp.h>

#define PWM_SLICE_OF(pin)	(((pin) >> 1) & 0x07u)
#define PWM_SLICE_BASE(slice)	(PWM_BASE + (0x14u * (slice)))

#define PWM_CSR_EN		(1u << 0)
#define PWM_CSR_DIVMODE_RISE	(2u << 4)	/* count rising edges on B */
#define PWM_DIV_INT_1		(1u << 4)	/* count every edge */
#define PWM_TOP_FULL		0xFFFFu		/* wrap at 16 bits */

#if ((ENC_L_PIN & 1u) == 0u) || ((ENC_R_PIN & 1u) == 0u)
#error "encoder pins must be odd (PWM channel B)"
#endif
#if (PWM_SLICE_OF(ENC_L_PIN) == PWM_SLICE_OF(ENC_R_PIN))
#error "encoders need two different PWM slices"
#endif
#if (PWM_SLICE_OF(ENC_L_PIN) == PWM_SLICE_OF(MOTOR_L_PIN_A)) || \
    (PWM_SLICE_OF(ENC_L_PIN) == PWM_SLICE_OF(MOTOR_R_PIN_A)) || \
    (PWM_SLICE_OF(ENC_R_PIN) == PWM_SLICE_OF(MOTOR_L_PIN_A)) || \
    (PWM_SLICE_OF(ENC_R_PIN) == PWM_SLICE_OF(MOTOR_R_PIN_A))
#error "an encoder slice is also used by a motor"
#endif

static const uint32_t g_enc_pin[ENCODER_COUNT] = { ENC_L_PIN, ENC_R_PIN };

ER encoder_init(void)
{
	ER		er = E_OK;
	uint32_t	idx;

	for (idx = 0u; (idx < (uint32_t)ENCODER_COUNT) && (er == E_OK); idx++)
	{
		uint32_t pin  = g_enc_pin[idx];
		uint32_t base = PWM_SLICE_BASE(PWM_SLICE_OF(pin));

		er = pwm_set_pin(pin);		/* pin -> PWM function, input on */
		if (er == E_OK)
		{
			/* Pull-up for open-collector sensor outputs (LM393 type) */
			set_w(GPIO(pin), GPIO_PUE);
			clr_w(GPIO(pin), GPIO_PDE);

			out_w(base + PWM_CHx_CSR, 0u);		/* stop slice */
			out_w(base + PWM_CHx_DIV, PWM_DIV_INT_1);
			out_w(base + PWM_CHx_TOP, PWM_TOP_FULL);
			out_w(base + PWM_CHx_CTR, 0u);
			out_w(base + PWM_CHx_CSR, PWM_CSR_DIVMODE_RISE | PWM_CSR_EN);
		}
	}
	return er;
}

uint16_t encoder_read_raw(encoder_id_t enc)
{
	uint16_t count = 0u;

	if ((uint32_t)enc < (uint32_t)ENCODER_COUNT)
	{
		uint32_t base = PWM_SLICE_BASE(PWM_SLICE_OF(g_enc_pin[enc]));
		count = (uint16_t)(in_w(base + PWM_CHx_CTR) & 0xFFFFu);
	}
	return count;
}
