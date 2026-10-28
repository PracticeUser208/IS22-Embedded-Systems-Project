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
 *  Encoders: each encoder needs a WHOLE PWM slice, used as a hardware
 *  edge counter on its channel-B pin, so the pin must be ODD and its
 *  slice (pin / 2 % 8) must not be used by anyone else for PWM.
 *  TODO move to the pins your encoders are actually wired to.
 * --------------------------------------------------------------------- */
#define MOTOR_L_PIN_A		8u	/* slice 4 A */
#define MOTOR_L_PIN_B		9u	/* slice 4 B */
#define MOTOR_R_PIN_A		10u	/* slice 5 A */
#define MOTOR_R_PIN_B		11u	/* slice 5 B */

#define ENC_L_PIN		3u	/* slice 1 B */
#define ENC_R_PIN		5u	/* slice 2 B */

/* Set to 1 if that wheel turns backwards on a "forward" command
   (motors are mounted mirror-image, so one of them usually is). */
#define MOTOR_L_INVERT		0
#define MOTOR_R_INVERT		1	/* TODO check on the bench */

/* ---------------------------------------------------------------------
 *  PWM: 125 MHz / 1 / (6249 + 1) = 20 kHz, above hearing range.
 * --------------------------------------------------------------------- */
#define MOTOR_PWM_TOP		6249u

/* ---------------------------------------------------------------------
 *  Geometry  (TODO measure)
 *    ENC_COUNTS_PER_REV : edges counted per WHEEL revolution
 *                         (slots in the disc x gear ratio if the disc
 *                          is on the motor shaft).  Rising edges only.
 *    WHEEL_DIAMETER_MM  : measure over the tyre, or better: drive 10
 *                         revolutions and divide distance by 10*pi.
 *    TRACK_WIDTH_MM     : centre of left tyre to centre of right tyre.
 *                         Fine-tune it until a 360 deg turn is 360.
 * --------------------------------------------------------------------- */
#define ENC_COUNTS_PER_REV	20.0f
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
#define MOTION_TURN_SPEED_MM_S		120.0f	/* each wheel, in turns */
#define MOTION_MIN_SPEED_MM_S		60.0f	/* creep speed near target */
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
#define MOTOR_FF_DEADBAND	0.25f	/* duty where the wheel starts to turn */
#define MOTOR_FF_GAIN		0.0018f	/* extra duty per mm/s */

#define SPEED_KP		0.0015f
#define SPEED_KI		0.0040f
#define SPEED_KD		0.0f
#define SPEED_I_LIMIT		0.30f	/* anti-windup: max integral duty */

/* Straight-line correction: mm/s of speed difference per mm that the
   left wheel is ahead of the right wheel. */
#define SYNC_KP			4.0f
#define SYNC_MAX_MM_S		60.0f

#endif /* MOTION_CONFIG_H */
