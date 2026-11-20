/*
 *	shared_bits.h
 *	Bits that more than one subsystem reads or writes: the ESTOP_REQ
 *	stop sources and the shared fault mask (Tribunal 1, "Minimum-Bit
 *	Data Representation").  This is the ONE place these values live;
 *	every subsystem uses these names instead of its own copy.
 *	Changed only by team agreement (Buddy 4 keeps the representation
 *	table).
 *
 *	Plain #defines, no kernel headers, so Buddy 1's PC unit tests can
 *	include it through telem_format.h.  The kernel objects that carry
 *	these bits (flg_estop_req, flg_sysfault) are declared in app.h.
 */

#ifndef SHARED_BITS_H
#define SHARED_BITS_H

/* ---------------------------------------------------------------------
 *  ESTOP_REQ  (event flag flg_estop_req, 3 bits used)
 *    - created by Motion Control (subsystem 2), which owns the motors
 *    - each stop source sets ONLY its own bit, with tk_set_flg()
 *    - Motion Control checks the flag every control tick, before it
 *      writes the motor PWM, and keeps both motors braked while any
 *      bit is set
 *    - ONLY the Vehicle Controller clears a bit (tk_clr_flg), once the
 *      cause is dealt with and the car is stopped
 * --------------------------------------------------------------------- */
#define ESTOP_IMU_LIFTED	(1u << 0)	/* IMU (Buddy 4): the car was
						   lifted or overturned (16 Nov;
						   was ESTOP_IMU_IMPACT) */
#define ESTOP_IMU_FAULT		(1u << 1)	/* IMU (Buddy 4): IMU not working */
#define ESTOP_OBSTACLE		(1u << 2)	/* Obstacle Scan (Buddy 5):
						   obstacle within 15 cm, or the
						   ultrasonic sensor failed */
#define ESTOP_ALL		(ESTOP_IMU_LIFTED | ESTOP_IMU_FAULT | ESTOP_OBSTACLE)

/* ---------------------------------------------------------------------
 *  Shared fault mask  (event flag flg_sysfault, and the 8-bit "fault"
 *  field in telemetry).  One bit per condition.  Each subsystem sets
 *  AND clears only its own bits.  A fault bit is for reporting; it
 *  does not stop the car by itself (ESTOP_REQ does).
 * --------------------------------------------------------------------- */
#define FAULT_MOTOR_STALL	(1u << 0)	/* Motion Control (Buddy 2)   */
#define FAULT_ENCODER		(1u << 1)	/* Motion Control (Buddy 2)   */
#define FAULT_IMU		(1u << 2)	/* IMU (Buddy 4)              */
#define FAULT_NO_ECHO		(1u << 3)	/* Obstacle Scan (Buddy 5):
						   ultrasonic sensor fault    */
#define FAULT_WIFI_LOSS		(1u << 4)	/* Telemetry (Buddy 1)        */
#define FAULT_LINE_LOST		(1u << 5)	/* Line and Barcode (Buddy 3) */
#define FAULT_NO_PATH		(1u << 6)	/* Vehicle Controller, when
						   Obstacle Scan gives up     */
#define FAULT_STARTUP		(1u << 7)	/* Vehicle Controller         */

/* Names used before this file existed (Buddy 4's IMU build,
   Obstacle Scan).  Same bits. */
#define SYSFAULT_IMU		FAULT_IMU
#define SYSFAULT_OBSTACLE	FAULT_NO_ECHO

#endif /* SHARED_BITS_H */
