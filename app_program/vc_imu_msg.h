/*
 *	vc_imu_msg.h
 *	Vehicle Controller -- the IMU's message formats, as Buddy 4 defined
 *	them (subsystem4/app_types.h IMU_MSG_PACK, subsystem4/app_fields.h
 *	imu_evt_type_t / motion_class_t / imu_cmd_t).
 *
 *	The VC cannot include Buddy 4's app_fields.h (it redefines sys_state_t
 *	and others that subsystem1/telem_format.h defines), so the values are
 *	repeated here, kernel-free, and subsystem4/imu_port_pico_utk.c checks
 *	at compile time that every one matches Buddy 4's (_Static_assert).
 *
 *	imu_evt_q word (IMU -> VC), 32 bits:
 *	  [31:16] timestamp, ms (low 16 bits, wraps every 65.5 s)
 *	  [15:9]  spare
 *	  [8:2]   payload (7 bits)
 *	  [1:0]   event type
 *	  HUMP          payload = height in 0.5 mm steps (127 = over range);
 *	                sent when the hump is FINISHED
 *	  MOTION        payload = motion class (below), on a change
 *	  LIFTED        (16 Nov, was IMPACT) the car was lifted or overturned;
 *	                payload = why (1 moved up, 2 tipped over, 3 both); the
 *	                IMU has already set ESTOP_IMU_LIFTED
 *	  SENSOR_FAULT  payload = IMU fault bits; 0 = fault cleared (the IMU
 *	                recalibrated); the IMU set ESTOP_IMU_FAULT on entry
 *	imu_q word (VC -> IMU): the command, IMU_CMD_RESET = 0 (recalibrate;
 *	the IMU only accepts it while the car is stopped).
 */

#ifndef VC_IMU_MSG_H
#define VC_IMU_MSG_H

#include <stdint.h>

#define VC_IMU_EVT_HUMP			0u
#define VC_IMU_EVT_MOTION		1u
#define VC_IMU_EVT_LIFTED		2u
#define VC_IMU_EVT_SENSOR_FAULT		3u

#define VC_IMU_MOT_STATIONARY		0u
#define VC_IMU_MOT_ACCELERATING		1u
#define VC_IMU_MOT_TURNING		2u
#define VC_IMU_MOT_STEADY		3u

#define VC_IMU_CMD_RESET		0u

#define VC_IMU_MSG_PACK(type, payload, ts16) \
	((uint32_t)(((uint32_t)(type) & 0x3u) | (((uint32_t)(payload) & 0x7Fu) << 2) | \
		    (((uint32_t)(ts16) & 0xFFFFu) << 16)))
#define VC_IMU_MSG_TYPE(m)		((uint32_t)(m) & 0x3u)
#define VC_IMU_MSG_PAYLOAD(m)		(((uint32_t)(m) >> 2) & 0x7Fu)
#define VC_IMU_MSG_TS16(m)		(((uint32_t)(m) >> 16) & 0xFFFFu)

/* 20 Nov: the car's tilt, for the dashboard.  The IMU task writes it with
 * one aligned 32-bit store every 0.5 s; the VC reads it once a second.
 *   [13:7] roll  (+ right side up), [6:0] pitch (+ nose up): each the
 *          value furthest from 0 in the last 0.5-1 s, signed, 0.5 deg
 *          steps (-64..63 = -32..+31.5 deg) */
#define VC_IMU_TILT_PACK(pitch_x2, roll_x2) \
	((uint32_t)(((uint32_t)(pitch_x2) & 0x7Fu) | (((uint32_t)(roll_x2) & 0x7Fu) << 7)))
#define VC_IMU_TILT_SEXT7(v)		((int32_t)(((v) ^ 0x40u) & 0x7Fu) - 64)
#define VC_IMU_TILT_PITCH(w)		VC_IMU_TILT_SEXT7((uint32_t)(w) & 0x7Fu)
#define VC_IMU_TILT_ROLL(w)		VC_IMU_TILT_SEXT7(((uint32_t)(w) >> 7) & 0x7Fu)
extern volatile uint32_t vc_imu_tilt_word;

#endif /* VC_IMU_MSG_H */
