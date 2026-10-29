/*
 * app_types.h  --  SHARED HEADER (Buddy 4 maintains the ESTOP_REQ + IMU sections)
 *
 * Every width below is a PLACEHOLDER until the real ranges are measured/agreed
 * by the team. Anything sent over MQTT must use explicit masks/shifts (below),
 * never compiler bit-fields.
 */
#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include "app_fields.h"

/* All encodings (enums, flag bits, widths) are GENERATED into app_fields.h from
 * tools/gen_data_repr.py: ESTOP_SRC_*, SYSFAULT_*, IMU_EVT_*, MOTION_*, IMU_FAULT_*, VOTE_*, IMU_ST_*.
 * ESTOP_REQ rules: each source sets ONLY its own bit; Motion Control reads the flag before every
 * PWM write; only the Vehicle Controller clears bits. The IMU never clears them and never writes PWM. */
#define IMU_FAULT_BLOCKING    (IMU_FAULT_NACK | IMU_FAULT_STALE | IMU_FAULT_BUS_STUCK | \
                               IMU_FAULT_CAL_FAIL | IMU_FAULT_TIMING)

/* ---------- 32-bit imu_evt_q message ----------
 * [31:16] timestamp (ms, low 16 bits, wraps every 65.5 s)
 * [15:9]  spare
 * [8:2]   payload (7 bits)
 * [1:0]   event type                                                            */
#define IMU_MSG_PACK(type, payload, ts16) \
    ((uint32_t)(((uint32_t)(type) & 0x3u) | (((uint32_t)(payload) & 0x7Fu) << 2) | \
                (((uint32_t)(ts16) & 0xFFFFu) << 16)))
#define IMU_MSG_TYPE(m)      ((uint32_t)(m) & 0x3u)
#define IMU_MSG_PAYLOAD(m)   (((uint32_t)(m) >> 2) & 0x7Fu)
#define IMU_MSG_TS16(m)      (((uint32_t)(m) >> 16) & 0xFFFFu)

/* ---------- Motion Control snapshot word (single writer = Motion Control) ----------
 * One aligned 32-bit store/load, so the IMU never sees a torn speed/distance pair.
 * [9:0] speed mm/s   [25:10] distance mm (wraps)   [31:26] sequence count
 * Ranges are placeholders; Buddy 2 confirms.                                       */
#define MC_SNAP_PACK(speed, dist, seq) \
    ((uint32_t)(((uint32_t)(speed) & 0x3FFu) | (((uint32_t)(dist) & 0xFFFFu) << 10) | \
                (((uint32_t)(seq) & 0x3Fu) << 26)))
#define MC_SNAP_SPEED(w)  ((uint16_t)((uint32_t)(w) & 0x3FFu))
#define MC_SNAP_DIST(w)   ((uint16_t)(((uint32_t)(w) >> 10) & 0xFFFFu))
#define MC_SNAP_SEQ(w)    ((uint8_t)(((uint32_t)(w) >> 26) & 0x3Fu))

#endif /* APP_TYPES_H */
