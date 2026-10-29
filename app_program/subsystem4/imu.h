#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"
#include "imu_cfg.h"

/* Diagnostic counters (saturating, not application data). */
typedef struct {
    uint16_t polls, impacts, single_votes, humps, drops, resets, reset_rejected;
    uint16_t snap_stale, ds_jumps;
} imu_stats_t;

void        imu_init(void);
void        imu_step(uint32_t late_us);   /* one poll cycle (called by imu_run, or by tests) */
void        imu_run(void);                /* task body: never returns                        */
imu_state_t imu_get_state(void);
uint8_t     imu_get_fault_mask(void);
imu_stats_t imu_get_stats(void);

#if IMU_ENABLE_DEBUG
typedef struct {
    float    body_mg[3];      /* forward, lateral, up (mg)            */
    float    a_dev_mg;        /* |a| - 1 g                            */
    float    pitch_rad;       /* filtered pitch, relative to baseline */
    uint8_t  votes;           /* VOTE_* mask active this poll         */
    bool     hump_active;
    float    hump_mm;         /* running height integral (gain applied) */
    uint16_t speed_mm_s;
} imu_debug_t;
imu_debug_t imu_get_debug(void);
#endif

#endif /* IMU_H */
