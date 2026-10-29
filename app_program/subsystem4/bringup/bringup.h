#ifndef BRINGUP_H
#define BRINGUP_H
#include <stdint.h>
#include <stdbool.h>

/* Simulated Motion Control / Vehicle Controller (bench testing only). main_bringup.c writes
 * these, imu_port_pico_bare.c hands them to the IMU through the normal port interface. */
extern uint32_t g_sim_snapshot;      /* MC_SNAP_PACK(speed, distance, seq)  */
extern bool     g_sim_braking;
extern bool     g_sim_have_cmd;
extern uint8_t  g_sim_cmd;
extern uint32_t g_estop_bits;        /* what the IMU asked for via ESTOP_REQ */
extern uint32_t g_sysfault_bits;     /* shared system fault bitmask (our bit = SYSFAULT_IMU_FAULT) */

bool bringup_pop_event(uint32_t *msg);   /* events the IMU posted               */
void bringup_i2c_init(void);
#endif
