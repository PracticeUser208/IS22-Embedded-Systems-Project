/*
 * imu_port.h -- the ONLY hardware/kernel boundary used by imu.c.
 * imu_port_pico_utk.c implements it for the Pico W + uT-Kernel build.
 * test/fake_port.c implements it for PC testing.
 */
#ifndef IMU_PORT_H
#define IMU_PORT_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

typedef enum { PORT_I2C_OK = 0, PORT_I2C_NACK = 1, PORT_I2C_TIMEOUT = 2 } port_i2c_rc_t;

port_i2c_rc_t port_i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val);
port_i2c_rc_t port_i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n);
void          port_i2c_recover(void);              /* clock out a stuck bus, re-init I2C1 */

uint64_t      port_time_us(void);                  /* 1 MHz timestamp                      */
uint32_t      port_time_ms(void);                  /* milliseconds since boot (wraps)      */
uint32_t      port_wait_next_poll(void);           /* block to next 10 ms tick; returns lateness in us */

bool          port_post_event(uint32_t msg);       /* imu_evt_q, ZERO timeout; false if full */
bool          port_recv_cmd(uint8_t *cmd);         /* imu_q, non-blocking                   */
uint32_t      port_read_mc_snapshot(void);         /* one aligned 32-bit load               */
bool          port_mc_braking(void);               /* Motion Control controlled stop active */
void          port_estop_set(uint32_t src_bits);   /* set bits in ESTOP_REQ (atomic)        */
void          port_sysfault_set(uint32_t bits);    /* set  OUR bit in the shared fault bitmask (atomic) */
void          port_sysfault_clear(uint32_t bits);  /* clear OUR bit only                     */

#endif /* IMU_PORT_H */
