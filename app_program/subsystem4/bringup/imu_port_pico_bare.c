/*
 * imu_port_pico_bare.c -- bench-test port: pico-sdk only, NO uT-Kernel.
 * Same imu_port.h interface as the real build, so imu.c / imu_logic.c are unchanged.
 */
#include "imu_port.h"
#include "imu_cfg.h"
#include "bringup.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#define IMU_I2C        i2c1
#define IMU_SDA_PIN    2            /* GP2 = I2C1 SDA */
#define IMU_SCL_PIN    3            /* GP3 = I2C1 SCL */
#define IMU_I2C_HZ     400000
#define I2C_TIMEOUT_US 2000

uint32_t g_sim_snapshot;
bool     g_sim_braking;
bool     g_sim_have_cmd;
uint8_t  g_sim_cmd;
uint32_t g_estop_bits;
uint32_t g_sysfault_bits;

#define EVQ_N 32
static uint32_t evq[EVQ_N];
static volatile unsigned evq_head, evq_tail;

void bringup_i2c_init(void)
{
    i2c_init(IMU_I2C, IMU_I2C_HZ);
    gpio_set_function(IMU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA_PIN);
    gpio_pull_up(IMU_SCL_PIN);
}

static port_i2c_rc_t map_rc(int rc, size_t expected)
{
    if (rc == PICO_ERROR_TIMEOUT) {
        return PORT_I2C_TIMEOUT;
    }
    if (rc < 0 || (size_t)rc != expected) {
        return PORT_I2C_NACK;
    }
    return PORT_I2C_OK;
}

port_i2c_rc_t port_i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return map_rc(i2c_write_blocking_until(IMU_I2C, addr, buf, 2, false,
                                           make_timeout_time_us(I2C_TIMEOUT_US)), 2);
}

port_i2c_rc_t port_i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n)
{
    int rc = i2c_write_blocking_until(IMU_I2C, addr, &reg, 1, true,
                                      make_timeout_time_us(I2C_TIMEOUT_US));
    if (rc == PICO_ERROR_TIMEOUT) {
        return PORT_I2C_TIMEOUT;
    }
    if (rc != 1) {
        return PORT_I2C_NACK;
    }
    return map_rc(i2c_read_blocking_until(IMU_I2C, addr, buf, n, false,
                                          make_timeout_time_us(I2C_TIMEOUT_US)), n);
}

void port_i2c_recover(void)
{
    i2c_deinit(IMU_I2C);
    gpio_init(IMU_SCL_PIN); gpio_set_dir(IMU_SCL_PIN, GPIO_OUT);
    gpio_init(IMU_SDA_PIN); gpio_set_dir(IMU_SDA_PIN, GPIO_IN);
    for (int i = 0; i < 9 && !gpio_get(IMU_SDA_PIN); i++) {
        gpio_put(IMU_SCL_PIN, 0); sleep_us(5);
        gpio_put(IMU_SCL_PIN, 1); sleep_us(5);
    }
    bringup_i2c_init();
}

uint64_t port_time_us(void) { return time_us_64(); }
uint32_t port_time_ms(void) { return to_ms_since_boot(get_absolute_time()); }

uint32_t port_wait_next_poll(void)
{
    static uint64_t next;
    uint64_t now = time_us_64();
    if (next == 0 || now > next + 5 * IMU_POLL_PERIOD_US) {
        next = now;   /* resync after a stall */
    }
    next += IMU_POLL_PERIOD_US;
    sleep_until(from_us_since_boot(next));
    uint64_t t = time_us_64();
    return t > next ? (uint32_t)(t - next) : 0;
}

bool port_post_event(uint32_t msg)
{
    unsigned nh = (evq_head + 1) % EVQ_N;
    if (nh == evq_tail) {
        return false;   /* full: IMU drops and counts it */
    }
    evq[evq_head] = msg;
    evq_head = nh;
    return true;
}

bool bringup_pop_event(uint32_t *msg)
{
    if (evq_tail == evq_head) {
        return false;
    }
    *msg = evq[evq_tail];
    evq_tail = (evq_tail + 1) % EVQ_N;
    return true;
}

bool port_recv_cmd(uint8_t *cmd)
{
    if (!g_sim_have_cmd) {
        return false;
    }
    *cmd = g_sim_cmd;
    g_sim_have_cmd = false;
    return true;
}

uint32_t port_read_mc_snapshot(void) { return g_sim_snapshot; }
bool     port_mc_braking(void)       { return g_sim_braking; }
void     port_estop_set(uint32_t b)  { g_estop_bits |= b; }
void     port_sysfault_set(uint32_t b)   { g_sysfault_bits |= b; }
void     port_sysfault_clear(uint32_t b) { g_sysfault_bits &= ~b; }
