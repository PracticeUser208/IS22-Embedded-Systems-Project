/*
 * main_bringup.c -- IMU bench-test firmware (USB serial, 115200 irrelevant: USB CDC).
 * Runs the real imu.c with a SIMULATED Motion Control. Use it to check the wiring and axes,
 * record labelled logs for threshold tuning, and watch events / ESTOP bits.
 */
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "imu.h"
#include "imu_cfg.h"
#include "imu_port.h"
#include "app_types.h"
#include "bringup.h"

#define SIM_SPEED_DEFAULT   300      /* mm/s the simulated car "drives" at          */
#define WHEEL_MODEL_MG      1000.0f  /* spike that collapses the simulated wheels   */

static const char *state_name(imu_state_t s)
{
    switch (s) { case IMU_ST_INIT: return "INIT"; case IMU_ST_CALIBRATE: return "CAL";
                 case IMU_ST_MONITORING: return "MON"; default: return "FAULT"; }
}
static const char *evt_name(uint32_t t)
{
    switch (t) { case IMU_EVT_HUMP: return "HUMP"; case IMU_EVT_MOTION: return "MOTION";
                 case IMU_EVT_IMPACT: return "IMPACT"; default: return "SENSOR_FAULT"; }
}
static const char *motion_name(uint32_t c)
{
    switch (c) { case MOTION_STATIONARY: return "stationary"; case MOTION_ACCELERATING: return "accelerating";
                 case MOTION_TURNING: return "turning"; default: return "steady"; }
}

static void help(void)
{
    printf("\n# IMU bring-up. Keys:\n"
           "#  h help   i I2C scan   p status/axes check   l toggle CSV log   0-6 set log label\n"
           "#  m sim drive   s sim stop   +/- sim speed   b toggle braking   w toggle wheel model\n"
           "#  r send RESET   c clear ESTOP (what the Vehicle Controller would do)\n"
           "#  labels: 0 none, 1 flat drive, 2 hump, 3 controlled stop, 4 collision, 5 tap/bump, 6 stationary\n");
}

int main(void)
{
    stdio_init_all();
    for (int i = 0; i < 50 && !stdio_usb_connected(); i++) sleep_ms(100);   /* wait up to 5 s */
    bringup_i2c_init();
    help();

    imu_init();
    bool logging = false, wheel_model = false;
    char label = '0';
    int sim_speed = 0, sim_cfg_speed = SIM_SPEED_DEFAULT;
    float dist_f = 0.0f;
    uint8_t seq = 0;
    uint32_t last_estop = 0;
    imu_state_t last_state = (imu_state_t)99;

    for (;;) {
        uint32_t late = port_wait_next_poll();

        /* --- keyboard --- */
        int ch = getchar_timeout_us(0);
        if (ch >= 0) {
            switch (ch) {
            case 'h': help(); break;
            case 'l': logging = !logging; printf("# log %s\n", logging ? "ON" : "OFF");
                      if (logging) printf("# D,t_ms,label,state,fault,speed,fwd,lat,up,a_dev,pitch_deg,votes,hump_mm,late_us\n"); break;
            case 'm': sim_speed = sim_cfg_speed; printf("# sim drive %d mm/s\n", sim_speed); break;
            case 's': sim_speed = 0; printf("# sim stop\n"); break;
            case '+': sim_cfg_speed += 50; printf("# sim speed %d\n", sim_cfg_speed); break;
            case '-': if (sim_cfg_speed > 50) sim_cfg_speed -= 50; printf("# sim speed %d\n", sim_cfg_speed); break;
            case 'b': g_sim_braking = !g_sim_braking; printf("# braking %d\n", g_sim_braking); break;
            case 'w': wheel_model = !wheel_model; printf("# wheel model %s\n", wheel_model ? "ON" : "OFF"); break;
            case 'r': g_sim_cmd = IMU_CMD_RESET; g_sim_have_cmd = true; printf("# RESET sent\n"); break;
            case 'c': g_estop_bits = 0; last_estop = 0; printf("# ESTOP cleared\n"); break;
            case 'i': {
                printf("# I2C1 scan (GP2/GP3):");
                int found = 0;
                for (int addr = 0x08; addr < 0x78; addr++) {
                    uint8_t b;
                    if (i2c_read_blocking_until(i2c1, (uint8_t)addr, &b, 1, false, make_timeout_time_us(2000)) >= 0) {
                        printf(" 0x%02X", addr); found++;
                    }
                }
                printf("%s\n# accelerometer should answer at 0x%02X (LSM303D: the magnetometer shares this address)\n",
                       found ? "" : " nothing found - check wiring/power", IMU_I2C_ADDR_ACCEL);
                break; }
            case 'p': {
                imu_debug_t d = imu_get_debug();
                printf("# state=%s fault=0x%02X estop=0x%X sysfault=0x%X\n# body mg: fwd=%.0f lat=%.0f up=%.0f  pitch=%.1f deg\n"
                       "# CHECK: level & still -> up ~ +1000, fwd ~ 0, lat ~ 0. Tilt nose UP -> pitch positive.\n"
                       "# Push the board FORWARD -> fwd goes positive. If not, change IMU_AXIS_*/IMU_SIGN_* in imu_cfg.h\n",
                       state_name(imu_get_state()), imu_get_fault_mask(), (unsigned)g_estop_bits, (unsigned)g_sysfault_bits,
                       d.body_mg[0], d.body_mg[1], d.body_mg[2], d.pitch_rad * 57.2958f);
                break; }
            default:
                if (ch >= '0' && ch <= '6') { label = (char)ch; printf("# label %c\n", label); }
            }
        }

        /* --- simulated Motion Control: integrates distance, writes the snapshot word --- */
        imu_debug_t d0 = imu_get_debug();
        if (wheel_model && sim_speed > 0 && fabsf(d0.a_dev_mg) > WHEEL_MODEL_MG) sim_speed = 0;
        dist_f += (float)sim_speed * (IMU_POLL_PERIOD_US / 1e6f);
        g_sim_snapshot = MC_SNAP_PACK(sim_speed, (uint32_t)dist_f, seq++);

        imu_step(late);

        /* --- report --- */
        uint32_t m;
        while (bringup_pop_event(&m)) {
            uint32_t t = IMU_MSG_TYPE(m), p = IMU_MSG_PAYLOAD(m);
            printf("EVT,%lu,%s,", (unsigned long)(time_us_64() / 1000), evt_name(t));
            if (t == IMU_EVT_HUMP)        printf("height_mm=%.1f\n", p / 2.0);
            else if (t == IMU_EVT_MOTION) printf("%s\n", motion_name(p));
            else if (t == IMU_EVT_IMPACT) printf("votes=0x%lX\n", (unsigned long)p);
            else                          printf("fault_mask=0x%02lX\n", (unsigned long)p);
        }
        if (g_estop_bits != last_estop) { printf("ESTOP,0x%X (bits now set)\n", (unsigned)g_estop_bits); last_estop = g_estop_bits; }
        if (imu_get_state() != last_state) { last_state = imu_get_state(); printf("STATE,%s\n", state_name(last_state)); }

        if (logging) {
            imu_debug_t d = imu_get_debug();
            printf("D,%lu,%c,%s,%02X,%d,%.0f,%.0f,%.0f,%.0f,%.2f,%u,%.1f,%lu\n",
                   (unsigned long)(time_us_64() / 1000), label, state_name(imu_get_state()), imu_get_fault_mask(),
                   sim_speed, d.body_mg[0], d.body_mg[1], d.body_mg[2], d.a_dev_mg, d.pitch_rad * 57.2958f,
                   d.votes, d.hump_mm, (unsigned long)late);
        }
    }
}
