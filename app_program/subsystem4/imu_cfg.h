/*
 * imu_cfg.h -- IMU tunables.
 * EVERY NUMBER HERE IS A PLACEHOLDER until tuned from logged data (A7, A8, A12).
 * Sensor: LSM303D (the GY-511 on Buddy 5's car; WHO_AM_I 0x0F reads 0x49). Accelerometer AND magnetometer
 * share one address, 0x1D (SA0 high). The older LSM303DLHC GY-511 answers at 0x19 and uses different
 * control registers, so this driver does not run on it unchanged.
 * VERIFY against your board's datasheet before trusting any of this.
 */
#ifndef IMU_CFG_H
#define IMU_CFG_H

/* ---- sensor ---- */
#define IMU_I2C_ADDR_ACCEL        0x1D     /* LSM303D, SA0 high (was 0x19 for the LSM303DLHC) */
#define IMU_POLL_PERIOD_US        10000
#define IMU_SENS_MG_PER_LSB       4.0f     /* +-8 g. The LSM303D is really 3.904 mg/count (readings ~2.5% high);
                                              calibration absorbs it. Use 3.904f for exact values. */
#define IMU_RAW_FULLSCALE         2047
/* Mounting. Which sensor axis (0=X, 1=Y, 2=Z) is the car's forward / lateral / up axis,
 * and the sign that makes each point the right way:
 *   forward axis: reads POSITIVE when the car pushes forward (accelerating)
 *   up axis     : reads POSITIVE (~ +1 g) when the car sits level
 * Use the bring-up firmware's 'p' command to check these (see README). */
#define IMU_AXIS_FWD   0
#define IMU_AXIS_LAT   1
#define IMU_AXIS_UP    2
/* Buddy 5's car: the IMU is mounted upside down and facing backwards, so
 *   fwd = -X, lat = +Y, up = -Z.
 * LAT is not yet verified on the car (lift the right side: lat should go negative).
 * The #ifndef lets the PC tests override these with -D (their fake sensor uses a +X/+Y/+Z mounting). */
#ifndef IMU_SIGN_FWD
#define IMU_SIGN_FWD   (-1.0f)
#endif
#ifndef IMU_SIGN_LAT
#define IMU_SIGN_LAT   (1.0f)
#endif
#ifndef IMU_SIGN_UP
#define IMU_SIGN_UP    (-1.0f)
#endif
/* Flip to -1 if a nose-UP tilt reports a NEGATIVE pitch. */
#define IMU_PITCH_SIGN 1.0f

/* ---- calibration (car must be stopped) ---- */
#define CAL_SAMPLES               50
#define CAL_NOISE_MAX_MG          60.0f
#define CAL_MAX_RETRIES           3

/* ---- fault detection ---- */
#define NACK_POLLS_TO_FAULT       2        /* 2 x 10 ms                               */
#define STALE_POLLS_TO_FAULT      3        /* 3 x 10 ms = 30 ms (NFR15 proposal)      */
#define SAT_POLLS_TO_FLAG         2
#define GOOD_READS_TO_RECOVER     3
#define LATE_POLL_US              3000
#define LATE_POLLS_TO_FAULT       10
/* Team decision pending: should an IMU sensor fault stop the car? 1 = yes. */
#define IMU_FAULT_TRIGGERS_ESTOP  1

/* ---- impact voting (2 of 3) ---- */
#define IMPACT_ACCEL_DEV_MG       2500.0f  /* |a| deviation from 1 g                  */
#define IMPACT_ACCEL_DEV_HUMP_MULT 2.0f    /* raise threshold while crossing a hump   */
#define IMPACT_DECEL_MG           600.0f   /* sustained forward deceleration          */
#define IMPACT_DECEL_POLLS        2
#define IMPACT_WHEEL_MIN_SPEED    100      /* mm/s before a wheel drop can count      */
#define IMPACT_WHEEL_DROP_PCT     40       /* % speed lost within one poll            */
#define IMPACT_WINDOW_POLLS       3        /* votes must fall within this window      */
#define IMPACT_COOLDOWN_POLLS     50

/* ---- hump (pitch x encoder distance) ---- */
#define HUMP_PITCH_ENTER_RAD      0.07f    /* about 4 degrees                         */
#define HUMP_PITCH_EXIT_RAD       0.03f
#define HUMP_END_POLLS            3
#define HUMP_MIN_SPEED            80       /* mm/s                                    */
#define HUMP_ABORT_STALL_POLLS    20       /* discard hump if the car stalls on it    */
#define HUMP_MIN_HEIGHT_MM        3.0f
#define HUMP_GAIN                 1.0f     /* fit from calibration passes (A7)        */
#define PITCH_LPF_ALPHA           0.3f

/* ---- Motion Control snapshot health ---- */
#define SNAP_STALE_POLLS          5        /* sequence unchanged this long = Motion Control not updating */
#define SNAP_DS_SAFETY_MULT       4        /* a distance jump bigger than this x expected is a discontinuity */

/* ---- debug read-out (bring-up build only; keep 0 in the kernel build) ---- */
#ifndef IMU_ENABLE_DEBUG
#define IMU_ENABLE_DEBUG          0
#endif

/* ---- motion classification ---- */
#define MOTION_STOP_SPEED         20       /* mm/s                                    */
#define MOTION_STOP_POLLS         5
#define MOTION_TURN_LAT_MG        250.0f
#define MOTION_ACCEL_FWD_MG       150.0f
#define MOTION_ACCEL_DSPEED       8        /* mm/s gained per poll                    */
#define MOTION_DEBOUNCE_POLLS     3

#endif /* IMU_CFG_H */
