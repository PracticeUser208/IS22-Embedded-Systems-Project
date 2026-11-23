/*
 *  hump_check.c
 *  PC check for hump_step() (imu_logic.c section of subsystem4.c), 19 Nov.
 *  Drives a simulated car at the patrol speed (25 mm/s, one poll every 10 ms)
 *  over trapezoid humps of known height and prints what the IMU reports.
 *  The car's pitch is the height difference between its front and rear
 *  contact points over the wheelbase, smoothed like the firmware
 *  (PITCH_LPF_ALPHA), with +-0.25 deg of sensor noise (about 1 LSB at 4 mg).
 *  Run:  python3 testing/pc_checks.py hump
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define OLD_ENTER_RAD 0.07f     /* threshold before 19 Nov */

static float ground(float x, float h, float ramp, float top)
{
    if ((x <= 0.0f) || (x >= ((2.0f * ramp) + top))) {
        return 0.0f;
    }
    if (x < ramp) {
        return h * (x / ramp);
    }
    if (x < (ramp + top)) {
        return h;
    }
    return h * (((2.0f * ramp) + top - x) / ramp);
}

static float noise(void)
{
    return ((float)rand() / (float)RAND_MAX - 0.5f) * 2.0f * (0.25f * 3.14159265f / 180.0f);
}

/* one pass; returns the height reported (mm, 0 = none) and the peak pitch */
static float pass(float h, float wheelbase, float ramp, float top, float *p_peak_deg, int *p_count)
{
    hump_t   hs;
    float    pitch_f = 0.0f;
    float    x = -wheelbase - 50.0f;
    float    peak = 0.0f;
    float    last = 0.0f;
    uint16_t dist = 0u;
    uint16_t prev = 0u;
    int      n;

    hump_init(&hs);
    *p_count = 0;
    for (n = 0; n < 40000; n++) {
        float   rear  = ground(x, h, ramp, top);
        float   front = ground(x + wheelbase, h, ramp, top);
        float   pitch = atanf((front - rear) / wheelbase) + noise();
        uint8_t h2 = 0u;

        pitch_f += PITCH_LPF_ALPHA * (pitch - pitch_f);
        if (pitch_f > peak) {
            peak = pitch_f;
        }
        x += 0.25f;                                  /* 25 mm/s for 10 ms */
        dist = (uint16_t)(x + 1000.0f);
        if (hump_step(&hs, pitch_f, noise(), (uint16_t)(dist - prev), true, HUMP_GAIN, &h2)) {
            last = (float)h2 / 2.0f;
            (*p_count)++;
        }
        prev = dist;
        if (x > ((2.0f * ramp) + top + HUMP_CALM_MM + 150.0f)) {
            break;
        }
    }
    *p_peak_deg = peak * 180.0f / 3.14159265f;
    return last;
}

int main(void)
{
    static const float H[] = {3.0f, 5.0f, 10.0f, 20.0f};
    static const float W[] = {100.0f, 140.0f};
    unsigned i, j;
    int      fail = 0;
    int      false_humps = 0;
    hump_t   hs;
    float    pitch_f = 0.0f;
    uint16_t dist = 0u, prev = 0u;
    long     k;

    srand(1u);
    printf("hump enter %.3f rad (%.1f deg), exit %.3f rad, min %.1f mm, gain %.2f\n",
           (double)HUMP_PITCH_ENTER_RAD, (double)(HUMP_PITCH_ENTER_RAD * 57.2958f),
           (double)HUMP_PITCH_EXIT_RAD, (double)HUMP_MIN_HEIGHT_MM, (double)HUMP_GAIN);
    printf("hump (ramp 30 mm, top 80 mm)  wheelbase  peak pitch  before 19 Nov   now\n");
    for (i = 0u; i < (sizeof H / sizeof H[0]); i++) {
        for (j = 0u; j < (sizeof W / sizeof W[0]); j++) {
            float peak;
            int   cnt;
            float got = pass(H[i], W[j], 30.0f, 80.0f, &peak, &cnt);
            int   old = (peak * 3.14159265f / 180.0f) >= OLD_ENTER_RAD;

            printf("  %4.0f mm                      %3.0f mm    %4.1f deg    %-13s  %s",
                   (double)H[i], (double)W[j], (double)peak,
                   old ? "entered" : "not detected",
                   (cnt > 0) ? "" : "NOT DETECTED\n");
            if (cnt > 0) {
                printf("%.1f mm (%d report%s)\n", (double)got, cnt, (cnt == 1) ? "" : "s");
            }
            if ((H[i] >= 10.0f) && (cnt == 0)) {
                fail = 1;
            }
        }
    }

    /* 20 Nov: hump under the right wheel only: the car tips sideways by the height over the
       track, and nose up by half that over the wheelbase */
    printf("hump under one wheel (ramp 30 mm, top 80 mm), track %.0f mm:\n", (double)HUMP_TRACK_MM);
    for (i = 0u; i < (sizeof H / sizeof H[0]); i++) {
        hump_t   h1;
        float    pf = 0.0f, rf = 0.0f, x = -200.0f, got = 0.0f, rpk = 0.0f;
        uint16_t d1 = 0u, p1 = 0u;
        int      cnt = 0;

        hump_init(&h1);
        while (x < 600.0f) {
            float   lift  = ground(x, H[i], 30.0f, 80.0f);
            float   roll  = asinf(lift / HUMP_TRACK_MM) + noise();
            float   pitch = atanf((lift / 2.0f) / 120.0f) + noise();
            uint8_t h2 = 0u;

            pf += PITCH_LPF_ALPHA * (pitch - pf);
            rf += PITCH_LPF_ALPHA * (roll - rf);
            if (fabsf(rf) > rpk) {
                rpk = fabsf(rf);
            }
            x += 0.25f;
            d1 = (uint16_t)(x + 1000.0f);
            if (hump_step(&h1, pf, rf, (uint16_t)(d1 - p1), true, HUMP_GAIN, &h2)) {
                got = (float)h2 / 2.0f;
                cnt++;
            }
            p1 = d1;
        }
        printf("  %4.0f mm: roll peak %4.1f deg -> %s", (double)H[i], (double)(rpk * 57.2958f),
               (cnt > 0) ? "" : "NOT DETECTED\n");
        if (cnt > 0) {
            printf("%.1f mm (%d report%s)\n", (double)got, cnt, (cnt == 1) ? "" : "s");
        }
        if ((H[i] >= 10.0f) && (cnt == 0)) {
            fail = 1;
        }
    }

    /* 20 Nov: one 10 mm hump under the right wheel, then under the caster 120 mm further on
       (nose up): two tilts, one hump */
    {
        hump_t   h1;
        float    pf = 0.0f, rf = 0.0f, x = -200.0f, got = 0.0f;
        uint16_t d1 = 0u, p1 = 0u;
        int      cnt = 0;

        hump_init(&h1);
        while (x < 900.0f) {
            float   wheel  = ground(x, 10.0f, 30.0f, 80.0f);
            float   caster = ground(x - 120.0f, 10.0f, 30.0f, 80.0f);
            float   roll   = asinf(wheel / HUMP_TRACK_MM) + noise();
            float   pitch  = atanf(((wheel / 2.0f) + caster) / 120.0f) + noise();
            uint8_t h2 = 0u;

            pf += PITCH_LPF_ALPHA * (pitch - pf);
            rf += PITCH_LPF_ALPHA * (roll - rf);
            x += 0.25f;
            d1 = (uint16_t)(x + 1000.0f);
            if (hump_step(&h1, pf, rf, (uint16_t)(d1 - p1), true, HUMP_GAIN, &h2)) {
                got = (float)h2 / 2.0f;
                cnt++;
            }
            p1 = d1;
        }
        printf("10 mm hump under a wheel, then the caster: %d report%s (%.1f mm)\n",
               cnt, (cnt == 1) ? "" : "s", (double)got);
        if (cnt != 1) {
            fail = 1;
        }
    }

    /* 20 Nov: the course hump (photo 20 Nov): one low peak, 10 mm, long ramps (90 mm up, 90 mm
       down), under one wheel only; driving jolts of +-2.5 deg each poll (the dashboard showed the
       car tipping 2-3 deg on a flat floor with the 30 ms filter).  Also 10 min of flat floor
       with the same jolts. */
    {
        hump_t   h1;
        float    pf = 0.0f, rf = 0.0f, x = -200.0f, got = 0.0f;
        uint16_t d1 = 0u, p1 = 0u;
        int      cnt = 0;
        int      flat = 0;
        long     k;

        hump_init(&h1);
        while (x < 700.0f) {
            float   wheel = ground(x, 10.0f, 90.0f, 0.0f);
            float   roll  = asinf(wheel / HUMP_TRACK_MM) + (10.0f * noise());
            float   pitch = atanf((wheel / 2.0f) / 120.0f) + (10.0f * noise());
            uint8_t h2 = 0u;

            pf += PITCH_LPF_ALPHA * (pitch - pf);
            rf += PITCH_LPF_ALPHA * (roll - rf);
            x += 0.25f;
            d1 = (uint16_t)(x + 1000.0f);
            if (hump_step(&h1, pf, rf, (uint16_t)(d1 - p1), true, HUMP_GAIN, &h2)) {
                got = (float)h2 / 2.0f;
                cnt++;
            }
            p1 = d1;
        }
        hump_init(&h1);
        pf = 0.0f;
        rf = 0.0f;
        p1 = 0u;
        for (k = 0; k < 60000L; k++) {
            uint8_t h2 = 0u;

            pf += PITCH_LPF_ALPHA * ((10.0f * noise()) - pf);
            rf += PITCH_LPF_ALPHA * ((10.0f * noise()) - rf);
            d1 = (uint16_t)((float)k * 0.25f);
            if (hump_step(&h1, pf, rf, (uint16_t)(d1 - p1), true, HUMP_GAIN, &h2)) {
                flat++;
            }
            p1 = d1;
        }
        printf("course hump (10 mm, 90 mm ramps, one wheel, +-2.5 deg jolts): %d report%s (%.1f mm); "
               "flat floor 10 min with the jolts: %d false hump%s\n",
               cnt, (cnt == 1) ? "" : "s", (double)got, flat, (flat == 1) ? "" : "s");
        if ((cnt != 1) || (flat != 0)) {
            fail = 1;
        }
        if (cnt != 1) {
            fail = 1;
        }
    }

    /* 20 Nov (evening): the same course hump with the car leaning 1 deg nose down and 0.8 deg
       to the side while it drives (as on the car), stopping on the hump now and then (speed
       reads 0 for 0.3 s every 4 cm, as on the car) */
    {
        hump_t   h1;
        float    pf = -0.017f, rf = -0.014f, x = -900.0f, got = 0.0f;
        uint16_t d1 = 0u, p1 = 0u;
        int      cnt = 0;
        int      n = 0;

        hump_init(&h1);
        while (x < 700.0f) {
            float   wheel = ground(x, 10.0f, 90.0f, 0.0f);
            float   roll  = -0.014f - asinf(wheel / HUMP_TRACK_MM) + (10.0f * noise());
            float   pitch = -0.017f - atanf((wheel / 2.0f) / 120.0f) + (10.0f * noise());
            bool    moving = ((n % 190) >= 30);
            uint8_t h2 = 0u;

            pf += PITCH_LPF_ALPHA * (pitch - pf);
            rf += PITCH_LPF_ALPHA * (roll - rf);
            if (moving) {
                x += 0.25f;
            }
            n++;
            d1 = (uint16_t)(x + 2000.0f);
            if (hump_step(&h1, pf, rf, (uint16_t)(d1 - p1), moving, HUMP_GAIN, &h2)) {
                got = (float)h2 / 2.0f;
                cnt++;
            }
            p1 = d1;
        }
        printf("course hump, the car leaning 1 / 0.8 deg while driving, with stops on the hump: %d report%s (%.1f mm)\n",
               cnt, (cnt == 1) ? "" : "s", (double)got);
        if (cnt != 1) {
            fail = 1;
        }
    }

    /* 20 Nov (evening): starting from standstill, the car leans 1 deg nose down and 0.8 deg
       sideways as soon as it drives; the course hump 300 mm after the start */
    {
        hump_t   h1;
        float    pf = 0.0f, rf = 0.0f, x = -300.0f, got = 0.0f;
        uint16_t d1 = 0u, p1 = 0u;
        int      cnt = 0;
        int      n;

        hump_init(&h1);
        for (n = 0; n < 300; n++) {                  /* 3 s standing, level */
            uint8_t h2 = 0u;
            (void)hump_step(&h1, pf, rf, 0u, false, HUMP_GAIN, &h2);
        }
        p1 = (uint16_t)(x + 2000.0f);
        while (x < 700.0f) {
            float   wheel = ground(x, 10.0f, 90.0f, 0.0f);
            float   roll  = -0.014f - asinf(wheel / HUMP_TRACK_MM) + (10.0f * noise());
            float   pitch = -0.017f - atanf((wheel / 2.0f) / 120.0f) + (10.0f * noise());
            uint8_t h2 = 0u;

            pf += PITCH_LPF_ALPHA * (pitch - pf);
            rf += PITCH_LPF_ALPHA * (roll - rf);
            x += 0.25f;
            d1 = (uint16_t)(x + 2000.0f);
            if (hump_step(&h1, pf, rf, (uint16_t)(d1 - p1), true, HUMP_GAIN, &h2)) {
                got = (float)h2 / 2.0f;
                cnt++;
            }
            p1 = d1;
        }
        printf("course hump 30 cm after starting, the car leaning 1 / 0.8 deg once it drives: %d report%s (%.1f mm)\n",
               cnt, (cnt == 1) ? "" : "s", (double)got);
        if (cnt != 1) {
            fail = 1;
        }
    }

    /* flat floor: 10 minutes of driving with sensor noise only */
    hump_init(&hs);
    for (k = 0; k < 60000L; k++) {
        uint8_t h2 = 0u;
        float   x = (float)k * 0.25f;

        pitch_f += PITCH_LPF_ALPHA * (noise() - pitch_f);
        dist = (uint16_t)x;
        if (hump_step(&hs, pitch_f, noise(), (uint16_t)(dist - prev), true, HUMP_GAIN, &h2)) {
            false_humps++;
        }
        prev = dist;
    }
    printf("flat floor, 10 min with +-0.25 deg noise: %d false hump%s\n",
           false_humps, (false_humps == 1) ? "" : "s");
    if (false_humps != 0) {
        fail = 1;
    }
    printf("%s\n", fail ? "FAILED" : "ALL PASSED");
    return fail;
}
