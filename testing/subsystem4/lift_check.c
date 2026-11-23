/*
 * lift_check.c -- PC check of the lifted / overturned detection (imu_logic.c, lift_step) with made-up
 * sensor data: level driving with vibration, humps, lifts of several speeds, tipping over.
 * Not part of the firmware (LIFT_HOST_TEST).  Build and run on a PC:
 *   gcc -std=c11 -Wall -Wextra -DLIFT_HOST_TEST lift_check.c imu_logic.c -lm -o lift_check && ./lift_check
 * Vibration of 100 mg is shown for comparison only (LIFT_UP_MM_S would have to be raised for it).
 */
#ifdef LIFT_HOST_TEST
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "imu_logic.h"

static double gauss(void)
{
    double u = (rand() + 1.0) / (RAND_MAX + 2.0), v = (rand() + 1.0) / (RAND_MAX + 2.0);
    return sqrt(-2.0 * log(u)) * cos(6.283185307 * v);
}
/* gravity seen by a car pitched p and rolled r (deg), plus upward acceleration up_mg and noise */
static void sample(double p, double r, double up_mg, double noise, float a[3])
{
    double pr = p * 0.0174533, rr = r * 0.0174533;
    double g = 1000.0 + up_mg;
    a[0] = (float)(g * sin(pr) + noise * gauss());
    a[1] = (float)(g * cos(pr) * sin(rr) + noise * gauss());
    a[2] = (float)(g * cos(pr) * cos(rr) + noise * gauss());
}
static double minjerk_acc(double s, double H, double T)   /* mm/s^2 */
{
    return (H / (T * T)) * (60 * s - 180 * s * s + 120 * s * s * s);
}
int main(void)
{
    const float base[3] = { 20.0f, -15.0f, 999.0f };   /* a slightly tilted mounting */
    float g = sqrtf(base[0]*base[0] + base[1]*base[1] + base[2]*base[2]);
    int fails = 0;
    double noises[3] = { 20.0, 50.0, 100.0 };

    for (int n = 0; n < 3; n++) {
        double nz = noises[n];
        lift_t l; float a[3]; int fires = 0;
        /* 1: 10 minutes of level driving with vibration */
        srand(1); lift_init(&l);
        for (int i = 0; i < 60000; i++) {
            sample(1.15, -0.86, 0.0, nz, a);
            if (lift_step(&l, a, base, g) != 0u) fires++;
        }
        printf("noise %3.0f mg  drive 10 min level: %d false stops\n", nz, fires);
        if (nz <= 50.0 && fires) fails++;
        /* 2: humps: nose up 15 / 25 deg, 20 passes each */
        for (int deg = 15; deg <= 25; deg += 10) {
            fires = 0; lift_init(&l);
            for (int k = 0; k < 20; k++) {
                for (int i = 0; i < 1000; i++) {
                    double ph = i / 1000.0, p = (ph < 0.2) ? deg * ph / 0.2 : (ph < 0.4) ? deg : (ph < 0.6) ? deg - 2 * deg * (ph - 0.4) / 0.2 : (ph < 0.8) ? -deg + deg * (ph - 0.6) / 0.2 : 0;
                    sample(1.15 + p, -0.86, 0.0, nz, a);
                    if (lift_step(&l, a, base, g) != 0u) fires++;
                }
            }
            printf("               20 humps, nose %d deg up then down: %d false stops\n", deg, fires);
            if (nz <= 50.0 && fires) fails++;
        }
        /* 3: lifts, kept level: H mm in T s */
        static const double HT[6][2] = { {150,0.4}, {150,0.6}, {200,0.8}, {100,0.5}, {150,1.0}, {100,1.0} };
        for (int c = 0; c < 6; c++) {
            int got = 0, at = -1; uint8_t why = 0;
            for (int rep = 0; rep < 50; rep++) {
                lift_init(&l); int hit = -1;
                for (int i = 0; i < 300; i++) {             /* 1 s driving, then the lift */
                    double t = (i - 100) * 0.01, s = t / HT[c][1];
                    double acc = (t >= 0 && s <= 1.0) ? minjerk_acc(s, HT[c][0], HT[c][1]) : 0.0;
                    sample(1.15, -0.86, acc / 9.80665, nz, a);
                    uint8_t w = lift_step(&l, a, base, g);
                    if (w && hit < 0) { hit = i - 100; why = w; }
                }
                if (hit >= 0) { got++; at = hit; }
            }
            printf("               lift %3.0f mm in %.1f s, level: stopped %2d/50%s", HT[c][0], HT[c][1], got, got ? "" : "\n");
            if (got) printf("  (after %d ms, cause %u)\n", at * 10, why);
            if ((nz <= 50.0) && (c < 4) && (got < 49)) fails++;
        }
        /* 4: tipped: roll 30 deg in 0.5 s; pitch 50 deg; upside down */
        static const double PR[3][2] = { {0, 30}, {50, 0}, {0, 180} };
        for (int c = 0; c < 3; c++) {
            lift_init(&l); int hit = -1; uint8_t why = 0;
            for (int i = 0; i < 300; i++) {
                double f = (i < 100) ? 0 : (i < 150) ? (i - 100) / 50.0 : 1.0;
                sample(1.15 + f * PR[c][0], -0.86 + f * PR[c][1], 0.0, nz, a);
                uint8_t w = lift_step(&l, a, base, g);
                if (w && hit < 0) { hit = i - 100; why = w; }
            }
            printf("               tipped pitch %.0f roll %.0f deg: %s", PR[c][0], PR[c][1], hit >= 0 ? "stopped" : "NOT stopped\n");
            if (hit >= 0) printf(" after %d ms (cause %u)\n", hit * 10, why);
            if ((nz <= 50.0) && (hit < 0)) fails++;
        }
        /* 5: one event per lift, ready again after 1 s level */
        lift_init(&l); fires = 0; int second = 0;
        for (int i = 0; i < 1500; i++) {
            double r = (i >= 100 && i < 600) ? 40 : 0;          /* held tipped for 5 s */
            sample(1.15, -0.86 + r, 0.0, nz, a);
            if (lift_step(&l, a, base, g)) fires++;
        }
        for (int i = 0; i < 300; i++) { sample(1.15, 39.0, 0.0, nz, a); if (lift_step(&l, a, base, g)) second++; }
        printf("               held tipped 5 s: %d event(s); tipped again later: %d event(s)\n", fires, second);
        if ((nz <= 50.0) && (fires != 1 || second != 1)) fails++;
    }
    printf("%s\n", fails ? "LIFT CHECK FAILED" : "LIFT CHECK PASSED");
    return fails;
}
#else
typedef int lift_check_not_built;   /* firmware build: nothing here */
#endif /* LIFT_HOST_TEST */
