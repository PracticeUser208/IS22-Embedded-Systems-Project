/*
 *	subsystem3_test.c
 *	On-target tests for subsystem 3.  Built only with: make TEST=1
 *	Record unit results in testing/subsystem3/TEST_LOG.md and the
 *	INT- result in testing/integration/TEST_LOG.md.
 */

#include "subsystem3.h"
#include "../test_report.h"






#if APP_TEST

#if TEMPLATE_DEMO
LOCAL APP_STATS read_stats(void)
{
	APP_STATS	s;

	tk_loc_mtx(mtx_stats, TMO_FEVR);
	s = app_stats;
	tk_unl_mtx(mtx_stats);
	return s;
}
#endif

EXPORT void subsystem3_test(void)
{
#if TEMPLATE_DEMO
	APP_STATS	before, after;
#endif

	TEST_INIT();

	/* Buddy 3: pure-logic regressions (s3_check.c) */
	TEST_CHECK("S3-CORE", s3_check_run() == 0u,
		   "IR, bounded pulse capture and recovery regressions");

	/* Line following logic (line_check.c) */
	TEST_CHECK("S3-LF", lf_check_run() == 0u,
		   "line following: calibration, steering, line lost, crossings");

#if S3_BARCODES
	/* Code 39 barcodes (bc_check.c): the course's codes, sampled */
	TEST_CHECK("S3-BC", bc_check_run() == 0u,
		   "barcodes: Code 39 *A* *F* *Z* *W* *X* *Y* *K*, both directions, 10-60 mm/s");
#endif

#if !TEMPLATE_DEMO
	/* S3-IR (live): the three IR sensors read through the ADC */
	{
		UH	raw[IR_COUNT] = {0U, 0U, 0U};
		ER	er = ir_read(raw);

		TEST_CHECK("S3-IR", (er == E_OK) && (raw[0] <= 4095U) &&
			   (raw[1] <= 4095U) && (raw[2] <= 4095U),
			   "IR sensors read through the ADC (GP26-GP28)");
		tm_printf((UB*)"[TEST] S3-IR raw L=%d C=%d B=%d\n",
			  (INT)raw[0], (INT)raw[1], (INT)raw[2]);
	}
#endif

#if TEMPLATE_DEMO

	/* INT-01 (integration, IF-01): samples from subsystem 2 arrive.
	   At one sample per 300 ms, 2 s should bring at least 5. */
	before = read_stats();
	tk_dly_tsk(2000);
	after = read_stats();
	TEST_CHECK("INT-01", after.received >= before.received + 5,
		   "samples flow from subsystem 2 to subsystem 3");

	/* S3-U01: the statistics are self-consistent */
	TEST_CHECK("S3-U01",
		   after.received > 0 && after.min <= after.max &&
		   after.min >= 0 && after.max <= 100,
		   "min <= max, both within 0..100");
#endif /* TEMPLATE_DEMO */

	TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */

/* ======== from s3_check.c ======== */


#if defined(S3_HOST_TEST) || (defined(APP_TEST) && APP_TEST)
#include "subsystem3.h"
#include "subsystem3.h"
#include "subsystem3.h"
#include "subsystem3.h"
#include <stddef.h>

static uint32_t check_ir(void)
{
    uint32_t failures = 0u;
    s3_ir_cal_t cal = {0u, 0u, 0u};
    bool b_black = false;
    uint16_t value = 0u;

    failures += !s3_ir_calibrate(&cal, 3000u, 500u, 100u, 100u);
    failures += !s3_ir_classify(&cal, 500u, false, &b_black) || !b_black;
    failures += !s3_ir_classify(&cal, 1800u, true, &b_black) || !b_black;
    failures += !s3_ir_classify(&cal, 1900u, true, &b_black) || b_black;
    failures += !s3_ir_normalize(&cal, 500u, &value) || (value != 1000u);
    failures += !s3_ir_normalize(&cal, 4000u, &value) || (value != 0u);
    failures += s3_ir_calibrate(&cal, 1000u, 1000u, 1u, 0u);
    failures += (cal.white != 3000u); /* Failed calibration is transactional. */
    failures += s3_ir_calibrate(&cal, 100u, 110u, 10u, 5u);
    failures += !s3_ir_calibrate(&cal, 0u, UINT16_MAX, 1u, 0u);
    failures += !s3_ir_normalize(&cal, UINT16_MAX, &value) || (value != 1000u);
    failures += !s3_ir_classify(&cal, UINT16_MAX, false, &b_black) || !b_black;
    failures += s3_ir_classify(NULL, 0u, false, &b_black);
    failures += s3_ir_normalize(&cal, 0u, NULL);
    return (failures);
}

static uint32_t check_pulse(void)
{
    uint32_t failures = 0u;
    s3_pulse_t capture = {0};
    uint32_t minimum[3] = {9u, 19u, 9u};
    uint32_t maximum[3] = {11u, 21u, 11u};

    failures += !s3_pulse_begin(&capture, UINT32_MAX - 9u, true, 20u, 100u);
    failures += !s3_pulse_sample(&capture, 0u, false);
    failures += !s3_pulse_sample(&capture, 20u, true);
    failures += s3_pulse_matches(&capture, true, minimum, maximum, 3u);
    failures += !s3_pulse_finish(&capture, 30u);
    failures += !s3_pulse_matches(&capture, true, minimum, maximum, 3u);
    failures += s3_pulse_matches(&capture, false, minimum, maximum, 3u);
    failures += s3_pulse_matches(&capture, true, minimum, maximum, 2u);
    failures += s3_pulse_finish(&capture, 31u);
    minimum[1] = 22u;
    failures += s3_pulse_matches(&capture, true, minimum, maximum, 3u);
    failures += !s3_pulse_begin(&capture, 100u, false, 10u, 50u);
    failures += s3_pulse_sample(&capture, 111u, true); /* Missing sample. */
    failures += s3_pulse_finish(&capture, 112u);
    failures += !s3_pulse_begin(&capture, 100u, true, 10u, 50u);
    failures += s3_pulse_sample(&capture, 99u, false); /* Backwards time. */
    failures += !s3_pulse_begin(&capture, 0u, true, 10u, 50u);
    failures += s3_pulse_sample(&capture, 0u, false); /* Zero-width run. */
    failures += !s3_pulse_begin(&capture, 0u, true, 100u, 20u);
    failures += s3_pulse_sample(&capture, 21u, true); /* Stuck run. */
    failures += !s3_pulse_begin(&capture, 0u, false, 10u, 100u);
    for (uint32_t index = 1u; index <= S3_PULSE_CAPACITY; index++)
    {
        failures += !s3_pulse_sample(&capture, index, ((index & 1u) != 0u));
    }
    failures += s3_pulse_finish(&capture, S3_PULSE_CAPACITY + 1u);
    failures += (capture.count != S3_PULSE_CAPACITY);
    failures += s3_pulse_begin(&capture, 0u, true, 0u, 100u);
    failures += s3_pulse_sample(NULL, 0u, false);
    return (failures);
}

static uint32_t check_state(void)
{
    uint32_t failures = 0u;
    s3_state_t state;
    uint32_t start = UINT32_MAX - 100u;

    s3_state_init(&state);
    failures += s3_state_can_track(&state);
    (void)s3_state_step(&state, S3_STATE_SUSPEND, 0u);
    (void)s3_state_step(&state, S3_STATE_RESUME_SEARCH, 0u);
    failures += (s3_state_get(&state) != S3_STATE_INIT);
    (void)s3_state_step(&state, S3_STATE_CALIBRATED, 0u);
    (void)s3_state_step(&state, S3_STATE_BARCODE_START, 1u);
    failures += (s3_state_get(&state) != S3_STATE_CAPTURE);
    failures += !s3_state_can_track(&state);
    (void)s3_state_step(&state, S3_STATE_BARCODE_END, 2u);
    failures += (s3_state_get(&state) != S3_STATE_DECODE);
    (void)s3_state_step(&state, S3_STATE_DECODE_DONE, 3u);
    failures += (s3_state_get(&state) != S3_STATE_FOLLOW);
    failures += (s3_state_step(&state, S3_STATE_LINE_LOST, start)
                  != S3_STATE_EVT_LOST);
    failures += (s3_state_step(&state, S3_STATE_LINE_LOST, start + 500u)
                  != S3_STATE_EVT_NONE); /* Must not restart timeout. */
    failures += (s3_state_step(&state, S3_STATE_POLL, start + S3_STATE_SEARCH_US - 1u)
                  != S3_STATE_EVT_NONE);
    failures += (s3_state_step(&state, S3_STATE_LINE_FOUND, start + S3_STATE_SEARCH_US)
                  != S3_STATE_EVT_TIMEOUT);
    failures += (s3_state_step(&state, S3_STATE_POLL, start + S3_STATE_SEARCH_US + 1u)
                  != S3_STATE_EVT_NONE);
    (void)s3_state_step(&state, S3_STATE_RESUME_SEARCH, 0u);
    failures += (s3_state_get(&state) != S3_STATE_FAULT);
    (void)s3_state_step(&state, S3_STATE_RESET, 0u);
    (void)s3_state_step(&state, S3_STATE_CALIBRATED, 0u);
    (void)s3_state_step(&state, S3_STATE_SUSPEND, 0u);
    failures += s3_state_can_track(&state);
    (void)s3_state_step(&state, S3_STATE_RESUME_SEARCH, 100u);
    failures += (s3_state_step(&state, S3_STATE_LINE_FOUND, 101u)
                  != S3_STATE_EVT_REACQUIRED);
    failures += (s3_state_step(&state, S3_STATE_SENSOR_FAULT, 102u)
                  != S3_STATE_EVT_FAULT);
    failures += (s3_state_step(&state, S3_STATE_SENSOR_FAULT, 103u)
                  != S3_STATE_EVT_NONE);
    return (failures);
}

uint32_t s3_check_run(void)
{
    uint32_t failures = check_ir() + check_pulse() + check_state();
    /* Synthetic configuration, not a claim about the real track. */
    s3_line_config_t config = {{0, -10, 10, 0}, 14u, 6u};
    s3_line_result_t result;
    for (uint32_t pattern = 0u; pattern < 4u; pattern++)
    {
        failures += !s3_line_evaluate(&config, (pattern & 1u) != 0u,
                                      (pattern & 2u) != 0u, &result);
        failures += (s3_line_is_present(&result) != (pattern != 0u));
        failures += (s3_line_has_steering(&result)
                      != ((pattern == 1u) || (pattern == 2u)));
        if (s3_line_has_steering(&result))
        {
            failures += (result.steering != config.steering[pattern]);
        }
    }
    config.steering_valid_mask = 1u;
    failures += s3_line_evaluate(&config, false, false, &result);
    failures += s3_line_has_steering(&result);
    return (failures);
}
#endif
/* End of file. */

/* ======== from line_check.c ======== */
/*
 *	line_check.c
 *	Subsystem 3 -- regression checks for the line following logic
 *	(line_follow.c).  No hardware: synthetic sensor values only, so they
 *	prove the logic, not the calibration on the real track.
 *
 *	On the car: subsystem3_test.c (make TEST=1), test S3-LF.
 *	On a PC, from app_program/subsystem3:
 *	  gcc -std=c11 -Wall -Wextra -Werror -Wconversion -Wshadow -pedantic \
 *	      -DLF_HOST_TEST s3_ir.c s3_line.c line_follow.c line_check.c \
 *	      -o /tmp/lf-tests && /tmp/lf-tests
 */




#include <stdbool.h>

/* Raw values: MH module A0 is high over black (LF_BLACK_IS_HIGH) */
#define RAW_WHITE	300u
#define RAW_BLACK	3000u
#define RAW_HALF	((RAW_WHITE + RAW_BLACK) / 2u)
#define RAW_OF(level)	((uint16_t)(RAW_WHITE + (((RAW_BLACK - RAW_WHITE) * (level)) / 1000u)))

static uint32_t	g_fail;
static uint32_t	g_now;

static void check(bool b_ok)
{
	if (!b_ok) {
		g_fail++;
	}
}

/* Feed the same reading n times, 10 ms apart; returns the last output */
static lf_out_t feed(lf_t *p, uint16_t left, uint16_t centre, uint16_t barcode,
		     uint32_t n)
{
	uint16_t raw[LF_SENSORS];
	lf_out_t out = {0, 0, false, false, 0u};
	uint32_t k;

	raw[LF_CH_PAIR_LEFT] = left;		/* by pin (line_cfg.h) */
	raw[LF_CH_PAIR_RIGHT] = centre;
	raw[LF_CH_SIDE] = barcode;
	for (k = 0u; k < n; k++) {
		g_now += LF_PERIOD_MS;
		lf_update(p, raw, g_now, &out);
	}
	return out;
}

/* Junctions: a pattern of white / black runs in mm ("w100 b18 ..."), the
   side sensor sampled every 0.5 mm; the middle pair on the line or not.
   How many junctions. */
static uint32_t jct_runs(const uint32_t *p_mm10, const bool *p_dark, uint32_t n, bool on_line)
{
	lf_jct_t j;
	uint32_t i;
	uint32_t k;
	uint32_t t = 0u;
	uint32_t hits = 0u;
	uint32_t us_per_half_mm = LF_JCT_US(1u) / 2u;

	lf_jct_reset(&j, 0u, false);
	for (i = 0u; i < n; i++) {
		for (k = 0u; k < (p_mm10[i] / 5u); k++) {	/* 0.5 mm steps */
			t += us_per_half_mm;
			if (lf_jct_step(&j, t, p_dark[i], on_line, LF_JCT_US(LF_JCT_RUN_MM),
					LF_JCT_US(LF_JCT_MAX_MM), LF_JCT_US(LF_JCT_GAP_MM))) {
				hits++;
			}
		}
	}
	return hits;
}

static uint32_t jct_count(const char *p_s, bool on_line)
{
	uint32_t mm10[32];
	bool	 dark[32];
	uint32_t n = 0u;

	while ((*p_s != '\0') && (n < 32u)) {
		uint32_t v = 0u;

		if ((*p_s == 'w') || (*p_s == 'b')) {
			dark[n] = (*p_s == 'b');
			p_s++;
			while ((*p_s >= '0') && (*p_s <= '9')) {
				v = (v * 10u) + (uint32_t)(*p_s - '0');
				p_s++;
			}
			mm10[n] = v * 10u;
			n++;
		} else {
			p_s++;
		}
	}
	return jct_runs(mm10, dark, n, on_line);
}

/* Code 39 patterns (bar, space, ... 9 elements, MSB first, 1 = wide):
   '*' and A-Z, 0-9 (the course's letters) */
static const uint16_t g_c39[] = {
	0x094u,
	0x109u, 0x049u, 0x148u, 0x019u, 0x118u, 0x058u, 0x00Du, 0x10Cu, 0x04Cu, 0x01Cu,
	0x103u, 0x043u, 0x142u, 0x013u, 0x112u, 0x052u, 0x007u, 0x106u, 0x046u, 0x016u,
	0x181u, 0x0C1u, 0x1C0u, 0x091u, 0x190u, 0x0D0u,
	0x034u, 0x121u, 0x061u, 0x160u, 0x031u, 0x130u, 0x070u, 0x025u, 0x124u, 0x064u
};

static uint32_t jct_barcodes(void)
{
	static const uint32_t narrow10[2] = { 30u, 35u };	/* 0.1 mm */
	uint32_t hits = 0u;
	uint32_t c;
	uint32_t q;
	uint32_t rev;
	uint32_t merge;

	for (q = 0u; q < 2u; q++) {
		for (c = 1u; c < (sizeof g_c39 / sizeof g_c39[0]); c++) {
			for (rev = 0u; rev < 2u; rev++) {
				for (merge = 0u; merge < 30u; merge++) {
					uint32_t mm10[64];
					bool	 dark[64];
					uint32_t e[29];
					uint32_t n = 0u;
					uint32_t i;
					uint32_t k;
					uint16_t pats[3] = { g_c39[0], g_c39[c], g_c39[0] };

					for (i = 0u; i < 3u; i++) {
						for (k = 0u; k < 9u; k++) {
							e[n++] = ((pats[i] >> (8u - k)) & 1u) ? 3u : 1u;
						}
						if (i < 2u) {
							e[n++] = 1u;
						}
					}
					/* white before, the elements, white after */
					mm10[0] = 1000u;
					dark[0] = false;
					for (i = 0u; i < 29u; i++) {
						uint32_t idx = (rev != 0u) ? (28u - i) : i;

						mm10[1u + i] = e[idx] * narrow10[q];
						dark[1u + i] = ((idx % 2u) == 0u);
					}
					mm10[30] = 1000u;
					dark[30] = false;
					n = 31u;
					/* merge: one narrow space (element 1..27, odd)
					   seen as black: two bars run together */
					if ((merge > 0u) && (merge < 29u) && ((merge % 2u) == 1u) &&
					    (e[(rev != 0u) ? (28u - merge) : merge] == 1u)) {
						dark[1u + merge] = true;
					}
					hits += jct_runs(mm10, dark, n, true);
				}
			}
		}
	}
	return hits;
}

/* LF-01..LF-17 test the edge mode (LF_MODE_EDGE); LF-18.. the centre mode */
static void lf_init_edge(lf_t *p)
{
	lf_init(p);
	p->mode = (uint8_t)LF_MODE_EDGE;
}

/* Every sensor sees white, then black, then white: calibrated */
static void calibrate_all(lf_t *p)
{
	(void)feed(p, RAW_WHITE, RAW_WHITE, RAW_WHITE, 12u);
	(void)feed(p, RAW_BLACK, RAW_BLACK, RAW_BLACK, 12u);
	(void)feed(p, RAW_WHITE, RAW_WHITE, RAW_WHITE, 12u);
}

uint32_t lf_check_run(void)
{
	lf_t		lf;
	lf_out_t	o;

	g_fail = 0u;
	g_now = 1000u;

	/* LF-01: one colour only -> not calibrated, no steering, lost */
	lf_init_edge(&lf);
	o = feed(&lf, RAW_BLACK, RAW_BLACK, RAW_BLACK, 20u);
	check(!o.calibrated && (o.steer_mm_s == 0) && !o.present && lf_lost(&lf, g_now));

	/* LF-02: too little contrast (span < LF_MIN_SPAN) -> not calibrated */
	lf_init_edge(&lf);
	(void)feed(&lf, 1000u, 1000u, 1000u, 12u);
	o = feed(&lf, 1300u, 1300u, 1300u, 12u);
	check(!o.calibrated);

	/* LF-03: white then black -> calibrated, ir bits follow the colour */
	lf_init_edge(&lf);
	calibrate_all(&lf);
	o = feed(&lf, RAW_WHITE, RAW_WHITE, RAW_WHITE, 6u);
	check(o.calibrated && (o.ir_bits == 0u));
	o = feed(&lf, RAW_BLACK, RAW_WHITE, RAW_BLACK, 6u);
	check(o.ir_bits == (LF_IR_LEFT | LF_IR_BARCODE));

	/* LF-04: centre half on the edge, left white -> (almost) straight */
	lf_init_edge(&lf);
	calibrate_all(&lf);
	o = feed(&lf, RAW_WHITE, RAW_OF(600u), RAW_WHITE, 6u);	/* onto the edge */
	o = feed(&lf, RAW_WHITE, (uint16_t)RAW_HALF, RAW_WHITE, 6u);
	check(o.present && (o.steer_mm_s >= -10) && (o.steer_mm_s <= 10));

	/* LF-05: centre darker (drifted right) -> steer left, proportional
	   (gentler than the hard turn; past the dead band) */
	o = feed(&lf, RAW_WHITE, RAW_OF(800u), RAW_WHITE, 6u);
	check(o.present && (o.steer_mm_s > 0) && (o.steer_mm_s < LF_STEER_HARD));

	/* LF-06: centre lighter (drifted left) -> steer right */
	o = feed(&lf, RAW_WHITE, RAW_OF(250u), RAW_WHITE, 6u);
	check(o.present && (o.steer_mm_s < 0) && (o.search_dir < 0));

	/* LF-07: left sensor on the line -> hard left, search to the left.
	   The line passed between the sensors first: centre white a while
	   (left dark right after the centre leaves the edge = a crossing) */
	(void)feed(&lf, RAW_WHITE, RAW_WHITE, RAW_WHITE, (LF_CROSS_WINDOW_MS / LF_PERIOD_MS) + 2u);
	o = feed(&lf, RAW_BLACK, RAW_WHITE, RAW_WHITE, 6u);
	check(o.present && (o.steer_mm_s == LF_STEER_HARD) && (o.search_dir > 0));

	/* LF-08: ... and when it goes off to the left, keep turning left */
	o = feed(&lf, RAW_WHITE, RAW_WHITE, RAW_WHITE, 3u);
	check(o.steer_mm_s == LF_STEER_HARD);

	/* LF-09: line lost only after LF_LOST_MS without it */
	lf_init_edge(&lf);
	calibrate_all(&lf);
	o = feed(&lf, RAW_WHITE, RAW_BLACK, RAW_WHITE, 6u);	/* on the line */
	check(o.present && !lf_lost(&lf, g_now));
	o = feed(&lf, RAW_WHITE, RAW_WHITE, RAW_WHITE, (LF_LOST_MS / LF_PERIOD_MS) - 2u);
	check(!o.present && !lf_lost(&lf, g_now));
	o = feed(&lf, RAW_WHITE, RAW_WHITE, RAW_WHITE, 6u);
	check(lf_lost(&lf, g_now));

	/* LF-10: steering never exceeds the hard limit */
	o = feed(&lf, RAW_BLACK, RAW_BLACK, RAW_BLACK, 6u);
	check((o.steer_mm_s <= LF_STEER_HARD) && (o.steer_mm_s >= -LF_STEER_HARD));

	/* LF-11: after both sensors saw black (e.g. while calibrating), the
	   centre back on the edge must trim (or, inside the dead band, go
	   straight), not keep turning hard left (that would spin the car
	   on the spot) */
	o = feed(&lf, RAW_WHITE, RAW_OF(600u), RAW_WHITE, 6u);
	check(o.present && (o.steer_mm_s >= 0) && (o.steer_mm_s < (LF_STEER_HARD / 2)) && (o.search_dir < 0));

	/* LF-12: a crossing (both sensors dark at once while the centre rides
	   the edge) -> straight on, no hard left; after it the car has not
	   taken it as "line on the left".  (The first sample or two, while
	   the smoothed readings darken, steer normally.) */
	lf_init_edge(&lf);
	calibrate_all(&lf);
	o = feed(&lf, RAW_WHITE, RAW_OF(600u), RAW_WHITE, 6u);	/* onto the edge */
	o = feed(&lf, RAW_WHITE, (uint16_t)RAW_HALF, RAW_WHITE, 10u);
	{
		bool ok = true;
		uint32_t k;

		for (k = 0u; k < 30u; k++) {			/* 300 ms over the tape */
			o = feed(&lf, RAW_BLACK, RAW_BLACK, RAW_WHITE, 1u);
			ok = ok && o.present && (o.steer_mm_s <= (LF_STEER_HARD / 2)) &&
			     ((k < 2u) || (o.steer_mm_s == 0));
		}
		check(ok);
	}
	o = feed(&lf, RAW_WHITE, (uint16_t)RAW_HALF, RAW_WHITE, 10u);	/* past it */
	check(o.present && (o.steer_mm_s >= -10) && (o.steer_mm_s <= 10));
	o = feed(&lf, RAW_WHITE, RAW_WHITE, RAW_WHITE, 3u);	/* line drifts right */
	check(o.steer_mm_s < 0);

	/* LF-13: barcode bars under the left sensor, centre on the edge ->
	   never a hard left */
	lf_init_edge(&lf);
	calibrate_all(&lf);
	o = feed(&lf, RAW_WHITE, RAW_OF(600u), RAW_WHITE, 6u);
	o = feed(&lf, RAW_WHITE, (uint16_t)RAW_HALF, RAW_WHITE, 10u);
	{
		bool ok = true;
		uint32_t k;

		for (k = 0u; k < 12u; k++) {			/* bars 30-60 ms, spaces 30 ms */
			o = feed(&lf, RAW_BLACK, (uint16_t)RAW_HALF, RAW_WHITE, ((k % 3u) == 0u) ? 6u : 3u);
			ok = ok && (o.steer_mm_s < (LF_STEER_HARD / 2));
			o = feed(&lf, RAW_WHITE, (uint16_t)RAW_HALF, RAW_WHITE, 3u);
			ok = ok && (o.steer_mm_s < (LF_STEER_HARD / 2));
		}
		check(ok);
	}

	/* LF-14: drifting right, realistically: centre fully black, back
	   through the edge, white, THEN the left sensor finds the line ->
	   hard left (not a crossing) */
	lf_init_edge(&lf);
	calibrate_all(&lf);
	o = feed(&lf, RAW_WHITE, RAW_OF(600u), RAW_WHITE, 6u);
	o = feed(&lf, RAW_WHITE, (uint16_t)RAW_HALF, RAW_WHITE, 10u);
	o = feed(&lf, RAW_WHITE, RAW_BLACK, RAW_WHITE, 30u);
	o = feed(&lf, RAW_WHITE, RAW_OF(300u), RAW_WHITE, 2u);
	o = feed(&lf, RAW_WHITE, RAW_WHITE, RAW_WHITE, 20u);
	o = feed(&lf, RAW_BLACK, RAW_WHITE, RAW_WHITE, 3u);
	check(o.present && (o.steer_mm_s == LF_STEER_HARD) && (o.search_dir > 0));

	/* LF-15: left sensor stays dark past LF_CROSS_MAX_MS next to the
	   edge: it is a real line after all -> hard left */
	lf_init_edge(&lf);
	calibrate_all(&lf);
	o = feed(&lf, RAW_WHITE, RAW_OF(600u), RAW_WHITE, 6u);
	o = feed(&lf, RAW_WHITE, (uint16_t)RAW_HALF, RAW_WHITE, 10u);
	o = feed(&lf, RAW_BLACK, (uint16_t)RAW_HALF, RAW_WHITE,
		 (LF_CROSS_MAX_MS / LF_PERIOD_MS) + 3u);
	check(o.steer_mm_s == LF_STEER_HARD);

	/* LF-16: steering never stops a wheel: on the edge -> straight;
	   off the edge either way -> proportional, never beyond
	   LF_STEER_HARD (below the patrol speed); no timed hard turns */
	lf_init_edge(&lf);
	calibrate_all(&lf);
	o = feed(&lf, RAW_WHITE, RAW_OF(600u), RAW_WHITE, 6u);
	o = feed(&lf, RAW_WHITE, (uint16_t)RAW_HALF, RAW_WHITE, 10u);
	check(o.present && (o.steer_mm_s == 0));
	o = feed(&lf, RAW_WHITE, RAW_WHITE, RAW_WHITE, 100u);		/* 1 s off, white side */
	check((o.steer_mm_s < 0) && (o.steer_mm_s >= -LF_STEER_HARD) &&
	      (o.steer_mm_s == -(((500 - (int32_t)LF_EDGE_DEADBAND) * LF_KP_PER_1000) / 1000)));
	o = feed(&lf, RAW_WHITE, (uint16_t)RAW_HALF, RAW_WHITE, 10u);
	o = feed(&lf, RAW_WHITE, RAW_BLACK, RAW_WHITE, 100u);		/* 1 s fully on the line */
	check(o.present && (o.steer_mm_s > 0) &&
	      (o.steer_mm_s == (((500 - (int32_t)LF_EDGE_DEADBAND) * LF_KP_PER_1000) / 1000)));

	/* LF-17: black/white decision at LF_BLACK_PCT: a sensor 45 % dark
	   (half on the line) counts as black, 20 % as white */
	lf_init_edge(&lf);
	calibrate_all(&lf);
	o = feed(&lf, RAW_OF(450u), RAW_OF(450u), RAW_WHITE, 6u);
	check((o.ir_bits & (LF_IR_LEFT | LF_IR_CENTRE)) == (LF_IR_LEFT | LF_IR_CENTRE));
	o = feed(&lf, RAW_OF(200u), RAW_OF(200u), RAW_WHITE, 6u);
	check((o.ir_bits & (LF_IR_LEFT | LF_IR_CENTRE)) == 0u);

	/* LF-18 (centre mode): the line between the sensors, both on it ->
	   straight */
	lf_init(&lf);
	lf.mode = (uint8_t)LF_MODE_CENTRE;
	calibrate_all(&lf);
	o = feed(&lf, RAW_BLACK, RAW_BLACK, RAW_WHITE, 10u);
	check(o.present && (o.steer_mm_s == 0));

	/* LF-19: the line moving left (the right sensor sliding off it) ->
	   steer left, more the further it goes, never beyond LF_STEER_HARD */
	o = feed(&lf, RAW_BLACK, RAW_OF(600u), RAW_WHITE, 10u);
	{
		int16_t a = o.steer_mm_s;

		o = feed(&lf, RAW_BLACK, RAW_OF(100u), RAW_WHITE, 10u);
		check((a > 0) && (o.steer_mm_s > a) && (o.steer_mm_s <= LF_STEER_HARD) &&
		      o.present && (o.search_dir > 0));
	}

	/* LF-20: ... both white: the line between them -> straight on; no
	   sensor touching it for LF_STRADDLE_MS -> gone off to the left:
	   turn left, search left, lost after LF_LOST_MS more */
	if (LF_STRADDLE_MS > 0u) {		/* straight on first */
		o = feed(&lf, RAW_WHITE, RAW_WHITE, RAW_WHITE, 5u);
		check(o.present && (o.steer_mm_s == 0));
	}
	o = feed(&lf, RAW_WHITE, RAW_WHITE, RAW_WHITE, (LF_STRADDLE_MS / LF_PERIOD_MS) + 5u);
	check(!o.present && (o.steer_mm_s == LF_STEER_HARD) && (o.search_dir > 0));
	o = feed(&lf, RAW_WHITE, RAW_WHITE, RAW_WHITE, LF_LOST_MS / LF_PERIOD_MS);
	check(lf_lost(&lf, g_now));

	/* LF-21: the mirror image: line moving right -> steer right; gone
	   off to the right -> turn right, search right */
	o = feed(&lf, RAW_BLACK, RAW_BLACK, RAW_WHITE, 10u);
	o = feed(&lf, RAW_OF(300u), RAW_BLACK, RAW_WHITE, 10u);
	check(o.present && (o.steer_mm_s < 0) && (o.search_dir < 0));
	o = feed(&lf, RAW_WHITE, RAW_WHITE, RAW_WHITE, (LF_STRADDLE_MS / LF_PERIOD_MS) + 7u);
	check(!o.present && (o.steer_mm_s == -LF_STEER_HARD) && (o.search_dir < 0));

	/* LF-22: a small difference (inside LF_C_DEADBAND) -> straight */
	o = feed(&lf, RAW_OF(950u), RAW_OF(900u), RAW_WHITE, 10u);
	check(o.present && (o.steer_mm_s == 0));

	/* LF-23: the side sensor (barcodes) never steers: bars under it
	   while the line is under both middle sensors -> straight */
	lf_init(&lf);
	lf.mode = (uint8_t)LF_MODE_CENTRE;
	calibrate_all(&lf);
	o = feed(&lf, RAW_BLACK, RAW_BLACK, RAW_WHITE, 10u);
	{
		bool ok = true;
		uint32_t k;

		for (k = 0u; k < 10u; k++) {
			o = feed(&lf, RAW_BLACK, RAW_BLACK, RAW_BLACK, 3u);
			ok = ok && (o.steer_mm_s == 0) && ((o.ir_bits & LF_IR_BARCODE) != 0u);
			o = feed(&lf, RAW_BLACK, RAW_BLACK, RAW_WHITE, 3u);
			ok = ok && (o.steer_mm_s == 0);
		}
		check(ok && o.present);
	}

	/* LF-24: telemetry bits by pin: side, centre, right */
	check(lf_ir_by_pin(LF_IR_LEFT) == (uint8_t)(1u << LF_CH_PAIR_LEFT));
	check(lf_ir_by_pin(LF_IR_CENTRE) == (uint8_t)(1u << LF_CH_PAIR_RIGHT));
	check(lf_ir_by_pin(LF_IR_BARCODE) == (uint8_t)(1u << LF_CH_SIDE));

	/* LF-25 (15 Nov): junctions.  The crossing tape (18 mm) under the
	   side sensor with white before and after -> one junction; only with
	   the middle pair on the line; a second crossing later -> one more */
	check(jct_count("w100 b18 w300 b18 w100", true) == 2u);
	check(jct_count("w100 b18 w100", false) == 0u);
	check(jct_count("w100 b13 w100", true) == 0u);		/* too short */
	check(jct_count("w100 b60 w100", true) == 0u);		/* along a line */
	check(jct_count("w10 b18 w100", true) == 0u);		/* no white before */
	check(jct_count("w100 b18 w10 b5 w100", true) == 0u);	/* none after */
	check(jct_count("w100 b22 w16", true) == 1u);		/* a wider tape */

	/* LF-26: no barcode is a junction: every Code 39 letter the course
	   could use, 3.0 and 3.5 mm narrow bars, both ways, also with two
	   bars run together across a narrow space (a blurry sensor) */
	check(jct_barcodes() == 0u);

	return g_fail;
}

#if defined(LF_HOST_TEST)
#include <stdio.h>

int main(void)
{
	uint32_t failures = lf_check_run();

	printf("line_follow regression failures: %lu\n", (unsigned long)failures);
	return (failures == 0u) ? 0 : 1;
}
#endif

/* ======== from bc_check.c ======== */
/*
 *	bc_check.c
 *	Subsystem 3 -- regression checks for the Code 39 decoder
 *	(s3_code39.c).  No hardware.
 *
 *	The test barcodes are the course's own: bar and space widths (pixels)
 *	measured on the images in BARCODE.pptx (A, F, Z, W, X, Y), and the
 *	printed "K" (2 Nov photo: narrow 3.5 mm, wide 10.5 mm, 164.5 mm long;
 *	widths below in 0.1 mm).  Then the
 *	car is simulated driving over a printed one: the sensor is sampled
 *	every 10 or 20 ms (the line task's period), so each width becomes a
 *	whole number of samples, at different start phases, speeds, a speed
 *	change across the barcode, and both directions -- first straight into
 *	the decoder, then sample by sample through bc_reader (start, quiet
 *	zone, end) as the line task does.  Also the letter table
 *	(barcode_map.h): no letter in two lines.
 *
 *	On the car: subsystem3_test.c (make TEST=1), test S3-BC.
 *	On a PC, from app_program/subsystem3 (also BC-09..14, the second look
 *	at a barcode, bc_rec.c, which needs more RAM than the car test has,
 *	and every letter through a simulated sensor):
 *	  gcc -std=c11 -Wall -Wextra -Werror -Wconversion -Wshadow -pedantic \
 *	      -DBC_HOST_TEST s3_code39.c s3_pulse.c bc_reader.c bc_rec.c \
 *	      bc_check.c -o /tmp/bc-tests && /tmp/bc-tests
 */






#include <stdbool.h>
#if defined(BC_HOST_TEST)
#include "subsystem3.h"
#endif

#define BC_N		29u	/* elements in a one-letter code */
#define BC_CODES	7u
#define BC_K		6u	/* index of the printed K */

/* Measured on the BARCODE.pptx slide images, first element = bar */
static const uint16_t bc_px[BC_CODES][BC_N] = {
	{ 41, 124, 41, 42, 124, 41, 124, 41, 41, 42, 123, 42, 41, 41, 41, 124, 42, 41,	/* A */
	  124, 41, 41, 124, 41, 42, 124, 41, 124, 41, 41 },
	{ 41, 124, 41, 42, 124, 41, 124, 41, 41, 42, 41, 41, 124, 41, 124, 124, 41, 42,	/* F */
	  41, 41, 41, 124, 41, 42, 124, 41, 124, 41, 41 },
	{ 41, 124, 41, 42, 124, 41, 124, 41, 41, 42, 41, 124, 123, 42, 124, 41, 41, 42,	/* Z */
	  41, 41, 41, 124, 41, 42, 124, 41, 124, 41, 41 },
	{ 39, 118, 39, 39, 117, 39, 118, 39, 39, 39, 118, 117, 118, 39, 39, 39, 39, 39,	/* W */
	  40, 39, 39, 117, 39, 40, 117, 39, 118, 39, 39 },
	{ 29, 87, 29, 29, 88, 29, 87, 29, 29, 29, 30, 87, 29, 29, 87, 30, 29, 29,	/* X */
	  87, 29, 29, 88, 29, 29, 87, 29, 88, 29, 29 },
	{ 21, 63, 22, 21, 63, 21, 64, 21, 21, 21, 64, 63, 21, 21, 64, 21, 21, 21,	/* Y */
	  21, 21, 21, 64, 21, 21, 64, 21, 63, 21, 21 },
	{ 35, 105, 35, 35, 105, 35, 105, 35, 35, 35,			/* *K* */
	  105, 35, 35, 35, 35, 35, 35, 105, 105, 35,
	  35, 105, 35, 35, 105, 35, 105, 35, 35 }
};
static const char bc_letter[BC_CODES] = { 'A', 'F', 'Z', 'W', 'X', 'Y', 'K' };

static uint32_t g_fail_bc;

static void check_bc(bool b_ok)
{
	if (!b_ok) {
		g_fail_bc++;
	}
}

static bool reads_as(const uint32_t *p_w, uint8_t n, char expect)
{
	char text[C39_MAX_TEXT + 1u];

	return (c39_decode(p_w, n, text, (uint8_t)sizeof text) == 1u) &&
	       (text[0] == expect) && (text[1] == '\0');
}

/* Drive over code c (printed len_mm long) at speed_mm_s, doubling to
   speed2 over the second half; sample every period_ms starting phase_ms
   in; reverse = cross it the other way.  Fills p_w with the sampled
   widths in microseconds; returns how many. */
static uint8_t bc_drive(uint32_t c, uint32_t len_mm, uint32_t speed_mm_s,
			uint32_t speed2_mm_s, uint32_t period_ms, uint32_t phase_ms,
			bool reverse, uint32_t *p_w)
{
	uint32_t	edge_um[BC_N + 1u];	/* element boundaries along the code */
	uint32_t	total_px = 0u;
	uint32_t	acc = 0u;
	uint32_t	i;
	uint32_t	t_ms;
	uint32_t	len_um = len_mm * 1000u;
	bool		started = false;
	bool		prev_dark = false;
	uint32_t	run = 0u;
	uint8_t		n = 0u;

	for (i = 0u; i < BC_N; i++) {
		total_px += bc_px[c][i];
	}
	edge_um[0] = 0u;
	for (i = 0u; i < BC_N; i++) {
		uint32_t k = reverse ? (BC_N - 1u - i) : i;

		acc += bc_px[c][k];
		edge_um[i + 1u] = (uint32_t)(((uint64_t)acc * len_um) / total_px);
	}

	/* from 5 mm before the code to 30 mm after it */
	for (t_ms = phase_ms; ; t_ms += period_ms) {
		uint32_t	half_ms = (len_um / 2u) / speed_mm_s;	/* um / (mm/s) = ms */
		uint32_t	x_um;
		bool		dark = false;
		int32_t		pos;

		if (t_ms <= half_ms) {
			x_um = t_ms * speed_mm_s;
		} else {
			x_um = (half_ms * speed_mm_s) + ((t_ms - half_ms) * speed2_mm_s);
		}
		pos = (int32_t)x_um - 5000;
		if (pos > (int32_t)(len_um + 30000u)) {
			break;
		}
		if ((pos >= 0) && (pos < (int32_t)len_um)) {
			for (i = 0u; i < BC_N; i++) {
				if (((uint32_t)pos >= edge_um[i]) && ((uint32_t)pos < edge_um[i + 1u])) {
					dark = ((i & 1u) == 0u);	/* even = bar */
					break;
				}
			}
		}
		if (!started) {
			if (dark) {
				started = true;
				prev_dark = true;
				run = 1u;
			}
			continue;
		}
		if (dark == prev_dark) {
			run++;
		} else {
			if (n < (C39_MAX_ELEMENTS + 2u)) {
				p_w[n] = run * period_ms * 1000u;
				n++;
			}
			run = 1u;
			prev_dark = dark;
		}
	}
	/* the run in progress is the white after the code: not a width */
	return n;
}

/* How much of a sensor spot spot_um wide, centred at pos, is on a bar:
   0..1000 (spot 0 = a point) */
static uint32_t bc_cover(const uint32_t *p_edge, int32_t pos, uint32_t spot_um)
{
	int32_t		a = pos - (int32_t)(spot_um / 2u);
	int32_t		b = a + (int32_t)spot_um;
	uint32_t	i;
	int64_t		black = 0;

	if (spot_um == 0u) {
		for (i = 0u; i < BC_N; i += 2u) {
			if ((pos >= (int32_t)p_edge[i]) && (pos < (int32_t)p_edge[i + 1u])) {
				return 1000u;
			}
		}
		return 0u;
	}
	for (i = 0u; i < BC_N; i += 2u) {		/* bars: even elements */
		int32_t lo = ((int32_t)p_edge[i] > a) ? (int32_t)p_edge[i] : a;
		int32_t hi = ((int32_t)p_edge[i + 1u] < b) ? (int32_t)p_edge[i + 1u] : b;

		if (hi > lo) {
			black += (int64_t)(hi - lo);
		}
	}
	return (uint32_t)((black * 1000) / (int64_t)spot_um);
}

/* The same drive, sample by sample through bc_reader, from 50 mm of
   white before the code to 100 mm after it, at speed_mm_s up to the
   middle of the code and speed2_mm_s after it.  The sensor sees a spot
   spot_um wide and calls it black above pct % (+-3 % hysteresis), as
   line_follow.c does for the side sensor.  Returns the letter read
   ('\0' = none), and how many times reading started. */
static char bc_read_x(uint32_t c, uint32_t len_um, uint32_t speed_mm_s,
		      uint32_t speed2_mm_s, uint32_t period_ms, uint32_t phase_ms,
		      bool reverse, uint32_t spot_um, uint32_t pct,
		      uint32_t *p_starts)
{
	uint32_t	edge_um[BC_N + 1u];
	uint32_t	total_px = 0u;
	uint32_t	acc = 0u;
	uint32_t	i;
	uint32_t	t_ms;
	bc_reader_t	rd;
	char		got = '\0';
	uint32_t	us0 = 0xFFFFF000u;	/* timer wraps during the drive */
	int32_t		pos;
	bool		dark = false;
	bool		known = false;

	for (i = 0u; i < BC_N; i++) {
		total_px += bc_px[c][i];
	}
	edge_um[0] = 0u;
	for (i = 0u; i < BC_N; i++) {
		uint32_t k = reverse ? (BC_N - 1u - i) : i;

		acc += bc_px[c][k];
		edge_um[i + 1u] = (uint32_t)(((uint64_t)acc * len_um) / total_px);
	}
	bc_reader_init(&rd);
	*p_starts = 0u;
	pos = (int32_t)(phase_ms * speed_mm_s) - 50000;
	for (t_ms = phase_ms; pos < (int32_t)(len_um + 100000u); t_ms += period_ms) {
		uint32_t	lvl = bc_cover(edge_um, pos, spot_um);
		uint8_t		r;

		if (!known) {
			dark = (lvl >= (pct * 10u));
			known = true;
		} else if (lvl > ((pct * 10u) + 30u)) {
			dark = true;
		} else if ((lvl + 30u) < (pct * 10u)) {
			dark = false;
		} else {
			/* inside the hysteresis band: keep */
		}
		r = bc_reader_sample(&rd, us0 + (t_ms * 1000u), dark, true);
		if (r == BC_RD_START) {
			(*p_starts)++;
		} else if (r == BC_RD_DONE) {
			char	text[C39_MAX_TEXT + 1u];
			uint8_t	count;

			if ((bc_reader_decode(&rd, text, (uint8_t)sizeof text, &count) == 1u) &&
			    (count == BC_N)) {
				got = text[0];
			}
		} else {
			/* reading, or nothing */
		}
		pos += (int32_t)(period_ms *
				 ((pos < (int32_t)(len_um / 2u)) ? speed_mm_s : speed2_mm_s));
	}
	return got;
}

static char bc_read(uint32_t c, uint32_t len_mm, uint32_t speed_mm_s,
		    uint32_t period_ms, uint32_t phase_ms, bool reverse,
		    uint32_t *p_starts)
{
	return bc_read_x(c, len_mm * 1000u, speed_mm_s, speed_mm_s, period_ms,
			 phase_ms, reverse, 0u, 50u, p_starts);
}

/* BC-08: one drive over the printed K; true = read as K, exactly once */
static bool bc_k_ok(uint32_t speed, uint32_t speed2, uint32_t phase,
		    bool reverse, uint32_t spot_um, uint32_t pct)
{
	uint32_t starts;

	return (bc_read_x(BC_K, 164500u, speed, speed2, 10u, phase, reverse,
			  spot_um, pct, &starts) == 'K') && (starts == 1u);
}

/* No letter in two lines of barcode_map.h */
static bool bc_map_ok(void)
{
	static const char *const lines[4] = {
		BARCODE_LEFT_LETTERS, BARCODE_RIGHT_LETTERS,
		BARCODE_STRAIGHT_LETTERS, BARCODE_UTURN_LETTERS
	};
	uint32_t a;
	uint32_t b;
	const char *p;

	for (a = 0u; a < 4u; a++) {
		for (p = lines[a]; *p != '\0'; p++) {
			for (b = a + 1u; b < 4u; b++) {
				if (bc_letter_in(lines[b], *p)) {
					return false;
				}
			}
		}
	}
	return true;
}

uint32_t bc_check_run(void)
{
	uint32_t	w[C39_MAX_ELEMENTS + 2u];
	uint32_t	c;
	uint32_t	i;
	uint32_t	phase;
	char		text[C39_MAX_TEXT + 1u];
	uint8_t		n;

	g_fail_bc = 0u;

	/* BC-01: the slides' widths read as their letters, both directions */
	for (c = 0u; c < BC_CODES; c++) {
		for (i = 0u; i < BC_N; i++) {
			w[i] = bc_px[c][i];
		}
		check_bc(reads_as(w, (uint8_t)BC_N, bc_letter[c]));
		for (i = 0u; i < BC_N; i++) {
			w[i] = bc_px[c][BC_N - 1u - i];
		}
		check_bc(reads_as(w, (uint8_t)BC_N, bc_letter[c]));
	}

	/* BC-02: a 130 mm barcode (the smaller one on the course mat) at
	   60 mm/s, sampled every 20 ms, every start phase, both ways */
	for (c = 0u; c < BC_CODES; c++) {
		for (phase = 0u; phase < 20u; phase++) {
			n = bc_drive(c, 130u, 60u, 60u, 20u, phase, false, w);
			check_bc(reads_as(w, n, bc_letter[c]));
			n = bc_drive(c, 130u, 60u, 60u, 20u, phase, true, w);
			check_bc(reads_as(w, n, bc_letter[c]));
		}
	}

	/* BC-03: 260 mm barcode, 10 ms samples, speeding up by 20 % half way */
	for (c = 0u; c < BC_CODES; c++) {
		for (phase = 0u; phase < 10u; phase++) {
			n = bc_drive(c, 260u, 60u, 72u, 10u, phase, false, w);
			check_bc(reads_as(w, n, bc_letter[c]));
		}
	}

	/* BC-04: slow (40 mm/s, obstacle-ahead speed) */
	n = bc_drive(0u, 130u, 40u, 40u, 20u, 7u, false, w);
	check_bc(reads_as(w, n, 'A'));

	/* BC-05: through bc_reader as on the car: white, the code, white.
	   Read exactly once, as the right letter, every phase, both ways
	   (the first wide space must not end it early) */
	for (c = 0u; c < BC_CODES; c++) {
		for (phase = 0u; phase < 20u; phase += 3u) {
			uint32_t starts;

			uint32_t len = ((phase & 1u) == 0u) ? 130u : 260u;
			uint32_t per = ((phase & 2u) == 0u) ? 20u : 10u;

			check_bc((bc_read(c, len, 60u, per, phase, false, &starts) == bc_letter[c]) &&
			      (starts == 1u));
			check_bc((bc_read(c, len, 60u, per, phase, true, &starts) == bc_letter[c]) &&
			      (starts == 1u));
		}
	}

	/* BC-08: the printed K (164.5 mm) at 10-40 mm/s, 10 ms samples, both
	   ways, a sharp sensor and a blurry spot at the side sensor's
	   threshold (LF_BC_BLACK_PCT), and the speed changing by 20 % half
	   way.  The live reading copes with a 5 mm spot at 35 %, but only
	   4 mm at 20 % (the narrow spaces stay above 20 %); the second look
	   (BC-09..12) reads blurrier ones. */
	{
		static const uint32_t speeds[] = { 10u, 20u, 25u, 40u };
		static const uint32_t spots[] = { 0u, (LF_BC_BLACK_PCT >= 30u) ? 5000u : 4000u };
		uint32_t s;
		uint32_t k;

		for (s = 0u; s < (sizeof speeds / sizeof speeds[0]); s++) {
			for (k = 0u; k < (sizeof spots / sizeof spots[0]); k++) {
				for (phase = 0u; phase < 10u; phase += 5u) {
					check_bc(bc_k_ok(speeds[s], speeds[s], phase, false,
						      spots[k], LF_BC_BLACK_PCT));
					check_bc(bc_k_ok(speeds[s], speeds[s], phase, true,
						      spots[k], LF_BC_BLACK_PCT));
				}
			}
		}
		check_bc(bc_k_ok(20u, 24u, 3u, false, (LF_BC_BLACK_PCT >= 30u) ? 4000u : 3000u,
			      LF_BC_BLACK_PCT));
		check_bc(bc_k_ok(20u, 16u, 7u, true, (LF_BC_BLACK_PCT >= 30u) ? 4000u : 3000u,
			      LF_BC_BLACK_PCT));
	}

	/* BC-06: the letter table: not case sensitive, no letter twice */
	check_bc(bc_letter_in("AF", 'F') && bc_letter_in("af", 'A') &&
	      !bc_letter_in("AF", 'Z') && !bc_letter_in("", 'A') &&
	      !bc_letter_in("A", '\0'));
	check_bc(bc_map_ok());

	/* BC-07: not barcodes -> nothing */
	w[0] = 300000u;				/* one bar: a crossing line */
	check_bc(c39_decode(w, 1u, text, (uint8_t)sizeof text) == 0u);
	for (i = 0u; i < BC_N; i++) {
		w[i] = 50000u;			/* all the same width */
	}
	check_bc(c39_decode(w, (uint8_t)BC_N, text, (uint8_t)sizeof text) == 0u);
	for (i = 0u; i < BC_N; i++) {
		w[i] = bc_px[0][i];
	}
	check_bc(c39_decode(w, (uint8_t)(BC_N - 1u), text, (uint8_t)sizeof text) == 0u);
	for (i = 0u; i < BC_N; i++) {		/* wide only 1.1 x narrow */
		w[i] = (bc_px[0][i] > 100u) ? 45u : 41u;
	}
	check_bc(c39_decode(w, (uint8_t)BC_N, text, (uint8_t)sizeof text) == 0u);
	for (i = 0u; i < BC_N; i++) {
		w[i] = bc_px[0][i];
	}
	w[9] = 400u;				/* gap far too wide */
	check_bc(c39_decode(w, (uint8_t)BC_N, text, (uint8_t)sizeof text) == 0u);
	w[9] = 42u;
	w[0] = 0u;				/* zero width */
	check_bc(c39_decode(w, (uint8_t)BC_N, text, (uint8_t)sizeof text) == 0u);

	return g_fail_bc;
}

#if defined(BC_HOST_TEST)
#include <stdio.h>

/* BC-09: the side sensor's first black is the barcode itself, seen through
   a 6 mm blurry spot.  Live, its black is learnt bar by bar (the darkest
   value so far), so the first narrow bars are judged against a grey
   "black".  The recording, classified again with the black it has at
   the end (bc_rec), must read K; the live reading is allowed to fail. */
static bc_rec_t	g_rec;
static bc_reader_t g_scratch;

static bool bc_rec_k_ok(uint32_t speed, uint32_t spot_um, bool reverse, bool overflow_it)
{
	uint32_t	edge_um[BC_N + 1u];
	uint32_t	acc = 0u;
	uint32_t	i;
	int32_t		pos = -60000;
	uint32_t	t_us = 0xFFF00000u;		/* wraps during the drive */
	uint16_t	white = 250u;
	uint16_t	black_seen = 250u;
	bool		dark = false;
	bc_reader_t	rd;
	char		text[C39_MAX_TEXT + 1u];
	uint8_t		count = 0u;
	uint8_t		n = 0u;
	bool		done = false;

	edge_um[0] = 0u;
	for (i = 0u; i < BC_N; i++) {
		acc += bc_px[BC_K][reverse ? (BC_N - 1u - i) : i];
		edge_um[i + 1u] = acc * 100u;			/* 0.1 mm -> um */
	}
	bc_reader_init(&rd);
	bc_rec_init(&g_rec);
	while ((pos < 300000) && !done) {
		uint16_t raw = (uint16_t)(white + ((bc_cover(edge_um, pos, spot_um) * 3350u) / 1000u));
		uint8_t r;

		if (raw > black_seen) {
			black_seen = raw;			/* learnt as it goes */
		}
		if (((uint32_t)black_seen - white) >= 400u) {	/* LF_MIN_SPAN */
			uint32_t thr = white + ((uint32_t)(black_seen - white) / 2u);
			uint32_t band = (uint32_t)(black_seen - white) / 32u;

			if (raw > (thr + band)) {
				dark = true;
			} else if ((raw + band) < thr) {
				dark = false;
			} else {
				/* keep */
			}
		}
		bc_rec_sample(&g_rec, t_us, raw);
		r = bc_reader_sample(&rd, t_us, dark, true);
		if (r == BC_RD_START) {
			bc_rec_start(&g_rec);
			if (overflow_it) {
				g_rec.overflow = true;
			}
		} else if ((r == BC_RD_DONE) || (r == BC_RD_ABORT)) {
			bc_rec_stop(&g_rec);
			if (r == BC_RD_DONE) {
				n = bc_reader_decode(&rd, text, (uint8_t)sizeof text, &count);
			}
			if (n == 0u) {
				uint8_t how = 0u;

				n = bc_rec_decode_auto(&g_rec, true, white, black_seen,
						       LF_BC_BLACK_PCT, true, LF_BC_MIN_OWN_SPAN,
						       LF_BC_HYST_DIV, &g_scratch, text,
						       (uint8_t)sizeof text, &count, &how);
			}
			done = (n > 0u) || (count >= 9u);
		} else {
			/* reading, or nothing */
		}
		pos += (int32_t)(speed * 10u);			/* 10 ms */
		t_us += 10000u;
	}
	return (n == 1u) && (text[0] == 'K') && (count == BC_N);
}

/* BC-10 (3 Nov car log): printed bars much lighter than the black tape
   the side sensor calibrated on.  Bars reach con_pct % of the tape's
   darkness, seen through a 4 mm spot, live black/white at
   LF_BC_BLACK_PCT % of the tape.  The recording window (bc_rec_watch,
   started at LF_BC_ACT_PCT %) and the second look with the barcode's
   own levels must read K. */
static bool bc_faint_k_ok(uint32_t speed, uint32_t con_pct, bool reverse)
{
	uint32_t	edge_um[BC_N + 1u];
	uint32_t	acc = 0u;
	uint32_t	i;
	int32_t		pos = -60000;
	uint32_t	t_us = 0xFFF00000u;
	const uint32_t	white = 150u;
	const uint32_t	tape = 1400u;
	bool		dark = false;
	bool		read = false;
	bool		handled = false;
	bc_reader_t	rd;
	char		text[C39_MAX_TEXT + 1u];
	uint8_t		count = 0u;
	uint8_t		how = 0u;

	edge_um[0] = 0u;
	for (i = 0u; i < BC_N; i++) {
		acc += bc_px[BC_K][reverse ? (BC_N - 1u - i) : i];
		edge_um[i + 1u] = acc * 100u;
	}
	bc_reader_init(&rd);
	bc_rec_init(&g_rec);
	while (pos < 330000) {
		uint32_t lvl = (bc_cover(edge_um, pos, 4000u) * con_pct) / 100u;	/* 0..1000 of the tape */
		uint16_t raw = (uint16_t)(white + ((lvl * (tape - white)) / 1000u));
		uint32_t thr = LF_BC_BLACK_PCT * 10u;
		uint8_t	 r;
		uint8_t	 ev;

		if (lvl > (thr + 30u)) {
			dark = true;
		} else if ((lvl + 30u) < thr) {
			dark = false;
		} else {
			/* keep */
		}
		ev = bc_rec_watch(&g_rec, t_us, raw, (lvl * 100u) > (LF_BC_ACT_PCT * 1000u), true,
				  LF_BC_REC_TAIL_MS * 1000u);
		if (ev == BC_REC_EV_STARTED) {
			handled = false;
		} else if ((ev == BC_REC_EV_ENDED) && !handled && !read) {
			read = (bc_rec_decode_auto(&g_rec, true, (uint16_t)white, (uint16_t)tape,
						   LF_BC_BLACK_PCT, true, LF_BC_MIN_OWN_SPAN,
						   LF_BC_HYST_DIV, &g_scratch, text,
						   (uint8_t)sizeof text, &count, &how) == 1u) &&
			       (text[0] == 'K');
		} else {
			/* recording, or nothing */
		}
		r = bc_reader_sample(&rd, t_us, dark, true);
		if ((r == BC_RD_DONE) && (bc_reader_decode(&rd, text, (uint8_t)sizeof text, &count) == 1u) &&
		    (text[0] == 'K')) {
			read = true;
			handled = true;
		}
		pos += (int32_t)(speed * 10u);
		t_us += 10000u;
	}
	return read;
}

/* ---- BC-11..13: every letter through a simulated side sensor ---- */

/* The drop-out level these tests use: the 4 Nov wiring fault (the car
   itself has the detector off since the fault was fixed, line_cfg.h) */
#define BC_TEST_DEAD_RAW	((uint16_t)450u)

/* Put samples straight into g_rec (20 ms apart) */
static void bc_rec_fill(const uint16_t *p_raw, uint32_t n)
{
	uint32_t i;

	bc_rec_init(&g_rec);
	for (i = 0u; (i < n) && (i < BC_REC_MAX); i++) {
		g_rec.raw[i] = p_raw[i];
		g_rec.ms[i] = (uint16_t)(i * 20u);
	}
	g_rec.n = (uint16_t)i;
}

/* The second look as the line task does it: drop-outs, then
   bc_rec_decode_auto with the side sensor's (bad) calibration */
static uint8_t bc_second_look(uint16_t cal_white, uint16_t cal_black, char *p_text,
			      uint8_t *p_how)
{
	uint8_t count = 0u;

	(void)bc_rec_find_dropouts(&g_rec, BC_TEST_DEAD_RAW, (uint16_t)LF_BC_DEAD_JUMP);
	return bc_rec_decode_auto(&g_rec, true, cal_white, cal_black, LF_BC_BLACK_PCT, true,
				  LF_BC_MIN_OWN_SPAN, LF_BC_HYST_DIV, &g_scratch, p_text,
				  (uint8_t)(C39_MAX_TEXT + 1u), &count, p_how);
}

/* e^-x for x >= 0, PC only (no libm) */
static double exp_neg(double x)
{
	double	r = 1.0;
	double	term = 1.0;
	int	k;
	int	halve = 0;

	while (x > 0.5) {
		x /= 2.0;
		halve++;
	}
	for (k = 1; k < 20; k++) {
		term *= -x / (double)k;
		r += term;
	}
	while (halve > 0) {
		r *= r;
		halve--;
	}
	return r;
}

/* Normal CDF (Abramowitz & Stegun 7.1.26), PC only */
static double bc_phi(double z)
{
	double x = (z < 0.0) ? -z : z;
	double t = 1.0 / (1.0 + (0.3275911 * x / 1.41421356));
	double y = 1.0 - ((((((1.061405429 * t) - 1.453152027) * t) + 1.421413741) * t -
			   0.284496736) * t + 0.254829592) * t * exp_neg(x * x / 2.0);

	return (z < 0.0) ? (0.5 * (1.0 - y)) : (0.5 * (1.0 + y));
}

#ifndef BC_SEED
#define BC_SEED		12345u
#endif
static uint32_t	g_seed = BC_SEED;

static uint32_t bc_rand(uint32_t m)
{
	g_seed = (g_seed * 1103515245u) + 12345u;
	return (m == 0u) ? 0u : ((g_seed >> 8) % m);
}

/* "*c*" (or c = 0: a plain black stripe of 18 mm, the line), edges in
   mm, read forward or reversed; returns the number of elements */
static uint32_t bc_sym_edges(char c, bool rev, double narrow, double ratio, double *p_e)
{
	double		w[BC_N];
	uint32_t	n = 0u;
	uint32_t	k;
	uint32_t	ch;
	double		acc = 0.0;

	if (c == '\0') {
		p_e[0] = 0.0;
		p_e[1] = 18.0;
		return 1u;
	}
	for (ch = 0u; ch < 3u; ch++) {
		uint16_t pat = C39_STAR_PATTERN;

		if (ch == 1u) {
			uint8_t idx;
			char	cc;

			for (idx = 0u; c39_table(idx, &cc, &pat) && (cc != c); idx++) {
			}
		}
		for (k = 0u; k < 9u; k++) {
			w[n++] = (((uint32_t)pat >> (8u - k)) & 1u) ? (narrow * ratio) : narrow;
		}
		if (ch < 2u) {
			w[n++] = narrow;
		}
	}
	for (k = 0u; k < n; k++) {
		p_e[k] = acc;
		acc += w[rev ? (n - 1u - k) : k];
	}
	p_e[n] = acc;
	return n;
}

/* Simulate a drive over it: Gaussian spot (fwhm mm), speed mm/s with a
   drift, swinging +-swing_pct % (a triangle wave, 0.6-3 s period: the
   car weaving), saturating response, noise, optional drop-outs */
static uint32_t bc_sim(char c, bool rev, double fwhm, double speed, uint32_t swing_pct,
		       double paper, double bars, uint32_t drops, uint16_t *p_raw)
{
	double		e[BC_N + 1u];
	uint32_t	ne = bc_sym_edges(c, rev, 3.5, 3.0, e);
	double		sigma = fwhm / 2.355;
	double		pos = -45.0 - (double)bc_rand(10u);
	double		drift = ((double)bc_rand(11u) - 5.0) / 100.0;	/* +-5 % over it */
	double		end = e[ne] + 60.0;
	uint32_t	period_ms = 600u + (bc_rand(25u) * 100u);
	uint32_t	t_ms = bc_rand(period_ms);
	uint32_t	n = 0u;
	uint32_t	d;

	while ((pos < end) && (n < BC_REC_MAX)) {
		double cov = 0.0;
		double v;
		uint32_t k;

		for (k = 0u; k < ne; k += 2u) {		/* bars */
			cov += bc_phi((e[k + 1u] - pos) / sigma) - bc_phi((e[k] - pos) / sigma);
		}
		v = 1.0 - ((1.0 - cov) * (1.0 - cov) * (1.0 - cov) * 0.4) - ((1.0 - cov) * 0.6);
		v = paper + ((bars - paper) * v) + (double)bc_rand(33u) - 16.0;
		p_raw[n++] = (uint16_t)((v < 0.0) ? 0.0 : ((v > 4095.0) ? 4095.0 : v));
		{
			double ph = (double)(t_ms % period_ms) / (double)period_ms;
			double tri = (ph < 0.5) ? ((4.0 * ph) - 1.0) : (3.0 - (4.0 * ph));
			double sw = 1.0 + (tri * (double)swing_pct / 100.0);

			pos += speed * 0.020 * (1.0 + (drift * pos / end)) * ((sw < 0.05) ? 0.05 : sw);
		}
		t_ms += 20u;
	}
	for (d = 0u; d < drops; d++) {			/* the loose wire */
		uint32_t at = bc_rand(n);
		uint32_t len = 1u + bc_rand(18u);
		uint32_t k;

		for (k = at; (k < (at + len)) && (k < n); k++) {
			p_raw[k] = (uint16_t)(144u + bc_rand(33u));
		}
	}
	return n;
}

static uint16_t	g_sim[BC_REC_MAX];

/* Read every character (not '*'), both ways, 12 runs each: paper at 150
   or 1,100, 12 / 20 / 40 mm/s, swinging +-swing_pct %, spot fwhm mm.
   Adds to *p_right, *p_wrong (the dangerous one), *p_unread. */
static void bc_letters(double fwhm, uint32_t swing_pct, uint32_t *p_right,
		       uint32_t *p_wrong, uint32_t *p_unread)
{
	static const double speeds[3] = { 12.0, 20.0, 40.0 };
	uint8_t		idx;
	char		c;
	uint16_t	pat;

	for (idx = 0u; c39_table(idx, &c, &pat); idx++) {
		uint32_t k;

		if (c == '*') {
			continue;
		}
		for (k = 0u; k < 12u; k++) {
			char	text[C39_MAX_TEXT + 1u];
			uint8_t	how = 0u;
			bool	rev = (k % 2u) == 1u;
			double	paper = ((k / 2u) % 2u) ? 1100.0 : 150.0;
			uint32_t n = bc_sim(c, rev, fwhm, speeds[(k / 4u) % 3u], swing_pct, paper,
					    3300.0 + (double)bc_rand(500u), 0u, g_sim);

			bc_rec_fill(g_sim, n);
			if (bc_second_look(144u, 4000u, text, &how) == 0u) {
				(*p_unread)++;
			} else if ((text[0] == c) && (text[1] == '\0')) {
				(*p_right)++;
			} else {
				printf("  WRONG: *%c* %s, spot %.0f mm, paper %.0f, speed +-%lu %%: "
				       "read %s (how %u)\n", c, rev ? "reversed" : "forward", fwhm,
				       paper, (unsigned long)swing_pct, text, (unsigned)how);
				(*p_wrong)++;
			}
		}
	}
}

/* BC-11 (8 Nov): every character through a 3 and a 5 mm spot (the side
   sensor 2-5 mm above the paper), nearly steady speed: never a wrong
   letter; with 3 mm nearly all read.
   BC-12 (8 Nov): the same with the speed swinging +-40 % within the
   barcode (the wide/narrow rule is 1.2 x since 8 Nov, s3_code39.c):
   never a wrong letter.
   BC-13: no barcode (an 18 mm stripe, the line) never reads, and sharp
   edges are not drop-outs. */
static uint32_t bc_letters_check(void)
{
	static const double	speeds[3] = { 12.0, 20.0, 40.0 };
	static const double	fwhms[2] = { 3.0, 5.0 };
	uint32_t	f = 0u;
	uint32_t	q;

	for (q = 0u; q < 2u; q++) {
		uint32_t right = 0u;
		uint32_t wrong = 0u;
		uint32_t unread = 0u;

		bc_letters(fwhms[q], 0u, &right, &wrong, &unread);
		printf("BC-11 spot %.0f mm: %lu read, %lu WRONG, %lu not\n", fwhms[q],
		       (unsigned long)right, (unsigned long)wrong, (unsigned long)unread);
		f += wrong;
		if (q == 0u) {
			f += ((right * 100u) >= ((right + unread) * 95u)) ? 0u : 1u;
		}
	}
	{
		uint32_t right = 0u;
		uint32_t wrong = 0u;
		uint32_t unread = 0u;

		bc_letters(3.0, 40u, &right, &wrong, &unread);
		printf("BC-12 speed +-40 %%: %lu read, %lu WRONG, %lu not\n",
		       (unsigned long)right, (unsigned long)wrong, (unsigned long)unread);
		f += wrong;
	}
	{
		uint32_t k;
		uint32_t reads = 0u;

		for (k = 0u; k < 40u; k++) {
			char	text[C39_MAX_TEXT + 1u];
			uint8_t	how = 0u;
			uint32_t n = bc_sim('\0', false, 3.0 + (2.0 * (double)(k % 3u)), speeds[k % 3u],
					    0u, (k % 2u) ? 1100.0 : 150.0, 3500.0, 0u, g_sim);

			bc_rec_fill(g_sim, n);
			reads += (bc_second_look(144u, 4000u, text, &how) > 0u) ? 1u : 0u;
		}
		printf("BC-13 (no barcode, just the line): %lu reads (must be 0)\n", (unsigned long)reads);
		f += reads;
	}
	{	/* sharp edges are not drop-outs: a 3 mm spot on white paper at 150 */
		uint32_t k;
		uint32_t flagged = 0u;

		for (k = 0u; k < 20u; k++) {
			uint32_t n = bc_sim('K', (k % 2u) == 1u, 3.0, speeds[k % 3u], 0u, 150.0,
					    3600.0, 0u, g_sim);

			bc_rec_fill(g_sim, n);
			flagged += bc_rec_find_dropouts(&g_rec, BC_TEST_DEAD_RAW,
							(uint16_t)LF_BC_DEAD_JUMP);
		}
		printf("BC-13 (sharp sensor, no drop-outs): %lu flagged as drop-outs (must be 0)\n",
		       (unsigned long)flagged);
		f += flagged;
	}
	return f;
}

/* BC-14 (4 Nov): the car weaves over the barcode, so the side sensor's
   speed swings (here 25 mm/s +- 20 mm/s, period p ms).  Read with the
   time as the clock, and with the distance as the clock (task_line.c
   s3_bc_clock: um x 1000 / LF_BC_DIST_MM_S).  Returns the letter, or 0. */
static char bc_weave_read(bool by_distance, uint32_t period_ms, uint32_t phase_ms, bool reverse)
{
	uint32_t	edge_um[BC_N + 1u];
	uint32_t	acc = 0u;
	uint32_t	i;
	double		pos_um = -60000.0;
	uint32_t	t_ms = 0u;
	bool		dark = false;
	bc_reader_t	rd;
	char		text[C39_MAX_TEXT + 1u];
	uint8_t		count = 0u;

	edge_um[0] = 0u;
	for (i = 0u; i < BC_N; i++) {
		acc += bc_px[BC_K][reverse ? (BC_N - 1u - i) : i];
		edge_um[i + 1u] = acc * 100u;
	}
	bc_reader_init(&rd);
	while (pos_um < 260000.0) {
		uint32_t lvl = bc_cover(edge_um, (int32_t)pos_um, 3000u);
		uint32_t clk = by_distance
			? (uint32_t)((pos_um + 60000.0) * 1000.0 / (double)LF_BC_DIST_MM_S)
			: (t_ms * 1000u);
		uint8_t r;
		double	ph = (double)((t_ms + phase_ms) % period_ms) / (double)period_ms;
		double	tri = (ph < 0.5) ? ((4.0 * ph) - 1.0) : (3.0 - (4.0 * ph));	/* -1..1 */
		double	v = 25.0 + (20.0 * tri);					/* mm/s */

		if (lvl > 380u) {
			dark = true;
		} else if (lvl < 320u) {
			dark = false;
		} else {
			/* keep */
		}
		r = bc_reader_sample(&rd, clk, dark, true);
		if ((r == BC_RD_DONE) && (bc_reader_decode(&rd, text, (uint8_t)sizeof text, &count) == 1u)) {
			return text[0];
		}
		pos_um += v * 20.0;			/* um in 20 ms */
		t_ms += 20u;
	}
	return '\0';
}

static uint32_t bc_weave_check(void)
{
	static const uint32_t periods[3] = { 800u, 1500u, 3000u };
	uint32_t ok_t = 0u;
	uint32_t ok_d = 0u;
	uint32_t n = 0u;
	uint32_t p;
	uint32_t ph;

	for (p = 0u; p < 3u; p++) {
		for (ph = 0u; ph < periods[p]; ph += periods[p] / 8u) {
			ok_t += (bc_weave_read(false, periods[p], ph, (ph & 1u) != 0u) == 'K') ? 1u : 0u;
			ok_d += (bc_weave_read(true, periods[p], ph, (ph & 1u) != 0u) == 'K') ? 1u : 0u;
			n++;
		}
	}
	printf("BC-14 (weaving 5-45 mm/s): K read by time %lu of %lu, by distance %lu of %lu\n",
	       (unsigned long)ok_t, (unsigned long)n, (unsigned long)ok_d, (unsigned long)n);
	return (ok_d == n) ? 0u : 1u;
}

int main(void)
{
	uint32_t failures = bc_check_run();
	uint32_t f9 = 0u;
	uint32_t sp;

	for (sp = 10u; sp <= 40u; sp += 10u) {
		f9 += bc_rec_k_ok(sp, 6000u, false, false) ? 0u : 1u;
		f9 += bc_rec_k_ok(sp, 6000u, true, false) ? 0u : 1u;
		f9 += bc_rec_k_ok(sp, 0u, false, false) ? 0u : 1u;
		f9 += bc_rec_k_ok(sp, 6000u, false, true) ? 1u : 0u;	/* overflowed: no re-read */
	}
	printf("BC-09 (second look) failures: %lu\n", (unsigned long)f9);
	failures += f9;
	{
		static const uint32_t cons[] = { 45u, 55u, 70u, 100u };
		uint32_t f10 = 0u;
		uint32_t c;

		for (c = 0u; c < 4u; c++) {
			for (sp = 10u; sp <= 40u; sp += 15u) {
				f10 += bc_faint_k_ok(sp, cons[c], false) ? 0u : 1u;
				f10 += bc_faint_k_ok(sp, cons[c], true) ? 0u : 1u;
			}
		}
		printf("BC-10 (faint bars, 45-100 %% of the tape) failures: %lu\n", (unsigned long)f10);
		failures += f10;
	}

	{
		uint32_t f11 = bc_letters_check();
		uint32_t f14 = bc_weave_check();

		printf("BC-11..14 failures: %lu\n", (unsigned long)(f11 + f14));
		failures += f11 + f14;
	}

	printf("code39 regression failures: %lu\n", (unsigned long)failures);
	{	/* how blurry may the side sensor be? (information only) */
		static const uint32_t pcts[] = { 35u, 50u };
		uint32_t p;
		uint32_t spot;
		uint32_t speed;

		for (p = 0u; p < 2u; p++) {
			for (speed = 10u; speed <= 40u; speed += 10u) {
				uint32_t best = 0u;

				for (spot = 0u; spot <= 9000u; spot += 250u) {
					uint32_t ph;
					bool ok = true;

					for (ph = 0u; ph < 10u; ph++) {
						ok = ok && bc_k_ok(speed, speed, ph, false, spot, pcts[p]) &&
						     bc_k_ok(speed, speed, ph, true, spot, pcts[p]);
					}
					if (!ok) {
						break;
					}
					best = spot;
				}
				printf("K at %2lu mm/s, threshold %lu %%: reads with a spot up to %.2f mm\n",
				       (unsigned long)speed, (unsigned long)pcts[p], (double)best / 1000.0);
			}
		}
	}
	return (failures == 0u) ? 0 : 1;
}
#endif
