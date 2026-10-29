#include "s3_check.h"

#if defined(S3_HOST_TEST) || (defined(APP_TEST) && APP_TEST)
#include "s3_ir.h"
#include "s3_line.h"
#include "s3_pulse.h"
#include "s3_state.h"
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
    failures += (s3_state_step(&state, S3_STATE_POLL, start + 1999999u)
                  != S3_STATE_EVT_NONE);
    failures += (s3_state_step(&state, S3_STATE_LINE_FOUND, start + 2000000u)
                  != S3_STATE_EVT_TIMEOUT);
    failures += (s3_state_step(&state, S3_STATE_POLL, start + 2000001u)
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

