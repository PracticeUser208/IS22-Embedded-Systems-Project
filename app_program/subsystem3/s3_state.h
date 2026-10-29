/* Pure seven-state transition engine. Values below are internal, NOT queue ABI. */
#ifndef S3_STATE_H
#define S3_STATE_H

#include <stdbool.h>
#include <stdint.h>

#define S3_STATE_INIT 0u
#define S3_STATE_FOLLOW 1u
#define S3_STATE_CAPTURE 2u
#define S3_STATE_DECODE 3u
#define S3_STATE_SUSPENDED 4u
#define S3_STATE_SEARCH 5u
#define S3_STATE_FAULT 6u
#define S3_STATE_SEARCH_US UINT32_C(2000000)

typedef enum
{
    S3_STATE_CALIBRATED,
    S3_STATE_BARCODE_START,
    S3_STATE_BARCODE_END,
    S3_STATE_DECODE_DONE,
    S3_STATE_LINE_LOST,
    S3_STATE_LINE_FOUND,
    S3_STATE_SUSPEND,
    S3_STATE_RESUME_SEARCH,
    S3_STATE_RESET,
    S3_STATE_SENSOR_FAULT,
    S3_STATE_POLL
} s3_state_input_t;

#define S3_STATE_EVT_NONE 0u
#define S3_STATE_EVT_LOST 1u
#define S3_STATE_EVT_REACQUIRED 2u
#define S3_STATE_EVT_TIMEOUT 4u
#define S3_STATE_EVT_FAULT 8u

typedef struct
{
    uint32_t search_start_us;
    /* bits 0..2 state; bit 3 calibrated; remaining bits reserved */
    uint8_t packed;
} s3_state_t;

void s3_state_init(s3_state_t * p_state);
uint8_t s3_state_get(s3_state_t const * p_state);
/* Returns internal event bits for the caller to translate to agreed nav_q.
 * Timeout wins at exactly 2 s, except RESET/SUSPEND/fault commands.
 * This is a proposed policy, documented for team review, not physical motion.
 * Call at least once per 2^31 us; timestamps are acquisition/monotonic time. */
uint8_t s3_state_step(s3_state_t * p_state, s3_state_input_t input,
                      uint32_t now_us);
bool s3_state_can_track(s3_state_t const * p_state);

#endif /* S3_STATE_H */
/* End of file. */

