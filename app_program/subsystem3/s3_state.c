#include "s3_state.h"
#include <stddef.h>

#define STATE_MASK 7u
#define CALIBRATED 8u

static void set_state(s3_state_t * p_state, uint8_t value)
{
    p_state->packed = (uint8_t)((p_state->packed & CALIBRATED) | value);
}

void s3_state_init(s3_state_t * p_state)
{
    if (NULL != p_state)
    {
        p_state->packed = S3_STATE_INIT;
        p_state->search_start_us = 0u;
    }
}

uint8_t s3_state_get(s3_state_t const * p_state)
{
    return ((NULL == p_state) ? S3_STATE_FAULT
                               : (uint8_t)(p_state->packed & STATE_MASK));
}

bool s3_state_can_track(s3_state_t const * p_state)
{
    uint8_t state = s3_state_get(p_state);
    return ((state == S3_STATE_FOLLOW) || (state == S3_STATE_CAPTURE)
            || (state == S3_STATE_DECODE));
}

uint8_t s3_state_step(s3_state_t * p_state, s3_state_input_t input,
                      uint32_t now_us)
{
    uint8_t state = s3_state_get(p_state);
    if (NULL == p_state)
    {
        return (S3_STATE_EVT_NONE);
    }
    if (input == S3_STATE_RESET)
    {
        s3_state_init(p_state);
        return (S3_STATE_EVT_NONE);
    }
    if (state == S3_STATE_FAULT)
    {
        return (S3_STATE_EVT_NONE);
    }
    if ((state > S3_STATE_FAULT) || (input == S3_STATE_SENSOR_FAULT))
    {
        set_state(p_state, S3_STATE_FAULT);
        return (S3_STATE_EVT_FAULT);
    }
    if (input == S3_STATE_SUSPEND)
    {
        set_state(p_state, S3_STATE_SUSPENDED);
        return (S3_STATE_EVT_NONE);
    }
    if ((state == S3_STATE_SEARCH)
        && ((now_us - p_state->search_start_us) >= S3_STATE_SEARCH_US))
    {
        set_state(p_state, S3_STATE_FAULT);
        return (S3_STATE_EVT_TIMEOUT);
    }
    if ((state == S3_STATE_INIT) && (input == S3_STATE_CALIBRATED))
    {
        p_state->packed = CALIBRATED | S3_STATE_FOLLOW;
    }
    else if ((state == S3_STATE_SUSPENDED) && (input == S3_STATE_RESUME_SEARCH))
    {
        if ((p_state->packed & CALIBRATED) == 0u)
        {
            set_state(p_state, S3_STATE_INIT);
        }
        else
        {
            set_state(p_state, S3_STATE_SEARCH);
            p_state->search_start_us = now_us;
        }
    }
    else if (s3_state_can_track(p_state) && (input == S3_STATE_LINE_LOST))
    {
        set_state(p_state, S3_STATE_SEARCH);
        p_state->search_start_us = now_us;
        return (S3_STATE_EVT_LOST);
    }
    else if ((state == S3_STATE_SEARCH) && (input == S3_STATE_LINE_FOUND))
    {
        set_state(p_state, S3_STATE_FOLLOW);
        return (S3_STATE_EVT_REACQUIRED);
    }
    else if ((state == S3_STATE_FOLLOW) && (input == S3_STATE_BARCODE_START))
    {
        set_state(p_state, S3_STATE_CAPTURE);
    }
    else if ((state == S3_STATE_CAPTURE) && (input == S3_STATE_BARCODE_END))
    {
        set_state(p_state, S3_STATE_DECODE);
    }
    else if ((state == S3_STATE_DECODE) && (input == S3_STATE_DECODE_DONE))
    {
        set_state(p_state, S3_STATE_FOLLOW);
    }
    else
    {
        /* Ignore inputs inappropriate to this state; never implicitly reset. */
    }
    return (S3_STATE_EVT_NONE);
}
/* End of file. */

