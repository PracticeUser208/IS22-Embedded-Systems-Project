#include "s3_pulse.h"
#include <stddef.h>

#define ACTIVE 1u
#define INVALID 2u
#define DARK 4u
#define FIRST_DARK 8u

static bool invalidate(s3_pulse_t * p_capture)
{
    p_capture->flags = (uint8_t)(((uint32_t)p_capture->flags | INVALID) & ~ACTIVE);
    return (false);
}

static bool check_time(s3_pulse_t * p_capture, uint32_t now_us)
{
    if ((NULL == p_capture) || ((p_capture->flags & ACTIVE) == 0u))
    {
        return (false);
    }
    if (((now_us - p_capture->last_sample_us) > p_capture->max_gap_us)
        || ((now_us - p_capture->last_edge_us) > p_capture->max_width_us))
    {
        return (invalidate(p_capture));
    }
    p_capture->last_sample_us = now_us;
    return (true);
}

static bool append_run(s3_pulse_t * p_capture, uint32_t now_us)
{
    uint32_t width = now_us - p_capture->last_edge_us;
    if ((width == 0u) || (p_capture->count >= S3_PULSE_CAPACITY))
    {
        return (invalidate(p_capture));
    }
    p_capture->widths_us[p_capture->count] = width;
    p_capture->count++;
    p_capture->last_edge_us = now_us;
    return (true);
}

bool s3_pulse_begin(s3_pulse_t * p_capture, uint32_t now_us, bool b_dark,
                    uint32_t max_gap_us, uint32_t max_width_us)
{
    if (NULL == p_capture)
    {
        return (false);
    }
    p_capture->flags = INVALID;
    p_capture->count = 0u;
    if ((max_gap_us == 0u) || (max_gap_us >= UINT32_C(0x80000000))
        || (max_width_us == 0u) || (max_width_us >= UINT32_C(0x80000000)))
    {
        return (false);
    }
    p_capture->last_edge_us = now_us;
    p_capture->last_sample_us = now_us;
    p_capture->max_gap_us = max_gap_us;
    p_capture->max_width_us = max_width_us;
    p_capture->flags = (uint8_t)(ACTIVE | (b_dark ? DARK | FIRST_DARK : 0u));
    return (true);
}

bool s3_pulse_sample(s3_pulse_t * p_capture, uint32_t now_us, bool b_dark)
{
    if (!check_time(p_capture, now_us))
    {
        return (false);
    }
    if (b_dark != ((p_capture->flags & DARK) != 0u))
    {
        if (!append_run(p_capture, now_us))
        {
            return (false);
        }
        p_capture->flags ^= DARK;
    }
    return (true);
}

bool s3_pulse_finish(s3_pulse_t * p_capture, uint32_t now_us)
{
    if (!check_time(p_capture, now_us) || !append_run(p_capture, now_us))
    {
        return (false);
    }
    p_capture->flags = (uint8_t)((uint32_t)p_capture->flags & ~ACTIVE);
    return (true);
}

bool s3_pulse_matches(s3_pulse_t const * p_capture, bool b_first_dark,
                      uint32_t const * p_min_us, uint32_t const * p_max_us,
                      uint8_t count)
{
    if ((NULL == p_capture) || (NULL == p_min_us) || (NULL == p_max_us)
        || (count == 0u) || (count > S3_PULSE_CAPACITY)
        || (count != p_capture->count)
        || ((p_capture->flags & (ACTIVE | INVALID)) != 0u)
        || (b_first_dark != ((p_capture->flags & FIRST_DARK) != 0u)))
    {
        return (false);
    }
    for (uint8_t index = 0u; index < count; index++)
    {
        if ((p_min_us[index] == 0u) || (p_min_us[index] > p_max_us[index])
            || (p_capture->widths_us[index] < p_min_us[index])
            || (p_capture->widths_us[index] > p_max_us[index]))
        {
            return (false);
        }
    }
    return (true);
}
/* End of file. */

