#include "s3_ir.h"
#include <stddef.h>

static bool cal_valid(s3_ir_cal_t const * p_cal)
{
    bool b_valid = false;
    if (NULL != p_cal)
    {
        uint32_t span = (p_cal->black > p_cal->white)
            ? (uint32_t)p_cal->black - p_cal->white
            : (uint32_t)p_cal->white - p_cal->black;
        b_valid = (span > (2u * p_cal->hysteresis));
    }
    return (b_valid);
}

bool s3_ir_calibrate(s3_ir_cal_t * p_cal, uint16_t white, uint16_t black,
                     uint16_t min_span, uint16_t hysteresis)
{
    s3_ir_cal_t candidate = {white, black, hysteresis};
    uint32_t span = (black > white) ? (uint32_t)black - white
                                          : (uint32_t)white - black;
    bool b_ok = (NULL != p_cal) && (min_span > 0u)
        && (span >= min_span) && cal_valid(&candidate);
    if (b_ok)
    {
        *p_cal = candidate;
    }
    return (b_ok);
}

bool s3_ir_classify(s3_ir_cal_t const * p_cal, uint16_t raw,
                    bool b_known, bool * p_black)
{
    if ((NULL == p_black) || !cal_valid(p_cal))
    {
        return (false);
    }

    int32_t direction = (p_cal->black > p_cal->white) ? 1 : -1;
    int32_t midpoint = ((int32_t)p_cal->white + (int32_t)p_cal->black) / 2;
    int32_t distance = ((int32_t)raw - midpoint) * direction;
    int32_t band = (int32_t)p_cal->hysteresis;
    if (!b_known)
    {
        *p_black = (distance >= 0);
    }
    else if (distance > band)
    {
        *p_black = true;
    }
    else if (distance < -band)
    {
        *p_black = false;
    }
    else
    {
        /* Retain the previous classification inside the hysteresis band. */
    }
    return (true);
}

bool s3_ir_normalize(s3_ir_cal_t const * p_cal, uint16_t raw,
                     uint16_t * p_blackness)
{
    if ((NULL == p_blackness) || !cal_valid(p_cal))
    {
        return (false);
    }

    int32_t span = (int32_t)p_cal->black - (int32_t)p_cal->white;
    int32_t value = (((int32_t)raw - (int32_t)p_cal->white) * 1000) / span;
    if (value < 0)
    {
        value = 0;
    }
    else if (value > 1000)
    {
        value = 1000;
    }
    else
    {
        /* In calibrated range. */
    }
    *p_blackness = (uint16_t)value;
    return (true);
}
/* End of file. */

