#include "s3_line.h"
#include <stddef.h>

bool s3_line_evaluate(s3_line_config_t const * p_config, bool b_left_dark,
                      bool b_centre_dark, s3_line_result_t * p_result)
{
    if (NULL == p_result)
    {
        return (false);
    }
    p_result->steering = 0;
    p_result->flags = 0u;
    if ((NULL == p_config)
        || ((p_config->present_mask & 0xf0u) != 0u)
        || ((p_config->steering_valid_mask & 0xf0u) != 0u)
        || (((uint32_t)p_config->steering_valid_mask
             & ~(uint32_t)p_config->present_mask) != 0u))
    {
        return (false);
    }

    uint8_t pattern = (uint8_t)((b_left_dark ? 1u : 0u)
                               | (b_centre_dark ? 2u : 0u));
    uint32_t selected = 1u << pattern;
    p_result->flags = pattern;
    if ((p_config->present_mask & selected) != 0u)
    {
        p_result->flags |= 4u;
    }
    if ((p_config->steering_valid_mask & selected) != 0u)
    {
        p_result->flags |= 8u;
        p_result->steering = p_config->steering[pattern];
    }
    return (true);
}

bool s3_line_is_present(s3_line_result_t const * p_result)
{
    return ((NULL != p_result) && ((p_result->flags & 4u) != 0u));
}

bool s3_line_has_steering(s3_line_result_t const * p_result)
{
    return ((NULL != p_result) && ((p_result->flags & 8u) != 0u));
}
/* End of file. */

