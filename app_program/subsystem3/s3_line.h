/* Configured two-sensor interpretation; no assumed sensor spacing or steering units. */
#ifndef S3_LINE_H
#define S3_LINE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    /* Index: bit 0 left dark, bit 1 centre dark. Each bit in the masks
     * describes one index. Caller must establish these mappings on the track.
     * Steering values are opaque caller-defined units, NOT PWM values. */
    int16_t steering[4];
    uint8_t present_mask;
    uint8_t steering_valid_mask;
} s3_line_config_t;

typedef struct
{
    int16_t steering;
    /* bits 0..1 sensor pattern; bit 2 line present; bit 3 steering valid */
    uint8_t flags;
} s3_line_result_t;

bool s3_line_evaluate(s3_line_config_t const * p_config, bool b_left_dark,
                      bool b_centre_dark, s3_line_result_t * p_result);
bool s3_line_is_present(s3_line_result_t const * p_result);
bool s3_line_has_steering(s3_line_result_t const * p_result);

#endif /* S3_LINE_H */
/* End of file. */

