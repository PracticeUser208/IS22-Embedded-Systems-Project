/* Buddy 3: hardware-independent, caller-calibrated IR classification. */
#ifndef S3_IR_H
#define S3_IR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint16_t white;
    uint16_t black;
    uint16_t hysteresis;
} s3_ir_cal_t;

/* References are measured on known surfaces. No electrical polarity assumed.
 * Hysteresis is in raw input units; min_span must be measured/selected by caller.
 * On failure the destination is not changed. */
bool s3_ir_calibrate(s3_ir_cal_t * p_cal, uint16_t white, uint16_t black,
                     uint16_t min_span, uint16_t hysteresis);

/* Returns false for invalid calibration/pointers. b_black is in/out state;
 * b_known=false initializes using the midpoint, without historical hysteresis. */
bool s3_ir_classify(s3_ir_cal_t const * p_cal, uint16_t raw,
                    bool b_known, bool * p_black);

/* Normalize measured reflectance: white=0, black=1000, clamped.
 * This is reflectance, NOT steering units or a motor command. */
bool s3_ir_normalize(s3_ir_cal_t const * p_cal, uint16_t raw,
                     uint16_t * p_blackness);

#endif /* S3_IR_H */
/* End of file. */

