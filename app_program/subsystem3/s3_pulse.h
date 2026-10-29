/* Bounded pulse capture. Framing and symbol definitions belong to configuration. */
#ifndef S3_PULSE_H
#define S3_PULSE_H

#include <stdbool.h>
#include <stdint.h>

/* Software storage limit, not an assertion about the barcode format. */
#define S3_PULSE_CAPACITY 64u

typedef struct
{
    uint32_t widths_us[S3_PULSE_CAPACITY];
    uint32_t last_edge_us;
    uint32_t last_sample_us;
    uint32_t max_gap_us;
    uint32_t max_width_us;
    uint8_t count;
    /* bit 0 active, bit 1 invalid, bit 2 current dark, bit 3 first dark */
    uint8_t flags;
} s3_pulse_t;

/* Call only on an externally established frame-start edge. Samples and edges
 * are monotonic modulo uint32_t, with gaps/widths below 2^31 microseconds. */
bool s3_pulse_begin(s3_pulse_t * p_capture, uint32_t now_us, bool b_dark,
                    uint32_t max_gap_us, uint32_t max_width_us);
bool s3_pulse_sample(s3_pulse_t * p_capture, uint32_t now_us, bool b_dark);
/* Finish at an established frame boundary; appends final run. Does not infer
 * a quiet zone from the lateral 20 mm track spacing. */
bool s3_pulse_finish(s3_pulse_t * p_capture, uint32_t now_us);

/* Compare a COMPLETE frame with a caller-supplied run pattern (including any
 * start/stop elements). Each expected run is an inclusive duration interval.
 * No encoding, command mapping, reverse policy or speed is assumed. */
bool s3_pulse_matches(s3_pulse_t const * p_capture, bool b_first_dark,
                      uint32_t const * p_min_us, uint32_t const * p_max_us,
                      uint8_t count);

#endif /* S3_PULSE_H */
/* End of file. */

