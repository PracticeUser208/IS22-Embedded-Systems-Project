#ifndef S3_CHECK_H
#define S3_CHECK_H

#include <stdint.h>

/* Pure regression checks; returns number of failed checks. No peripherals. */
uint32_t s3_check_run(void);

#endif /* S3_CHECK_H */
/* End of file. */

