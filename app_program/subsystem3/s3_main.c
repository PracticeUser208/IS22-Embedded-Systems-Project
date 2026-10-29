#include "s3_main.h"

#if defined(S3_HOST_TEST)
#include "s3_check.h"
#include <stdio.h>
#include <inttypes.h>

int main(void)
{
    uint32_t failures = s3_check_run();
    printf("Buddy 3 pure-logic regression failures: %" PRIu32 "\n", failures);
    return ((failures == 0u) ? 0 : 1);
}
#endif
/* End of file. */

