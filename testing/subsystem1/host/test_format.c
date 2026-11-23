/*
 *  test_format.c
 *  PC check of subsystem1/telem_format.h (Buddy 1): prints the sizes
 *  that go into the Week 13 message-size table and checks them.
 *  Build and run from testing/subsystem1/host:
 *    gcc -Wall -Wextra -I../../../app_program/subsystem1 test_format.c -o test_format
 *    ./test_format
 */

#include <stdio.h>
#include "telem_format.h"

int main(void)
{
    int fail = 0;

    printf("telemetry : %3u bits -> %2u bytes on the wire\n", TF_TM_BITS, TF_TM_LEN);
    printf("heartbeat : %3u bits -> %2u bytes on the wire\n", TF_HB_BITS, TF_HB_LEN);
    printf("command   : %3u bits -> %2u bytes on the wire\n", TF_CMD_BITS, TF_CMD_LEN);
    printf("ack       : %3u bits -> %2u bytes on the wire\n", TF_ACK_BITS, TF_ACK_LEN);
    printf("telem_q message  (telem_snapshot_t): %zu bytes\n", sizeof(telem_snapshot_t));
    printf("remote_cmd_q msg (remote_cmd_t)    : %zu bytes\n", sizeof(remote_cmd_t));

    printf("format version   : %u\n", TF_VERSION);

    if ((19u != TF_TM_LEN) || (7u != TF_HB_LEN) || (1u != TF_CMD_LEN) || (2u != TF_ACK_LEN))
    {
        printf("FAIL  wire sizes differ from the v2 layout 19/7/1/2\n");
        fail = 1;
    }
    if (sizeof(telem_snapshot_t) > 20u)
    {
        printf("FAIL  telem_snapshot_t is larger than 20 bytes\n");
        fail = 1;
    }

    printf("\n%s\n", (0 == fail) ? "ALL PASSED" : "FAILED");
    return fail;
}
