/*
 *  test_bitpack.c
 *  PC unit tests for subsystem1/bitpack.c (Buddy 1). Not built into the Pico image.
 *  Build and run from testing/subsystem1/host:
 *    gcc -Wall -Wextra -I../../../app_program/subsystem1 \
 *        test_bitpack.c ../../../app_program/subsystem1/bitpack.c -o test_bitpack
 *    ./test_bitpack
 */

#include <stdio.h>
#include "bitpack.h"

static int g_fail = 0;

static void check(char const *p_name, bool b_ok)
{
    printf("%s  %s\n", b_ok ? "PASS" : "FAIL", p_name);
    if (!b_ok)
    {
        g_fail++;
    }
}

int main(void)
{
    bitstream_t bs;
    uint8_t     buf[4];
    uint8_t     guard[3] = {0xAAu, 0x00u, 0xAAu};   /* [0],[2] = guards, [1] = 1-byte buffer */

    /* B1: two fields in one byte: code=1 (3 bits), seq=5 (5 bits) -> 0x29 */
    bs_init_write(&bs, buf, sizeof buf);
    bs_put(&bs, 1u, 3u);
    bs_put(&bs, 5u, 5u);
    check("B1 pack 1|5<<3 == 0x29", (0x29u == buf[0]) && (1u == bs_bytes_used(&bs)));

    /* B2: field crossing a byte boundary: 0xABC (12 bits) -> BC 0A */
    bs_init_write(&bs, buf, sizeof buf);
    bs_put(&bs, 0xABCu, 12u);
    check("B2 cross-byte 0xABC -> BC 0A", (0xBCu == buf[0]) && (0x0Au == buf[1]));

    /* B3: signed round trip: -1 and -64 and +63 in 7 bits */
    bs_init_write(&bs, buf, sizeof buf);
    bs_put(&bs, (uint32_t)(int32_t)-1, 7u);
    bs_put(&bs, (uint32_t)(int32_t)-64, 7u);
    bs_put(&bs, 63u, 7u);
    bs_init_read(&bs, buf, sizeof buf);
    {
        int32_t a = bs_get_signed(&bs, 7u);
        int32_t b = bs_get_signed(&bs, 7u);
        int32_t c = bs_get_signed(&bs, 7u);
        check("B3 signed -1/-64/+63 round trip", (-1 == a) && (-64 == b) && (63 == c));
    }

    /* B4: overflow: 9 bits into a 1-byte buffer must not write past it */
    bs_init_write(&bs, &guard[1], 1u);
    bs_put(&bs, 0x1FFu, 9u);
    check("B4 overflow flagged, guards untouched",
          bs.b_overflow && (0xAAu == guard[0]) && (0x00u == guard[1]) && (0xAAu == guard[2]));

    /* B5: once overflowed, later small writes are ignored too */
    bs_put(&bs, 1u, 1u);
    check("B5 overflow is sticky", (0x00u == guard[1]) && (0u == bs.pos));

    /* B6: unsigned round trip of many widths, back to back */
    bs_init_write(&bs, buf, sizeof buf);
    bs_put(&bs, 1u, 1u);
    bs_put(&bs, 6u, 3u);
    bs_put(&bs, 4095u, 12u);
    bs_put(&bs, 511u, 9u);
    bs_init_read(&bs, buf, sizeof buf);
    {
        uint32_t a = bs_get(&bs, 1u);
        uint32_t b = bs_get(&bs, 3u);
        uint32_t c = bs_get(&bs, 12u);
        uint32_t d = bs_get(&bs, 9u);
        check("B6 unsigned 1/6/4095/511 round trip",
              (1u == a) && (6u == b) && (4095u == c) && (511u == d) && !bs.b_overflow);
    }

    printf("\n%s: %d failure(s)\n", (0 == g_fail) ? "ALL PASSED" : "FAILED", g_fail);
    return (0 == g_fail) ? 0 : 1;
}