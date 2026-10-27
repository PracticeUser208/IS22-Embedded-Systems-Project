/*
 *  bitpack.h
 *  Subsystem 1 (Buddy 1) -- bit-level writer/reader for compact MQTT payloads.
 *
 *  Packing rule: LSB-first. The first field starts at bit 0 of byte 0;
 *  each later field starts at the next free bit and may cross bytes.
 *  Only masks and shifts are used, so the layout is the same on the Pico,
 *  on a PC and in any ground-station script.
 *
 *  Uses only <stdint.h>/<stdbool.h> so it also builds on a PC for tests.
 */

#ifndef BITPACK_H
#define BITPACK_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint8_t  *p_buf;        /* byte buffer being written or read       */
    uint16_t  cap_bits;     /* buffer size in bits                     */
    uint16_t  pos;          /* next bit to write or read               */
    bool      b_overflow;   /* set on any out-of-range access; sticky  */
} bitstream_t;

/* Start writing: clears the buffer, then pos = 0 */
void     bs_init_write(bitstream_t *p_bs, uint8_t *p_buf, uint16_t len_bytes);

/* Start reading: does NOT touch the buffer, pos = 0 */
void     bs_init_read(bitstream_t *p_bs, uint8_t *p_buf, uint16_t len_bytes);

/* Write the low nbits (1..32) of value. Negative numbers: pass the value
   cast to uint32_t; the low nbits are its two's-complement form. */
void     bs_put(bitstream_t *p_bs, uint32_t value, uint8_t nbits);

/* Read nbits (1..32) as an unsigned value */
uint32_t bs_get(bitstream_t *p_bs, uint8_t nbits);

/* Read nbits (1..32) as a two's-complement signed value */
int32_t  bs_get_signed(bitstream_t *p_bs, uint8_t nbits);

/* Whole bytes touched so far (bits rounded up) */
uint16_t bs_bytes_used(bitstream_t const *p_bs);

#endif /* BITPACK_H */