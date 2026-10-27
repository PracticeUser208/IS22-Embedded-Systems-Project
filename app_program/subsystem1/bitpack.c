/*
 *  bitpack.c
 *  Subsystem 1 (Buddy 1) -- bit-level writer/reader (see bitpack.h).
 *
 *  Works one bit at a time: simple, no shift by 32 (undefined in C),
 *  and fast enough for a 16-byte frame once per second.
 */

#include <string.h>
#include "bitpack.h"

/* true if nbits more bits fit and the stream has not already failed */
static bool bs_room(bitstream_t const *p_bs, uint8_t nbits)
{
    bool b_ok = false;

    if ((false == p_bs->b_overflow) && (nbits > 0u) && (nbits <= 32u) &&
        (((uint32_t)p_bs->pos + nbits) <= p_bs->cap_bits))
    {
        b_ok = true;
    }

    return b_ok;
}

void bs_init_read(bitstream_t *p_bs, uint8_t *p_buf, uint16_t len_bytes)
{
    p_bs->p_buf      = p_buf;
    p_bs->cap_bits   = (uint16_t)(len_bytes * 8u);
    p_bs->pos        = 0u;
    p_bs->b_overflow = false;
}

void bs_init_write(bitstream_t *p_bs, uint8_t *p_buf, uint16_t len_bytes)
{
    (void)memset(p_buf, 0, len_bytes);
    bs_init_read(p_bs, p_buf, len_bytes);
}

void bs_put(bitstream_t *p_bs, uint32_t value, uint8_t nbits)
{
    uint8_t i;

    if (false == bs_room(p_bs, nbits))
    {
        p_bs->b_overflow = true;    /* write nothing, keep pos */
    }
    else
    {
        for (i = 0u; i < nbits; i++)
        {
            if ((value & 1u) != 0u)
            {
                p_bs->p_buf[p_bs->pos / 8u] |= (uint8_t)(1u << (p_bs->pos % 8u));
            }
            value >>= 1;
            p_bs->pos++;
        }
    }
}

uint32_t bs_get(bitstream_t *p_bs, uint8_t nbits)
{
    uint32_t value = 0u;
    uint8_t  i;

    if (false == bs_room(p_bs, nbits))
    {
        p_bs->b_overflow = true;    /* return 0, keep pos */
    }
    else
    {
        for (i = 0u; i < nbits; i++)
        {
            if ((p_bs->p_buf[p_bs->pos / 8u] & (uint8_t)(1u << (p_bs->pos % 8u))) != 0u)
            {
                value |= ((uint32_t)1u << i);
            }
            p_bs->pos++;
        }
    }

    return value;
}

int32_t bs_get_signed(bitstream_t *p_bs, uint8_t nbits)
{
    uint32_t raw = bs_get(p_bs, nbits);

    /* sign-extend: if the top bit of the field is 1, set all higher bits */
    if ((nbits > 0u) && (nbits < 32u) &&
        ((raw & ((uint32_t)1u << (nbits - 1u))) != 0u))
    {
        raw |= ~(((uint32_t)1u << nbits) - 1u);
    }

    return (int32_t)raw;
}

uint16_t bs_bytes_used(bitstream_t const *p_bs)
{
    return (uint16_t)((p_bs->pos + 7u) / 8u);
}