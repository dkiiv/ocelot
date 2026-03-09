#include "can_fuzzer_helpers.h"
#include <stdlib.h>   /* rand() - swap for hardware RNG if preferred */

/* ------------------------------------------------------------------ */
/* Global rolling counter, shared across every message that has one.  */
/* ------------------------------------------------------------------ */
static uint8_t g_counter = 0;

/* ------------------------------------------------------------------ */

void fill_random(uint8_t *data, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        data[i] = (uint8_t)(rand() & 0xFF);
    }
}

void tick_counter(void)
{
    g_counter = (g_counter + 1) & 0x0F;
}

void insert_counter(uint8_t *data, uint8_t byte_idx, uint8_t shift)
{
    const uint8_t mask = (uint8_t)(0x0F << shift);

    /* clear the nibble, then stamp the current counter value - no increment */
    data[byte_idx] = (data[byte_idx] & (uint8_t)~mask)
                   | ((g_counter & 0x0F) << shift);
}

void insert_checksum(uint8_t *data, uint8_t len,
                     uint16_t can_id, uint8_t checksum_byte)
{
    uint8_t sum = (uint8_t)(can_id & 0xFF)
                + (uint8_t)((can_id >> 8) & 0xFF);

    for (uint8_t i = 0; i < len; i++) {
        if (i != checksum_byte) {
            sum += data[i];
        }
    }

    data[checksum_byte] = sum & 0xFF;
}