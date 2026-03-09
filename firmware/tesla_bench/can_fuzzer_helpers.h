#include <stdint.h>

/**
 * Fill `len` bytes of `data` with pseudo-random values.
 * Replace rand() with your hardware RNG call if available
 * (e.g. HAL_RNG_GenerateRandomNumber on STM32).
 */
void fill_random(uint8_t *data, uint8_t len);

/**
 * Advance the global counter by one, wrapping 0->15.
 * Call this ONCE per ISR cycle, before sending any messages.
 * All insert_counter() calls within that cycle will then stamp
 * the same counter value into every message.
 */
void tick_counter(void);

/**
 * Write the current global counter (0-15) into one nibble of data[byte_idx].
 * Does NOT advance the counter - call tick_counter() once per cycle instead.
 *
 *   shift = 0  ->  low  nibble  (bits [3:0])
 *                  used when COUNTER start_bit % 8 == 0   (e.g. bit 8  -> byte 1 low)
 *
 *   shift = 4  ->  high nibble  (bits [7:4])
 *                  used when COUNTER start_bit % 8 == 4   (e.g. bit 52 -> byte 6 high)
 */
void insert_counter(uint8_t *data, uint8_t byte_idx, uint8_t shift);

/**
 * Compute and write the Tesla-style additive checksum.
 *
 *   sum = (can_id & 0xFF) + ((can_id >> 8) & 0xFF)
 *       + every data byte EXCEPT data[checksum_byte]
 *   data[checksum_byte] = sum & 0xFF
 *
 * Always call this LAST (after fill_random and insert_counter) so
 * that the counter value is already included in the sum.
 *
 *   can_id        - 11-bit CAN ID of this message
 *   len           - DBC-declared payload length in bytes
 *   checksum_byte - byte index where the checksum is written
 */
void insert_checksum(uint8_t *data, uint8_t len,
                     uint16_t can_id, uint8_t checksum_byte);