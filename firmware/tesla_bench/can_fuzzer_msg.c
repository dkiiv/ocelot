#include "can_fuzzer_helpers.h"

void msg_send(uint32_t id, uint8_t *data, uint8_t len)
{
    CAN_FIFOMailBox_TypeDef to_send = {0};

    for (uint8_t i = 0U; i < len && i < 4U; i++) {
        to_send.RDLR |= ((uint32_t)data[i] << (i * 8U));
    }

    for (uint8_t i = 4U; i < len && i < 8U; i++) {
        to_send.RDHR |= ((uint32_t)data[i] << ((i - 4U) * 8U));
    }

    to_send.RDTR = len & 0x0FU;
    to_send.RIR = (id << 21U) | 1U;
    can_send(&to_send, 0, false);
}

/* ------------------------------------------------------------------
 * 0x3C2 | VCLEFT_switchStatus | 8 bytes
 * No CHECKSUM, no COUNTER - pure random fuzz.
 * ------------------------------------------------------------------ */
void send_MSG_0x3C2(void)
{
    uint8_t data[8];
    fill_random(data, 8);
    msg_send(0x3C2, data, 8);
}

/* ------------------------------------------------------------------
 * 0x103 | VCRIGHT_doorStatus | 8 bytes
 * No CHECKSUM, no COUNTER - pure random fuzz.
 * ------------------------------------------------------------------ */
void send_MSG_0x103(void)
{
    uint8_t data[8];
    fill_random(data, 8);
    msg_send(0x103, data, 8);
}

/* ------------------------------------------------------------------
 * 0x249 | SCCM_leftStalk | 3 bytes
 *   CHECKSUM : start_bit 0  -> byte 0 (full byte)
 *   COUNTER  : start_bit 8  -> byte 1, low nibble (shift 0)
 * ------------------------------------------------------------------ */
void send_MSG_0x249(void)
{
    uint8_t data[3];
    fill_random(data, 3);
    insert_counter(data, 1, 0);
    insert_checksum(data, 3, 0x249, 0);
    msg_send(0x249, data, 3);
}

/* ------------------------------------------------------------------
 * 0x118 | DI_systemStatus | 8 bytes
 *   CHECKSUM : start_bit 0  -> byte 0 (full byte)
 *   COUNTER  : start_bit 8  -> byte 1, low nibble (shift 0)
 * ------------------------------------------------------------------ */
void send_MSG_0x118(void)
{
    uint8_t data[8];
    fill_random(data, 8);
    insert_counter(data, 1, 0);
    insert_checksum(data, 8, 0x118, 0);
    msg_send(0x118, data, 8);
}

/* ------------------------------------------------------------------
 * 0x343 | VCRIGHT_status | 8 bytes
 * No CHECKSUM, no COUNTER - pure random fuzz.
 * ------------------------------------------------------------------ */
void send_MSG_0x343(void)
{
    uint8_t data[8];
    fill_random(data, 8);
    msg_send(0x343, data, 8);
}

/* ------------------------------------------------------------------
 * 0x229 | SCCM_rightStalk | 3 bytes
 *   CHECKSUM : start_bit 0  -> byte 0 (full byte)
 *   COUNTER  : start_bit 8  -> byte 1, low nibble (shift 0)
 * ------------------------------------------------------------------ */
void send_MSG_0x229(void)
{
    uint8_t data[3];
    fill_random(data, 3);
    insert_counter(data, 1, 0);
    insert_checksum(data, 3, 0x229, 0);
    msg_send(0x229, data, 3);
}

/* ------------------------------------------------------------------
 * 0x129 | SCCM_steeringAngleSensor | 8 bytes
 *   CHECKSUM : start_bit 0  -> byte 0 (full byte)
 *   COUNTER  : start_bit 8  -> byte 1, low nibble (shift 0)
 * ------------------------------------------------------------------ */
void send_MSG_0x129(void)
{
    uint8_t data[8];
    fill_random(data, 8);
    insert_counter(data, 1, 0);
    insert_checksum(data, 8, 0x129, 0);
    msg_send(0x129, data, 8);
}

/* ------------------------------------------------------------------
 * 0x102 | VCLEFT_doorStatus | 8 bytes
 * No CHECKSUM, no COUNTER - pure random fuzz.
 * ------------------------------------------------------------------ */
void send_MSG_0x102(void)
{
    uint8_t data[8];
    fill_random(data, 8);
    msg_send(0x102, data, 8);
}

/* ------------------------------------------------------------------
 * 0x238 | STW_ACTN_RQ | 8 bytes
 *   CHECKSUM : start_bit 56 -> byte 7 (full byte)
 *   No COUNTER signal. MC_STW_ACTN_RQ (bit 52) is fuzzed randomly.
 * ------------------------------------------------------------------ */
void send_MSG_0x238(void)
{
    uint8_t data[8];
    fill_random(data, 8);
    insert_checksum(data, 8, 0x238, 7);
    msg_send(0x238, data, 8);
}

/* ------------------------------------------------------------------
 * 0x3E9 | DAS_bodyControls | 8 bytes
 *   COUNTER  : start_bit 52 -> byte 6, high nibble (shift 4)
 *   CHECKSUM : start_bit 56 -> byte 7 (full byte)
 * ------------------------------------------------------------------ */
void send_MSG_0x3E9(void)
{
    uint8_t data[8];
    fill_random(data, 8);
    insert_counter(data, 6, 4);
    insert_checksum(data, 8, 0x3E9, 7);
    msg_send(0x3E9, data, 8);
}

/* ------------------------------------------------------------------
 * 0x3F5 | ID3F5VCFRONT_lighting | 8 bytes
 * No CHECKSUM, no COUNTER - pure random fuzz.
 * ------------------------------------------------------------------ */
void send_MSG_0x3F5(void)
{
    uint8_t data[8];
    fill_random(data, 8);
    msg_send(0x3F5, data, 8);
}