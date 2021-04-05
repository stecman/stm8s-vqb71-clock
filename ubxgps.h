#pragma once

#include <stdint.h>

// Macros to generate entries for UBX message payloads in a const uint8_t array
// Values are split out into LSB-first order (little endian) per the UBX protocol docs
// Signed variants exist only to help describe intention

// 8-bit values
#define UBX_VALUE_U8(x) (uint8_t)(x & 0xFF)
#define UBX_VALUE_S8(x) UBX_VALUE_U8(x)

// 16-bit values
#define UBX_VALUE_U16(x) (uint8_t)(((uint16_t)x) & 0xFF), (uint8_t)((((uint16_t)x) >> 8) & 0xFF)
#define UBX_VALUE_S16(x) UBX_VALUE_U16(x)

// 32-bit values
#define UBX_VALUE_U32(x) (uint8_t)(((uint32_t)x) & 0xFF), (uint8_t)((((uint32_t)x) >> 8) & 0xFF), (uint8_t)((((uint32_t)x) >> 16) & 0xFF), (uint8_t)((((uint32_t)x) >> 24) & 0xFF)
#define UBX_VALUE_S32(x) UBX_VALUE_U32(x)


/**
 * Add a value to a checksum (8-Bit Fletcher Algorithm)
 * The initial checksum value must be {0,0}
 */
inline void ubx_update_checksum(uint8_t* checksum, uint8_t value)
{
    checksum[0] += value;
    checksum[1] += checksum[0];
}

/**
 * Calculate the checksum for an array of bytes
 */
void ubx_update_checksum_multi(uint8_t* checksum, uint8_t* data, uint16_t length);


enum UbxResponse {
    kUbxNack = 0, // Matches the NACK message ID
    kUbxAck = 1,  // Matches the ACK message ID
    kUbxResponseTimeout = 0x55,
    kUbxBadResponse = 0xBD
};

/**
 * Send a UBX format message over serial to the GPS module
 * This 
 */
enum UbxResponse ubx_send(uint8_t msgClass, uint8_t msgId, uint8_t* data, uint16_t length);