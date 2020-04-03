#pragma once

// Macros to generate entries for UBX message payloads in a const uint8_t array
// Values are split out into LSB-first order (little endian) per the UBX protocol docs
// Signed variants exist only to help describe intention

// 8-bit values
#define UBX_VALUE_U8(x) (uint8_t)(x & 0xFF)
#define UBX_VALUE_S8(x) UBX_VALUE_U8(x)

// 16-bit values
#define UBX_VALUE_U16(x) (uint8_t)(x & 0xFF), (uint8_t)((x >> 8) & 0xFF)
#define UBX_VALUE_S16(x) UBX_VALUE_U16(x)

// 32-bit values
#define UBX_VALUE_U32(x) (uint8_t)(x & 0xFF), (uint8_t)((x >> 8) & 0xFF), (uint8_t)((x >> 16) & 0xFF), (uint8_t)((x >> 24) & 0xFF)
#define UBX_VALUE_S32(x) UBX_VALUE_U32(x)