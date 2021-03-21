#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct CircBuf {
    uint8_t data[64];
    uint8_t writeIndex;
    uint8_t readIndex;
} CircBuf;

inline static bool circbuf_is_empty(CircBuf* buf)
{
    return buf->writeIndex == buf->readIndex;
}

inline static void circbuf_append(CircBuf* buf, uint8_t byte)
{
    uint8_t nextWriteIndex = buf->writeIndex + 1;

    if (nextWriteIndex >= sizeof(buf->data)) {
        nextWriteIndex = 0;
    }

    // Append the received byte if the buffer has room
    if (nextWriteIndex != buf->readIndex) {
        buf->data[buf->writeIndex] = byte;
        buf->writeIndex = nextWriteIndex;
    }
}

inline static uint8_t circbuf_pop(CircBuf* buf)
{
    const uint8_t value = buf->data[buf->readIndex];

    uint8_t nextReadIndex = buf->readIndex + 1;
    if (nextReadIndex >= sizeof(buf->data)) {
        nextReadIndex = 0;
    }

    buf->readIndex = nextReadIndex;

    return value;
}