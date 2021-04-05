#include "ubxgps.h"

#include "uart.h"

void ubx_update_checksum_multi(uint8_t* checksum, uint8_t* data, uint16_t length)
{
    for (uint16_t i = 0; i < length; ++i) {
        ubx_update_checksum(checksum, data[i]);
    }
}

enum UbxResponse ubx_send(uint8_t msgClass, uint8_t msgId, uint8_t* data, uint16_t length)
{
    // Send packet to receiver
    {
        uint8_t header[6] = {
            0xB5, 0x62, // Every message starts with these sync characters
            msgClass,
            msgId,
            (uint8_t) (length & 0xFF), // Payload length as little-endian (LSB first)
            (uint8_t) (length >> 8),
        };

        uint8_t checksum[2] = {0, 0};

        // Checksum includes the payload and the header minus its two fixed bytes
        ubx_update_checksum_multi(checksum, (uint8_t*)(&header) + 2, sizeof(header) - 2);
        ubx_update_checksum_multi(checksum, data, length);

        // Send the message over serial
        uart_send_stream_blocking(header, sizeof(header));
        uart_send_stream_blocking(data, length);
        uart_send_stream_blocking(checksum, sizeof(checksum));
    }

    // Look for receiver response
    // TODO: make this a more generic UBX packet reading routine that verifies checksum
    // TODO: handle response timeout. Currently this blocks forever if the GPS doesn't respond
    {
        const uint8_t response_header[] = {0xB5, 0x62, 0x05};

        uint8_t searchIndex = 0;
        enum UbxResponse response = kUbxBadResponse;

        // Wait for the ACK/NACK response header
        while (searchIndex < sizeof(response_header)) {
            const char byte = uart_read_byte();

            if (byte == response_header[searchIndex]) {
                ++searchIndex;
            }
        }

        // Read message ID as response
        response = uart_read_byte();

        // Discard packet length as we're not using it here
        uart_read_byte();
        uart_read_byte();

        if (uart_read_byte() == msgClass &&
            uart_read_byte() == msgId) {
            return response;
        } else {
            return kUbxBadResponse;
        }
    }
}