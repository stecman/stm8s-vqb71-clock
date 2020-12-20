#!/usr/bin/env python3

print("Debugging tool for parsing digit data sent to the MAX7221")
print("Enter 8 digit bytes sent to the MAX7221 as a hex string to see the displayed characters")

# Map of segments to displayed character
bcd_map = {
    0b00111111: "0",
    0b00000110: "1",
    0b01011011: "2",
    0b01001111: "3",
    0b01100110: "4",
    0b01101101: "5",
    0b01111101: "6",
    0b00000111: "7",
    0b01111111: "8",
    0b01101111: "9",
    0b01000000: "-",
    0b01111001: "E",
    0b01110110: "H",
    0b00111000: "L",
    0b01110011: "P",
    0b00000000: " ",
}

# Map of logical digit index to rev 1.0 board wiring using a MAX7221
# This allows unscrambling of the out-of-order wiring of digits
digit_order_map = {
    0: 0,
    4: 1,
    3: 2,
    1: 3,
    5: 4,
    2: 5,
};

while True:
    raw = input('> ')
    data = bytes.fromhex(raw)

    if len(data) != 8:
        print("Data must be 8 bytes! %d given." % len(data))
        continue

    digits = bytearray([0, 0, 0, 0, 0, 0, 0, 0])

    for segmentIndex in range(8):
        for digitIndex in range(8):
            if data[segmentIndex] & (1 << digitIndex) != 0:
                input_index = digit_order_map[digitIndex]
                digits[input_index] |= (1 << segmentIndex)

    for digit in digits:
        print(bcd_map[digit], end="")

    print("")