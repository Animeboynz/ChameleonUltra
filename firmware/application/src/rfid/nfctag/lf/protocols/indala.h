#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "protocols.h"

// 64-bit Indala. PSK1, RF/32, PSK carrier RF/2.
// 224-bit Indala is PSK2 and does not fit the 20-byte LF slot buffer.
#define INDALA_DATA_SIZE (8)
#define INDALA_BIT_COUNT (64)

// One carrier-cycle sample per bit-clock tick (RF/32). The sliding window
// holds a frame plus the long zero run in the preamble.
#define INDALA_BIT_WINDOW 96

typedef struct {
    uint8_t data[INDALA_DATA_SIZE];
    int32_t mean;
    int16_t count;
    uint8_t bits[INDALA_BIT_WINDOW];
    uint8_t nbits;
    uint8_t phase;
    int8_t last_sign;
    bool seeded;
    bool locked;
} indala_codec;

extern const protocol indala;

uint8_t indala_t55xx_writer(uint8_t *raw, uint32_t *blks);

// fc is 8 bits, cn is 16 bits. fmt_4041x selects the 4041X parity layout.
// raw is the 8-byte frame, MSB first, matching Proxmark3 getIndalaBits().
bool indala_encode_fc_cn(uint8_t fc, uint16_t cn, bool fmt_4041x, uint8_t raw[INDALA_DATA_SIZE]);
