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
#define INDALA_CLOCK 32
#define INDALA_WIN_BYTES (INDALA_BIT_WINDOW / 8)

// Thirty-two bit alignments are tracked together. The SAADC value is a
// unipolar ripple, so the bit is the sign of a 32-sample correlation with
// the RF/2 subcarrier, not a per-sample zero crossing.
typedef struct {
    uint8_t data[INDALA_DATA_SIZE];
    uint8_t confirm[INDALA_DATA_SIZE];
    bool has_confirm;
    int32_t mean_q8;
    int32_t run_sum;
    int32_t hist[INDALA_CLOCK];
    uint8_t win[INDALA_CLOCK][INDALA_WIN_BYTES];
    uint8_t nbits[INDALA_CLOCK];
    uint16_t filled;
    uint32_t n;
} indala_codec;

extern const protocol indala;

uint8_t indala_t55xx_writer(uint8_t *raw, uint32_t *blks);

// fc is 8 bits, cn is 16 bits. fmt_4041x selects the 4041X parity layout.
// raw is the 8-byte frame, MSB first, matching Proxmark3 getIndalaBits().
bool indala_encode_fc_cn(uint8_t fc, uint16_t cn, bool fmt_4041x, uint8_t raw[INDALA_DATA_SIZE]);
