#pragma once

#include "protocols.h"
#include "utils/fskdemod.h"

#define PARADOX_DATA_SIZE (12)
#define PARADOX_RAW_BITS  (96)

typedef struct {
    uint8_t data[PARADOX_DATA_SIZE];
    fsk_t *modem;
    uint8_t n_bits;
    uint8_t bits[PARADOX_RAW_BITS];
} paradox_codec;

extern const protocol paradox;

uint8_t paradox_t55xx_writer(uint8_t *raw12, uint32_t *blks);

/** Build 12-byte Paradox raw (T55xx blocks 1–3 wire image) from FC/CN; fc ≤ 999, cn ≤ 99999. */
bool paradox_encode_fc_cn(uint32_t fc, uint32_t cn, uint8_t raw12[PARADOX_DATA_SIZE]);
