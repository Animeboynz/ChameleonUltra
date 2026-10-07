#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "nrf_pwm.h"
#include "utils/fskdemod.h"

#define LF_PARADOX_TAG_ID_SIZE (12)
#define PARADOX_RAW_BITS       (96)

typedef enum {
    PARADOX_DEC_WAIT,
    PARADOX_DEC_COLLECT,
} paradox_dec_state_t;

typedef struct {
    uint8_t data[LF_PARADOX_TAG_ID_SIZE];
    fsk_t *modem;
    uint8_t pre;
    paradox_dec_state_t st;
    uint8_t n_bits;
    uint8_t bits[PARADOX_RAW_BITS];
} paradox_codec_t;

void paradox_decoder_start(paradox_codec_t *d);
bool paradox_decoder_feed(paradox_codec_t *d, uint16_t sample);
uint8_t *paradox_get_data(paradox_codec_t *d);

const nrf_pwm_sequence_t *paradox_modulator(paradox_codec_t *d, uint8_t *buf);

uint8_t paradox_t55xx_writer(uint8_t raw12[LF_PARADOX_TAG_ID_SIZE], uint32_t *blks);

/** Build 12-byte Paradox raw (T55xx blocks 1–3 wire image) from FC/CN; fc ≤ 999, cn ≤ 99999. */
bool paradox_encode_fc_cn(uint32_t fc, uint32_t cn, uint8_t raw12[LF_PARADOX_TAG_ID_SIZE]);

bool paradox_read(uint8_t *data, uint32_t timeout_ms);
