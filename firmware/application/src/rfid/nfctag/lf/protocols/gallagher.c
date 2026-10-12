#include "gallagher.h"

#include <stdlib.h>
#include <string.h>

#include "hex_utils.h"
#include "nrf_pwm.h"
#include "t55xx.h"
#include "tag_base_type.h"
#include "utils/manchester.h"

/*
 * Gallagher LF. ASK/Manchester, RF/32, 96 bits.
 *
 *   16-bit preamble 0111111111101010 (0x7FEA)
 *   8 data bytes, each followed by the inverse of that byte's LSB
 *   CRC-8/CARDX over the 8 data bytes (poly 0x07, init 0x2C)
 *
 * The 8 data bytes are the scrambled credential block from Proxmark3
 * gallagher_encode_creds(). Edge timing matches Viking (also RF/32).
 */

#define GALLAGHER_RAW_BITS (96)
#define GALLAGHER_T55XX_BLOCK_COUNT (4)

#define GALLAGHER_READ_TIME1_BASE (0x20)
#define GALLAGHER_READ_TIME2_BASE (0x30)
#define GALLAGHER_READ_TIME3_BASE (0x40)
#define GALLAGHER_READ_JITTER_TIME_BASE (0x07)

static nrf_pwm_values_wave_form_t m_gallagher_pwm_seq_vals[GALLAGHER_RAW_BITS] = {};

static nrf_pwm_sequence_t const m_gallagher_pwm_seq = {
    .values.p_wave_form = m_gallagher_pwm_seq_vals,
    .length = NRF_PWM_VALUES_LENGTH(m_gallagher_pwm_seq_vals),
    .repeats = 0,
    .end_delay = 0,
};

typedef struct {
    uint8_t data[GALLAGHER_DATA_SIZE];
    uint8_t window[GALLAGHER_DATA_SIZE];
    uint8_t n_bits;
    manchester *modem;
} gallagher_codec;

static const uint8_t gallagher_scramble_lut[256] = {
    0xa3, 0xb0, 0x80, 0xc6, 0xb2, 0xf4, 0x5c, 0x6c, 0x81, 0xf1, 0xbb, 0xeb, 0x55, 0x67, 0x3c, 0x05,
    0x1a, 0x0e, 0x61, 0xf6, 0x22, 0xce, 0xaa, 0x8f, 0xbd, 0x3b, 0x1f, 0x5e, 0x44, 0x04, 0x51, 0x2e,
    0x4d, 0x9a, 0x84, 0xea, 0xf8, 0x66, 0x74, 0x29, 0x7f, 0x70, 0xd8, 0x31, 0x7a, 0x6d, 0xa4, 0x00,
    0x82, 0xb9, 0x5f, 0xb4, 0x16, 0xab, 0xff, 0xc2, 0x39, 0xdc, 0x19, 0x65, 0x57, 0x7c, 0x20, 0xfa,
    0x5a, 0x49, 0x13, 0xd0, 0xfb, 0xa8, 0x91, 0x73, 0xb1, 0x33, 0x18, 0xbe, 0x21, 0x72, 0x48, 0xb6,
    0xdb, 0xa0, 0x5d, 0xcc, 0xe6, 0x17, 0x27, 0xe5, 0xd4, 0x53, 0x42, 0xf3, 0xdd, 0x7b, 0x24, 0xac,
    0x2b, 0x58, 0x1e, 0xa7, 0xe7, 0x86, 0x40, 0xd3, 0x98, 0x97, 0x71, 0xcb, 0x3a, 0x0f, 0x01, 0x9b,
    0x6e, 0x1b, 0xfc, 0x34, 0xa6, 0xda, 0x07, 0x0c, 0xae, 0x37, 0xca, 0x54, 0xfd, 0x26, 0xfe, 0x0a,
    0x45, 0xa2, 0x2a, 0xc4, 0x12, 0x0d, 0xf5, 0x4f, 0x69, 0xe0, 0x8a, 0x77, 0x60, 0x3f, 0x99, 0x95,
    0xd2, 0x38, 0x36, 0x62, 0xb7, 0x32, 0x7e, 0x79, 0xc0, 0x46, 0x93, 0x2f, 0xa5, 0xba, 0x5b, 0xaf,
    0x52, 0x1d, 0xc3, 0x75, 0xcf, 0xd6, 0x4c, 0x83, 0xe8, 0x3d, 0x30, 0x4e, 0xbc, 0x08, 0x2d, 0x09,
    0x06, 0xd9, 0x25, 0x9e, 0x89, 0xf2, 0x96, 0x88, 0xc1, 0x8c, 0x94, 0x0b, 0x28, 0xf0, 0x47, 0x63,
    0xd5, 0xb3, 0x68, 0x56, 0x9c, 0xf9, 0x6f, 0x41, 0x50, 0x85, 0x8b, 0x9d, 0x59, 0xbf, 0x9f, 0xe2,
    0x8e, 0x6a, 0x11, 0x23, 0xa1, 0xcd, 0xb5, 0x7d, 0xc7, 0xa9, 0xc8, 0xef, 0xdf, 0x02, 0xb8, 0x03,
    0x6b, 0x35, 0x3e, 0x2c, 0x76, 0xc9, 0xde, 0x1c, 0x4b, 0xd1, 0xed, 0x14, 0xc5, 0xad, 0xe9, 0x64,
    0x4a, 0xec, 0x8d, 0xf7, 0x10, 0x43, 0x78, 0x15, 0x87, 0xe4, 0xd7, 0x92, 0xe1, 0xee, 0xe3, 0x90,
};

static uint8_t gallagher_bit(const uint8_t *raw, int pos) {
    return (uint8_t)((raw[pos / 8] >> (7 - (pos % 8))) & 1u);
}

static void gallagher_set_bit(uint8_t *raw, int pos, uint8_t bit) {
    uint8_t mask = (uint8_t)(1u << (7 - (pos % 8)));
    if (bit) {
        raw[pos / 8] |= mask;
    } else {
        raw[pos / 8] &= (uint8_t)~mask;
    }
}

static void gallagher_set_bits(uint8_t *raw, int *pos, uint32_t value, int len) {
    for (int i = len - 1; i >= 0; i--) {
        gallagher_set_bit(raw, *pos, (uint8_t)((value >> i) & 1u));
        (*pos)++;
    }
}

// CRC-8/CARDX: poly 0x07, init 0x2C, no reflection, xorout 0.
static uint8_t crc8_cardx(const uint8_t *buff, size_t size) {
    uint8_t crc = 0x2C;
    for (size_t i = 0; i < size; i++) {
        crc ^= buff[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static bool gallagher_raw_valid(const uint8_t raw[GALLAGHER_DATA_SIZE]) {
    if (raw[0] != 0x7F || raw[1] != 0xEA) {
        return false;
    }

    uint8_t data[8];
    for (int i = 0; i < 8; i++) {
        int pos = 16 + (9 * i);
        uint8_t byte = 0;
        for (int b = 0; b < 8; b++) {
            byte = (uint8_t)((byte << 1) | gallagher_bit(raw, pos + b));
        }
        uint8_t parity = gallagher_bit(raw, pos + 8);
        if (parity != (uint8_t)((byte & 1u) ^ 1u)) {
            return false;
        }
        data[i] = byte;
    }

    uint8_t crc = 0;
    for (int b = 0; b < 8; b++) {
        crc = (uint8_t)((crc << 1) | gallagher_bit(raw, 88 + b));
    }
    return crc == crc8_cardx(data, sizeof(data));
}

static void gallagher_build_raw(const uint8_t data[8], uint8_t raw[GALLAGHER_DATA_SIZE]) {
    memset(raw, 0, GALLAGHER_DATA_SIZE);
    int pos = 0;
    gallagher_set_bits(raw, &pos, 0x7FEA, 16);
    for (int i = 0; i < 8; i++) {
        gallagher_set_bits(raw, &pos, data[i], 8);
        gallagher_set_bits(raw, &pos, (uint32_t)((data[i] & 1u) ^ 1u), 1);
    }
    gallagher_set_bits(raw, &pos, crc8_cardx(data, 8), 8);
}

bool gallagher_encode_creds(uint8_t region, uint16_t facility, uint32_t card, uint8_t issue, uint8_t raw12[GALLAGHER_DATA_SIZE]) {
    if (region > 0x0F || card > 0x00FFFFFF || issue > 0x0F) {
        return false;
    }

    uint8_t arr[8];
    arr[0] = (uint8_t)((card & 0xFFFFFF) >> 16);
    arr[1] = (uint8_t)((facility & 0xFFF) >> 4);
    arr[2] = (uint8_t)((card & 0x7FF) >> 3);
    arr[3] = (uint8_t)(((card & 0x7) << 5) | ((region & 0x0F) << 1));
    arr[4] = (uint8_t)((card & 0xFFFF) >> 11);
    arr[5] = (uint8_t)((facility & 0xFFFF) >> 12);
    arr[6] = 0;
    arr[7] = (uint8_t)(((facility & 0x0F) << 4) | (issue & 0x0F));
    for (int i = 0; i < 8; i++) {
        arr[i] = gallagher_scramble_lut[arr[i]];
    }
    gallagher_build_raw(arr, raw12);
    return true;
}

static bool gallagher_get_time(uint8_t interval, uint8_t base) {
    return interval >= (base - GALLAGHER_READ_JITTER_TIME_BASE) &&
           interval <= (base + GALLAGHER_READ_JITTER_TIME_BASE);
}

static uint8_t gallagher_period(uint8_t interval) {
    if (gallagher_get_time(interval, GALLAGHER_READ_TIME1_BASE)) {
        return 0;
    }
    if (gallagher_get_time(interval, GALLAGHER_READ_TIME2_BASE)) {
        return 1;
    }
    if (gallagher_get_time(interval, GALLAGHER_READ_TIME3_BASE)) {
        return 2;
    }
    return 3;
}

static gallagher_codec *gallagher_alloc(void) {
    gallagher_codec *codec = malloc(sizeof(gallagher_codec));
    if (codec == NULL) {
        return NULL;
    }
    codec->modem = malloc(sizeof(manchester));
    if (codec->modem == NULL) {
        free(codec);
        return NULL;
    }
    codec->modem->rp = gallagher_period;
    return codec;
}

static void gallagher_free(gallagher_codec *d) {
    if (d == NULL) {
        return;
    }
    free(d->modem);
    free(d);
}

static uint8_t *gallagher_get_data(gallagher_codec *d) {
    return d->data;
}

static void gallagher_decoder_start(gallagher_codec *d, uint8_t format) {
    (void)format;
    memset(d->data, 0, sizeof(d->data));
    memset(d->window, 0, sizeof(d->window));
    d->n_bits = 0;
    manchester_reset(d->modem);
}

static bool gallagher_push_bit(gallagher_codec *d, bool bit) {
    uint8_t carry = bit ? 1u : 0u;
    for (int i = GALLAGHER_DATA_SIZE - 1; i >= 0; i--) {
        uint8_t next = (d->window[i] & 0x80u) ? 1u : 0u;
        d->window[i] = (uint8_t)((d->window[i] << 1) | carry);
        carry = next;
    }
    if (d->n_bits < GALLAGHER_RAW_BITS) {
        d->n_bits++;
        if (d->n_bits < GALLAGHER_RAW_BITS) {
            return false;
        }
    }
    if (!gallagher_raw_valid(d->window)) {
        return false;
    }
    memcpy(d->data, d->window, GALLAGHER_DATA_SIZE);
    return true;
}

static bool gallagher_feed_bits(gallagher_codec *d, const bool *bits, int8_t bitlen) {
    for (int i = 0; i < bitlen; i++) {
        if (gallagher_push_bit(d, bits[i])) {
            return true;
        }
    }
    return false;
}

static bool gallagher_decoder_feed(gallagher_codec *d, uint16_t interval) {
    bool bits[2] = {0};
    int8_t bitlen = 0;

    // Same edge-merge workaround as Viking: a missed short gap shows up as one long interval.
    if (interval > (uint16_t)(GALLAGHER_READ_TIME3_BASE + GALLAGHER_READ_JITTER_TIME_BASE)) {
        interval = (uint16_t)(interval - GALLAGHER_READ_TIME2_BASE);
        manchester_feed(d->modem, GALLAGHER_READ_TIME2_BASE, bits, &bitlen);
        if (bitlen == -1) {
            memset(d->window, 0, sizeof(d->window));
            d->n_bits = 0;
            return false;
        }
        if (gallagher_feed_bits(d, bits, bitlen)) {
            return true;
        }
    }

    manchester_feed(d->modem, (uint8_t)interval, bits, &bitlen);
    if (bitlen == -1) {
        memset(d->window, 0, sizeof(d->window));
        d->n_bits = 0;
        return false;
    }
    return gallagher_feed_bits(d, bits, bitlen);
}

static const nrf_pwm_sequence_t *gallagher_modulator(gallagher_codec *d, uint8_t *buf) {
    (void)d;
    for (int i = 0; i < GALLAGHER_RAW_BITS; i++) {
        uint16_t msb = gallagher_bit(buf, i) ? (uint16_t)(1u << 15) : 0;
        m_gallagher_pwm_seq_vals[i].channel_0 = msb | 16;
        m_gallagher_pwm_seq_vals[i].counter_top = 32;
    }
    return &m_gallagher_pwm_seq;
}

const protocol gallagher = {
    .tag_type = TAG_TYPE_GALLAGHER,
    .data_size = GALLAGHER_DATA_SIZE,
    .alloc = (codec_alloc)gallagher_alloc,
    .free = (codec_free)gallagher_free,
    .get_data = (codec_get_data)gallagher_get_data,
    .modulator = (modulator)gallagher_modulator,
    .decoder = {
        .start = (decoder_start)gallagher_decoder_start,
        .feed = (decoder_feed)gallagher_decoder_feed,
    },
};

uint8_t gallagher_t55xx_writer(uint8_t *raw12, uint32_t *blks) {
    blks[0] = T5577_GALLAGHER_CONFIG;
    blks[1] = (uint32_t)bytes_to_num(raw12, 4);
    blks[2] = (uint32_t)bytes_to_num(raw12 + 4, 4);
    blks[3] = (uint32_t)bytes_to_num(raw12 + 8, 4);
    return GALLAGHER_T55XX_BLOCK_COUNT;
}
