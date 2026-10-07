#include "paradox.h"

#include <stdlib.h>
#include <string.h>

#include "hex_utils.h"
#include "protocols/t55xx.h"

#define LF_FSK2a_PWM_LO_FREQ_LOOP (5)
#define LF_FSK2a_PWM_LO_FREQ_TOP_VALUE (10)
#define LF_FSK2a_PWM_HI_FREQ_LOOP (6)
#define LF_FSK2a_PWM_HI_FREQ_TOP_VALUE (8)

static nrf_pwm_values_wave_form_t m_paradox_pwm_seq_vals[PARADOX_RAW_BITS * 6] = {};

static nrf_pwm_sequence_t m_paradox_pwm_seq = {
    .values.p_wave_form = m_paradox_pwm_seq_vals,
    .length = NRF_PWM_VALUES_LENGTH(m_paradox_pwm_seq_vals),
    .repeats = 0,
    .end_delay = 0,
};

// From Proxmark3 common/lfdemod.c (manchesterEncode2Bytes)
static uint32_t manchester_encode_2bytes(uint16_t datain) {
    uint32_t output = 0;
    for (uint8_t i = 0; i < 16; i++) {
        uint8_t b = (uint8_t)((datain >> (15 - i)) & 1);
        output |= (1UL << (((15 - i) * 2) + b));
    }
    return output;
}

// CRC-8/MAXIM (Proxmark3 common/crc.c)
static uint8_t crc8_maxim(const uint8_t *buff, size_t size) {
    uint8_t crc = 0;
    for (size_t i = 0; i < size; i++) {
        crc ^= buff[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (uint8_t)((crc >> 1) ^ 0x8C);
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// Proxmark3 client/src/cmdlfparadox.c GetParadoxBits (blocks[1..3] = data words)
static uint8_t paradox_get_paradox_bits(uint32_t fc, uint32_t cn, uint32_t *blocks) {
    uint8_t manchester[13] = {0};
    uint32_t t1;

    manchester[0] = 0x0F;
    manchester[1] = 0x05;
    manchester[2] = 0x55;
    manchester[3] = 0x55;

    t1 = manchester_encode_2bytes((uint16_t)fc);
    manchester[4] = (uint8_t)((t1 >> 8) & 0xFF);
    manchester[5] = (uint8_t)(t1 & 0xFF);

    t1 = manchester_encode_2bytes((uint16_t)cn);
    manchester[6] = (uint8_t)((t1 >> 24) & 0xFF);
    manchester[7] = (uint8_t)((t1 >> 16) & 0xFF);
    manchester[8] = (uint8_t)((t1 >> 8) & 0xFF);
    manchester[9] = (uint8_t)(t1 & 0xFF);

    uint8_t crc = (uint8_t)((crc8_maxim(manchester + 1, 9) ^ 0x6U) & 0xFFU);

    t1 = manchester_encode_2bytes(crc);
    manchester[10] = (uint8_t)((t1 >> 8) & 0xFF);
    manchester[11] = (uint8_t)(t1 & 0xFF);

    for (int i = 1; i < 12; i++) {
        manchester[i] = (uint8_t)((manchester[i] << 4) + (manchester[i + 1] >> 4));
    }

    manchester[11] |= (1U << 3);
    manchester[11] |= (1U << 1);

    blocks[1] = 0;
    blocks[2] = 0;
    blocks[3] = 0;
    for (int i = 0; i < 12; i++) {
        blocks[1 + (i / 4)] += ((uint32_t)manchester[i]) << (8 * (3 - (i % 4)));
    }
    return crc;
}

bool paradox_encode_fc_cn(uint32_t fc, uint32_t cn, uint8_t raw12[LF_PARADOX_TAG_ID_SIZE]) {
    if (fc > 999u || cn > 99999u) {
        return false;
    }
    uint32_t blocks[4] = {0};
    paradox_get_paradox_bits(fc, cn, blocks);
    for (uint8_t i = 1; i < 4; i++) {
        num_to_bytes(blocks[i], sizeof(uint32_t), raw12 + ((i - 1) * 4));
    }
    return true;
}

void paradox_decoder_start(paradox_codec_t *d) {
    memset(d->data, 0, sizeof(d->data));
    d->pre = 0;
    d->st = PARADOX_DEC_WAIT;
    d->n_bits = 0;
    if (d->modem == NULL) {
        d->modem = fsk_alloc(FSK_BITRATE_HID);
    }
}

bool paradox_decoder_feed(paradox_codec_t *d, uint16_t val) {
    bool bit = false;
    if (!fsk_feed(d->modem, val, &bit)) {
        return false;
    }

    if (d->st == PARADOX_DEC_WAIT) {
        d->pre = (uint8_t)((d->pre << 1) | (bit ? 1U : 0U));
        if (d->pre == 0x0FU) {
            d->st = PARADOX_DEC_COLLECT;
            d->n_bits = 0;
            for (uint8_t i = 0; i < 8; i++) {
                d->bits[i] = (uint8_t)((d->pre >> (7 - i)) & 1);
            }
            d->n_bits = 8;
        }
        return false;
    }

    if (d->st == PARADOX_DEC_COLLECT) {
        if (d->n_bits >= PARADOX_RAW_BITS) {
            d->st = PARADOX_DEC_WAIT;
            d->pre = 0;
            d->n_bits = 0;
            return false;
        }
        d->bits[d->n_bits++] = bit ? 1U : 0U;
        if (d->n_bits < PARADOX_RAW_BITS) {
            return false;
        }
        for (uint8_t i = 0, m = 0, p = 1; i < PARADOX_RAW_BITS; i++) {
            d->data[m] <<= 1;
            d->data[m] |= d->bits[i];
            if (p == 8) {
                m++;
                p = 1;
            } else {
                p++;
            }
        }
        d->st = PARADOX_DEC_WAIT;
        d->pre = 0;
        d->n_bits = 0;
        return true;
    }

    return false;
}

uint8_t *paradox_get_data(paradox_codec_t *d) {
    return d->data;
}

const nrf_pwm_sequence_t *paradox_modulator(paradox_codec_t *d, uint8_t *buf) {
    (void)d;
    int k = 0;
    for (int i = 0; i < PARADOX_RAW_BITS; i++) {
        bool b = (buf[i / 8] >> (7 - (i % 8))) & 1;
        if (!b) {
            for (int j = 0; j < LF_FSK2a_PWM_HI_FREQ_LOOP; j++) {
                m_paradox_pwm_seq_vals[k].channel_0 = LF_FSK2a_PWM_HI_FREQ_TOP_VALUE / 2;
                m_paradox_pwm_seq_vals[k].counter_top = LF_FSK2a_PWM_HI_FREQ_TOP_VALUE;
                k++;
            }
        } else {
            for (int j = 0; j < LF_FSK2a_PWM_LO_FREQ_LOOP; j++) {
                m_paradox_pwm_seq_vals[k].channel_0 = LF_FSK2a_PWM_LO_FREQ_TOP_VALUE / 2;
                m_paradox_pwm_seq_vals[k].counter_top = LF_FSK2a_PWM_LO_FREQ_TOP_VALUE;
                k++;
            }
        }
    }
    m_paradox_pwm_seq.length = (uint16_t)(k * 4);
    return &m_paradox_pwm_seq;
}

uint8_t paradox_t55xx_writer(uint8_t raw12[LF_PARADOX_TAG_ID_SIZE], uint32_t *blks) {
    blks[0] = T5577_PARADOX_CONFIG;
    blks[1] = (uint32_t)bytes_to_num(raw12, 4);
    blks[2] = (uint32_t)bytes_to_num(raw12 + 4, 4);
    blks[3] = (uint32_t)bytes_to_num(raw12 + 8, 4);
    return 4;
}
