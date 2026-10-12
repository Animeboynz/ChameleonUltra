#include "indala.h"

#include <stdlib.h>
#include <string.h>

#include "nrf_pwm.h"
#include "parity.h"
#include "t55xx.h"
#include "tag_base_type.h"
#include "utils/psk1.h"

/*
 * Indala 64-bit. PSK1 at RF/32, subcarrier RF/2.
 *
 * The 33-bit preamble is bit 0 = 1, bit 2 = 1, bit 32 = 1, and the rest of
 * the first 33 bits clear. Facility and card bits are scattered by the
 * Proxmark3 getIndalaBits() / getIndalaBits4041x() maps.
 *
 * The reader samples the op-amp once per 125 kHz carrier cycle. PSK1 with
 * an RF/2 subcarrier is a high/low ripple on that stream. Each bit is the
 * sign of a 32-sample correlation against that ripple. Proxmark3 builds the
 * same waveform in pskSimBit() with carrier 2 and clock 32, and the phase
 * during a bit equals the data bit (either polarity).
 */

#define INDALA_PWM_ENTRIES       (INDALA_BIT_COUNT * LF_PSK1_RF32_SUBCYCLES_PER_BIT)
#define INDALA_T55XX_BLOCK_COUNT (3)

static nrf_pwm_values_wave_form_t m_indala_pwm_seq_vals[INDALA_PWM_ENTRIES] = {};

static nrf_pwm_sequence_t m_indala_pwm_seq = {
    .values.p_wave_form = m_indala_pwm_seq_vals,
    .length = NRF_PWM_VALUES_LENGTH(m_indala_pwm_seq_vals),
    .repeats = 0,
    .end_delay = 0,
};

static void indala_decoder_reset(indala_codec *d) {
    d->mean_q8 = 0;
    d->run_sum = 0;
    d->filled = 0;
    d->n = 0;
    d->has_confirm = false;
    memset(d->confirm, 0, sizeof(d->confirm));
    memset(d->hist, 0, sizeof(d->hist));
    memset(d->win, 0, sizeof(d->win));
    memset(d->nbits, 0, sizeof(d->nbits));
}

static indala_codec *indala_alloc(void) {
    indala_codec *d = malloc(sizeof(indala_codec));
    if (d == NULL) {
        return NULL;
    }
    memset(d, 0, sizeof(*d));
    indala_decoder_reset(d);
    return d;
}

static void indala_free(indala_codec *d) {
    free(d);
}

static uint8_t *indala_get_data(indala_codec *d) {
    return d->data;
}

static void indala_decoder_start(indala_codec *d, uint8_t format) {
    (void)format;
    memset(d->data, 0, INDALA_DATA_SIZE);
    indala_decoder_reset(d);
}

// 33-bit Indala preamble: 1, 0, 1, then 29 zeros, then 1.
static bool indala_preamble_at(const uint8_t *bits, uint8_t invert) {
    if ((bits[0] ^ invert) != 1 || (bits[1] ^ invert) != 0 || (bits[2] ^ invert) != 1) {
        return false;
    }
    if ((bits[32] ^ invert) != 1) {
        return false;
    }
    for (int i = 3; i < 32; i++) {
        if ((bits[i] ^ invert) != 0) {
            return false;
        }
    }
    return true;
}

static void indala_pack_frame(indala_codec *d, const uint8_t *bits, uint8_t invert) {
    memset(d->data, 0, INDALA_DATA_SIZE);
    for (int i = 0; i < INDALA_BIT_COUNT; i++) {
        if (bits[i] ^ invert) {
            d->data[i / 8] |= (uint8_t)(1u << (7 - (i % 8)));
        }
    }
}

static uint8_t indala_bit_at(const uint8_t *win, int pos) {
    return (uint8_t)((win[pos / 8] >> (7 - (pos % 8))) & 1u);
}

// A frame is only returned after the same 8 bytes show up twice. The preamble
// is the only check, and one noisy alignment can stumble into it.
static bool indala_accept(indala_codec *d) {
    if (d->has_confirm && memcmp(d->confirm, d->data, INDALA_DATA_SIZE) == 0) {
        return true;
    }
    memcpy(d->confirm, d->data, INDALA_DATA_SIZE);
    d->has_confirm = true;
    return false;
}

static bool indala_push_bit(indala_codec *d, int lane, uint8_t bit) {
    uint8_t *win = d->win[lane];
    uint8_t carry = bit;
    for (int i = INDALA_WIN_BYTES - 1; i >= 0; i--) {
        uint8_t next = (win[i] & 0x80u) ? 1u : 0u;
        win[i] = (uint8_t)((win[i] << 1) | carry);
        carry = next;
    }
    if (d->nbits[lane] < INDALA_BIT_WINDOW) {
        d->nbits[lane]++;
    }
    if (d->nbits[lane] < INDALA_BIT_COUNT) {
        return false;
    }

    // The new bit lands at the end of the window, so only the 64-bit slice
    // that ends there is new. Older slices were checked on earlier bits.
    uint8_t frame[INDALA_BIT_COUNT];
    int start = INDALA_BIT_WINDOW - INDALA_BIT_COUNT;
    for (int i = 0; i < INDALA_BIT_COUNT; i++) {
        frame[i] = indala_bit_at(win, start + i);
    }
    for (uint8_t invert = 0; invert < 2; invert++) {
        if (indala_preamble_at(frame, invert)) {
            indala_pack_frame(d, frame, invert);
            if (indala_accept(d)) {
                return true;
            }
        }
    }
    return false;
}

static bool indala_decoder_feed(indala_codec *d, uint16_t val) {
    int32_t sample = (int16_t)val;
    if (d->filled == 0 && d->n == 0) {
        d->mean_q8 = sample << 8;
    }
    d->mean_q8 += ((sample << 8) - d->mean_q8) >> 4;
    int32_t hp = sample - (d->mean_q8 >> 8);
    int32_t weight = ((d->n & 1u) == 0u) ? hp : -hp;

    uint32_t slot = d->n % INDALA_CLOCK;
    d->run_sum += weight - d->hist[slot];
    d->hist[slot] = weight;

    uint32_t n = d->n++;
    if (d->filled < INDALA_CLOCK) {
        d->filled++;
        return false;
    }

    uint32_t start = n - (INDALA_CLOCK - 1);
    int32_t score = ((start & 1u) == 0u) ? d->run_sum : -d->run_sum;
    return indala_push_bit(d, (int)(start % INDALA_CLOCK), (uint8_t)(score >= 0));
}

static const nrf_pwm_sequence_t *indala_modulator(indala_codec *d, uint8_t *buf) {
    (void)d;

    size_t n = lf_psk1_build_sequence(buf, INDALA_BIT_COUNT, m_indala_pwm_seq_vals, INDALA_PWM_ENTRIES);
    m_indala_pwm_seq.length = (uint16_t)(n * 4);
    return &m_indala_pwm_seq;
}

const protocol indala = {
    .tag_type = TAG_TYPE_INDALA,
    .data_size = INDALA_DATA_SIZE,
    .alloc = (codec_alloc)indala_alloc,
    .free = (codec_free)indala_free,
    .get_data = (codec_get_data)indala_get_data,
    .modulator = (modulator)indala_modulator,
    .decoder =
        {
            .start = (decoder_start)indala_decoder_start,
            .feed = (decoder_feed)indala_decoder_feed,
        },
};

uint8_t indala_t55xx_writer(uint8_t *raw, uint32_t *blks) {
    uint32_t hi = 0;
    uint32_t lo = 0;
    for (int i = 0; i < 4; i++) {
        hi = (hi << 8) | raw[i];
    }
    for (int i = 4; i < 8; i++) {
        lo = (lo << 8) | raw[i];
    }
    blks[0] = T5577_INDALA_CONFIG;
    blks[1] = hi;
    blks[2] = lo;
    return INDALA_T55XX_BLOCK_COUNT;
}

static void indala_pack(const uint8_t bits[INDALA_BIT_COUNT], uint8_t raw[INDALA_DATA_SIZE]) {
    memset(raw, 0, INDALA_DATA_SIZE);
    for (int i = 0; i < INDALA_BIT_COUNT; i++) {
        if (bits[i]) {
            raw[i / 8] |= (uint8_t)(1u << (7 - (i % 8)));
        }
    }
}

// Proxmark3 client/src/cmdlfindala.c getIndalaBits(). The bit array must
// start cleared; that function only writes the bits it owns.
static void indala_bits_standard(uint8_t fc, uint16_t cn, uint8_t bits[INDALA_BIT_COUNT]) {
    bits[0] = 1;
    bits[2] = 1;
    bits[32] = 1;

    bits[57] = (uint8_t)((fc >> 7) & 1);
    bits[49] = (uint8_t)((fc >> 6) & 1);
    bits[44] = (uint8_t)((fc >> 5) & 1);
    bits[47] = (uint8_t)((fc >> 4) & 1);
    bits[48] = (uint8_t)((fc >> 3) & 1);
    bits[53] = (uint8_t)((fc >> 2) & 1);
    bits[39] = (uint8_t)((fc >> 1) & 1);
    bits[58] = (uint8_t)(fc & 1);

    bits[42] = (uint8_t)((cn >> 15) & 1);
    bits[45] = (uint8_t)((cn >> 14) & 1);
    bits[43] = (uint8_t)((cn >> 13) & 1);
    bits[40] = (uint8_t)((cn >> 12) & 1);
    bits[52] = (uint8_t)((cn >> 11) & 1);
    bits[36] = (uint8_t)((cn >> 10) & 1);
    bits[35] = (uint8_t)((cn >> 9) & 1);
    bits[51] = (uint8_t)((cn >> 8) & 1);
    bits[46] = (uint8_t)((cn >> 7) & 1);
    bits[33] = (uint8_t)((cn >> 6) & 1);
    bits[37] = (uint8_t)((cn >> 5) & 1);
    bits[54] = (uint8_t)((cn >> 4) & 1);
    bits[56] = (uint8_t)((cn >> 3) & 1);
    bits[59] = (uint8_t)((cn >> 2) & 1);
    bits[50] = (uint8_t)((cn >> 1) & 1);
    bits[41] = (uint8_t)(cn & 1);

    uint8_t chk = 0;
    chk = (uint8_t)(chk + ((cn >> 14) & 1));
    chk = (uint8_t)(chk + ((cn >> 12) & 1));
    chk = (uint8_t)(chk + ((cn >> 9) & 1));
    chk = (uint8_t)(chk + ((cn >> 8) & 1));
    chk = (uint8_t)(chk + ((cn >> 6) & 1));
    chk = (uint8_t)(chk + ((cn >> 5) & 1));
    chk = (uint8_t)(chk + ((cn >> 2) & 1));
    chk = (uint8_t)(chk + (cn & 1));
    if ((chk & 1) == 0) {
        bits[62] = 1;
        bits[63] = 0;
    } else {
        bits[62] = 0;
        bits[63] = 1;
    }

    bits[34] = 0;
    bits[38] = 0;
    uint8_t p1 = 1;
    uint8_t p2 = 1;
    for (int i = 33; i < 64; i++) {
        if (i % 2) {
            p1 ^= bits[i];
        } else {
            p2 ^= bits[i];
        }
    }
    bits[34] = p1;
    bits[38] = p2;
}

// Proxmark3 getIndalaBits4041x(). Parity replaces the standard checksum.
static void indala_bits_4041x(uint8_t fc, uint16_t cn, uint8_t bits[INDALA_BIT_COUNT]) {
    bits[0] = 1;
    bits[2] = 1;
    bits[32] = 1;
    bits[40] = 1;
    bits[55] = 1;

    bits[57] = (uint8_t)((fc >> 7) & 1);
    bits[49] = (uint8_t)((fc >> 6) & 1);
    bits[44] = (uint8_t)((fc >> 5) & 1);
    bits[47] = (uint8_t)((fc >> 4) & 1);
    bits[48] = (uint8_t)((fc >> 3) & 1);
    bits[53] = (uint8_t)((fc >> 2) & 1);
    bits[39] = (uint8_t)((fc >> 1) & 1);
    bits[58] = (uint8_t)(fc & 1);

    bits[42] = (uint8_t)((cn >> 15) & 1);
    bits[45] = (uint8_t)((cn >> 14) & 1);
    bits[43] = (uint8_t)((cn >> 13) & 1);
    bits[40] = (uint8_t)((cn >> 12) & 1);
    bits[52] = (uint8_t)((cn >> 11) & 1);
    bits[36] = (uint8_t)((cn >> 10) & 1);
    bits[35] = (uint8_t)((cn >> 9) & 1);
    bits[51] = (uint8_t)((cn >> 8) & 1);
    bits[46] = (uint8_t)((cn >> 7) & 1);
    bits[33] = (uint8_t)((cn >> 6) & 1);
    bits[37] = (uint8_t)((cn >> 5) & 1);
    bits[54] = (uint8_t)((cn >> 4) & 1);
    bits[56] = (uint8_t)((cn >> 3) & 1);
    bits[59] = (uint8_t)((cn >> 2) & 1);
    bits[50] = (uint8_t)((cn >> 1) & 1);
    bits[41] = (uint8_t)(cn & 1);

    bits[34] = evenparity32(((uint32_t)fc << 4) | ((uint32_t)cn >> 12));
    bits[38] = oddparity32((uint32_t)cn & 0x0fffu);
}

bool indala_encode_fc_cn(uint8_t fc, uint16_t cn, bool fmt_4041x, uint8_t raw[INDALA_DATA_SIZE]) {
    uint8_t bits[INDALA_BIT_COUNT];
    memset(bits, 0, sizeof(bits));
    if (fmt_4041x) {
        indala_bits_4041x(fc, cn, bits);
    } else {
        indala_bits_standard(fc, cn, bits);
    }
    indala_pack(bits, raw);
    return true;
}
