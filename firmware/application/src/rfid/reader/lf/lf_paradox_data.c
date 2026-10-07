#include <stdlib.h>
#include <string.h>

#include "bsp_delay.h"
#include "bsp_time.h"
#include "circular_buffer.h"
#include "lf_125khz_radio.h"
#include "lf_reader_data.h"
#include "lf_reader_main.h"
#include "nrfx_saadc.h"
#include "protocols/paradox.h"
#include "fskdemod.h"

#define NRF_LOG_MODULE_NAME lf_paradox
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();

#define PARADOX_BUFFER_SIZE (6144)

static circular_buffer cb;

static void saadc_cb(nrf_saadc_value_t *vals, size_t size) {
    for (size_t i = 0; i < size; i++) {
        nrf_saadc_value_t val = vals[i];
        if (!cb_push_back(&cb, &val)) {
            return;
        }
    }
}

static void init_paradox_hw(void) {
    lf_125khz_radio_saadc_enable(saadc_cb);
}

static void uninit_paradox_hw(void) {
    lf_125khz_radio_saadc_disable();
}

bool paradox_read(uint8_t *data, uint32_t timeout_ms) {
    paradox_codec_t *codec = (paradox_codec_t *)calloc(1, sizeof(paradox_codec_t));
    if (codec == NULL) {
        return false;
    }
    paradox_decoder_start(codec);

    cb_init(&cb, PARADOX_BUFFER_SIZE, sizeof(uint16_t));
    init_paradox_hw();
    start_lf_125khz_radio();

    bool ok = false;
    autotimer *p_at = bsp_obtain_timer(0);
    while (!ok && NO_TIMEOUT_1MS(p_at, timeout_ms)) {
        uint16_t val = 0;
        while (!ok && NO_TIMEOUT_1MS(p_at, timeout_ms) && cb_pop_front(&cb, &val)) {
            if (paradox_decoder_feed(codec, val)) {
                memcpy(data, paradox_get_data(codec), LF_PARADOX_TAG_ID_SIZE);
                ok = true;
                break;
            }
        }
    }

    bsp_return_timer(p_at);
    stop_lf_125khz_radio();
    uninit_paradox_hw();
    cb_free(&cb);

    if (codec->modem) {
        fsk_free(codec->modem);
    }
    free(codec);
    return ok;
}
