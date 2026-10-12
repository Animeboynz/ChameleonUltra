#include "lf_reader_data.h"

#include <string.h>

#include "bsp_delay.h"
#include "bsp_time.h"
#include "circular_buffer.h"
#include "lf_125khz_radio.h"
#include "protocols/gallagher.h"

#define NRF_LOG_MODULE_NAME gallagher_reader
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();

#define GALLAGHER_BUFFER_SIZE (1024)

static circular_buffer cb;

static void gallagher_gpio_int0_cb(void) {
    uint32_t cntr = get_lf_counter_value();
    uint16_t val = cntr > 0xff ? 0xff : (uint16_t)(cntr & 0xff);
    cb_push_back(&cb, &val);
    clear_lf_counter_value();
}

static void init_gallagher_hw(void) {
    register_rio_callback(gallagher_gpio_int0_cb);
    lf_125khz_radio_gpiote_enable();
}

static void uninit_gallagher_hw(void) {
    lf_125khz_radio_gpiote_disable();
    unregister_rio_callback();
}

bool gallagher_read(uint8_t *data, uint32_t timeout_ms) {
    void *codec = gallagher.alloc();
    if (codec == NULL) {
        return false;
    }
    gallagher.decoder.start(codec, 0);

    cb_init(&cb, GALLAGHER_BUFFER_SIZE, sizeof(uint16_t));
    init_gallagher_hw();
    start_lf_125khz_radio();

    bool ok = false;
    autotimer *p_at = bsp_obtain_timer(0);
    while (!ok && NO_TIMEOUT_1MS(p_at, timeout_ms)) {
        uint16_t val = 0;
        while (!ok && NO_TIMEOUT_1MS(p_at, timeout_ms) && cb_pop_front(&cb, &val)) {
            if (gallagher.decoder.feed(codec, val)) {
                memcpy(data, gallagher.get_data(codec), gallagher.data_size);
                ok = true;
                break;
            }
        }
    }

    bsp_return_timer(p_at);
    stop_lf_125khz_radio();
    uninit_gallagher_hw();
    cb_free(&cb);

    gallagher.free(codec);
    return ok;
}
