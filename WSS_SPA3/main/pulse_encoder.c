/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_check.h"
#include "pulse_encoder.h"
#include "driver/gpio.h"

#define WSS_RESOLUTION_HZ 1000000 // 1MHz resolution, 1 tick = 1us 

int _Tp = 50;  // microseconds

static const char *TAG = "pulse_encoder";

typedef struct {
    rmt_encoder_t base;           // the base "class", declares the standard encoder interface
    rmt_encoder_t *copy_encoder;  // use the copy_encoder to encode the pulse, pause1, pause2

    rmt_symbol_word_t pulse_rem;  // Remainder of time, after all data bits/pause2, before next pulse
    int state;
    float period;
} rmt_pulse_encoder_t;

static size_t rmt_encode_pulse(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
{   
    rmt_pulse_encoder_t *pulse_encoder = __containerof(encoder, rmt_pulse_encoder_t, base);
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;

    rmt_encoder_handle_t copy_encoder = pulse_encoder->copy_encoder;
   
    switch (pulse_encoder->state) {

        // fall-through
        case 0: // send remainder
            encoded_symbols += copy_encoder->encode(copy_encoder, channel, &pulse_encoder->pulse_rem,
                                                    sizeof(rmt_symbol_word_t), &session_state);
            ESP_LOGI(TAG, "Complete");
            if (session_state & RMT_ENCODING_COMPLETE) {
                pulse_encoder->state = RMT_ENCODING_RESET; // back to the initial encoding session
                state |= RMT_ENCODING_COMPLETE;
            }
            if (session_state & RMT_ENCODING_MEM_FULL) {
                state |= RMT_ENCODING_MEM_FULL;
                goto out; // yield if there's no free space to put other encoding artifacts
            }
        }
        out:
            *ret_state = state;
            return encoded_symbols;
    }

static esp_err_t rmt_del_pulse_encoder(rmt_encoder_t *encoder)
{
    rmt_pulse_encoder_t *pulse_encoder = __containerof(encoder, rmt_pulse_encoder_t, base);
    rmt_del_encoder(pulse_encoder->copy_encoder);
    free(pulse_encoder);
    return ESP_OK;
}

static esp_err_t rmt_pulse_encoder_reset(rmt_encoder_t *encoder)
{
    rmt_pulse_encoder_t *pulse_encoder = __containerof(encoder, rmt_pulse_encoder_t, base);
    rmt_encoder_reset(pulse_encoder->copy_encoder);
    pulse_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

esp_err_t rmt_new_pulse_encoder(const pulse_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder)
{
    esp_err_t ret = ESP_OK;
    rmt_pulse_encoder_t *pulse_encoder = NULL;
    ESP_GOTO_ON_FALSE(config && ret_encoder, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    pulse_encoder = rmt_alloc_encoder_mem(sizeof(rmt_pulse_encoder_t));
    ESP_GOTO_ON_FALSE(pulse_encoder, ESP_ERR_NO_MEM, err, TAG, "no mem for encoder");
    pulse_encoder->base.encode = rmt_encode_pulse;
    pulse_encoder->base.del = rmt_del_pulse_encoder;
    pulse_encoder->base.reset = rmt_pulse_encoder_reset;

    pulse_encoder->period = config->period;

    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_GOTO_ON_ERROR(rmt_new_copy_encoder(&copy_encoder_config, &pulse_encoder->copy_encoder), err, TAG, "create copy encoder failed");
    
    // construct the various symbols in RMT symbol format
    float rem = config->period - _Tp;
    //ESP_LOGI(TAG, "Period: %f ; Tp: %d ; Remainder: %f", config->period, _Tp, rem);

    // Configure last RMT symbol with variable duration to make up remainder of pulsePeriod
    pulse_encoder->pulse_rem = (rmt_symbol_word_t) {
        .level0 = 1,
        .duration0 = 50 * WSS_RESOLUTION_HZ / 1000000,
        .level1 = 0,
        .duration1 = rem * WSS_RESOLUTION_HZ / 1000000,
    };

    *ret_encoder = &pulse_encoder->base;
    return ESP_OK;
err:
    if (pulse_encoder) {
        if (pulse_encoder->copy_encoder) {
            rmt_del_encoder(pulse_encoder->copy_encoder);
        }
        free(pulse_encoder);
    }
    return ret;
}
