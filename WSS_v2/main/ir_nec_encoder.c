/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_check.h"
#include "ir_nec_encoder.h"
#include "driver/gpio.h"

#define WSS_RESOLUTION_HZ 1000000 // 1MHz resolution, 1 tick = 1us 
#define WSS_GPIO_0      0
#define WSS_GPIO_2      2
#define WSS_GPIO_4      4
#define WSS_GPIO_5      5
#define WSS_GPIO_15      15
#define WSS_GPIO_17      17
#define WSS_GPIO_18      18
#define WSS_GPIO_19      19

//int availDataBits = 9;
//float period = 2100.0;  // us

int Tp = 50;  // microseconds
float teeth = 48.0;
float wheelCirc = 1950.0;

static const char *TAG = "wss_encoder";

typedef struct {
    rmt_encoder_t base;           // the base "class", declares the standard encoder interface
    rmt_encoder_t *copy_encoder;  // use the copy_encoder to encode the pulse, pause1, pause2
    rmt_symbol_word_t wss_pulse_pause; // WSS Pulse and Pause1 combined, in RMT representation
    rmt_symbol_word_t wss_pause2;  // WSS Pause2 in RMT representation
    rmt_symbol_word_t wss_one;  // WSS ONE in RMT representation
    rmt_symbol_word_t wss_zero;  // WSS ZERO in RMT representation
    rmt_symbol_word_t wss_rem;  // Remainder of time, after all data bits/pause2, before next pulse
    int state;
    float period;
    int availDataBits;
    gpio_num_t pin2;
} rmt_wss_encoder_t;

static size_t rmt_encode_wss(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
{   
    rmt_wss_encoder_t *wss_encoder = __containerof(encoder, rmt_wss_encoder_t, base);
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;
    const uint8_t *data = (const uint8_t *)primary_data;
    rmt_encoder_handle_t copy_encoder = wss_encoder->copy_encoder;
   
    switch (wss_encoder->state) {

    case 0: // send start Pulse and Pause1
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &wss_encoder->wss_pulse_pause,
                                                sizeof(rmt_symbol_word_t), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {

            wss_encoder->state = 1; // we can only switch to next state when current encoder finished
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space to put other encoding artifacts
        }

   // fall-through
   case 1:  // send data bits
        
        for (int i=0; i<wss_encoder->availDataBits;i++){
            //ESP_LOGI(TAG, "Bit %d: %d", i, data[i]);
            ESP_LOGI(TAG, "Data: %d", data[i]);

            if (data[i] == 1){
                encoded_symbols += copy_encoder->encode(copy_encoder, channel, &wss_encoder->wss_one,
                                                sizeof(rmt_symbol_word_t), &session_state);
            }
            else if (data[i] == 0){
                encoded_symbols += copy_encoder->encode(copy_encoder, channel, &wss_encoder->wss_zero,
                                                sizeof(rmt_symbol_word_t), &session_state);
            }
        }
        // Might have to fix this - currently only checks after attempting to encode all bits, should prob check after each.
        // the encode function above gets a handle to &session_state and modifies it according to whether or not the encoding was successful,
        if (session_state & RMT_ENCODING_COMPLETE) {
            ESP_LOGI(TAG, "Case 1: Encode complete, %d symbols!", encoded_symbols);
            wss_encoder->state = 2; // we can only switch to next state when current encoder finished
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            ESP_LOGI(TAG, "Case 1: Mem full after %d symbols!", encoded_symbols);
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space to put other encoding artifacts
        }
    
    case 2: // send Pause2 if necessary
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &wss_encoder->wss_pause2,
                                                sizeof(rmt_symbol_word_t), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            wss_encoder->state = 3; // we can only switch to next state when current encoder finished
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space to put other encoding artifacts
        }

    // fall-through
    case 3: // send remainder
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &wss_encoder->wss_rem,
                                                sizeof(rmt_symbol_word_t), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            wss_encoder->state = RMT_ENCODING_RESET; // back to the initial encoding session
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

static esp_err_t rmt_del_wss_encoder(rmt_encoder_t *encoder)
{
    rmt_wss_encoder_t *wss_encoder = __containerof(encoder, rmt_wss_encoder_t, base);
    rmt_del_encoder(wss_encoder->copy_encoder);
    free(wss_encoder);
    return ESP_OK;
}

static esp_err_t rmt_wss_encoder_reset(rmt_encoder_t *encoder)
{
    rmt_wss_encoder_t *wss_encoder = __containerof(encoder, rmt_wss_encoder_t, base);
    rmt_encoder_reset(wss_encoder->copy_encoder);
    wss_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

esp_err_t rmt_new_wss_encoder(const wss_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder)
{
    esp_err_t ret = ESP_OK;
    rmt_wss_encoder_t *wss_encoder = NULL;
    ESP_GOTO_ON_FALSE(config && ret_encoder, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    wss_encoder = rmt_alloc_encoder_mem(sizeof(rmt_wss_encoder_t));
    ESP_GOTO_ON_FALSE(wss_encoder, ESP_ERR_NO_MEM, err, TAG, "no mem for ir nec encoder");
    wss_encoder->base.encode = rmt_encode_wss;
    wss_encoder->base.del = rmt_del_wss_encoder;
    wss_encoder->base.reset = rmt_wss_encoder_reset;

    wss_encoder->period = config->period;
    wss_encoder->pin2 = config->pin2;
    wss_encoder->availDataBits = config->availDataBits;

    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_GOTO_ON_ERROR(rmt_new_copy_encoder(&copy_encoder_config, &wss_encoder->copy_encoder), err, TAG, "create copy encoder failed");

    // construct the various symbols in RMT symbol format
    wss_encoder->wss_pulse_pause = (rmt_symbol_word_t) {
        .level0 = 1,
        .duration0 = 50 * WSS_RESOLUTION_HZ / 1000000, // HIGH for 50us - Pulse
        .level1 = 0,
        .duration1 = 25 * WSS_RESOLUTION_HZ / 1000000, // LOW for 25us - Pause1
    };

    wss_encoder->wss_pause2 = (rmt_symbol_word_t) {
        .level0 = 0,
        .duration0 = 12 * WSS_RESOLUTION_HZ / 1000000, //12
        .level1 = 0,
        .duration1 = 13 * WSS_RESOLUTION_HZ / 1000000, //13
    };
    
    wss_encoder->wss_zero = (rmt_symbol_word_t) {
        .level0 = 1,
        .duration0 = 25 * WSS_RESOLUTION_HZ / 1000000,
        .level1 = 0,
        .duration1 = 25 * WSS_RESOLUTION_HZ / 1000000,
    };
    wss_encoder->wss_one = (rmt_symbol_word_t) {
        .level0 = 0,
        .duration0 = 25 * WSS_RESOLUTION_HZ / 1000000,
        .level1 = 1,
        .duration1 = 25 * WSS_RESOLUTION_HZ / 1000000,
    };

    float rem = config->period / 2 - (config->availDataBits * Tp + 2 * Tp); // pulse + p1 + p2 = 2*Tp

    wss_encoder->wss_rem = (rmt_symbol_word_t) {
        .level0 = 0,
        .duration0 = rem/2.0 * WSS_RESOLUTION_HZ / 1000000,
        .level1 = 0,
        .duration1 = rem/2.0 * WSS_RESOLUTION_HZ / 1000000,
    };
    

    *ret_encoder = &wss_encoder->base;
    return ESP_OK;
err:
    if (wss_encoder) {
        if (wss_encoder->copy_encoder) {
            rmt_del_encoder(wss_encoder->copy_encoder);
        }
        free(wss_encoder);
    }
    return ret;
}
