/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_check.h"
#include "data_encoder.h"
#include "driver/gpio.h"

#define SIG_RESOLUTION_HZ 1000000 // 1MHz resolution, 1 tick = 1us 

int Tp = 50;  // microseconds

static const char *TAG = "data_encoder";

typedef struct {
    rmt_encoder_t base;           // the base "class", declares the standard encoder interface
    rmt_encoder_t *copy_encoder;  // use the copy_encoder to encode the pulse, pause1, pause2
    rmt_symbol_word_t data_pulse_pause; // Pulse and Pause1 combined, in RMT representation
    rmt_symbol_word_t data_pause2;  // Pause2 in RMT representation
    rmt_symbol_word_t data_one;  // ONE in RMT representation
    rmt_symbol_word_t data_zero;  // ZERO in RMT representation
    rmt_symbol_word_t data_rem;  // Remainder of time, after all data bits/pause2, before next pulse
    int state;
    float period;
    int availDataBits;
} rmt_data_encoder_t;

static size_t rmt_encode_data(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
{   
    rmt_data_encoder_t *data_encoder = __containerof(encoder, rmt_data_encoder_t, base);
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;
    const uint8_t *data = (const uint8_t *)primary_data;
    rmt_encoder_handle_t copy_encoder = data_encoder->copy_encoder;
   
    switch (data_encoder->state) {

    case 0: // send start Pulse and Pause1
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &data_encoder->data_pulse_pause,
                                                sizeof(rmt_symbol_word_t), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {

            data_encoder->state = 1; // we can only switch to next state when current encoder finished
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space to put other encoding artifacts
        }

   // fall-through
   case 1:  // send data bits
        
        for (int i=0; i<data_encoder->availDataBits;i++){
            //ESP_LOGI(TAG, "Bit %d: %d", i, data[i]);
           // ESP_LOGI(TAG, "Data: %d", data[i]);

            if (data[i] == 1){
                encoded_symbols += copy_encoder->encode(copy_encoder, channel, &data_encoder->data_one,
                                                sizeof(rmt_symbol_word_t), &session_state);
            }
            else if (data[i] == 0){
                encoded_symbols += copy_encoder->encode(copy_encoder, channel, &data_encoder->data_zero,
                                                sizeof(rmt_symbol_word_t), &session_state);
            }
        }
        // Might have to fix this - currently only checks after attempting to encode all bits, should prob check after each.
        // the encode function above gets a handle to &session_state and modifies it according to whether or not the encoding was successful,
        if (session_state & RMT_ENCODING_COMPLETE) {
            //ESP_LOGI(TAG, "Case 1: Encode complete, %d symbols!", encoded_symbols);
            data_encoder->state = 3; // we can only switch to next state when current encoder finished //EDIT - 2->3
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            //ESP_LOGI(TAG, "Case 1: Mem full after %d symbols!", encoded_symbols);
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space to put other encoding artifacts
        }
    /* //EDIT
    case 2: // send Pause2 if necessary
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &data_encoder->data_pause2,
                                                sizeof(rmt_symbol_word_t), &session_state);
        ESP_LOGI(TAG, "Case 2: Encoded Pause2, %d symbols!", encoded_symbols);
        if (session_state & RMT_ENCODING_COMPLETE) {
            data_encoder->state = 3; // we can only switch to next state when current encoder finished
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space to put other encoding artifacts
        }
*/
    // fall-through
    case 3: // send remainder
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &data_encoder->data_rem,
                                                sizeof(rmt_symbol_word_t), &session_state);
        //ESP_LOGI(TAG, "Case 3: Encoded Remainder, %d symbols!", encoded_symbols);
        if (session_state & RMT_ENCODING_COMPLETE) {
            data_encoder->state = RMT_ENCODING_RESET; // back to the initial encoding session
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

static esp_err_t rmt_del_data_encoder(rmt_encoder_t *encoder)
{
    rmt_data_encoder_t *data_encoder = __containerof(encoder, rmt_data_encoder_t, base);
    rmt_del_encoder(data_encoder->copy_encoder);
    free(data_encoder);
    return ESP_OK;
}

static esp_err_t rmt_data_encoder_reset(rmt_encoder_t *encoder)
{
    rmt_data_encoder_t *data_encoder = __containerof(encoder, rmt_data_encoder_t, base);
    rmt_encoder_reset(data_encoder->copy_encoder);
    data_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

esp_err_t rmt_new_data_encoder(const data_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder)
{
    esp_err_t ret = ESP_OK;
    rmt_data_encoder_t *data_encoder = NULL;
    ESP_GOTO_ON_FALSE(config && ret_encoder, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    data_encoder = rmt_alloc_encoder_mem(sizeof(rmt_data_encoder_t));
    ESP_GOTO_ON_FALSE(data_encoder, ESP_ERR_NO_MEM, err, TAG, "no mem for encoder");
    data_encoder->base.encode = rmt_encode_data;
    data_encoder->base.del = rmt_del_data_encoder;
    data_encoder->base.reset = rmt_data_encoder_reset;

    data_encoder->period = config->period;
    data_encoder->availDataBits = config->availDataBits;

    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_GOTO_ON_ERROR(rmt_new_copy_encoder(&copy_encoder_config, &data_encoder->copy_encoder), err, TAG, "create copy encoder failed");

    // construct the various symbols in RMT symbol format
    data_encoder->data_pulse_pause = (rmt_symbol_word_t) {
        .level0 = 1,
        .duration0 = 50 * SIG_RESOLUTION_HZ / 1000000, // HIGH for 50us - Pulse
        .level1 = 0,
        .duration1 = 25 * SIG_RESOLUTION_HZ / 1000000, // LOW for 25us - Pause1
    };

    data_encoder->data_pause2 = (rmt_symbol_word_t) {
        .level0 = 0,
        .duration0 = 12 * SIG_RESOLUTION_HZ / 1000000, //12
        .level1 = 0,
        .duration1 = 13 * SIG_RESOLUTION_HZ / 1000000, //13
    };
    
    data_encoder->data_zero = (rmt_symbol_word_t) {
        .level0 = 1,
        .duration0 = 25 * SIG_RESOLUTION_HZ / 1000000,
        .level1 = 0,
        .duration1 = 25 * SIG_RESOLUTION_HZ / 1000000,
    };
    data_encoder->data_one = (rmt_symbol_word_t) {
        .level0 = 0,
        .duration0 = 25 * SIG_RESOLUTION_HZ / 1000000,
        .level1 = 1,
        .duration1 = 25 * SIG_RESOLUTION_HZ / 1000000,
    };

    //float rem = config->period / 2 - (config->availDataBits * Tp + 2 * Tp); // pulse + p1 + p2 = 2*Tp
    //float rem = config->period - (config->availDataBits * Tp + 2 * Tp); // pulse + p1 + p2 = 2*Tp
    float rem = config->period - (config->availDataBits * Tp + 1.5 * Tp); // pulse + p1 = 1.5*Tp (no pause 2)
    ESP_LOGI(TAG, "Period: %f ; AvailBits: %d ; Tp: %d ; Remainder: %f", config->period, config->availDataBits, Tp, rem);

    // Configure last RMT symbol with variable duration to make up remainder of pulsePeriod
    data_encoder->data_rem = (rmt_symbol_word_t) {
        .level0 = 0,
        .duration0 = 10 * SIG_RESOLUTION_HZ / 1000000,
        .level1 = 0,
        .duration1 = (rem - 10) * SIG_RESOLUTION_HZ / 1000000,  // Use [10, (rem-10)] to avoid float divide issues with rem/2
    };

    *ret_encoder = &data_encoder->base;
    return ESP_OK;
err:
    if (data_encoder) {
        if (data_encoder->copy_encoder) {
            rmt_del_encoder(data_encoder->copy_encoder);
        }
        free(data_encoder);
    }
    return ret;
}
