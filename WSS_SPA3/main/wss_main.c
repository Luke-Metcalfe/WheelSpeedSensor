#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/ledc.h"
#include "hal/ledc_types.h"

#include "data_encoder.h"
#include "pulse_encoder.h"

#include "hal/gpio_ll.h"
#include "driver/gpio.h"
//#include "components/driver/rmt/rmt_private.h"
//#include "rmt_private.h"

#include "wheels.h"

#define WSS_RESOLUTION_HZ 1000000 // 1MHz resolution, 1 tick = 1us 
static const char *TAG = "wss_main";

int numChannels = 4;

void app_main(void)
{
    
    // *******************************************************************************************************************
    ESP_LOGI(TAG, "Create RMT TX channel");

    for (int i = 0; i < numChannels; i++) {

        rmt_tx_channel_config_t tx_chan_config_data = {
            .gpio_num = wheelArray[i]->pin1,    
            .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
            .resolution_hz = WSS_RESOLUTION_HZ,
            .mem_block_symbols = 64, // increase the block size can make the LED less flickering
            .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
        };
        ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config_data, &wheelArray[i]->data_chan));
        
        rmt_tx_channel_config_t tx_chan_config_pulse = {
            .gpio_num = wheelArray[i]->pin2,
            .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock  
            .resolution_hz = WSS_RESOLUTION_HZ,
            .mem_block_symbols = 64, // increase the block size can make the LED less flickering
            .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
        };
        ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config_pulse, &wheelArray[i]->pulse_chan));
        
    }

    // Send Signal Frames in a loop
    rmt_transmit_config_t transmit_config = {
        .loop_count = -1, // -1 loop, 0 no loop
    };

    ESP_LOGI(TAG, "Install WSS DATA Encoders");

    for (int i = 0; i < numChannels; i++) {
        data_encoder_config_t tempConfig = {
            .resolution = WSS_RESOLUTION_HZ,
            .period = wheelArray[i]->pulsePeriod,
            .availDataBits = wheelArray[i]->availDataBits,
        };
        wheelArray[i]->data_encoder_config = tempConfig;
        ESP_ERROR_CHECK(rmt_new_data_encoder(&wheelArray[i]->data_encoder_config, &wheelArray[i]->data_encoder));
    }

    ESP_LOGI(TAG, "Install WSS PULSE Encoders");

    for (int i = 0; i < numChannels; i++) {
        pulse_encoder_config_t tempConfig = {
            .resolution = WSS_RESOLUTION_HZ,
            .period = wheelArray[i]->pulsePeriod,
        };
        wheelArray[i]->pulse_encoder_config = tempConfig;
        ESP_ERROR_CHECK(rmt_new_pulse_encoder(&wheelArray[i]->pulse_encoder_config, &wheelArray[i]->pulse_encoder));
    }

    ESP_LOGI(TAG, "Enable RMT TX channels");
    for (int i = 0; i < numChannels; i++) {
        ESP_ERROR_CHECK(rmt_enable(wheelArray[i]->data_chan));
        ESP_ERROR_CHECK(rmt_enable(wheelArray[i]->pulse_chan));
    }

    ESP_LOGI(TAG, "Setup Transmission");
    // Modification - This will set up transmission, but not start it.
    for (int i = 0; i < numChannels; i++) {
        ESP_ERROR_CHECK(rmt_transmit(wheelArray[i]->data_chan, wheelArray[i]->data_encoder, &wheelArray[i]->serialData, sizeof(wheelArray[i]->serialData), &transmit_config));
        ESP_ERROR_CHECK(rmt_transmit(wheelArray[i]->pulse_chan, wheelArray[i]->pulse_encoder, &wheelArray[i]->serialData, sizeof(wheelArray[i]->serialData), &transmit_config));
    }

    ESP_LOGI(TAG, "Start Transmission");
    // Custom Function to start transmission of pair in quick succession
    for (int i = 0; i < numChannels; i++) {
        rmt_tx_start(wheelArray[i]->data_chan, wheelArray[i]->pulse_chan);
        vTaskDelay(10);
    }

    while(true){
        vTaskDelay(1000);
    }

}
