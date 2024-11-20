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


#define WSS_RESOLUTION_HZ 1000000 // 1MHz resolution, 1 tick = 1us 
static const char *TAG = "example";

int numChannels = 4;

//——————————————————————————————————————————————————————————————————————————————
//  Wheel Structures
//——————————————————————————————————————————————————————————————————————————————
// ---------------------------- Wheel Struct Definition
typedef struct {
    gpio_num_t pin1;  // WSS Data
    gpio_num_t pin2;  // WSS Pulse
    gpio_num_t pin3;  // WSS Frequency

    float wheelSpeed;    // Wheel speed in kph
    float toothFreq;     // Frequency in Hz of sensor gear
    float pulsePeriod;   // Period between start pulses -> Double the tooth freq, since we send a pulse on each gear tooth transition
    int availDataBits;   // Number of data bits that will be able to be transmitted between pulse periods
    uint8_t sensorType;  // 0:Normal | 1:Directional
    uint8_t serialData[9];

    ledc_channel_t ledc_chan; // LEDC channel (for PWM out)
    ledc_timer_t ledc_timer;  // Timer for LEDC channel

    rmt_channel_handle_t data_chan;
    rmt_channel_handle_t pulse_chan;

    rmt_encoder_handle_t data_encoder;
    data_encoder_config_t data_encoder_config;

    rmt_encoder_handle_t pulse_encoder;
    pulse_encoder_config_t pulse_encoder_config;

} Wheel;

// ---------------------------- Wheel Object Declarations/Initialisations
Wheel FL = {
    .pin1 = GPIO_NUM_15,
    .pin2 = GPIO_NUM_2,
    .pin3 = GPIO_NUM_27,
    .wheelSpeed = 101.0,
    .sensorType = 0,
    .serialData = { 1, 1, 0, 1, 1, 0, 1, 0, 1 },

    .pulsePeriod = 800.0, // microseconds (us)
    .availDataBits = 9, // Number of databits to send

    .ledc_chan = LEDC_CHANNEL_0,
    .ledc_timer = LEDC_TIMER_0,
    .data_chan = NULL,
    .pulse_chan = NULL,

    .data_encoder = NULL,
    .pulse_encoder = NULL,
};

Wheel FR = {
    .pin1 = GPIO_NUM_0,
    .pin2 = GPIO_NUM_4,
    .pin3 = GPIO_NUM_26,

    .wheelSpeed = 102.0,
    .sensorType = 0,
    .serialData = { 1, 1, 0, 1, 1, 0, 1, 0, 1 },

    .pulsePeriod = 800.0, // microseconds (us)
    .availDataBits = 8, // Number of databits to send

    .ledc_chan = LEDC_CHANNEL_2,
    .ledc_timer = LEDC_TIMER_1,

    .data_chan = NULL,
    .pulse_chan = NULL,

    .data_encoder = NULL,
    .pulse_encoder = NULL,
};

Wheel RL = {
    .pin1 = GPIO_NUM_17,
    .pin2 = GPIO_NUM_5,
    .pin3 = GPIO_NUM_25,

    .wheelSpeed = 103.0,
    .sensorType = 0,
    .serialData = { 1, 1, 0, 1, 1, 0, 1, 0, 1 },

    .pulsePeriod = 800.0, // microseconds (us)
    .availDataBits = 5, // Number of databits to send

    .ledc_chan = LEDC_CHANNEL_4,
    .ledc_timer = LEDC_TIMER_2,

    .data_chan = NULL,
    .pulse_chan = NULL,

    .data_encoder = NULL,
    .pulse_encoder = NULL,
};

Wheel RR = {
    .pin1 = GPIO_NUM_12,
    .pin2 = GPIO_NUM_14,
    .pin3 = GPIO_NUM_33,

    .wheelSpeed = 104.0,
    .sensorType = 0,
    .serialData = { 1, 1, 0, 1, 1, 0, 1, 0, 1 },

    .pulsePeriod = 800.0, // microseconds (us)
    .availDataBits = 4, // Number of databits to send

    .ledc_chan = LEDC_CHANNEL_6,
    .ledc_timer = LEDC_TIMER_3,

    .data_chan = NULL,
    .pulse_chan = NULL,

    .data_encoder = NULL,
    .pulse_encoder = NULL,
};

// ---------------------------- Wheel Object Array
Wheel* wheelArray[4] = { &FL, &FR, &RL, &RR };  // array of pointers to wheel objects


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
        //vTaskDelay(500);
        ESP_LOGI(TAG, "Enable Channel %d", i);
        rmt_tx_start(wheelArray[i]->data_chan, wheelArray[i]->pulse_chan);
        vTaskDelay(10);
    }

    while(true){
        vTaskDelay(1000);
    }

}
