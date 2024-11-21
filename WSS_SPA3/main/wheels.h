#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/ledc.h"
#include "hal/ledc_types.h"

#include "data_encoder.h"
#include "pulse_encoder.h"

#include "hal/gpio_ll.h"
#include "driver/gpio.h"

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
    

    .pulsePeriod = 800.0, // microseconds (us)
    .availDataBits = 9, // Number of databits to send

    .sensorType = 0,
    .serialData = { 1, 1, 0, 1, 1, 0, 1, 0, 1 },

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

    .pulsePeriod = 800.0, // microseconds (us)
    .availDataBits = 8, // Number of databits to send
    
    .sensorType = 0,
    .serialData = { 1, 1, 0, 1, 1, 0, 1, 0, 1 },

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

    .pulsePeriod = 800.0, // microseconds (us)
    .availDataBits = 5, // Number of databits to send

    .sensorType = 0,
    .serialData = { 1, 1, 0, 1, 1, 0, 1, 0, 1 },

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

    .pulsePeriod = 800.0, // microseconds (us)
    .availDataBits = 4, // Number of databits to send

    .sensorType = 0,
    .serialData = { 1, 1, 0, 1, 1, 0, 1, 0, 1 },
    
    .ledc_chan = LEDC_CHANNEL_6,
    .ledc_timer = LEDC_TIMER_3,

    .data_chan = NULL,
    .pulse_chan = NULL,

    .data_encoder = NULL,
    .pulse_encoder = NULL,
};

// ---------------------------- Wheel Object Array
Wheel* wheelArray[4] = { &FL, &FR, &RL, &RR };  // array of pointers to wheel objects