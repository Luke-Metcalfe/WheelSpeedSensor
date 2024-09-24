//——————————————————————————————————————————————————————————————————————————————
//  CAN Receive for WSS, with LEDC (IDF methods) integrated so far.
//——————————————————————————————————————————————————————————————————————————————

#include <ACAN2517FD.h>
#include <SPI.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "driver/ledc.h"
#include "hal/ledc_types.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"

#define WSS_RESOLUTION_HZ 1000000  // 1MHz resolution, 1 tick = 1us

//——————————————————————————————————————————————————————————————————————————————
// Wheel Speed Sensor Definitions
//——————————————————————————————————————————————————————————————————————————————
const int Tp = 50;  // microseconds
float teeth = 48.0;
float wheelCirc = 1950.0;

//——————————————————————————————————————————————————————————————————————————————
//  CAN Comms Definitions
//——————————————————————————————————————————————————————————————————————————————
// ---------------------------- CAN IDs and Filters
long inputID = 0x33;
tFrameFormat format = kExtended;  // kStandard | kExtended

// If Standard, address is limited up to 0x3F4
long boxID;       // CAN ID of Box, to be formatted correctly
long responseID;  // ID at which Box will respond
long filterID_1;  // Filter for changing speed in Hz
long filterID_2;  // Filter for changing speed in Kph

// ---------------------------- CAN Controller/Transceiver Pin Definitions
static const byte MCP2517_SCK = 18;   // SCK input of MCP2517FD // 26
static const byte MCP2517_MOSI = 23;  // SDI input of MCP2517FD // 19
static const byte MCP2517_MISO = 19;  // SDO output of MCP2517FD // 18

static const byte MCP2517_CS = 16;   // CS input of MCP2517FD // 16
static const byte MCP2517_INT = 32;  // INT output of MCP2517FD // 32

// ----------------------------  ACAN2517FD Driver object
ACAN2517FD can(MCP2517_CS, SPI, MCP2517_INT);

// ---------------------------- Other Pin Definitions
const int LED21 = 21;

//——————————————————————————————————————————————————————————————————————————————
//  PWM Definitions
//——————————————————————————————————————————————————————————————————————————————
// ESP32 can generate 16 independent PWM channels - 8 high speed, 8 low speed
// However, each set of 8 channels shares the resources of only 4 independent timers.
// Thus, adjacent channel numbers (0-1,2-3...) share a timer and will always have the same frequency.
// Thus, to have 4 independently varying frequencies, we need to use channels eg. 0,2,4,6

// The max duty cycle value based on PWM resolution (will be 255 if resolution is 8 bits)
const int PWM_RESOLUTION = 13;  // ESP32 can go up to 16 bits. If changed, must also update timer_config.duty_resolution !!
const int MAX_DUTY_CYCLE = (int)(pow(2, PWM_RESOLUTION) - 1);
const int MAX_HIGH_POINT = (int)(pow(2, PWM_RESOLUTION) - 1);

ledc_timer_config_t timer_config = {
  .speed_mode = LEDC_LOW_SPEED_MODE,
  .duty_resolution = LEDC_TIMER_13_BIT,  // Duty resolution in bits
  .timer_num = LEDC_TIMER_0,             // Initial value
  .freq_hz = 1000,                       // Initial value. 0 does not work here, channel will be turned off by updatePWM() later.
  .clk_cfg = LEDC_AUTO_CLK
};

ledc_channel_config_t channel_config = {
  .gpio_num = GPIO_NUM_0,  // Initial value
  .speed_mode = LEDC_LOW_SPEED_MODE,
  .channel = LEDC_CHANNEL_0,  // Initial value
  .intr_type = LEDC_INTR_DISABLE,
  .timer_sel = LEDC_TIMER_0,   // Initial value
  .duty = MAX_DUTY_CYCLE / 2,  // Set duty to 50%
  .hpoint = MAX_HIGH_POINT
};

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

  ledc_channel_t ledc_chan;  // LEDC channel (for PWM out)
  ledc_timer_t ledc_timer;   // Timer for LEDC channel

  rmt_channel_handle_t data_chan;
  rmt_channel_handle_t pulse_chan;
} Wheel;

// ---------------------------- Wheel Object Declarations/Initialisations
Wheel FL = {
  .pin1 = GPIO_NUM_4,
  .pin2 = GPIO_NUM_15,
  .pin3 = GPIO_NUM_33,
  .wheelSpeed = 101.0,
  .sensorType = 0,
  .serialData = { 1, 1, 0, 1, 1, 0, 1, 0, 1 },

  .ledc_chan = LEDC_CHANNEL_0,
  .ledc_timer = LEDC_TIMER_0,
  .data_chan = NULL,
  .pulse_chan = NULL,
};

Wheel FR = {
  .pin1 = GPIO_NUM_5,
  .pin2 = GPIO_NUM_19,
  .pin3 = GPIO_NUM_25,

  .wheelSpeed = 102.0,
  .sensorType = 0,
  .serialData = { 1, 1, 0, 1, 1, 0, 1, 0, 1 },

  .ledc_chan = LEDC_CHANNEL_2,
  .ledc_timer = LEDC_TIMER_1,

  .data_chan = NULL,
  .pulse_chan = NULL,
};

Wheel RL = {
  .pin1 = GPIO_NUM_0,
  .pin2 = GPIO_NUM_17,
  .pin3 = GPIO_NUM_26,

  .wheelSpeed = 103.0,
  .sensorType = 0,
  .serialData = { 1, 1, 0, 1, 1, 0, 1, 0, 1 },

  .ledc_chan = LEDC_CHANNEL_4,
  .ledc_timer = LEDC_TIMER_2,

  .data_chan = NULL,
  .pulse_chan = NULL,
};

Wheel RR = {
  .pin1 = GPIO_NUM_2,
  .pin2 = GPIO_NUM_18,
  .pin3 = GPIO_NUM_27,

  .wheelSpeed = 104.0,
  .sensorType = 0,
  .serialData = { 1, 1, 0, 1, 1, 0, 1, 0, 1 },

  .ledc_chan = LEDC_CHANNEL_6,
  .ledc_timer = LEDC_TIMER_3,

  .data_chan = NULL,
  .pulse_chan = NULL,
};

// ---------------------------- Wheel Object Array
Wheel* wheelArray[4] = { &FL, &FR, &RL, &RR };  // array of pointers to wheel objects

//——————————————————————————————————————————————————————————————————————————————
//  RMT Symbol Units
//——————————————————————————————————————————————————————————————————————————————

static const rmt_symbol_word_t wss_zero = {
  .duration0 = 25 * WSS_RESOLUTION_HZ / 1000000,
  .level0 = 1,
  .duration1 = 25 * WSS_RESOLUTION_HZ / 1000000,
  .level1 = 0,
};

static const rmt_symbol_word_t wss_one = {
  .duration0 = 25 * WSS_RESOLUTION_HZ / 1000000,
  .level0 = 0,
  .duration1 = 25 * WSS_RESOLUTION_HZ / 1000000,
  .level1 = 1,
};

static const rmt_symbol_word_t wss_pulse = {
  .duration0 = 25 * WSS_RESOLUTION_HZ / 1000000,
  .level0 = 1,
  .duration1 = 25 * WSS_RESOLUTION_HZ / 1000000,
  .level1 = 1,
};

static const rmt_symbol_word_t wss_pause1 = {
  .duration0 = 12 * WSS_RESOLUTION_HZ / 1000000,
  .level0 = 0,
  .duration1 = 13 * WSS_RESOLUTION_HZ / 1000000,
  .level1 = 0,
};

static const rmt_symbol_word_t wss_pause2 = {
  .duration0 = 12 * WSS_RESOLUTION_HZ / 1000000,
  .level0 = 0,
  .duration1 = 13 * WSS_RESOLUTION_HZ / 1000000,
  .level1 = 0,
};

//——————————————————————————————————————————————————————————————————————————————
// ----------------------------------- SETUP -----------------------------------
//——————————————————————————————————————————————————————————————————————————————

void setup() {
  //--- Switch on builtin led
  pinMode(LED21, OUTPUT);
  digitalWrite(LED21, HIGH);
  //--- Start serial
  Serial.begin(115200);
  //--- Wait for serial (blink led at 10 Hz during waiting)
  while (!Serial) {
    delay(50);
    digitalWrite(LED21, !digitalRead(LED21));
  }

  Serial.println("Starting CAN Setup.");
  //----------------------------------- Begin SPI
  SPI.begin(MCP2517_SCK, MCP2517_MISO, MCP2517_MOSI);

  //--- Configure ACAN2517FD
  ACAN2517FDSettings settings(ACAN2517FDSettings::OSC_4MHz10xPLL, 1000 * 1000, DataBitRateFactor::x1);
  // oscillator frequency, desired arbitration bit rate, data bit rate factor (MBps)

  //----------------------------------- Append CAN filters
  buildAddresses();  // Create Filter and Response addresses based on Initial Configuration

  ACAN2517FDFilters filters;
  filters.appendFrameFilter(format, boxID, filterGeneral);        // Filter #0: receive standard frame with identifier 0x123
  filters.appendFrameFilter(format, filterID_1, filterSpeedHz);   // Filter #1: receive standard frame with identifier 0x123
  filters.appendFrameFilter(format, filterID_2, filterSpeedKph);  // Filter #2: receive standard frame with identifier 0x123

  // Check if filters are okay.
  if (filters.filterStatus() != ACAN2517FDFilters::kFiltersOk) {
    Serial.print("Error filter ");
    Serial.print(filters.filterErrorIndex());
    Serial.print(": ");
    Serial.println(filters.filterStatus());
  } else {
    Serial.println("Filters okay.");
  }

  //------------------------------------- Begin CAN
  const uint32_t errorCode = can.begin(
    settings, [] {
      can.isr();
    },
    filters);
  // NB!! adding "filters" here is the main difference.

  Serial.println("Finished CAN Setup.");
  Serial.print("Error code: 0x");
  Serial.println(errorCode, HEX);

  //----------------------------------- Initialise Wheel Speeds
  for (int i = 0; i < 4; i++) {
    setSpeed(100.0, wheelArray[i]);
  }

  // -------------------- Config LEDC Timers, Config LEDC Channels, Bind Channel to Timer
  for (int i = 0; i < 4; i++) {

    // Update Timer Config Object
    timer_config.timer_num = wheelArray[i]->ledc_timer;
    // Install Timer
    ledc_timer_config(&timer_config);

    // Update Channel Config Object
    channel_config.channel = wheelArray[i]->ledc_chan;
    channel_config.timer_sel = wheelArray[i]->ledc_timer;
    channel_config.gpio_num = wheelArray[i]->pin3;
    // Install Channel
    ledc_channel_config(&channel_config);

    // Bind channel to timer
    ledc_bind_channel_timer(LEDC_LOW_SPEED_MODE, wheelArray[i]->ledc_chan, wheelArray[i]->ledc_timer);
  }
  updatePWM();  // Update PWM output based on current Wheel Speed values

  //——————————————————————————————————————————————————————————————————————————————
  //   RMT Setup
  //——————————————————————————————————————————————————————————————————————————————

  // -------------------- Create RMT Tx Channels
  for (int i = 0; i < 4; i++) {

    rmt_tx_channel_config_t tx_chan_config_data = {
      .gpio_num = wheelArray[i]->pin1,
      .clk_src = RMT_CLK_SRC_DEFAULT,  // select source clock
      .resolution_hz = WSS_RESOLUTION_HZ,
      .mem_block_symbols = 64,  // increase the block size can make the LED less flickering
      .trans_queue_depth = 4,  // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config_data, &wheelArray[i]->data_chan));

    rmt_tx_channel_config_t tx_chan_config_pulse = {
      .gpio_num = wheelArray[i]->pin2,
      .clk_src = RMT_CLK_SRC_DEFAULT,  // select source clock
      .resolution_hz = WSS_RESOLUTION_HZ,
      .mem_block_symbols = 64,  // increase the block size can make the LED less flickering
      .trans_queue_depth = 4,  // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config_pulse, &wheelArray[i]->pulse_chan));
  }

  // -------------------- Create Encoders
  rmt_encoder_handle_t data_encoder_handle_array[4] = { NULL };
  rmt_encoder_handle_t pulse_encoder_handle_array[4] = { NULL };
  // Could put this in the Wheel struct instead

  // -------------------- Create DATA callback-based encoder
  // Have to define an encoder for each channel, as the .arg is passed when defined, not during runtime.
  for (int i = 0; i < 4; i++) {
    rmt_simple_encoder_config_t data_encoder_cfg = {
      .callback = data_encoder_callback,
      .arg = wheelArray[i],  // pass the Wheel object as an argument into the encoder
      //Note we don't set min_chunk_size here as the default of 64 is good enough.
    };
    ESP_ERROR_CHECK(rmt_new_simple_encoder(&data_encoder_cfg, &data_encoder_handle_array[i]));
  }

  // -------------------- Create PULSE callback-based encoder
  for (int i = 0; i < 4; i++) {
    rmt_simple_encoder_config_t pulse_encoder_cfg = {
      .callback = pulse_encoder_callback,
      .arg = wheelArray[i],  // pass the Wheel object as an argument into the encoder
      //Note we don't set min_chunk_size here as the default of 64 is good enough.
    };
    ESP_ERROR_CHECK(rmt_new_simple_encoder(&pulse_encoder_cfg, &pulse_encoder_handle_array[i]));
  }

  // -------------------- Enable RMT Tx Channels - 4xDATA + 4xPULSE
  for (int i = 0; i < 4; i++) {
    ESP_ERROR_CHECK(rmt_enable(wheelArray[i]->data_chan));
    ESP_ERROR_CHECK(rmt_enable(wheelArray[i]->pulse_chan));
  }

  // -------------------- Transmission DONE Callback - NB! STILL TO COMPLETE
  /*
bool trans_done_callback() {
  ESP_LOGI(TAG, "Pulse Done Callback!!");
}

rmt_tx_event_callbacks_t callback_config = {
  .on_trans_done = trans_done_callback,
};
ESP_ERROR_CHECK(rmt_tx_register_event_callbacks(wheelArray[i]->pulse_chan, &callback_config));
*/

  // -------------------- Transmission Config
  // Can reuse this for all channels, since they all run in looping config
  rmt_transmit_config_t tx_config = {
    .loop_count = -1,  // loop
  };

  // -------------------- Start Transmission
  for (int i = 0; i < 4; i++) {
    //data_encoder_cfg.arg = wheelArray[i];
    ESP_ERROR_CHECK(rmt_transmit(wheelArray[i]->data_chan, data_encoder_handle_array[i], wheelArray[i]->serialData, (wheelArray[i]->availDataBits + 4) * 4, &tx_config));

    //pulse_encoder_cfg.arg = wheelArray[i];
    ESP_ERROR_CHECK(rmt_transmit(wheelArray[i]->pulse_chan, pulse_encoder_handle_array[i], wheelArray[i]->serialData, 2 * 4, &tx_config));
  }
  //ESP_ERROR_CHECK(rmt_tx_wait_all_done(wss_chan, portMAX_DELAY));
}

//——————————————————————————————————————————————————————————————————————————————
// ----------------------------------- LOOP -----------------------------------
//——————————————————————————————————————————————————————————————————————————————

void loop() {
  can.dispatchReceivedMessage();
}



//——————————————————————————————————————————————————————————————————————————————
// --------------------------- FUNCTION DEFINITIONS ----------------------------
//——————————————————————————————————————————————————————————————————————————————

//——————————————————————————————————————————————————————————————————————————————
//   RECEIVE FILTERS
//——————————————————————————————————————————————————————————————————————————————
// --------------------Filter0
void filterGeneral(const CANFDMessage& inMessage) {
  Serial.println("Match filter 0");
  Serial.print("Byte1: ");
  Serial.println(inMessage.data[0], HEX);

  // Start formatting response
  CANFDMessage response;
  response.id = responseID;
  response.len = 8;
  if (format == kStandard) {
    response.ext = false;
  } else if (format == kExtended) {
    response.ext = true;
  }
  bool valuesOK = true;

  switch (inMessage.data[0]) {

    // *** READ ***
    case 0x00:
      // RESET
      break;

    case 0x10:  // CONFIGURE SENSOR TYPE
      valuesOK = configSensorType(inMessage);
      for (int i = 0; i < 4; i++) {
        response.data[i + 1] = wheelArray[i]->sensorType;  // configure bytes 1-3
      }
      break;

    case 0x11:  // *** NOT USED *** // CONFIGURE WHEEL INFO
      break;

    case 0x30:  // READ SENSOR TYPE
      for (int i = 0; i < 4; i++) {
        response.data[i + 1] = wheelArray[i]->sensorType;  // configure bytes 1-3
      }
      break;

    case 0x31:  // *** NOT USED *** // READ WHEEL INFO
      break;

    case 0x32:  // *** NOT USED *** // READ STARTUP VALUES OF DWSS DATA BITS
      break;

    case 0x3F:  // READ SOFTWARE VERSION

      break;

    case 0xF1:  // EXTRA FUNCTION ADDED BY HEIMHOLDT
      // GET ALL DATA
      break;

      // *** SET ***

    case 0x50:  // *** NOT USED *** // SET PARITY BIT
      break;

    case 0x51:  // *** NOT USED *** // SET STATUS MODE BIT
      break;

    case 0x52:  // *** NOT USED *** // SET DIRECTION VALIDITY BIT
      break;

    case 0x53:  // *** NOT USED *** // CONFIGURE STARTUP VALUES OF DWSS DATA BITS
      break;
  }

  // Configure Byte0 of response/Error check
  if (valuesOK) {
    response.data[0] = inMessage.data[0];  // Send with same byte0
  } else {
    response.data[0] = 0xFF;  // Indicate error in assignment
  }

  // Send response frame
  const bool responseOK = can.tryToSend(response);
  if (responseOK) {
    Serial.print("Sent Response with identifier: ");
    Serial.println(response.data[0], HEX);
  }
}

// --------------------Filter1
void filterSpeedHz(const CANFDMessage& inMessage) {
  Serial.println("Match filter 1");

  int16_t* WheelValues = extractWordsAndConvert(inMessage);

  float WheelValuesHz[4] = {};
  for (int i = 0; i < 4; i++) {
    WheelValuesHz[i] = WheelValues[i] / 10.0;
    setFrequency(WheelValuesHz[i], wheelArray[i]);
  }
  updatePWM();
}

// --------------------Filter2
void filterSpeedKph(const CANFDMessage& inMessage) {
  Serial.println("Match filter 2");

  int16_t* WheelValues = extractWordsAndConvert(inMessage);

  float WheelValuesKPH[4] = {};
  for (int i = 0; i < 4; i++) {
    WheelValuesKPH[i] = WheelValues[i] / 10.0;
    setSpeed(WheelValuesKPH[i], wheelArray[i]);
  }
  updatePWM();
}

//——————————————————————————————————————————————————————————————————————————————
//   Filter Callback Functions
//——————————————————————————————————————————————————————————————————————————————

bool configSensorType(const CANFDMessage& inMessage) {
  bool ok = true;
  for (int i = 0; i < 4; i++) {
    if (inMessage.data[i + 1] == 0) {
      // Set to Normal WSS
      wheelArray[i]->sensorType = 0;
    } else if (inMessage.data[i + 1] == 1) {
      // Set to Directional WSS
      wheelArray[i]->sensorType = 1;
    }
    ok = ok && (wheelArray[i]->sensorType == inMessage.data[i + 1]);  // check if target value = actual value, AND with "ok" to see if any wheel produces error
  }
  return ok;
}

void configWheelInfo(const CANFDMessage& inMessage) {
  teeth = inMessage.data[1];
  wheelCirc = inMessage.data[2];
  // STILL TO COMPLETE THIS FUNCTION
}

//——————————————————————————————————————————————————————————————————————————————
//   Data Format Functions
//——————————————————————————————————————————————————————————————————————————————
void buildAddresses() {
  if (format == kStandard) {
    boxID = inputID * 0x10;
  } else if (format == kExtended) {
    boxID = inputID * 0x100000 + 0x10000000;
  }
  responseID = boxID + 0x1;
  filterID_1 = boxID + 0x2;
  filterID_2 = boxID + 0x4;
}

int16_t convertFromTwosComp(uint16_t value) {
  if (value & 0x8000) {  // if neg, subtract 2^16
    return value - 0x10000;
  } else {  // else do nothing.
    return value;
  }
}

uint16_t convertToTwosComp(int16_t value) {
  if (value < 0) {
    return (uint16_t)(value + 0x10000);  // Convert negative to two's complement
  } else {
    return (uint16_t)value;  // Return positive value or zero as is
  }
}

int16_t* extractWordsAndConvert(const CANFDMessage& inMessage) {
  // Extract 4x Words (16-bit) from CAN message, each word corresponding to one wheel.
  // Convert from 16-bit two's complement to signed int
  // Message structure explained in documentation.
  static int16_t valueArray[4];
  for (int i = 0; i < 4; i++) {
    valueArray[i] = convertFromTwosComp((inMessage.data[2 * i] << 8) | inMessage.data[2 * i + 1]);
  }
  return valueArray;
}

uint8_t* splitWordIntoBytes(uint16_t input) {
  // Split 16-bit int into 2x 8-bit ints: 0b1111111100000000 => 0b11111111, 0b00000000
  static uint8_t byteArray[2];
  byteArray[0] = (input >> 8) & 0xFF;  // Upper 8 bits (MSB)
  byteArray[1] = input & 0xFF;         // Lower 8 bits (LSB)
  return byteArray;
}

//——————————————————————————————————————————————————————————————————————————————
//   Wheel Calculation Functions
//——————————————————————————————————————————————————————————————————————————————

float calcPeriod(float new_speed) {
  float period;
  if (new_speed == 0.0) {
    period = 1500.0;
    //freq = 1.0 / period * 1000000.0;
  } else {
    period = (1.0 / (new_speed * teeth / wheelCirc * (1000.0 / 3.6)) * 1000000.0) / 2.0;  // divide by two, to get pulsePeriod;
    //freq = speed * teeth / wheelCirc * (1000.0 / 3.6);
  }
  return abs(period);
}

int calcDataBits(float period) {
  int availDataBits = 9;

  if (period >= 1050.0) {
    availDataBits = 9;
  } else if ((period < 1050.0) && (period >= 950.0)) {
    availDataBits = 8;
  } else if ((period < 950.0) && (period >= 850.0)) {
    availDataBits = 7;
  } else if ((period < 850.0) && (period >= 750.0)) {
    availDataBits = 6;
  } else if ((period < 750.0) && (period >= 650.0)) {
    availDataBits = 5;
  } else if ((period < 650.0) && (period >= 550.0)) {
    availDataBits = 4;
  } else if ((period < 550.0)) {
    availDataBits = 3;
  }

  return availDataBits;
}

void calcParity(Wheel* w) {
  int sum = 0;
  for (int i = 0; i < 8; i++) {  // add bits, determine parity
    sum += w->serialData[i];
  }
  w->serialData[8] = sum % 2;  // even parity
}

//——————————————————————————————————————————————————————————————————————————————
//   Setter Functions
//——————————————————————————————————————————————————————————————————————————————

void setSpeed(float new_speed, Wheel* w) {
  w->wheelSpeed = new_speed;  // allow negative value
  w->toothFreq = abs(new_speed) * (teeth / wheelCirc) * (1000.0 / 3.6);
  w->pulsePeriod = calcPeriod(new_speed);
  w->availDataBits = calcDataBits(w->pulsePeriod);
  w->serialData[4] = (new_speed < 0) * 1;  // set wheel direction bit
  calcParity(w);
}

void setFrequency(float new_freq, Wheel* w) {
  w->toothFreq = abs(new_freq);
  w->wheelSpeed = new_freq * (wheelCirc / teeth) * (3.6 / 1000.0);  // allow negative value
  w->pulsePeriod = calcPeriod(w->wheelSpeed);
  w->availDataBits = calcDataBits(w->pulsePeriod);
  w->serialData[4] = (w->wheelSpeed < 0) * 1;  // set wheel direction bit
  calcParity(w);
  //w->pulsePeriod = 1.0/(2.0*new_freq)*1000000.0;
}

//——————————————————————————————————————————————————————————————————————————————
//   RMT Encoder Functions
//——————————————————————————————————————————————————————————————————————————————

static size_t pulse_encoder_callback(const void* data, size_t data_size,
                                     size_t symbols_written, size_t symbols_free,
                                     rmt_symbol_word_t* symbols, bool* done, void* arg) {

  if (symbols_written < 1) {
    symbols[0] = wss_pulse;
    return 1;
  } else {
    Wheel* wheelPtr = (Wheel*)arg;
    float period = wheelPtr->pulsePeriod;
    float rem = period / 2 - Tp;

    rmt_symbol_word_t wss_rem = {
      .level0 = 0,
      .duration0 = rem / 2.0 * WSS_RESOLUTION_HZ / 1000000,
      .level1 = 0,
      .duration1 = rem / 2.0 * WSS_RESOLUTION_HZ / 1000000,
    };

    symbols[0] = wss_rem;
    *done = 1;  //Indicate end of the transaction.
    return 1;
  }
}


static size_t data_encoder_callback(const void* data, size_t data_size,
                                    size_t symbols_written, size_t symbols_free,
                                    rmt_symbol_word_t* symbols, bool* done, void* arg) {

  int* data_int = (int*)data;
  Wheel* wheelPtr = (Wheel*)arg;
  int availDataBits = wheelPtr->availDataBits;

  if (symbols_free < 8) {
    return 0;
  }

  if (symbols_written < data_size / 4 - 1) {
    size_t symbol_pos = 0;

    switch (symbols_written) {
      case 0:
        symbols[symbol_pos++] = wss_pulse;
        break;

      case 1:
        symbols[symbol_pos++] = wss_pause1;
        break;

      default:
        if (symbols_written >= availDataBits + 2) {
          symbols[symbol_pos++] = wss_pause2;
        }

        if (symbols_written - 2 < availDataBits) {
          if (data_int[symbols_written - 2] == 1) {
            symbols[symbol_pos++] = wss_one;
          } else {
            symbols[symbol_pos++] = wss_zero;
          }
        }
        break;
    }
    return symbol_pos;
  } else {

    float period = wheelPtr->pulsePeriod;
    float rem = period / 2 - (availDataBits * Tp + 2 * Tp);  // pulse + p1 + p2 = 2*Tp

    rmt_symbol_word_t wss_rem = {
      .level0 = 0,
      .duration0 = rem / 2.0 * WSS_RESOLUTION_HZ / 1000000,
      .level1 = 0,
      .duration1 = rem / 2.0 * WSS_RESOLUTION_HZ / 1000000,
    };

    symbols[0] = wss_rem;
    *done = 1;  //Indicate end of the transaction.
    return 1;
  }
}

//——————————————————————————————————————————————————————————————————————————————
//   Update Waveform Functions
//——————————————————————————————————————————————————————————————————————————————

void updatePWM() {

  for (int i = 0; i < 4; i++) {
    //ledcWrite(wheelArray[i]->pin3, MAX_DUTY_CYCLE/2);
    int freqInt = int(trunc(wheelArray[i]->toothFreq));  // truncate float to int. Only have resolution of 1Hz.

    if (freqInt == 0) {
      ledc_stop(LEDC_LOW_SPEED_MODE, wheelArray[i]->ledc_chan, 0);
    } else {
      ledc_update_duty(LEDC_LOW_SPEED_MODE, wheelArray[i]->ledc_chan);  // Resume after ledc_stop(). Does nothing otherwise.
      ledc_set_freq(LEDC_LOW_SPEED_MODE, wheelArray[i]->ledc_timer, freqInt);
    }
  }
}
