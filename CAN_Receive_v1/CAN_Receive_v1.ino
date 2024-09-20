//——————————————————————————————————————————————————————————————————————————————
//  ACAN2517FD Demo in loopback mode, for ESP32
//——————————————————————————————————————————————————————————————————————————————

#include <ACAN2517FD.h>
#include <SPI.h>

//——————————————————————————————————————————————————————————————————————————————
//  For using SPI on ESP32, see demo sketch SPI_Multiple_Buses
//  Two SPI busses are available in Arduino, HSPI and VSPI.
//  By default, Arduino SPI use VSPI, leaving HSPI unused.
//  Default VSPI pins are: SCK=18, MISO=19, MOSI=23.
//  You can change the default pin with additional begin arguments
//    SPI.begin (MCP2517_SCK, MCP2517_MISO, MCP2517_MOSI)
//  CS input of MCP2517 should be connected to a digital output port
//  INT output of MCP2517 should be connected to a digital input port, with interrupt capability
//  Notes:
//    - GPIOs 34 to 39 are GPIs – input only pins. These pins don’t have internal pull-ups or
//      pull-down resistors. They can’t be used as outputs.
//    - some pins do not support INPUT_PULLUP (see https://www.esp32.com/viewtopic.php?t=439)
//    - All GPIOs can be configured as interrupts
// See https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
//——————————————————————————————————————————————————————————————————————————————
int Tp = 50;  // microseconds
float teeth = 48.0;
float wheelCirc = 1950.0;

long inputID = 0x33;
tFrameFormat format = kExtended;  // kStandard | kExtended
// If Standard, address is limited up to 0x3F4
long boxID;
long responseID;
long filterID_1;
long filterID_2;

static const byte MCP2517_SCK = 18;   // SCK input of MCP2517FD // 26
static const byte MCP2517_MOSI = 23;  // SDI input of MCP2517FD // 19
static const byte MCP2517_MISO = 19;  // SDO output of MCP2517FD // 18

static const byte MCP2517_CS = 16;   // CS input of MCP2517FD // 16
static const byte MCP2517_INT = 32;  // INT output of MCP2517FD // 32

int LED21 = 21;

//——————————————————————————————————————————————————————————————————————————————
//  Wheel Structures
//——————————————————————————————————————————————————————————————————————————————

typedef struct {
  //gpio_num_t pin1;
  //gpio_num_t pin2;
  float wheelSpeed;    // Wheel speed in kph
  float toothFreq;     // Frequency in Hz of sensor gear
  float pulsePeriod;   // Period between start pulses -> Double the tooth freq, since we send a pulse on each gear tooth transition
  int availDataBits;   // Number of data bits that will be able to be transmitted between pulse periods
  uint8_t sensorType;  // 0:Normal | 1:Directional
  uint8_t serialData[9];
  //rmt_channel_handle_t data_chan;
  //rmt_channel_handle_t pulse_chan;
} Wheel;

Wheel FL = {
  //.pin1 = GPIO_NUM_4,
  //.pin2 = GPIO_NUM_15,
  .wheelSpeed = 101.0,
  .sensorType = 0,
  .serialData = { 1, 1, 0, 1, 1, 0, 1, 0, 1 },
  //.data_chan = NULL,
  //.pulse_chan = NULL,
};

Wheel FR = {
  //.pin1 = GPIO_NUM_5,
  //.pin2 = GPIO_NUM_19,
  .wheelSpeed = 102.0,
  .sensorType = 0,
  .serialData = { 1, 1, 0, 1, 1, 0, 1, 0, 1 },
  //.data_chan = NULL,
  //.pulse_chan = NULL,
};

Wheel RL = {
  //.pin1 = GPIO_NUM_0,
  //.pin2 = GPIO_NUM_17,
  .wheelSpeed = 103.0,
  .sensorType = 0,
  .serialData = { 1, 1, 0, 1, 1, 0, 1, 0, 1 },
  //.data_chan = NULL,
  //.pulse_chan = NULL,
};

Wheel RR = {
  //.pin1 = GPIO_NUM_2,
  //.pin2 = GPIO_NUM_18,
  .wheelSpeed = 104.0,
  .sensorType = 0,
  .serialData = { 1, 1, 0, 1, 1, 0, 1, 0, 1 },
  //.data_chan = NULL,
  //.pulse_chan = NULL,
};

Wheel* wheelArray[4] = { &FL, &FR, &RL, &RR };  // array of pointers to wheel objects

//——————————————————————————————————————————————————————————————————————————————
//  ACAN2517FD Driver object
//——————————————————————————————————————————————————————————————————————————————

ACAN2517FD can(MCP2517_CS, SPI, MCP2517_INT);

//——————————————————————————————————————————————————————————————————————————————
//   RECEIVE FILTERS
//——————————————————————————————————————————————————————————————————————————————

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
        response.data[i + 1] = wheelArray[i]->sensorType; // configure bytes 1-3
      }
      break;

    case 0x11:  // *** NOT USED *** // CONFIGURE WHEEL INFO
      break;

    case 0x30:  // READ SENSOR TYPE
      for (int i = 0; i < 4; i++) {
        response.data[i + 1] = wheelArray[i]->sensorType; // configure bytes 1-3
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

void filterSpeedHz(const CANFDMessage& inMessage) {
  Serial.println("Match filter 1");

  int16_t* WheelValues = extractWordsAndConvert(inMessage);

  float WheelValuesHz[4] = {};
  for (int i = 0; i < 4; i++) {
    WheelValuesHz[i] = WheelValues[i] / 10.0;
    setFrequency(WheelValuesHz[i], wheelArray[i]);
  }
  /*
  Serial.println(WheelValuesHz[0]);
  Serial.println(WheelValuesHz[1]);
  Serial.println(WheelValuesHz[2]);
  Serial.println(WheelValuesHz[3]);
  */
}

void filterSpeedKph(const CANFDMessage& inMessage) {
  Serial.println("Match filter 2");

  int16_t* WheelValues = extractWordsAndConvert(inMessage);

  float WheelValuesKPH[4] = {};
  for (int i = 0; i < 4; i++) {
    WheelValuesKPH[i] = WheelValues[i] / 10.0;
    setSpeed(WheelValuesKPH[i], wheelArray[i]);
  }

  /*
  for (int i = 0; i < 4; i++) {
    Serial.println(wheelArray[i]->sensorType);
    Serial.println(wheelArray[i]->wheelSpeed);
    Serial.println(wheelArray[i]->pulsePeriod);
  }
  */
}

//——————————————————————————————————————————————————————————————————————————————
//   SETUP
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

  //----------------------------------- Append CAN filters
  buildAddresses();

  ACAN2517FDFilters filters;
  filters.appendFrameFilter(format, boxID, filterGeneral);        // Filter #0: receive standard frame with identifier 0x123
  filters.appendFrameFilter(format, filterID_1, filterSpeedHz);   // Filter #1: receive standard frame with identifier 0x123
  filters.appendFrameFilter(format, filterID_2, filterSpeedKph);  // Filter #2: receive standard frame with identifier 0x123

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

  //----------------------------------- Setup Wheel Speeds
  for (int i = 0; i < 4; i++) {
    setSpeed(100.0, wheelArray[i]);
  }
}


//——————————————————————————————————————————————————————————————————————————————
//   LOOP
//——————————————————————————————————————————————————————————————————————————————

void loop() {
  can.dispatchReceivedMessage();
}


//——————————————————————————————————————————————————————————————————————————————
//   DATA FORMAT FUNCTIONS
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

  //Serial.println(boxID, HEX);
  //Serial.println(responseID, HEX);
  //Serial.println(filterID_1, HEX);
  //Serial.println(filterID_2, HEX);
}

int16_t convertFromTwosComp(uint16_t value) {
  if (value & 0x8000) {  // if neg, subtract 2^16
    return value - 0x10000;
  } else {  // else do nothing.
    return value;
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

//——————————————————————————————————————————————————————————————————————————————
//   CAN FUNCTIONS
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
//   WHEEL CALCULATION FUNCTIONS
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
//   SET FUNCTIONS
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
//   CAN RESPONSE FUNCTIONS
//——————————————————————————————————————————————————————————————————————————————

uint16_t convertToTwosComp(int16_t value) {
  if (value < 0) {
    return (uint16_t)(value + 0x10000);  // Convert negative to two's complement
  } else {
    return (uint16_t)value;  // Return positive value or zero as is
  }
}

uint8_t* splitIntoBytes(uint16_t input) {
  static uint8_t byteArray[2];
  byteArray[0] = (input >> 8) & 0xFF;  // Upper 8 bits (MSB)
  byteArray[1] = input & 0xFF;         // Lower 8 bits (LSB)
  return byteArray;
}