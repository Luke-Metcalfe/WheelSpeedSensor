#include "hal/gpio_ll.h"
#include "driver/gpio.h"
#include <cstdint>
#include <cstddef>
#include "Wheel.h"

const unsigned long pin0 = 0;  // ...00001
const unsigned long pin1 = 1;  // ...00010
const unsigned long pin2 = 2;
const unsigned long pin4 = 4;
const unsigned long pin5 = 5;
const unsigned long pin15 = 15;
const unsigned long pin17 = 17;
const unsigned long pin18 = 18;
const unsigned long pin19 = 19;

// duration 0, level 0, duration 1, level 1
rmt_data_t zero = { 25, 1, 25, 0 };  // 400kHz = 2.5us per cycle, 10 cycles  = 25us // 1MHz = 1us/cycle
rmt_data_t one = { 25, 0, 25, 1 };
rmt_data_t blank = { 1, 0, 1, 0 };  // duration must be min 1 cycle, otherwise this appears as an EOF.
rmt_data_t EndOfFrame = { 0, 0, 0, 0 };

rmt_data_t pulse = { 25, 1, 25, 1 };
rmt_data_t pause1 = { 12, 0, 13, 0 };
rmt_data_t pause2 = { 12, 0, 13, 0 };
rmt_data_t currentWheelData[13];

// Sensor pulse width
int Tp = 50;  // microseconds

float teeth = 48.0;
float wheelCirc = 1950.0;

Wheel wheelArray[4] = {

  Wheel(pin4, pin15, 190.0),  // FL
  Wheel(pin5, pin19, 191.1),  // FR
  Wheel(pin0, pin17, 192.2),  // RL
  Wheel(pin2, pin18, 193.3)   // RR

};


void setup() {
  Serial.begin(115200);
  delay(2000);

  for (int i = 0; i < 4; i++) {
    Wheel& w = wheelArray[i];
    rmtInit(w.getPin1(), RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 1000000);
  }

  Wheel& w1 = wheelArray[0];
  w1.setSpeed(200.0);
  updatePulse(w1);
  Wheel& w2 = wheelArray[1];
  w2.setSpeed(210.0);
  updatePulse(w2);
  Wheel& w3 = wheelArray[2];
  w3.setSpeed(220.0);
  updatePulse(w3);
  Wheel& w4 = wheelArray[3];
  w4.setSpeed(230.0);
  updatePulse(w4);
}

void loop() {
  
}


void updatePulse(Wheel& w) {
  currentWheelData[0] = pulse;     // insert large pulse
  currentWheelData[1] = pause1;    // insert pause 1
  for (int i = 2; i <= 10; i++) {  // insert serial data
    if (i - 2 > w.availDataBits - 1) {
      currentWheelData[i] = blank;  // insert blank bit if serial data exceeds available bits
    } else if (w.serialData[i - 2] == 1) {
      currentWheelData[i] = one;
    } else if (w.serialData[i - 2] == 0) {
      currentWheelData[i] = zero;
    }
  }
  // Insert Pause 2
  if (w.availDataBits < 9) {       // No pause 2
    currentWheelData[11] = blank;  // insert blank bit
  } else {
    currentWheelData[11] = pause2;  // insert pause 2
  }
  float rem = w.period / 2 - (w.availDataBits * Tp + 1.5 * Tp);
  currentWheelData[12] = { rem / 2, 0, rem / 2, 0 };  // remaining time until next large pulse
  currentWheelData[13] = EndOfFrame;                  // End Of Frame

  rmtWriteLooping(w.getPin1(), currentWheelData, RMT_SYMBOLS_OF(currentWheelData));
}