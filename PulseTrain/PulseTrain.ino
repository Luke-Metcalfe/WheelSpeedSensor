#include "hal/gpio_ll.h"
#include "driver/gpio.h"
#include <cstdint>
#include <cstddef>
#include "Wheel.h"

// Bit masks for setting/clearing GPIO register
const unsigned long pin0 = 1ul << 0;  // ...00001
const unsigned long pin1 = 1ul << 1;  // ...00010
const unsigned long pin2 = 1ul << 2;
const unsigned long pin4 = 1ul << 4;
const unsigned long pin5 = 1ul << 5;
const unsigned long pin15 = 1ul << 15;
const unsigned long pin17 = 1ul << 17;
const unsigned long pin18 = 1ul << 18;
const unsigned long pin19 = 1ul << 19;

// Sensor pulse width
int Tp = 50;  // microseconds

float teeth = 48.0;
float wheelCirc = 1950.0;

Wheel wheelArray[4] = { 

  Wheel(pin4, pin15, 190.0),  // FL
  Wheel(pin5, pin19, 191.1),  // FR
  Wheel(pin0, pin17, 192.2),  // RL
  Wheel(pin2, pin18, 193.3) // RR

};


void setup() {
  Serial.begin(115200);
  delay(2000);

  //ESP32 config io
  gpio_config_t io_conf;
  //ESP32 config io to output
  io_conf.mode = GPIO_MODE_OUTPUT;
  //Bitmask GPIOs as Output
  io_conf.pin_bit_mask = pin4 | pin5 | pin0 | pin2 | pin15 | pin19 | pin17 | pin18;
  //Set BitMask
  gpio_config(&io_conf);
}

void loop() {
  checkTransitions();
}

void checkTransitions() {
  for (int i = 0; i < 4; i++) {
    Wheel &w = wheelArray[i];
    if (micros() >= w.getNextTransition()) {  // current time exceeds next transition time
      newTransition(w);
    }
  }
}

void newTransition(Wheel& w) {

  switch (w.getZone()) {

    case 0:                                       // Large Pulse
      if (w.getSpeed() == 0.0){
        GPIO.out_w1ts = w.getPin2();  // set
      }
      else {
        GPIO.out_w1ts = w.getPin1() | w.getPin2();  // set
      }
      w.incrNextTransition(Tp);
      w.setZone(1);
      break;

    case 1:                                       // Pause 1
      GPIO.out_w1tc = w.getPin1() | w.getPin2();  // clear
      w.incrNextTransition(Tp/2);

      // Entering data transmission
      w.setZone(2);
      w.setTransmissionHalf(1);
      w.setCurrentDataBit(0);
      break;

    case 2:  // Data Transmission

      switch (w.getTransmissionHalf()) {
        case 1:
          firstHalfTransition(w);
          w.incrNextTransition(Tp/2);
          w.setTransmissionHalf(2);
          break;

        case 2:
          secondHalfTransition(w);

          if (w.getCurrentDataBit() >= w.getAvailDataBits() - 1) {  // last data bit, short transition time
            w.incrNextTransition(Tp/2);

            // exit data transmission
            if (w.getAvailDataBits() < 9) {
              GPIO.out_w1tc = w.getPin2();
              w.setZone(100);  // skip zone 3, no pause 2
            } else {
              w.setZone(3);  // add pause 2
            }
            w.setTransmissionHalf(100);
            w.setCurrentDataBit(100);

          } else if (w.getSerialData(w.getCurrentDataBit()) ^ w.getSerialData(w.getCurrentDataBit() + 1)) {  // xor: if current bit diff to next, next transition occurs later.
            w.incrNextTransition(Tp);
            w.setTransmissionHalf(2); // skip ahead to 2nd half transition of next bit, 50us later

          } else {  // adjacent bits have same value, short transition
            w.incrNextTransition(Tp/2);
            w.setTransmissionHalf(1); // move to 1st half of next bit, 25us later.
          }

          w.setCurrentDataBit(w.getCurrentDataBit() + 1);  // move to next data bit
          break;
      }
      break;

    case 3:  // Pause 2
      GPIO.out_w1tc = w.getPin2();
      w.incrNextTransition(Tp/2);
      w.setZone(100);
      break;

    case 100:  // Transition out of NULL zone
      w.setZone(0);
      w.incrNextTransition( w.getPeriod()/2 - (w.getAvailDataBits()*Tp + (1.5*Tp)) ); // remaining time in period, after pulse, pause, and transmission of available bits

      if (w.getAvailDataBits() >= 9) {    // NB!! Investigate this
        w.incrNextTransition(Tp/2); // add pause 2 if wheel period is large enough (all bits are available)
      }

      break;
  }
}

void firstHalfTransition(Wheel& w) {
  switch (w.getSerialData(w.getCurrentDataBit())) {
    case 1:
      GPIO.out_w1tc = w.getPin2();  // clear
      break;

    case 0:
      GPIO.out_w1ts = w.getPin2();  // set
      break;
  }
}

void secondHalfTransition(Wheel& w) {
  switch (w.getSerialData(w.getCurrentDataBit())) {
    case 1:
      GPIO.out_w1ts = w.getPin2();  // set
      break;

    case 0:
      GPIO.out_w1tc = w.getPin2();  // clear
      break;
  }
}