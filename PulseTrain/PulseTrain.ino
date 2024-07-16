#include "hal/gpio_ll.h"
#include "driver/gpio.h"
#include <cstdint>
#include <cstddef>
#include "Wheel.h"
//#include "Wheel.cpp"


const unsigned long pin0 = 1ul << 0;
const unsigned long pin1 = 1ul << 1;
const unsigned long pin2 = 1ul << 2;
const unsigned long pin4 = 1ul << 4;
const unsigned long pin5 = 1ul << 5;
const unsigned long pin15 = 1ul << 15;
const unsigned long pin17 = 1ul << 17;
const unsigned long pin18 = 1ul << 18;
const unsigned long pin19 = 1ul << 19;

// Generate pulse train based on number of data bits.
int Tp = 50;  // microseconds

char LR = 1;
char M = 1;
char DE = 0;  //
char GDR = 1;
char DR = 1;
char LM0 = 0;  //
char LM1 = 1;
char LM2 = 0;  //
char P = 1;

char data[9] = { LR, M, DE, GDR, DR, LM0, LM1, LM2, P };

float teeth = 48.0;
float wheelCirc = 1950.0;

Wheel FL = Wheel(pin4, pin15, 190.0);
Wheel FR = Wheel(pin5, pin19, 191.0);
Wheel RL = Wheel(pin0, pin17, 189.0);
Wheel RR = Wheel(pin2, pin18, 190.0);
Wheel wheelArray[4] = { FL, FR, RL, RR };

long int loopStart = millis();
float currentSpeed = 50.0;

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

  long int timeStart = micros();

  FL.setStart(timeStart);
  FR.setStart(timeStart);
  RL.setStart(timeStart);
  RR.setStart(timeStart);

  FL.setNextTransition(timeStart);
  FR.setNextTransition(timeStart);
  RL.setNextTransition(timeStart);
  RR.setNextTransition(timeStart);
}

void loop() {
  checkTransitions();
}

void checkTransitions() {
  long int currentTime = micros();
  for (int i = 0; i < 4; i++) {
    Wheel& w = wheelArray[i];
    if (currentTime >= w.nextTransition) {  // current time exceeds next transition time
      newTransition(w);
    }
  }
}

void newTransition(Wheel& w) {

  switch (w.zone) {

    case 0:                                       // Large Pulse
      GPIO.out_w1ts = w.getPin1() | w.getPin2();  // set

      //w.start = w.nextTransition;  // NEW CHANGE - save start of pulse

      w.nextTransition += 50;
      w.zone = 1;
      break;

    case 1:                                       // Pause 1
      GPIO.out_w1tc = w.getPin1() | w.getPin2();  // clear
      w.nextTransition += 25;

      // Entering data transmission
      w.zone = 2;
      w.transmissionHalf = 1;
      w.currentDataBit = 0;
      break;

    case 2:  // Data Transmission

      switch (w.transmissionHalf) {
        case 1:
          firstHalfTransition(w);
          w.nextTransition += 25;
          w.transmissionHalf = 2;
          break;

        case 2:
          secondHalfTransition(w);

          if (w.currentDataBit >= w.availDataBits - 1) {  // last data bit, short transition time
            w.nextTransition += 25;

            // exit data transmission
            if (w.availDataBits < 9) {
              GPIO.out_w1tc = w.getPin2();
              w.zone = 100;  // skip zone 3, no pause 2
            } else {

              w.zone = 3;  // add pause 2
            }
            w.transmissionHalf = 100;
            w.currentDataBit = 100;

          } else if (data[w.currentDataBit] ^ data[w.currentDataBit + 1]) {  // xor: if current bit diff to next, next transition occurs later.
            w.nextTransition += 50;
            w.transmissionHalf = 2;  // skip ahead to 2nd half transition of next bit, 50us later

          } else {  // adjacent bits have same value, short transition
            w.nextTransition += 25;
            w.transmissionHalf = 1;  // move to 1st half of next bit, 25us later.
          }

          w.currentDataBit += 1;  // move to next data bit
          //w.transmissionHalf = 1;  // in first half of next bit.
          break;
      }
      break;

    case 3:  // Pause 2
      GPIO.out_w1tc = w.getPin2();
      w.nextTransition += 25;
      w.zone = 100;
      break;

    case 100:  // Transition out of NULL zone
      w.zone = 0;

      w.nextTransition += (w.getPeriod() / 2 - (w.getAvailDataBits() * Tp + (1.5 * Tp)));  // remaining time in period, after pulse, pause, and transmission of available bits

      if (w.availDataBits >= 9) {    // NB!! Investigate this
        w.nextTransition += Tp / 2;  // add pause 2 if wheel period is large enough (all bits are available)
      }

      break;
  }
}

void firstHalfTransition(Wheel& w) {
  switch (data[w.currentDataBit]) {
    case 1:
      GPIO.out_w1tc = w.getPin2();  // clear
      break;

    case 0:
      GPIO.out_w1ts = w.getPin2();  // set
      break;
  }
}

void secondHalfTransition(Wheel& w) {
  switch (data[w.currentDataBit]) {
    case 1:
      GPIO.out_w1ts = w.getPin2();  // set
      break;

    case 0:
      GPIO.out_w1tc = w.getPin2();  // clear
      break;
  }
}