// Example creates 4 independent PWM channels, and varies their frequency based on user input
// ESP32 can generate 16 independent PWM channels - 8 high speed, 8 low speed
// However, each set of 8 channels shares the resources of only 4 independent timers.
// Thus, adjacent channel numbers (0-1,2-3...) share a timer and will always have the same frequency.
// Thus, to have 4 independently varying frequencies, we need to use channels eg. 0,2,4,6

#include "driver/include/driver/ledc.h"

const int PWM_CHANNEL = 0;    // ESP32 has 16 channels which can generate 16 independent waveforms
const int PWM_FREQ = 1000;     // Recall that Arduino Uno is ~490 Hz. Official ESP32 example uses 5,000Hz
const int PWM_RESOLUTION = 10; // We'll use same resolution as Uno (8 bits, 0-255) but ESP32 can go up to 16 bits 

// The max duty cycle value based on PWM resolution (will be 255 if resolution is 8 bits)
const int MAX_DUTY_CYCLE = (int)(pow(2, PWM_RESOLUTION) - 1); 

const int LED_OUTPUT_PIN = 18;

int pin1 = 15;
int pin2 = 0;
int pin3 = 16;
int pin4 = 5;

void setup() {

  Serial.begin(115200);

  // Sets up a channel (0-15), a PWM duty cycle frequency, and a PWM resolution (1 - 16 bits) 
  // ledcSetup(uint8_t channel, double freq, uint8_t resolution_bits);
  
  /*ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(3, PWM_FREQ, PWM_RESOLUTION);

  // ledcAttachPin(uint8_t pin, uint8_t channel);
  ledcAttachPin(pin1, 0);
  ledcAttachPin(pin2, 1);
  ledcAttachPin(pin3, 2);
  ledcAttachPin(pin4, 3);
  */

  ledcAttachChannel(pin1, PWM_FREQ, PWM_RESOLUTION, 0); // pin, freq, res, channel
  ledcAttachChannel(pin2, PWM_FREQ, PWM_RESOLUTION, 2);
  ledcAttachChannel(pin3, PWM_FREQ, PWM_RESOLUTION, 4);
  ledcAttachChannel(pin4, PWM_FREQ, PWM_RESOLUTION, 6);

  ledcWrite(pin1, MAX_DUTY_CYCLE/2);
  ledcWrite(pin2, MAX_DUTY_CYCLE/2);
  ledcWrite(pin3, MAX_DUTY_CYCLE/2);
  ledcWrite(pin4, MAX_DUTY_CYCLE/2); // pin, duty cycle (0-resolution)

  Serial.print("Enter frequency: ");

}

void loop() {

  if (Serial.available() > 0) {

    float wheelFreq = Serial.parseFloat();
    int freqInt = int(trunc(wheelFreq));
    Serial.println(freqInt);

    ledcChangeFrequency(pin1, freqInt, PWM_RESOLUTION); // pin, freq., resolution
    ledcChangeFrequency(pin2, freqInt*2, PWM_RESOLUTION);
    ledcChangeFrequency(pin3, freqInt*3, PWM_RESOLUTION);
    ledcChangeFrequency(pin4, freqInt*4, PWM_RESOLUTION);

    //Serial.println(ledcChangeFrequency(pin1, freqInt, 8));

    Serial.println("Pin " + String(pin1) + ": " + String(ledcReadFreq(pin1)) + "Hz");
    Serial.println("Pin " + String(pin2) + ": " + String(ledcReadFreq(pin2)) + "Hz");
    Serial.println("Pin " + String(pin3) + ": " + String(ledcReadFreq(pin3)) + "Hz");
    Serial.println("Pin " + String(pin4) + ": " + String(ledcReadFreq(pin4)) + "Hz");

    Serial.print("Enter frequency: ");
    delay(500);
  }

  
}