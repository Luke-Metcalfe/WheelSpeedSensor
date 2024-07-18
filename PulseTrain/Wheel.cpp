#include "Wheel.h"
#include <cstdint>
#include <cstddef>
//#include "Utils.h"

Wheel::Wheel(unsigned long pin1, unsigned long pin2, float speed) {
  this->pin1 = pin1;
  this->pin2 = pin2;
  this->setSpeed(speed);

  // Initialise Transmission States
  nextTransition = 0;
  zone = 0;
  currentDataBit = 100;
  transmissionHalf = 100;
}

unsigned long Wheel::getPin1() {
  return pin1;
}

unsigned long Wheel::getPin2() {
  return pin2;
}

float Wheel::getSpeed() {
  return speed;
}

float Wheel::getPeriod() {
  return period;
}

float Wheel::getFrequency() {
  return freq;
}

long int Wheel::getNextTransition() {
  return nextTransition;
}

int Wheel::getZone() {
  return zone;
}

int Wheel::getCurrentDataBit() {
  return currentDataBit;
}

int Wheel::getTransmissionHalf() {
  return transmissionHalf;
}

int Wheel::getAvailDataBits() {
  return availDataBits;
}

int Wheel::getSerialData(int bit) {
  return serialData[bit];
}

// Setters
void Wheel::setSpeed(float newSpeed) { // km/h
  speed = newSpeed;

  if (newSpeed == 0.0){
    period = 1500.0*2;
    freq = 1.0 / period * 1000000.0;
  }
  else{
    period = 1.0 / (speed * teeth / wheelCirc * (1000.0 / 3.6)) * 1000000.0;
    freq = speed * teeth / wheelCirc * (1000.0 / 3.6);
  }
  this->calcDataBits();
  this->calcParity();
}

void Wheel::setFrequency(float newFreq) { // Hz
  freq = newFreq;
  period = 1.0 / freq * 1000000.0;
  speed = freq / (teeth / wheelCirc * (1000.0 / 3.6));
  this->calcDataBits();
  this->calcParity();
}

void Wheel::setPeriod(float newPeriod) { // us
  period = newPeriod;
  freq = 1.0 / period * 1000000.0;
  speed = freq / (teeth / wheelCirc * (1000.0 / 3.6));
  this->calcDataBits();
  this->calcParity();
}



void Wheel::setNextTransition(long int newTransition) {
  nextTransition = newTransition;
}

void Wheel::incrNextTransition(long int newTransition) {
  nextTransition += newTransition;
}

void Wheel::setZone(int newZone) {
  zone = newZone;
}

void Wheel::setCurrentDataBit(int newBit) {
  currentDataBit = newBit;
}

void Wheel::setTransmissionHalf(int newHalf) {
  transmissionHalf = newHalf;
}

void Wheel::setAvailDataBits(int newAvailBits) {  // REMOVE?
  availDataBits = newAvailBits;
}

void Wheel::setData(int index, int value){
  serialData[index] = value;
  this->calcParity();
}

void Wheel::calcDataBits() {
  if (period >= 1050) {
    availDataBits = 9;
  } else if ((period < 1050) && (period >= 950)) {
    availDataBits = 8;
  } else if ((period < 950) && (period >= 850)) {
    availDataBits = 7;
  } else if ((period < 850) && (period >= 750)) {
    availDataBits = 6;
  } else if ((period < 750) && (period >= 650)) {
    availDataBits = 5;
  } else if ((period < 650) && (period >= 550)) {
    availDataBits = 4;
  } else if ((period < 550)) {
    availDataBits = 3;
  }
}

void Wheel::calcParity(){
  int sum = 0;
  for (int i=0; i<8;i++){ // add bits, determine parity
    sum += serialData[i];
  }
  serialData[8] = sum % 2; // even parity
}