#include "Wheel.h"
#include <cstdint>
#include <cstddef>
//#include "Utils.h"

Wheel::Wheel(unsigned long pin1, unsigned long pin2, float speed) {
  this->pin1 = pin1;
  this->pin2 = pin2;

  this->speed = speed;
  period = 1.0/(speed*teeth/wheelCirc*(1000.0/3.6))*1000000;
  freq = speed*teeth/wheelCirc*(1000.0/3.6);
  //start = micros();

  zone = 100;
  currentDataBit = 100;
  transmissionHalf = 100;
  this->calcDataBits();

}

unsigned long Wheel::getPin1() {
  return pin1;
}

unsigned long Wheel::getPin2() {
  return pin2;
}

float Wheel::getSpeed(){
  return speed;
}

long int Wheel::getPeriod() {
  return period;
}

long int Wheel::getFrequency() {
  return freq;
}

long int Wheel::getStart() {
  return start;
}

long int Wheel::getDelta() {
  return delta;
}


/*
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
*/


// Setters
void Wheel::setSpeed(float newSpeed){
  speed = newSpeed;
  period = 1.0/(speed*teeth/wheelCirc*(1000.0/3.6))*1000000;
  freq = speed*teeth/wheelCirc*(1000.0/3.6);
  this->calcDataBits();
}

void Wheel::setDelta(long int newDelta){
  delta = newDelta;
}

void Wheel::setStart(long int newStart){
  start = newStart;
  nextTransition = newStart + period/2;
}

void Wheel::setZone(int newZone){
  zone = newZone;
}

void Wheel::calcDataBits(){
  if (period >= 1100) {  // = 550 * 2
    availDataBits = 9;
  } else if (period < 550) {  // = 225 * 2
    availDataBits = 3;
  } else {
    availDataBits = (period / 2.0 - 225.0) / 50.0 + 3;
  }
}

/*
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

void Wheel::setAvailDataBits(int newAvailBits) {
  availDataBits = newAvailBits;
}
*/