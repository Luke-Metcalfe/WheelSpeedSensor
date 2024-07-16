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

  zone = 0;
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



// Setters
void Wheel::setSpeed(float newSpeed){
  speed = newSpeed;
  period = 1.0/(speed*teeth/wheelCirc*(1000.0/3.6))*1000000;
  freq = speed*teeth/wheelCirc*(1000.0/3.6);
  this->calcDataBits();
}

void Wheel::setStart(long int newStart){
  start = newStart;
  //nextTransition = newStart + period/2;
}

void Wheel::calcDataBits(){

  /*
  if (period >= 1100) {  // = 550 * 2
    availDataBits = 9;
  } else if (period <= 550) {  // = 225 * 2
    availDataBits = 3;
  } else {
    availDataBits = (period / 2.0 - 225.0) / 50.0 + 3;
  }
*/
/*
if (period >= 1100){
availDataBits = 9;
}

else*/ if (period >= 1050){
availDataBits = 9;
}
else if ((period < 1050)&&(period >= 950)){
availDataBits = 8;
}
else if ((period < 950)&&(period >= 850)){
availDataBits = 7;
}
else if ((period < 850)&&(period >= 750)){
availDataBits = 6;
}
else if ((period < 750)&&(period >= 650)){
availDataBits = 5;
}
else if ((period < 650)&&(period >= 550)){
availDataBits = 4;
}
else if ((period < 550)){
availDataBits = 3;
}

}

void Wheel::setNextTransition(long int newTransition){
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

void Wheel::setAvailDataBits(int newAvailBits) {
  availDataBits = newAvailBits;
}
