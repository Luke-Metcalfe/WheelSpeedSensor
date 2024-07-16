#ifndef Wheel_h
#define Wheel_h

class Wheel {

public:
	Wheel(unsigned long pin1, unsigned long pin2, float speed);

  // Getters
  
  unsigned long getPin1();
  unsigned long getPin2();
  
  float getSpeed();
  long int getPeriod();
  long int getFrequency();

  long int getNextTransition();
  int getZone();
  int getCurrentDataBit();
  int getTransmissionHalf();
  int getAvailDataBits();

  
  // Setters
  void setSpeed(float newSpeed);
  void setDelta(long int newDelta);
  void setNextTransition(long int newTransition);
  void incrNextTransition(long int newTransition);
  void setZone(int newZone);
  void setCurrentDataBit(int newBit);
  void setTransmissionHalf(int newHalf);
  void setData(int index, int value);
  void setAvailDataBits(int newAvailBits);
  void calcDataBits();
  void calcParity();

  long int nextTransition;
  
  int zone; // 0 in pulse, 1 in pause 1, 2 in data transmission, 3 in pause 2, 100 when resting
  int currentDataBit; // 100 not in data transmission, 0-9 when in data
  int transmissionHalf; // 100 not in data transmission, 1 first half, 2 second half
  int availDataBits; // available data bits based on wheel frequency.

  char serialData[9] = { 1, 1, 0, 1, 1, 0, 1, 0, 1 };
  //              LR, M, DE, GDR, DR, LM0, LM1, LM2, P

private:
  unsigned long pin1;
  unsigned long pin2; 
  float speed;

  long int period;
  long int freq;

  float teeth = 48.0;
  float wheelCirc = 1950.0;

};
#endif
