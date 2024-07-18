#ifndef Wheel_h
#define Wheel_h

class Wheel {

public:
	Wheel(unsigned long pin1, unsigned long pin2, float speed);

  // Getters
  unsigned long getPin1();
  unsigned long getPin2();
  
  float getSpeed();
  float getPeriod();
  float getFrequency();

  long int getNextTransition();
  int getZone();
  int getCurrentDataBit();
  int getTransmissionHalf();
  int getAvailDataBits();
  int getSerialData(int bit);

  // Setters
  void setSpeed(float newSpeed);
  void setFrequency(float newFreq);
  void setPeriod(float newPeriod);

  void setNextTransition(long int newTransition);
  void incrNextTransition(long int newTransition);
  void setZone(int newZone);
  void setCurrentDataBit(int newBit);
  void setTransmissionHalf(int newHalf);
  void setData(int index, int value);
  void setAvailDataBits(int newAvailBits);

  // Other Functions
  void calcDataBits();
  void calcParity();

private:
  unsigned long pin1; // Pin for triggering large pulse
  unsigned long pin2; // Pin for triggering small pulse

  float speed;  // km/h
  float period; // us
  float freq;   // Hz

  char serialData[9] = { 1, 1, 0, 1, 1, 0, 1, 0, 1 };
  //                    LR, M, DE, GDR, DR, LM0, LM1, LM2, P - see documentation
  
  // Data Transmission Variables
  long int nextTransition;  // time stamp of next transition
  int zone; // 0 in pulse, 1 in pause 1, 2 in data transmission, 3 in pause 2, 100 when resting
  int currentDataBit; // 100 not in data transmission, 0-8 when in data
  int transmissionHalf; // 100 not in data transmission, 1 first half, 2 second half
  int availDataBits; // available data bits based on wheel frequency

  // Constants for speed/freq/period calculations
  float teeth = 48.0; // teeth/poles on encoder track 
  float wheelCirc = 1950.0; // circumference of wheel (mm)
};
#endif
