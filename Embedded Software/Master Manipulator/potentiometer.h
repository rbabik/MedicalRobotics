//make sure don't double include
#ifndef pt
#define pt

//make sure we act have arduino version necessary and 
//library included to use arduino functions
#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

class Potentiometer {
  private:
  float angDegrees;
  int potVal;
  int maxCount, ROM_degrees;
  int zeroCount;
  
  public:
  //constructor
  Potentiometer(int maxCnt, int ROM_deg, int countZero);

  float getAngle(int potVal);
  
};

#endif
