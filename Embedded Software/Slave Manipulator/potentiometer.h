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
  int signFactor;
  
  public:
  //constructor
  Potentiometer(int maxCnt, int ROM_deg, int countZero);

  float getAngle(int potVal);

  //set scaling to 1 or -1 as motors and pots were not mounted on same sides
  void directionFactor(int signAdjust);
  
};

#endif
