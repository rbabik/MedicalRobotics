#include "potentiometer.h"

Potentiometer::Potentiometer(int maxCnt, int ROM_deg, int countZero){
  maxCount = maxCnt;
  ROM_degrees = ROM_deg;
  angDegrees = 0;
  zeroCount = countZero;
  signFactor = 1;
}

float Potentiometer::getAngle(int potVal)
{
  //calculate distance off of defined zero point
  float distFromSet= potVal - zeroCount;
  //account for pot nonlinearities...
  if (ROM_degrees <= 300 && distFromSet > 0)
  {
    angDegrees = distFromSet / 350 * 90; //350 is counts to go 90 degrees
  }
   else angDegrees = distFromSet / maxCount * ROM_degrees;

   //need sign check so that master and slave convention align
   return angDegrees * signFactor;
}

//set scaling to 1 or -1 as motors and pots were not mounted on same sides
void Potentiometer::directionFactor(int signAdjust)
{
  signFactor = signAdjust;
}
