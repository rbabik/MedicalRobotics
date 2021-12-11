//make sure don't double include
#ifndef button
#define button

//make sure we act have arduino version necessary and 
//library included to use arduino functions
#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

class Button {
  private:
  int buttonPin;
  bool currState;
  
  public:
  //constructor
  Button(int pinButton);

  //check to see if start switch is off...
  bool buttonStateOff(void);

  //check to see if a button is on...
  bool buttonStateOn(void);

  //check to see if button was pressed once...
  bool buttonSinglePress(void);

  //check to see if switch was pressed once on...
  bool switchTurnedOn(void);

  //check to see if switch was pressed once off...
  bool switchTurnedOff(void);
  
};

#endif
