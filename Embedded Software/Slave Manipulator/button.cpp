#include "button.h"

///note all button stuff is backwards from intuition. can only set internal pullup for arduino input pins,
//so HIGH = button off and LOW = button on!

Button::Button(int pinButton)
{
  buttonPin = pinButton;
  //initialize off
  currState = HIGH;
}

//check to see if start switch is on...
bool Button::buttonStateOn(void)
{
  currState = digitalRead(buttonPin);
  if (currState == LOW) return true;
  else return false;
}

  //check to see if a button is off...
bool Button::buttonStateOff(void)
{
  currState = digitalRead(buttonPin);
  if (currState == HIGH) return true;
  else return false;
}

//check to see if button was pressed once...
bool Button::buttonSinglePress(void)
{
  static double prevTime = 0;
  double currTime = millis();
  double waitPeriod = currTime - prevTime;
  double waitThreshold = 2000;
  //Serial.println(currTime);

  //read current state
  bool tempState = digitalRead(buttonPin);
  
  //check to see if button pressed and wasn't immediately just pressed and hasn't been held down (been low for a while)...
  if ((currState = HIGH) && (tempState == LOW) && (waitPeriod > waitThreshold)) 
  {
    currState = tempState;
    prevTime = currTime;
    return true;
  }
  else 
  {
    currState = HIGH;
    return false;
  }
}

//check to see if switch was pressed once on...
//remember logic is backwards because of pullup resistor
bool Button::switchTurnedOn(void)
{
  static double prevTime = 0;
  double currTime = millis();
  double waitPeriod = currTime - prevTime;
  double waitThreshold = 50;
  //Serial.println(currTime);

  //read current state
  bool tempState = digitalRead(buttonPin);

  if ((tempState == 0) && (currState != tempState) && (waitPeriod > waitThreshold))
  {
    currState = tempState;
    prevTime = currTime;
    return true;
  }
  else return false;
}

//check to see if switch was pressed once off...
//remember logic is backwards because of pullup resistor
bool Button::switchTurnedOff(void)
{
  static double prevTime = 0;
  double currTime = millis();
  double waitPeriod = currTime - prevTime;
  double waitThreshold = 50;
  //Serial.println(currTime);

  //read current state
  bool tempState = digitalRead(buttonPin);

  if ((tempState > 0) && (currState != tempState) && (waitPeriod > waitThreshold))
  {
    currState = tempState;
    prevTime = currTime;
    return true;
  }
  else return false;  
}
