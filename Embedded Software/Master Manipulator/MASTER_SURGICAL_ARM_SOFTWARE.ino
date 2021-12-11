/*
 * NOTE:
 * This code only needs to be uploaded to the master arduino Uno if you are trying to 
 * use the bluetooth comm for passing data from master to slave. Othewise, the master is tethered to the 
 * slave and thus the sensors are powered over wire comm.
 */

#include "potentiometer.h"
#include "button.h"

#include <SoftwareSerial.h>
SoftwareSerial BTserial(2, 3); // RX | TX
// Connect the HC-05 TX to Arduino pin 2 RX. 
// Connect the HC-05 RX to Arduino pin 3 TX through a voltage divider.

//Define MACROS
#define NUM_POTS 4
#define POT1 A1
#define POT2 A2
#define POT3 A3
#define POT4 A4
const uint8_t potPins[4] = {POT1, POT2, POT3, POT4};

//for switch for fine vs coarse control
#define FINE_CONTROL_BUTTON 10

//create button objects
Button fineControlButton(FINE_CONTROL_BUTTON);

//store button state
//1 means coarse control b/c internal pull-up, 0 means fine
unsigned int switchState = 1;

//store pot angles
float potAngles[4] = {0, 0, 0, 0};

//create potentiometer objects
Potentiometer pot1(1024, 300, 610);
Potentiometer pot2(1024, 300, 660);
Potentiometer pot3(1024, 300, 673);
Potentiometer pot4(1024, 210, 335);
Potentiometer pots[4] = {pot1, pot2, pot3, pot4};


unsigned char buff[NUM_POTS*2+2];  //2 per potentiometer! plus two for fine control switch
//most likely would also want to add two for the zero encoder button!

void setup() {
  // put your setup code here, to run once:
  
  //set potentiometer pins modes
  for (int i = 0; i < NUM_POTS; i++) pinMode(potPins[i], INPUT);

  //set the fine control switch pin mode
  pinMode(FINE_CONTROL_BUTTON, INPUT_PULLUP);

  Serial.begin(38400);  //sped up so that encoder value printing catches every increment
  BTserial.begin(38400);  
}

void loop() {
  // put your main code here, to run repeatedly:
  //read the fine control switch value
  readSwitch();
  //potentiometer readings to drive reference angles
  readPotVals(potAngles);

  //print statements to check switch/pots are reading well
  Serial.print(switchState);
  Serial.print("\t");
  Serial.print(potAngles[0]);
  Serial.print("\t");
  Serial.print(potAngles[1]);
  Serial.print("\t");
  Serial.print(potAngles[2]);
  Serial.print("\t");
  Serial.println(potAngles[3]);

  //convert the pot readings
  potsToBytes(potAngles);

  //write over serial
  BTserial.write(&buff[0],9);
  
  delay(40);  //need this delay so slave BT can process reading quick enough!!
}

//read all potentiometer raw values and get angles from raw values
void readPotVals(float (& currPotAngles)[NUM_POTS])
{
  static float prevPotValues[4] = {0, 0, 0, 0};
  //loop through pots and get val, then convert to angle and add to return array
  for (int i = 0; i < NUM_POTS; i++)
  {
    int currPotVal = analogRead(potPins[i]);
    currPotAngles[i] = pots[i].getAngle(currPotVal);
  }
} 

//get current fine control switch state...
void readSwitch(void)
{
  switchState = digitalRead(FINE_CONTROL_BUTTON);
}


/*
 * BYTE 1 - Switch
 * BYTE 2 - Pot 1
 * BYTE 3 - Pot 1
 * BYTE 4 - Pot 2
 * BYTE 5 - Pot 2
 * BYTE 6 - Pot 3
 * BYTE 7 - Pot 3
 * BYTE 8 - Pot 4
 * BYTE 9 - Pot 4
 */
 
//convert pot readings...
void potsToBytes(float angles[4])
{
  //reserve first byte for switch!
  buff[0] = (unsigned char) switchState;

  //rest of bytes for pot values!
  for (int i = 0; i < NUM_POTS; i++)
  {
    double num = (angles[i] + 360) * 10;
    unsigned int num2 = (unsigned int) num;
    unsigned char byte1, byte2;
    byte2 = num2 >> 8;
    byte1 = num2 & 0xFF;
    buff[2*i+1] = byte1;
    buff[2*i+2] = byte2;
  }
}
