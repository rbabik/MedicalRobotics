/* Includes ------------------------------------------------------------------*/
#include <Servo.h>
#include <SoftwareSerial.h>
#include "state_machine.h"
#include "potentiometer.h"
#include "motor_control.h"
#include "button.h"  //main ino needs access to the button objects!

/* Mactros ------------------------------------------------------------------*/

/* Module Variables ----------------------------------------------------*/

//create software serial for the slave BT
SoftwareSerial BTSerial(50,51); // RX | TX

//variable for enum machineState
machineState currState; 

////////////////////////////////////////////////////////////////////////////
////////////////////////////MACROS//////////////////////////////////////////

//all the stuff originally at top of ino file.. ino loop still needs access to put in .h not .cpp

//Define MACROS
#define NUM_POTS 4
#define POT1 A1
#define POT2 A2
#define POT3 A5
#define POT4 A4 //this pot will be for servo motor!
const int potPins[4] = {POT1, POT2, POT3, POT4};

#define START_BUTTON 22
#define ZERO_ENCODERS_BUTTON 23
#define FINE_CONTROL_BUTTON 24

#define SERVO_PIN 44 //need PWM pin!

//motor defines...
#define NUM_MOTORS 3
const int enca[3] = {2, 18, 20};
const int encb[3] = {3, 19, 21};
const int pwm[3] = {4, 5, 6};
const int en1[3] = {7, 9, 11};
const int en2[3] = {8, 10, 12};
const int CS[3] = {A0, A6, A7};

////////////////////////////////////////////////////////////////////////////
////////////////////INSTANTIATE OBJECTS/////////////////////////////////////

//create button objects
Button startButton(START_BUTTON);
Button initEncodersButton(ZERO_ENCODERS_BUTTON);
Button fineControlButton(FINE_CONTROL_BUTTON);

//create potentiometer objects for actual master pots
Potentiometer pot1(1024, 300, 610);
Potentiometer pot2(1024, 300, 660);
Potentiometer pot3(1024, 300, 660);
Potentiometer pot4(1024, 210, 735);

Potentiometer pots[4] = {pot1, pot2, pot3, pot4};

//create servo object
Servo servo;

//create motor objects
Motor motor1(pwm[0], enca[0], encb[0], en1[0], en2[0], CS[0], 3300.0); // base  --labeling of these needs to be fixed!
Motor motor2(pwm[1], enca[1], encb[1], en1[1], en2[1], CS[1], 2249.0); // next motor --need to update gear ratios to include belt drive!!
Motor motor3(pwm[2], enca[2], encb[2], en1[2], en2[2], CS[2], 2249.0); // end motor
Motor motors[3] = {motor1, motor2, motor3};

////////////////////////////////////////////////////////////////////////////
////////////////////GLOBAL VARIABLES////////////////////////////////////////

//general variables
int sensorValue = 0; int sensorValue2 = 0;
int sensorValuePrev = 0;
int potVal = 0;
float angDegrees;
int countVal = 0;
float angVal = 0;
double inputU, inputU2;
int analogInput;
float potAngles[4] = {0, 0, 0, 0};
bool motorControlMode = false;
float fineControlGain = 2.0;
bool shutDown = false;

//for moving average
const int numReadings  = 10;
int readings [numReadings];
int readIndex  = 0;
long total  = 0;

//for BT communication!
int state;
int inBuffer[9];
byte bufIndex = 0;
unsigned int fineControlSwitchState;
float prevPotAngles[4] = {0,0,0,0};


///////////////////////////////define a boolean to set if we're in bluetooth mode or not!!!
bool bluetoothMode = false;
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////


/* functions ------------------------------------------------------------------*/

//set state to initialize
void initStateMachine(void)
{

  //putting all of the ino setup function in here!!!

   //set potentiometer pins modes
  for (int i = 0; i < NUM_POTS; i++) pinMode(potPins[i], INPUT);

  //set the start switch pin mode 
  pinMode(START_BUTTON, INPUT_PULLUP);

  //set the encoder zero-ing button pin mode 
  pinMode(ZERO_ENCODERS_BUTTON, INPUT_PULLUP);

  //set the fine control switch pin mode
  pinMode(FINE_CONTROL_BUTTON, INPUT_PULLUP);

  //attach servo
  servo.attach(SERVO_PIN);

  // set motor pin modes
  for(int k = 0; k < NUM_MOTORS; k++){
    pinMode(enca[k],INPUT);
    pinMode(encb[k],INPUT);
    pinMode(pwm[k],OUTPUT);
    pinMode(en1[k],OUTPUT);
    pinMode(en2[k],OUTPUT);
    pinMode(CS[k],INPUT);
  }

  //use quadrature encoding and add for rising and falling edges!!
  //motor 1
  attachInterrupt(digitalPinToInterrupt(enca[0]),isrA<0>,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encb[0]),isrB<0>,CHANGE);
  //motor 2
  attachInterrupt(digitalPinToInterrupt(enca[1]),isrA<1>,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encb[1]),isrB<1>,CHANGE);
  //motor 3
  attachInterrupt(digitalPinToInterrupt(enca[2]),isrA<2>,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encb[2]),isrB<2>,CHANGE);

  //setup serial comm for both serial monitor and BT!!
  //need 38400 baud rate for BT
  Serial.begin(38400);
  Serial.println("Arduino with HC-06 is ready");
  // HC-06 baud rate is 38400
  BTSerial.begin(38400);  
  Serial.println("BTserial started at 38400");

  //correct signage on pot readings for certian link
  pots[1].directionFactor(-1);
  
  //set gains
  motors[0].setParams(2, 0.2, 0.000);
  motors[1].setParams(3,.6, 0); //(5, .05, 0); // 0.0001);
  motors[2].setParams(3, .4, 0); //10, 2.5, 0); //0.0001);

  //set deadzones -- motor 0 is different from motor 1 and 2
  motors[0].setDeadzone(30);
  motors[1].setDeadzone(30);
  motors[2].setDeadzone(30);

  //also will need to set belt drive ratios!!
  motors[0].setBeltRatio(2.149/1.194); //OD ratio is 2.149:1.194
  motors[1].setBeltRatio(31.0/11.6); //OD ratio is 31:11.6
  motors[2].setBeltRatio(31.0/11.6); //OD ratio is 31:11.6

  //initialize encoders! Motors not back drive-able so before powering down we always command
  //back to start orientation which is straight down. Then know motors will never leave this orientation
  //unless we command them to
  for(int k = 0; k < NUM_MOTORS; k++) motors[k].initEncoder();

  //set initial state to WAIT
  currState = WAIT;

  //push state machine to wait state
  addToQueue(NO_EVENT);
}

//state machine to receive states and change states accordingly
void stateMachine(event_type newEvent)
{
    //switch case with state
    switch (currState)
    {
        //case 1 WAIT
        case WAIT:

        //make sure motor control is off
        turnOffMotorControl();
        
        Serial.println("IN WAIT STATE");
        
        //check for event to initialize encoders --- this can be deleted now that we initialize them automatically in initStateMachine()!!
        if (newEvent == ENCODER_INIT_BUTTON_PRESSED) 
        {
          for(int k = 0; k < NUM_MOTORS; k++) motors[k].initEncoder(); //setRefAngle(potAngles[k]);
        }
        
        //if the start switched is turned on and we have already initialized the motors (only need to check one because all get initialized together)
        if (newEvent == START_BUTTON_PRESSED && motors[0].encodersInitialized())
        {
          //check if fine control button is on - if is, then do fine control, otherwise coarse control state
          if (fineControlButton.buttonStateOn()) currState = FINE_CONTROL; //enter fine control!!
          else currState = COARSE_CONTROL;

          //enable motor control
          turnOnMotorControl();
        }
        break;
        
        //case 2 COARSE_CONTROL
        case COARSE_CONTROL:
        
        Serial.println("IN COARSE CONTROL STATE");
        
        //check for button press
        if (newEvent == FINE_CONTROL_BUTTON_PRESSED) currState = FINE_CONTROL;
        if (newEvent == START_BUTTON_UNPRESSED) 
        {
          turnOffMotorControl();
          shutDown = true;
          currState = SHUT_DOWN;
        }
        break;
    
        //case 3 FINE_CONTROL
        case FINE_CONTROL:

        Serial.println("IN FINE CONTROL STATE");
        
        if (newEvent == FINE_CONTROL_BUTTON_UNPRESSED) currState = COARSE_CONTROL;
        if (newEvent == START_BUTTON_UNPRESSED) 
        {
          turnOffMotorControl();
          shutDown = true;
          currState = SHUT_DOWN;
        }
        break;

        //case 4 SHUT DOWN PROCESS
        case SHUT_DOWN:
        //see if error in all motor states is greater than say 2 degrees
        //if they are, set shut down to false and go to wait state
        Serial.println("IN SHUT DOWN STATE");
        if (newEvent == SHUT_DOWN_COMPLETE)
        {
          //set that we complete the shut down
          shutDown = false;
          currState = WAIT;
          addToQueue(NO_EVENT);
        }
        
        //if the start switched is turned on and we have already initialized the motors (only need to check one because all get initialized together)
        if (newEvent == START_BUTTON_PRESSED && motors[0].encodersInitialized())
        {
          shutDown = false;
          //check if fine control button is on - if is, then do fine control, otherwise coarse control state
          if (fineControlButton.buttonStateOn()) currState = FINE_CONTROL; //enter fine control!!
          else currState = COARSE_CONTROL;

          //enable motor control
          turnOnMotorControl();
        }
        break;
    }
} 


//function for main ino loop to call!
void updatePots(void)
{
  //this is to read pot values over bluetooth!!
  if (bluetoothMode) readPotValsBT(potAngles);
  //this is to read pot values directly from wired pots!
  else readPotVals(potAngles);
}

//read all potentiometer raw values and get angles from raw values
void readPotVals(float (& currPotAngles)[NUM_POTS])
{
  //loop through pots and get val, then convert to angle and add to return array
  for (int i = 0; i < NUM_POTS; i++)
  {
    int currPotVal = analogRead(potPins[i]);    
    currPotAngles[i] = pots[i].getAngle(currPotVal);
  }
  //correct signage 
  currPotAngles[3] = -currPotAngles[3];
}

//read all BT potentiometer angles and decode them
void readPotValsBT(float (& currPotAngles)[NUM_POTS])
{
  // Keep reading from HC-06 and send to Arduino Serial Monitor
  if (BTSerial.available())
  {
    unsigned int startCharCheck = BTSerial.read();
    if (startCharCheck == 120)
    {
      //store the actual readings -- fine button value then four pots
      while (BTSerial.available() > 0 && bufIndex < 9)
      {
        inBuffer[bufIndex] = BTSerial.read();
        bufIndex = bufIndex + 1;
      }
    }
    //decode the readings
    decodeBT(currPotAngles);
    //reset the buffer
    bufIndex = 0;
  }
}

//decode the bits received over bluetooth
void decodeBT(float (& currPotAngles)[NUM_POTS])
{
  //decode the button first
  unsigned int num = inBuffer[0];
  //check for bad readings to filter out
  if (num <= 1) fineControlSwitchState = num;
  
  //decode the pots next
  for (int i = 0; i < 4; i++)
  {
    unsigned int num = inBuffer[2*i+2];
    num = (num << 8) + inBuffer[2*i+1];
    double num2 = num;
    num2 = (num2 / 10.0) - 360;
    //check for bad readings to filter out
    if (abs(num2 - currPotAngles[i]) < 300) currPotAngles[i] = num2;
  }
} 

//for encoder A
template <int j>
void isrA ()  {
  //EncInt is which encoder interrupt fired! -- 1 for A, 2 for B
  motors[j].readEncoder(1);
}

//for encoder B
template <int j>
void isrB ()  {
  //EncInt is which encoder interrupt fired! -- 1 for A, 2 for B
  motors[j].readEncoder(2);
}

void writePWM(int i, double inputVoltage){
  analogInput = inputVoltage/12.0 * 255;
  analogWrite(motors[i].getPWMpin(), analogInput);

//  //to check motor draw... --couldn't get it working
//  if (i == 1)
//  {
//    Serial.print("Analog Input:  ");
//    Serial.println(analogInput);
//  }
  //analogWrite(motors[i].getPWMpin(), 100);
}

//turn on motor control
void turnOnMotorControl(void)
{
  motorControlMode = true;
}

//turn off motor control
void turnOffMotorControl(void)
{
  motorControlMode = false;
}

//boolean to return the motors to start pos prior to shutting off the system
bool returningToStartPos(void)
{
  return shutDown;
}

//sets motor commands to return to start pos where 2nd and 3rd motors point straight down
void returnToStartPos(void)
{  
  for(int k = 0; k < NUM_MOTORS; k++)
  {
    //drive motors back to initialized position before shutting down
    motors[k].setRefAngle(0.0);
    //get desired input
    inputU = motors[k].computePID();
    //soften input so that motors don't slam back to zero position - tune this!
    inputU = .3 * inputU;
    writePWM(k, inputU);
  }
}

//checks to see if we have completed returning to start pos at shut down 
bool checkReturnToStartComplete(void)
{
//  //to see if we have taken too long and have stalled...
//  static float startTime = millis();
//  float currTime = millis();
//  if ((currTime - startTime) >= 4000)
//  {
//    Serial.println("STALLED OUT");
//    shutDown = false;
//    return true;
//  }

  //create array and fill with placeholder large numbers
  float errorVals[3] = {10, 10, 10};

  //get current errors in motors
  for(int k = 0; k < NUM_MOTORS; k++) errorVals[k] = motors[k].getError();
  
  //set complete shut down if all of the three angles are less than threshold degrees
  //currently large so will gaurantee shut down
  //but note this then means that at start up will need to re-calibrate the motors
  if (max(abs(errorVals[0]), max(abs(errorVals[1]), abs(errorVals[2]))) < 180.0) 
  {
    Serial.println("MET STOP CRITERIA!");
    shutDown = false;
    return true;
  }
  else return false;
}

//check to see if the encoder init button was pressed
void checkEncInitButton(void)
{
  if (initEncodersButton.buttonSinglePress())
  {
    //addToQueue(ENCODER_INIT_BUTTON_PRESSED);
    //Serial.println(fineControlButton.buttonSinglePress());
    Serial.println("INIT ENCODER BUTTON PRESSED!");
    for(int k = 0; k < NUM_MOTORS; k++) motors[k].initEncoder();
    Serial.println("ENCODER INITIALIZED");
  }
}


/////////////////////wrapper function for the main ino loop/////////////

//function for main to call to command motors to the set angles
void controlMotors(void)
{
  //static variable to store servo ref angle
  static float servoRefAngle = 0;
  //store a variable for fine control mode where we can keep the previous master angles
  static float prevPotAngles[4] = {0,0,0,0};
  
  //if turned on motor control to follow given angles
  if (motorControlMode)
  {    
    for(int k = 0; k < NUM_MOTORS; k++)
    {
      //check the current fine vs coarse state to determine how to set reference angles!!
      if (currState == FINE_CONTROL)
      {
        //get the change in the master potentiometer angle
        float desiredAngleAdjust = potAngles[k] - prevPotAngles[k];
        //would 1 to 1 map it in coarse control mode, but for fine control mode we map it down for finer control
        float fineDesiredAngle = motors[k].getRefAngle() + (desiredAngleAdjust / fineControlGain);
        //set new ref to the motor
        motors[k].setRefAngle(fineDesiredAngle);
      }
      //if coarse not fine control then 1 to 1 map
      else motors[k].setRefAngle(potAngles[k]);
      //then regardless do motor command updates
      inputU = motors[k].computePID();
            
      writePWM(k, inputU);
    }

//    //now need to do the same for the servo end effector!
//    if (currState == FINE_CONTROL)
//      {
//        float desiredAngleAdjust = potAngles[3] - prevPotAngles[3];
//        //would 1 to 1 map it in coarse control mode, but for fine control mode we map it down for finer control
//        float fineDesiredAngle = servoRefAngle + (desiredAngleAdjust / fineControlGain); //maybe need to adjust this to act store previous ref angle like with DC
//        Serial.print(desiredAngleAdjust);
//        Serial.print("\t");
//        Serial.println(fineDesiredAngle);
//        servoRefAngle = fineDesiredAngle;
//        //write servo to new ref angle
//        servo.write(int(fineDesiredAngle));
//      }
//      //if coarse not fine control then 1 to 1 map and write servo to angle
//      else
//      {
//        servo.write(int(potAngles[3]));
//        servoRefAngle = potAngles[3];
//      }

      //note need to map pot angle from -90->90 to 0->180 for the servo
      int servoAng = smooth(potAngles[3])+90;  //smooth with moving average

      servo.write(servoAng);
  }
  
  //if not meant to be following the set angles AND not in shut down mode
  else if (!returningToStartPos())
  {
    for(int k = 0; k < NUM_MOTORS; k++) analogWrite(motors[k].getPWMpin(), 0);
  }
  
  //update previous stored master angles for fine control mode
  for(int k = 0; k < NUM_POTS; k++) prevPotAngles[k] = potAngles[k];
}

//check if the start switch is turned on
bool startSwitchTurnedOn(void)
{
  if (startButton.switchTurnedOn()) return true;
  else return false;
}

//check if the start switch is turned off
bool startSwitchTurnedOff(void)
{
  if (startButton.switchTurnedOff()) return true;
  else return false;
}

//see if fine control switch was turned on
bool fineControlSwitchedOn(void)
{
  //if button state is being passed over bluetooth
  if (bluetoothMode)
  {
    //this is for if button is from master over BT!
    if (fineControlSwitchState == 1) return true;
    else return false;
  }
  
  //this is for if button is wired to slave!
  else
  {
    if (fineControlButton.switchTurnedOn()) return true;
    else return false;
  }
}

//see if fine control switch was turned off
bool fineControlSwitchedOff(void)
{
  //if button state is being passed over bluetooth
  if (bluetoothMode)
  {
    //this is for if button is from master over BT!
    if (fineControlSwitchState == 0) return true;
    else return false;
  }
  
  //this is for if button is wired to slave!
  else
  {
    if (fineControlButton.switchTurnedOff()) return true;
    else return false;
  }
}

//moving average on microservo pot readings
int smooth(float potAngle) 
{
  ////Perform average on sensor readings
  long average;
  // subtract the last reading:
  total = total - readings[readIndex];
  // read the sensor:
  readings[readIndex] = potAngle;
  // add value to total:
  total = total + readings[readIndex];
  // handle index
  readIndex = readIndex + 1;
  if (readIndex >= numReadings) readIndex = 0;
  // calculate the average:
  average = total / numReadings;

  return (int) average;
}
