/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STATEMACHINE_H_
#define __STATEMACHINE_H_


/* Includes ------------------------------------------------------------------*/
#include "Event.h" 

/*State Enums/Macros---------------------------------------------------------*/

//enumerated for the different states of the state machine
typedef enum
{
    WAIT,
    COARSE_CONTROL, //tihs is fine control switch open
    FINE_CONTROL,  //this is fine control switch closed
    SHUT_DOWN     //this is for when we need to give the arm time to go back to zero position at shut down
} machineState;
//make machine state enum

/*Initializations---------------------------------------------------------*/


/*Function definitions---------------------------------------------------------*/

//set state to initialize
void initStateMachine(void);

//state machine to receive states and change states accordingly
void stateMachine(event_type newEvent);

//turn on motor control
void turnOnMotorControl(void);

//boolean to return the motors to start pos prior to shutting off the system
bool returningToStartPos(void);

//sets motor commands to return to start pos where 2nd and 3rd motors point straight down
void returnToStartPos(void);

//checks to see if we have completed returning to start pos at shut down 
bool checkReturnToStartComplete(void);

//turn off motor control
void turnOffMotorControl(void);

//function for main ino loop to call!
void updatePots(void);

//moving average on microservo pot readings
int smooth(float potAngle);

//read all potentiometer raw values and get angles from raw values
void readPotVals(float (& currPotAngles)[4]); //[NUM_POTS]);

//read all BT potentiometer angles and decode them
void readPotValsBT(float (& currPotAngles)[4]); //[NUM_POTS]);

//decode the bits received over bluetooth
void decodeBT(float (& currPotAngles)[4]); //[NUM_POTS]);

//check to see if the encoder init button was pressed
void checkEncInitButton(void);

//for encoder A
template <int j>
void isrA (void);

template <int j>
void isrB (void);

//////////wrapper functions for the ino main loop to call!!!///////////////

//check if the start switch is turned on
bool startSwitchTurnedOn(void);

//check if the start switch is turned off
bool startSwitchTurnedOff(void);

//see if fine control switch was turned on
bool fineControlSwitchedOn(void);

//see if fine control switch was turned off
bool fineControlSwitchedOff(void);

//function for main to call to command motors to the set angles
void controlMotors(void);


#endif /*__STATEMACHINE_H_ */
