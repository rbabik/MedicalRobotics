#include "motor_control.h"
#include "potentiometer.h"
#include "button.h"
#include "state_machine.h"
#include "Event.h"


/*
 * README
 * CODE OVERVIEW
 * This is event-driven programming with a state machine to control active behavior
 * The main loop is used to constantly check for "interrupts" due to human interaction and also
 * continually manage other aspects that need constant update, such as reference angle readings and 
 * motor commands based on PID controls.
 * 
 * In the state machine file there is a boolean variable to set whether you are communicating master
 * reference angles over bluetooth or tethered connection. By swtiching that varaible, you will switch between 
 * the two modes. NOTE that bluetooth mode is too noisy and has too much latency to be effectively used as is. It
 * is highly recommended that before just "plugging and playing," revisions to the wireless comm method be made. Trying
 * immediately to use BT as is with the full hardware manipulator may damage the motors due to jerky and/or noisy commands.
 * Also note that for BT to work, you must first unplug the tethered potentiometer cables from the master and plug them into
 * the master Arduino Uno, which must also be powered on.
 * Tethered, which is the default, works great!
 * 
 * To use the robot, plug in the 12V power supply on the slave as well as the separate USB power supply for the arduino
 * Then, on the slave flip the start switch to on, and then the slave will begin following the master.
 * Often you need to recalibrate the slave DC motor encoders to effectively mimic the master joint angles, so if this is the case, 
 * then adjus the master links until the slave links (minus the end effector joint as this is servo driven not DC) 
 * both point directly down and the top rotation is at a good orientation to call "zero." Then, on the master just press once the 
 * zero encoder button and now the slave arm will accurately reflect the master configuration.
 * 
 * On the master there is also a fine control switch. When turned on, joint angle adjustments on the master map 2:1 to the slave. 
 * This allows for more fine control of the slave and is intended for use when you have already moved right to the surgical site and
 * can sacrifice a large workspace for more precise control. 
 * 
 * As always, use caution with the robotic arm and operate safely. Monitor heat dissipation in top slave controls box, and be sure to 
 * turn off the system power before ever adjusting electronics. Enjoy!
 * 
*/



void setup() {
  
  //initialize the state machine
  initStateMachine();

  delay(5);
}

void loop() {
  // put your main code here, to run repeatedly:

  //check for start switch turned on
  if (startSwitchTurnedOn()) 
  {
    Serial.println("START SWITCH TURNED ON"); 
    addToQueue(START_BUTTON_PRESSED);
  }
  
  //check for start switch turned off
  if (startSwitchTurnedOff()) 
  {
    Serial.println("START SWITCH TURNED OFF"); 
    addToQueue(START_BUTTON_UNPRESSED);
  }

  //check for if trying to swtich to fine control from coarse control
  if (fineControlSwitchedOn())
  {
    Serial.println("FINE CONTROL SWITCH TURNED ON");
    addToQueue(FINE_CONTROL_BUTTON_PRESSED);
  }

  //check for if trying to swtich to fine control from coarse control
  if (fineControlSwitchedOff()) 
  {
    Serial.println("FINE CONTROL SWITCH TURNED OFF");
    addToQueue(FINE_CONTROL_BUTTON_UNPRESSED); 
  }

  //check for if we need to return to 
  if (returningToStartPos())
  {
    Serial.println("Returning to start pos");
    returnToStartPos();
    if (checkReturnToStartComplete())
    {
      Serial.println("SHUT DOWN COMPLETE");
      addToQueue(SHUT_DOWN_COMPLETE);
    }
  }

  //continually check for if the encoder init button is pressed
  checkEncInitButton();
  
  //continually read new master pot values
  updatePots();

  //continually update motor controls if activated
  controlMotors();

  //service an event from the queue if one is there
  serviceEventQueue();
}
