//make sure don't double include
#ifndef mc
#define mc

//make sure we act have arduino version necessary and 
//library included to use arduino functions
#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

class Motor {
  private:
  float motorMax, deadzone; // max motor voltage and deadzone of motor
  float countsPerRev;  //counts per 1 full revolution of 47:1 motor
  float currAngle; 
  double kp, kd, ki, umax; //PID params
  float ref, refAngle; //reference
  // Updated by the ISR (Interrupt Service Routine)
  volatile int count;  //volatile because can update outside of functions that call it, updates in interrupt
  double motor_command, prev_motor_command; //voltage command
  double e, e_prev, e_deriv, e_deriv_prev, e_integral; //storage values
  unsigned long currTime, prevTime, elapsedTime; //time since last loop iteration in ms
  int PWMpin, ENCApin, ENCBpin, EN1pin, EN2pin, currMonitor; //setting up pins motor control
  bool initEncoders;
  
  public:
  //constructor
  Motor(int pinPWM, int pinENCA, int pinENCB, int pinEN1, int pinEN2, int currSense, int revCounts);

  //methods

  //get the current motor error
  double getError(void);

  //method to set 
  void setParams(float kpNew, float kdNew, float kiNew);

  //method to account for belt drive ratio
  void setBeltRatio(float ratio);

  //update the reference counts
  void setRef(float refVal);
  
  //update the reference angle
  void setRefAngle(float refAngle);

  //get the reference angle
  float getRefAngle(void);

  //generate the motor command based on PID controller
  double computePID();
  
  //read encoder pos - called by ISR
  void readEncoder(int encInt);

  //set the motor deadzone range
  void setDeadzone(float rangeDeadzone);

  //set encoders to zero for zero angle reference
  void initEncoder(void);

  //get method for seeing if the encoders have been initialized
  bool encodersInitialized(void);
  
  //get PWM pin!
  int getPWMpin();

  //returns current count of the motor
  int getCount();
  
  //returns current angle of the motor
  float getAngle();

  //returns the current draw of the motor
  float getCurrentDraw();

  //returns count converted to angle - RELATIVE TO INITIALIZED ZERO
  float countToDegree(int count);

  //set this to tell main to initialize the encoders
  void setEncoderInit(bool logic);

  //check this to tell main to initialize the encoders
  bool doEncoderInit(void);
};

#endif
