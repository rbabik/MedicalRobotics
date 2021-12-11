#include "motor_control.h"


Motor::Motor(int pinPWM, int pinENCA, int pinENCB, int pinEN1, int pinEN2, int currSense, int revCounts){
  //instantiate some stuff for the motor/PID control
  prevTime = 0.0; //ms
  e_deriv_prev = 0.0;
  motorMax = 12.0; //volts NOTE: should be 255 in PWM
  deadzone = 0; //PWM NOTE: need to experimentally update with full links
  count = 0;
  currAngle = 0;
  countsPerRev = revCounts; //2249; //motor spec for 47:1 gear ratio
  prev_motor_command = 0;
  initEncoders = false;
  refAngle = 0;

  //define this motor's pins for communication
  PWMpin = pinPWM; ENCApin = pinENCA; ENCBpin = pinENCB; EN1pin = pinEN1, EN2pin = pinEN2; currMonitor = currSense;
}

void Motor::setParams(float kpNew, float kdNew, float kiNew){
  kp = kpNew; kd = kdNew; ki = kiNew;
  }

//method to account for belt drive ratio
void Motor::setBeltRatio(float ratio)
{
  countsPerRev = countsPerRev * ratio;
}
  
void Motor::setRef(float refVal){
  ref = refVal;
}

//get the current motor error
double Motor::getError(void)
{
  //return current motor error
  return e;  
}

void Motor::setRefAngle(float angleRef){
  refAngle = angleRef;
}

//get the reference angle
float Motor::getRefAngle(void)
{
  return refAngle;
}

int Motor::getPWMpin(){
  return PWMpin;
}

int Motor::getCount(){
  return count;
}

float Motor::getAngle(){
  return currAngle;
}

//returns the current draw of the motor
float Motor::getCurrentDraw()
{
  float current = analogRead(currMonitor);///140; //channel reads up to 12A current, 140mV per amp
  return current;
}

//set encoders to zero for zero angle reference
void Motor::initEncoder()
{
  count = 0;
  currAngle = 0;
  setEncoderInit(true);
}

//set the motor deadzone range
void Motor::setDeadzone(float rangeDeadzone)
{
  deadzone = rangeDeadzone;
}

void Motor::setEncoderInit(bool logic)
{
  initEncoders = logic;
}

bool Motor::doEncoderInit(void)
{
  bool temp = initEncoders; //get curr value
  setEncoderInit(false); //reset initencoders value
  return temp;
}

//get method for seeing if the encoders have been initialized
bool Motor::encodersInitialized(void)
{
  return initEncoders;
}
  

//returns count converted to angle - RELATIVE TO INITIALIZED ZERO
float Motor::countToDegree(int count)
{
  currAngle = count/countsPerRev * 360.0;
  return currAngle;
}

double Motor::computePID(){
  //get time update since last loop run
  currTime = millis();
  elapsedTime = (double)(currTime - prevTime);
  
  //get time for deriv calc...
  static unsigned long prevTime_deriv = 0;
  unsigned long currTime_deriv = millis();
  double elapsedTime_deriv = (double)(currTime_deriv - prevTime_deriv);

  //update angles based on interrupt counts
  currAngle = countToDegree(count);
  //update errors
  e = refAngle - currAngle;  //error
  e_integral += e * elapsedTime; //integral error
  if (elapsedTime_deriv >= 5)
  {
    e_deriv = (e - e_prev) / elapsedTime;  //derivative error
  }
  else 
  {
    e_deriv = e_deriv_prev;
  }
  //Filter derivative error to prevent noisy spikes
  e_deriv = .8 * (e_deriv) + .2 * e_deriv_prev;

  //calc motor command
  motor_command = kp*e + ki*e_integral + kd*e_deriv;

  if (motor_command <= 0){
    digitalWrite(EN1pin, LOW);
    digitalWrite(EN2pin, HIGH);
  }
  else {
    digitalWrite(EN1pin, HIGH);
    digitalWrite(EN2pin, LOW);
  }


  //check for saturation
  motor_command = min(motorMax,max(-motorMax, motor_command));

  //update so it is only positive
  motor_command = abs(motor_command);

  //need to now account for deadzone in motor command...
  motor_command = motor_command + deadzone/255*12;

  //check for positive saturation only now...
  motor_command = min(motor_command, motorMax);
  
  //update new previous values
  e_prev = e; e_deriv_prev = e_deriv; 
  prevTime = currTime; prevTime_deriv = currTime_deriv;
  prev_motor_command = motor_command;

  return motor_command;
}

//https://howtomechatronics.com/tutorials/arduino/rotary-encoder-works-use-arduino/
//quadrature encoder!! Clockwise is ++ in count!!
void Motor::readEncoder(int encInt){
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();

  //see if encoder A fired...
  if (encInt == 1)
  {
    //see if encoder A is high
    if (digitalRead(ENCApin) == HIGH)
    {
      if (digitalRead(ENCBpin) == LOW) count++;
      if (digitalRead(ENCBpin) == HIGH) count--;
      }
      //see if encoder A is low
      if (digitalRead(ENCApin) == LOW)
      {
      if (digitalRead(ENCBpin) == HIGH) count++;
      if (digitalRead(ENCBpin) == LOW) count--;
    }
  }
  //see if encoder B fired...
  if (encInt == 2)
  {
    //see if encoder B is high
    if (digitalRead(ENCBpin) == HIGH)
    {
      if (digitalRead(ENCApin) == HIGH) count++;
      if (digitalRead(ENCApin) == LOW) count--;
      }
      //see if encoder B is low
      if (digitalRead(ENCBpin) == LOW)
      {
      if (digitalRead(ENCApin) == LOW) count++;
      if (digitalRead(ENCApin) == HIGH) count--;
    }
  }

  //check if we have wrapped a full 360 degree rotation
  if (abs(count) > countsPerRev) count = 0;
}
