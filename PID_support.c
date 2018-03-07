/**********************************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

#include "stm32f10x.h"
#include "PID_support.h"

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
void PID(PIDParams * PID_Instance, float Kp, float Ki, float Kd, uint8_t POn, uint8_t ControllerDirection)
{
    PID_Instance->inAuto = 0;

    SetOutputLimits(PID_Instance, 0.0, 255.0);				//default output limit
										
    PID_Instance->SampleTime = 100;										//default Controller Sample Time is 0.1 seconds

    SetControllerDirection(PID_Instance, ControllerDirection);
    SetTunings(PID_Instance, Kp, Ki, Kd, POn);
}

/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
void Compute(PIDParams * PID_Instance)
{
	if(!PID_Instance->inAuto) return;

	/*Compute all the working error variables*/
	float input = PID_Instance->Input;
	float error = PID_Instance->Setpoint - input;
	float dInput = (input - PID_Instance->lastInput);
	PID_Instance->outputSum += (PID_Instance->ki * error);

	/*Add Proportional on Measurement, if P_ON_M is specified*/
	if(!PID_Instance->pOnE) PID_Instance->outputSum -= PID_Instance->kp * dInput;

	/*Constrain outputSum*/
	if(PID_Instance->outputSum > PID_Instance->outMax) PID_Instance->outputSum = PID_Instance->outMax;
	else if(PID_Instance->outputSum < PID_Instance->outMin) PID_Instance->outputSum = PID_Instance->outMin;

	/*Add Proportional on Error, if P_ON_E is specified*/
	float output;
	if(PID_Instance->pOnE) output = PID_Instance->kp * error;
	else output = 0;

	/*Compute Rest of PID Output*/
	output += PID_Instance->outputSum - PID_Instance->kd * dInput;

	/*Constrain output*/
	if(output > PID_Instance->outMax) output = PID_Instance->outMax;
	else if(output < PID_Instance->outMin) output = PID_Instance->outMin;
	PID_Instance->Output = output;

	/*Remember some variables for next time*/
	PID_Instance->lastInput = input;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void SetTunings(PIDParams * PID_Instance, float Kp, float Ki, float Kd, uint8_t POn)
{
   if (Kp<0 || Ki<0 || Kd<0) return;

   PID_Instance->pOn = POn;
   PID_Instance->pOnE = POn == P_ON_E;

   PID_Instance->dispKp = Kp; PID_Instance->dispKi = Ki; PID_Instance->dispKd = Kd;

   float SampleTimeInSec = ((float)PID_Instance->SampleTime)/1000;
   PID_Instance->kp = Kp;
   PID_Instance->ki = Ki * SampleTimeInSec;
   PID_Instance->kd = Kd / SampleTimeInSec;

  if(PID_Instance->controllerDirection ==REVERSE)
   {
      PID_Instance->kp = (0 - PID_Instance->kp);
      PID_Instance->ki = (0 - PID_Instance->ki);
      PID_Instance->kd = (0 - PID_Instance->kd);
   }
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void SetSampleTime(PIDParams * PID_Instance, uint32_t NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      float ratio  = (float)NewSampleTime
                      / (float)PID_Instance->SampleTime;
      PID_Instance->ki *= ratio;
      PID_Instance->kd /= ratio;
      PID_Instance->SampleTime = (unsigned long)NewSampleTime;
   }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void SetOutputLimits(PIDParams * PID_Instance, float Min, float Max)
{
   if(Min >= Max) return;
   PID_Instance->outMin = Min;
   PID_Instance->outMax = Max;

   if(PID_Instance->inAuto)
   {
	   if(PID_Instance->Output > PID_Instance->outMax) PID_Instance->Output = PID_Instance->outMax;
	   else if(PID_Instance->Output < PID_Instance->outMin) PID_Instance->Output = PID_Instance->outMin;

	   if(PID_Instance->outputSum > PID_Instance->outMax) PID_Instance->outputSum = PID_Instance->outMax;
	   else if(PID_Instance->outputSum < PID_Instance->outMin) PID_Instance->outputSum = PID_Instance->outMin;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void SetMode(PIDParams * PID_Instance, uint8_t Mode)
{
    uint8_t newAuto = (Mode == AUTOMATIC);
    if(newAuto && !PID_Instance->inAuto)
    {  /*we just went from manual to auto*/
        Initialize(PID_Instance);
    }
    PID_Instance->inAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void Initialize(PIDParams * PID_Instance)
{
   PID_Instance->outputSum = PID_Instance->Output;
   PID_Instance->lastInput = PID_Instance->Input;
   if(PID_Instance->outputSum > PID_Instance->outMax) PID_Instance->outputSum = PID_Instance->outMax;
   else if(PID_Instance->outputSum < PID_Instance->outMin) PID_Instance->outputSum = PID_Instance->outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void SetControllerDirection(PIDParams * PID_Instance, uint8_t Direction)
{
   if(PID_Instance->inAuto && Direction != PID_Instance->controllerDirection)
   {
	    PID_Instance->kp = (0 - PID_Instance->kp);
      PID_Instance->ki = (0 - PID_Instance->ki);
      PID_Instance->kd = (0 - PID_Instance->kd);
   }
   PID_Instance->controllerDirection = Direction;
}




