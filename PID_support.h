#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.2.1

#define AUTOMATIC	1
#define MANUAL	0
#define DIRECT  0
#define REVERSE  1
#define P_ON_M 0
#define P_ON_E 1


typedef struct PIDParams
{
	float 		Input;
	float			Output;	
	float			Setpoint;

	float 		dispKp;								// * we'll hold on to the tuning parameters in user-entered 
	float 		dispKi;								//   format for display purposes
	float 		dispKd;								//
		
	float 		kp;                  	// * (P)roportional Tuning Parameter
	float 		ki;                  	// * (I)ntegral Tuning Parameter
	float 		kd;                  	// * (D)erivative Tuning Parameter

	uint8_t 	controllerDirection;
	uint8_t 	pOn;
				
	float 		outputSum, lastInput;

	uint32_t 	SampleTime;
	float 		outMin, outMax;
	uint8_t 	inAuto, pOnE;
	
} PIDParams;


//commonly used functions **************************************************************************
void PID(PIDParams *, float, float, float, uint8_t, uint8_t);
														
void SetMode(PIDParams *, uint8_t Mode);               // * sets PID to either Manual (0) or Auto (non-0)

void Compute(PIDParams *);                      
																			
void SetOutputLimits(PIDParams *, float, float); 
																			
//available but not commonly used functions ********************************************************
void SetTunings(PIDParams *, float, float, float, uint8_t);         	  

void SetControllerDirection(PIDParams *, uint8_t);
									
void SetSampleTime(PIDParams *, uint32_t);            
																		
void Initialize(PIDParams *);

#endif
