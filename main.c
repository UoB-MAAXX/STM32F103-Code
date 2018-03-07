/*
 * This program receives CPPM from a RF receiver and passes it through to the flight controller.
 * 
 * Depending on channel 6:
 * 1) CPPM pass through, full manual control.
 * 2) Altitude hold by adjusting throttle value using a PID and sensor readings from the Px4Flow sonar sensor
 * 3) Serial data from RPi3 is received for full autonomous control - Not implemented yet!
 * 
 * Some PID loops are run:
 * - To keep altitude constant
 * - To reach a postion setpoint - Not implemented yet!
 * 
 * 
 * We move the position setpoint around in order to get the quad to follow this setpoint route. 
 * 
 * We can initially just have the setpoint moving around the track blind. We can then introduce some correction info from the Pi. Eg setting X or Y
 * based on visual info.
 * 
 * 
 * Aside from this regular serial transmissions are made to send data over telementry. 
 * Eg, Battery voltage and current draw, Optical flow readings, Sonar readings etc.
 * 
 * Data can also be recieved from the computer to command the quad to stay put, move or adjust PID values etc
 * 
 * 
 * So the periodically run fxs include:
 * - Read Px4Flow
 * - Rx/Tx Telementry
 * - Rx from RPi3
 * 
 * Currently working on:
 * - Getting the quad to hold its own altitude using its sonar readings. 
 *
 * 
 * To Do:
 *  - TEST, TEST, TEST!!!
 *
 *
 *  - Position_Hold mode in the Pass_through task
 *  - Tune X & Y PIDs for position hold mode.
 *  - In the PID_Task, allow for position setpoint adjustment based on a map of position coordinates to work through. Slow sections have higher density of coords etc  

 *  - Get UART from Pi working.
 *  - Allow Pi to send X,Y,Yaw corrections
 *  - Allow for Yaw correction
 *  
 */
 
#include "stm32f10x.h"
#include "cmsis_os2.h"
#include "main.h"
#include "support.h"
#include "I2C_support.h"
#include "UART_support.h"
#include "PID_support.h"

#include "string.h"
#include "stdio.h"

PIDParams SonarPID;

i2c_integral_frame iframe;

volatile uint16_t PPM_In[PPM_CHANNELS] = {1500, 1500, 1000, 1500, 1000, 1000, 1500, 1000};
volatile uint16_t PPM_Out[PPM_CHANNELS] = {1500, 1500, 1000, 1500, 1000, 1000, 1500, 1000};

volatile uint8_t MODE = MANUAL_CTRL;

volatile int16_t constained_ground_distance;
volatile int16_t median_filtered_ground_distance;
volatile int16_t slew_limited_ground_distance;

volatile int16_t Filtered_PID_Output;

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
void Task_Read_Px4Flow(void *argument) 
{ 
	// This Task reads from the Px4Flow over I2C and updates the iframe struct with the infomation
	while(1) 
	{	
		osDelay(50);	
		update_integral(&iframe);

		constained_ground_distance = constrain(iframe.ground_distance, 300, 2000);															
		median_filtered_ground_distance = median_filter(constained_ground_distance);
		slew_limited_ground_distance = slew_limiter(median_filtered_ground_distance);	
		SonarPID.Input = (float)slew_limited_ground_distance;
	}
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
void Task_Alive_LED(void *argument) 
{				
	// This task blinks an LED to show that the STM32F103 is running our code.
	while(1)
	{
		GPIO_SetBits(GPIOC, GPIO_Pin_13);
		osDelay(200);
		
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		osDelay(200);
	}	
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
void Task_Ctrl_Mode(void *argument) 
{ 
	// This task sets the MODE (MANUAL / ALT_HOLD / ALT_HOLD) based on the postion of a 3-position switch on the Tx. (ANOTHER MODE TO BE ADDED)
	// Altitide hold mode is only activated once a certain upper_threshold height is reached. It is deactivated again below a lower_threshold
  // This allows one to take off in ALT_HOLD and for the system to snap to the setpoint height only once the throttle is roughly correct.
	static int16_t height_threshold = 500;

	while(1) 
	{
		osDelay(200);	
		
		if(PPM_In[MODE_CH] < 1250 || (int16_t)SonarPID.Input < height_threshold)
		{
			MODE = MANUAL_CTRL;
			SetMode(&SonarPID, MANUAL);
			height_threshold = 500;
		}
		else
		{
			MODE = ALT_HOLD_CTRL;
			SetMode(&SonarPID, AUTOMATIC);
			height_threshold = 325;
		}		
	}
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
void Task_Telemetry(void *argument) 
{ 	
	// This task transmits debugging info over telementry to the computer
	static char Buffer[16];
	float fvalue;
	
	UART_putstrln("Starting up!");
	
	while(1) 
	{
		osDelay(50);
		UART_flush_TX();
		   
		// ------------------- Transmit info for debugging purposes:
		UART_putnum((int32_t)SonarPID.Input); 							UART_putstr(",");																						// Print PID Input 1st ie. filtered ultrasound readings
		//UART_putnum((int32_t)SonarPID.Setpoint); 						UART_putstr(",");																						// Print setpoint height 2nd
		UART_putnum((int32_t)SonarPID.Output); 						  UART_putstr(",");
		UART_putnumln(Filtered_PID_Output); 						  //UART_putstr(",");		
		
		
		// ------------------- Receive info for PID tuning etc:
		if(UART_available())																																														
		{
			UART_getstr(Buffer, '\n');																																										// Writes to buffer until a NL character is recieved. Appends NULL terminator
			fvalue = ((float)stoi(&Buffer[1])) * 0.001;
			
			switch(Buffer[0])																																															// Format:    i0.045e
			{				
				case 'p':
					SetTunings(&SonarPID, fvalue, SonarPID.dispKi, SonarPID.dispKd, P_ON_E);	
					UART_putstr("P updated: ");
					break;
				
				case 'i':
					SetTunings(&SonarPID, SonarPID.dispKp, fvalue, SonarPID.dispKd, P_ON_E);
					UART_putstr("I updated: ");
					break;
				
				case 'd':
					SetTunings(&SonarPID, SonarPID.dispKp, SonarPID.dispKi, fvalue, P_ON_E);
					UART_putstr("D updated: ");
					break;

				default:
					UART_putstr("PIDs ->");
					break;
			}
			
			UART_putstr("  P: "); UART_putnum((int32_t)(SonarPID.dispKp * 1000.0)); 
			UART_putstr("  I: "); UART_putnum((int32_t)(SonarPID.dispKi * 1000.0)); 
			UART_putstr("  D: "); UART_putnumln((int32_t)(SonarPID.dispKd * 1000.0)); 
			
			osDelay(1000);
			UART_flush_RX();
		}		
	}
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
void Task_PID(void *argument) 
{		
	// This task runs frequently. It configures PIDs and adjusts setpoints. -> This could be moved and the while loop done as a periodic fnx
	PID(&SonarPID, 0.03, 0.0, 0.07, P_ON_E, DIRECT);
	SetSampleTime(&SonarPID, 55);
	SetOutputLimits(&SonarPID, -150.0, 150.0);
	SonarPID.Setpoint = 700.0;
		
	while(1) 
	{		
		osDelay(100);
		
//		SonarPID.Setpoint = (float)(constrain(PPM_In[ALT_CH], 1000, 2000) - 600);																				// Adjust Setpoint based on ALT_CH Tx Knob between 400 - 1400 millimetres
	}
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
void Task_Pass_Through(void *argument) 
{ 		
	// This task runs regularly and passes unmodified/modified PPM values from RF rx to FC depending on MODE (Manual, Altitude hold)
	while(1) 
	{
		osDelay(4);			
		
		if(MODE == ALT_HOLD_CTRL)	
		{		
			PPM_Out[ROLL_CH] 			= PPM_In[ROLL_CH];
			PPM_Out[PITCH_CH] 		= PPM_In[PITCH_CH];			
																
			PPM_Out[THROTTLE_CH] 	= PPM_In[THROTTLE_CH] + Filtered_PID_Output;																				// Adjust Throttle based on Sonar PID output									
			
			PPM_Out[YAW_CH] 			= PPM_In[YAW_CH];			
			PPM_Out[ARM_CH] 			= PPM_In[ARM_CH];
			PPM_Out[MODE_CH] 			= PPM_In[MODE_CH];
			PPM_Out[ALT_CH] 			= PPM_In[ALT_CH];
			PPM_Out[AUX_CH] 			= PPM_In[AUX_CH];		
		}  
		
		else	
		{
			PPM_Out[ROLL_CH] 			= PPM_In[ROLL_CH];
			PPM_Out[PITCH_CH] 		= PPM_In[PITCH_CH];
			PPM_Out[THROTTLE_CH] 	= PPM_In[THROTTLE_CH];
			PPM_Out[YAW_CH] 			= PPM_In[YAW_CH];			
			PPM_Out[ARM_CH] 			= PPM_In[ARM_CH];
			PPM_Out[MODE_CH] 			= PPM_In[MODE_CH];
			PPM_Out[ALT_CH] 			= PPM_In[ALT_CH];
			PPM_Out[AUX_CH] 			= PPM_In[AUX_CH];			
		}			
	}
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
static void Periodic_Sonar_PID (void *argument) 
{	
	Compute(&SonarPID);
	Filtered_PID_Output = PID_Output_average((int16_t)SonarPID.Output);
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
void app_main (void *argument) 
{	
	osDelay(5000);
	
	// Start some tasks
	osThreadNew(Task_Alive_LED, 		NULL, NULL);		
	osThreadNew(Task_Ctrl_Mode, 		NULL, NULL);
	osThreadNew(Task_Pass_Through, 	NULL, NULL);
	osThreadNew(Task_Telemetry, 		NULL, NULL);
	osThreadNew(Task_Read_Px4Flow, 	NULL, NULL);
	osThreadNew(Task_PID, 					NULL, NULL);

	// Start periodic timers to handle the PIDs for Altitude hold, etc
	osTimerId_t Periodic_Sonar_PID_id;
	Periodic_Sonar_PID_id = osTimerNew(Periodic_Sonar_PID, osTimerPeriodic, NULL, NULL);
	osTimerStart(Periodic_Sonar_PID_id, 55);	
	
	// This forms the idle task
	osDelay(osWaitForever);
	while(1) {} 
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
int main(void)
{	
	// Initialise timers and interrupts
	gpio_init();
	ppm_init();
	TIM3_Init();																					// Initialises a 1us counter for CPPM output timings. Generates compare interrupts
	USART1_Init();																				// Initialises UART1 functions, FIFO & interrupts for serial tx through telementry
	I2C_Init_fnc();																				// Initialises I2C functions, FIFO & interrupts

	// OS System Initialization
	SystemCoreClockUpdate();
	
	osKernelInitialize(); 																// Initialize CMSIS-RTOS		 
	osThreadNew(app_main, NULL, NULL); 										// Create application main thread
	osKernelStart(); 																			// Start thread execution
	while(1) {}		
}























