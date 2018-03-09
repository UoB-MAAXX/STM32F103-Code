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
 * To Do:
 *  - Make average filter use a struct and pass a pointer etc - TEST THIS!
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

PIDParams 							Altitude_PID, X_PID, Y_PID;
average_filter					Altitude_Avg, X_Avg, Y_Avg;
i2c_integral_frame 			Px4Flow_iframe;

volatile uint16_t PPM_In[PPM_CHANNELS] = {1500, 1500, 1000, 1500, 1000, 1000, 1500, 1000};
volatile uint16_t PPM_Out[PPM_CHANNELS] = {1500, 1500, 1000, 1500, 1000, 1000, 1500, 1000};

volatile int16_t Altitude_PID_Output_Avg, X_PID_Output_Avg, Y_PID_Output_Avg;
volatile int16_t constained_ground_distance, LIDAR_ground_distance, LIDAR_signal_strength;

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
void Task_Read_LIDAR(void *argument)
{
	// This Task configures then reads from the TFmini LIDAR module
	osDelay(100);
	UART_putc(0x42, UART3);
	UART_putc(0x57, UART3);
	UART_putc(0x02, UART3);
	UART_putc(0x00, UART3);
	UART_putc(0x00, UART3);
	UART_putc(0x00, UART3);
	UART_putc(0x01, UART3);
	UART_putc(0x06, UART3);
	osDelay(100);	
	
	while(1)	
	{
		// Step 1: Read the serial stream until we see the beginning of the TF Mini header
		uint8_t lastChar = 0x00;
		
		while (1) {
			if (UART_available(UART3)) {      
				uint8_t curChar = UART_getc(UART3);

				if ((lastChar == 0x59) && (curChar == 0x59)) {       
					break; 																																																		// Break to begin frame
				}        
				else {                 
					lastChar = curChar;																																												// Not seen two 0x59's in a row yet 
				}	
			}
			
			osDelay(1);
		}
		
		// Step 2: Read one frame from the TFMini
		uint8_t frame[TFMINI_FRAME_SIZE];

		for (int i = 0; i < TFMINI_FRAME_SIZE; i++) {
			while (!UART_available(UART3)) {
				osDelay(1);
			}    
			
			frame[i] = UART_getc(UART3);
		}

		// Step 3: Interpret frame and write out values
		uint16_t distance = (frame[1] << 8) + frame[0];
		uint16_t strength = (frame[3] << 8) + frame[2];  
		
		LIDAR_ground_distance = distance * 10;
		LIDAR_signal_strength = strength;
		
		constained_ground_distance = constrain(LIDAR_ground_distance, 300, 1500);
		Altitude_PID.Input = (float)constained_ground_distance;
		osDelay(5);			
	}
}	

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
void Task_Read_Px4Flow(void *argument) 
{ 
	// This Task reads from the Px4Flow over I2C and updates the Px4Flow_iframe struct with the infomation
	while(1) 
	{	
		osDelay(50);	
		update_integral(&Px4Flow_iframe);
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
	// This task sets the mode (MANUAL / ALT_HOLD / ALT_HOLD) based on the postion of a 3-position switch on the Tx. 

	while(1) 
	{
		osDelay(200);	
		
		if(PPM_In[MODE_CH] < 1250)			
		{
			SetMode(&Altitude_PID, MANUAL);
			SetMode(&X_PID, MANUAL);
			SetMode(&Y_PID, MANUAL);
		}
		else if(PPM_In[MODE_CH] < 1750) 
		{
			SetMode(&Altitude_PID, AUTOMATIC);
			SetMode(&X_PID, MANUAL);
			SetMode(&Y_PID, MANUAL);
		}		
		else
		{
			SetMode(&Altitude_PID, AUTOMATIC);
			SetMode(&X_PID, AUTOMATIC);
			SetMode(&Y_PID, AUTOMATIC);			
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
	
	UART_putstrln("Starting up!", UART1);
	
	while(1) 
	{
		osDelay(50);
		UART_flush_TX(UART1);
		   
		// ------------------- Transmit info for debugging purposes:
		UART_putnum((int32_t)Altitude_PID.Input, UART1); 							UART_putstr(",", UART1);												
//  UART_putnum(Px4Flow_iframe.sonar_ground_distance, UART1); 		UART_putstr(",", UART1);
//	UART_putnum((int32_t)Altitude_PID.Setpoint); 									UART_putstr(",", UART1);															
//	UART_putnum((int32_t)Altitude_PID.Output); 						  			UART_putstr(",", UART1);
//  UART_putnum(Altitude_PID_Output_Avg, UART1); 									UART_putstr(",", UART1);	  		
		UART_putnum(Px4Flow_iframe.pixel_flow_x_integral, UART1);			UART_putstr(",", UART1);
		UART_putnumln(Px4Flow_iframe.pixel_flow_y_integral, UART1);
		
		// ------------------- Receive info for PID tuning etc:
		if(UART_available(UART1))																																														
		{
			UART_getstr(Buffer, '\n', UART1);																																							// Writes to buffer until a NL character is recieved. Appends NULL terminator
			fvalue = ((float)stoi(&Buffer[1])) * 0.001;
			
			switch(Buffer[0])																																															// Format:    p45
			{				
				case 'p':
					SetTunings(&Altitude_PID, fvalue, Altitude_PID.dispKi, Altitude_PID.dispKd, P_ON_E);	
					UART_putstr("New P: ", UART1);
					break;
				
				case 'i':
					SetTunings(&Altitude_PID, Altitude_PID.dispKp, fvalue, Altitude_PID.dispKd, P_ON_E);
					UART_putstr("New I: ", UART1);
					break;
				
				case 'd':
					SetTunings(&Altitude_PID, Altitude_PID.dispKp, Altitude_PID.dispKi, fvalue, P_ON_E);
					UART_putstr("New D: ", UART1);
					break;
				
				case 's':
					Altitude_PID.Setpoint = fvalue * 1000.0;
					UART_putstr("New S: ", UART1); UART_putnumln((int32_t)Altitude_PID.Setpoint, UART1);
					break;

				default:
					UART_putstr("#", UART1);
					break;
			}
			
			UART_putstr(" P:", UART1); UART_putnum((int32_t)(Altitude_PID.dispKp * 1000.0), UART1); 
			UART_putstr(" I:", UART1); UART_putnum((int32_t)(Altitude_PID.dispKi * 1000.0), UART1); 
			UART_putstr(" D:", UART1); UART_putnum((int32_t)(Altitude_PID.dispKd * 1000.0), UART1); 
			UART_putstr(" S:", UART1); UART_putnumln((int32_t)Altitude_PID.Setpoint, UART1);
			
			osDelay(1000);
			UART_flush_RX(UART1);
		}		
	}
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
void Task_PID(void *argument) 
{		
	// This task runs frequently. It configures PIDs and adjusts setpoints. -> This could be moved and the while loop done as a periodic fnx
	PID(&Altitude_PID, 0.020, 0.004, 0.040, P_ON_E, DIRECT);
	SetSampleTime(&Altitude_PID, ALTITUDE_PID_SAMPLE_T);
	SetOutputLimits(&Altitude_PID, -150.0, 150.0);
	Altitude_PID.Setpoint = 800.0;																																										// Default setpoint (mm)

	PID(&X_PID, 0.010, 0.000, 0.020, P_ON_E, DIRECT);
	SetSampleTime(&X_PID, X_PID_SAMPLE_T);
	SetOutputLimits(&X_PID, -50.0, 50.0);
	X_PID.Setpoint = 0.0;
	
	PID(&Y_PID, 0.010, 0.000, 0.020, P_ON_E, DIRECT);
	SetSampleTime(&Y_PID, Y_PID_SAMPLE_T);
	SetOutputLimits(&Y_PID, -50.0, 50.0);
	Y_PID.Setpoint = 0.0;
	
	while(1) 
	{		
		osDelay(100);
		
		if(PPM_In[THROTTLE_CH] < 1100)	{																																								// Avoid PID windup if we're sat on the floor
			Altitude_PID.outputSum = 0;
			X_PID.outputSum = 0;
			Y_PID.outputSum = 0;
		}																		
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
		osDelay(5);			

		PPM_Out[ROLL_CH] 			= PPM_In[ROLL_CH]; 			// + X_PID_Output_Avg
		PPM_Out[PITCH_CH] 		= PPM_In[PITCH_CH];			// + Y_PID_Output_Avg															
		PPM_Out[THROTTLE_CH] 	= PPM_In[THROTTLE_CH] + Altitude_PID_Output_Avg;																								
		PPM_Out[YAW_CH] 			= PPM_In[YAW_CH];			
		PPM_Out[ARM_CH] 			= PPM_In[ARM_CH];
		PPM_Out[MODE_CH] 			= PPM_In[MODE_CH];
		PPM_Out[ALT_CH] 			= PPM_In[ALT_CH];
		PPM_Out[AUX_CH] 			= PPM_In[AUX_CH];	
	}
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
static void Periodic_Altitude_PID (void *argument) 
{	
	Compute(&Altitude_PID);
	Altitude_PID_Output_Avg = Average(&Altitude_Avg, (int16_t)Altitude_PID.Output);
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
static void Periodic_X_PID (void *argument) 
{	
	Compute(&X_PID);
	X_PID_Output_Avg = Average(&X_Avg, (int16_t)X_PID.Output);
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
static void Periodic_Y_PID (void *argument) 
{	
	Compute(&Y_PID);
	Y_PID_Output_Avg = Average(&Y_Avg, (int16_t)Y_PID.Output);
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
	osThreadNew(Task_Read_LIDAR, 		NULL, NULL);
	osThreadNew(Task_PID, 					NULL, NULL);

	// Start periodic timers to handle the PIDs for Altitude hold, etc
	osTimerId_t Periodic_Altitude_PID_id, Periodic_X_PID_id, Periodic_Y_PID_id;
	
	Periodic_Altitude_PID_id = osTimerNew(Periodic_Altitude_PID, osTimerPeriodic, NULL, NULL);
	osTimerStart(Periodic_Altitude_PID_id, ALTITUDE_PID_SAMPLE_T);		

//	Periodic_X_PID_id = osTimerNew(Periodic_X_PID, osTimerPeriodic, NULL, NULL);
//	osTimerStart(Periodic_X_PID_id, X_PID_SAMPLE_T);	

//	Periodic_Y_PID_id = osTimerNew(Periodic_Y_PID, osTimerPeriodic, NULL, NULL);
//	osTimerStart(Periodic_Y_PID_id, Y_PID_SAMPLE_T);		
	
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
	gpio_init();																					// Initialises GPIO for blinked LED
	ppm_init();																						// Initialises for CPPM input
	TIM3_Init();																					// Initialises a 1us counter for CPPM output timings. Generates compare interrupts
	USART1_Init();																				// Initialises UART1 functions, FIFO & interrupts for serial tx through telementry
	USART3_Init();																				// Initialises UART3 functions, FIFO & interrupts for comms with TFmini Lidar module
	I2C_Init_fnc();																				// Initialises I2C functions, FIFO & interrupts

	// Initialise variables
	Altitude_Avg.Filter_Size = 20;
	X_Avg.Filter_Size = 10;
	Y_Avg.Filter_Size = 10;
	
	// OS System Initialization
	SystemCoreClockUpdate();
	
	osKernelInitialize(); 																// Initialize CMSIS-RTOS		 
	osThreadNew(app_main, NULL, NULL); 										// Create application main thread
	osKernelStart(); 																			// Start thread execution
	
	osDelay(osWaitForever);
	while(1) {}		
}
























