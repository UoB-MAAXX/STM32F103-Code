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
 * We use the px4Flow to measure a velocity vector. We take data from the pi and modifiy the setpoint velocity vector. PIDs 
 * 
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
 *  - Reduce stick sensitivity in betaflight
 *  - TEST X_PID_Output_Avg PID outputs etc and moving from alt_hold to full auto mode in the air. Engage the PIDs in the pass through routine
 *  - Tune X & Y PIDs for velocity hold mode
 *  - Get UART from Pi working.
 *  - Allow Pi to send X,Y corrections
 *
 *
 *	Hardware:
 *  - Add LED downlighting for shorter camera exposure times and less motion blur
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

PIDParams 							Altitude_PID, X_PID, Y_PID;																																	// Our PID structures
average_filter					Altitude_Avg, X_Avg, Y_Avg, V_Avg, I_Avg;																										// Our average filter structures
i2c_integral_frame 			Px4Flow_iframe;																																							// Structure copied from Px4Flow containing flow data etc

volatile uint16_t PPM_In[PPM_CHANNELS] = {1500, 1500, 1000, 1500, 1000, 1000, 1500, 1000};													// These are updated with fresh values from the Rx
volatile uint16_t PPM_Out[PPM_CHANNELS] = {1500, 1500, 1000, 1500, 1000, 1000, 1500, 1000};													// These values get output to the FC

volatile int16_t Altitude_PID_Output_Avg, X_PID_Output_Avg, Y_PID_Output_Avg;																				// These hold averages of our PID outputs

volatile uint16_t LIDAR_ground_distance, LIDAR_signal_strength;																											
volatile uint16_t Current, Voltage;																																									// mA and mV
volatile float    Charge = 0;																																												// mAh accumulated since power on

volatile int32_t X_Position = 0, X_Velocity, X_Setpoint = 0, Y_Position = 0, Y_Velocity, Y_Setpoint = 0;

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
		
		LIDAR_ground_distance = distance * 10;																																					// Convert cm to mm
		LIDAR_signal_strength = strength;
		
		Altitude_PID.Input = (float)constrain(LIDAR_ground_distance, 300, 2000);
		osDelay(5);			
	}
}	

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
void Task_IV(void *argument) 
{ 
	// This Task reads the ADC which is connected to the PDB to measure current from the battery and voltage.

	V_Avg.Filter_Size = 20;
	I_Avg.Filter_Size = 20;
	Charge = 0;
	
	//Enable ADC1 reset calibaration register
   ADC_ResetCalibration(ADC1);
   while (ADC_GetResetCalibrationStatus(ADC1)); 																																		//Check the end of ADC1 reset calibration register

   //Start ADC1 calibaration
   ADC_StartCalibration(ADC1);
   while (ADC_GetCalibrationStatus(ADC1)); 																																					//Check the end of ADC1 calibration

	while(1) 
	{	
		osDelay(100);	
		
		ADC_RegularChannelConfig(ADC1, 8, 1, ADC_SampleTime_28Cycles5);
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);
		while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
		uint32_t V = ADC_GetConversionValue(ADC1) * 4468;  																															// 4.468mv per count (There's 10Kohm | 2.2Kohm resistor divider)
		V /= 1000;		
		
		ADC_RegularChannelConfig(ADC1, 9, 1, ADC_SampleTime_28Cycles5);
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);
		while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
		uint32_t I = ADC_GetConversionValue(ADC1) * 8057;																																// No resistor divider on the I sensing
		I /= 472;																																																		  	// 23.6mv/A 
		
		// Average update globals
		Voltage = Average(&V_Avg, V);
		Current = Average(&I_Avg, I) * 2;																																								// Undoes the I /= 472 rather than 236 above which allows us to 
	}																																																									// measure currents up to 65 Amps with 16bit variables since Average()
}																																																										// takes uint16_t variables as input for speed.

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
		
		X_Velocity = Px4Flow_iframe.pixel_flow_x_integral;
		Y_Velocity = Px4Flow_iframe.pixel_flow_y_integral;
		
		X_Position += Px4Flow_iframe.pixel_flow_x_integral;
		Y_Position += Px4Flow_iframe.pixel_flow_y_integral;
		
		X_PID.Input = ((float)X_Velocity) / 100.0;
		Y_PID.Input = ((float)Y_Velocity) / 100.0;
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
			X_PID.outputSum = 0;
			Y_PID.outputSum = 0;
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
		osDelay(40);
		UART_flush_TX(UART1);
		   
		// ------------------- Transmit info for debugging purposes:
		UART_putnum((int32_t)Altitude_PID.Input, UART1); 							
		UART_putstr(",", UART1);
		osDelay(5);
		
		UART_putnum((int32_t)Charge, UART1); 													
		UART_putstr(",", UART1);
		osDelay(5);
		
		UART_putnum(Current, UART1);																	
		UART_putstr(",", UART1);
		osDelay(5);
		
		UART_putnumln(Voltage, UART1);
		
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
	// This task runs frequently. It configures PIDs and adjusts setpoints.
	PID(&Altitude_PID, 0.020, 0.004, 0.040, P_ON_E, DIRECT);
	SetSampleTime(&Altitude_PID, ALTITUDE_PID_SAMPLE_T);
	SetOutputLimits(&Altitude_PID, -150.0, 150.0);
	Altitude_PID.Setpoint = 800.0;																																										// Default setpoint (mm)

	PID(&X_PID, 0.100, 0.005, 0.040, P_ON_E, DIRECT);
	SetSampleTime(&X_PID, X_PID_SAMPLE_T);
	SetOutputLimits(&X_PID, -50.0, 50.0);
	X_PID.Setpoint = (float)X_Setpoint;															// These will need to be updated constantly based on RPi3 data!!																									
	
	PID(&Y_PID, 0.100, 0.005, 0.040, P_ON_E, DIRECT);
	SetSampleTime(&Y_PID, Y_PID_SAMPLE_T);
	SetOutputLimits(&Y_PID, -50.0, 50.0);
	Y_PID.Setpoint = (float)Y_Setpoint;
	
	uint16_t threshold_count = 0;
	
	while(1) 
	{		
		osDelay(100);
		
		if(PPM_In[THROTTLE_CH] < 1100)	{																																								// Avoid PID windup if we're sat on the floor with a low throttle value
			threshold_count++;																																														// (>1 second) We don't want a spurious midflight reading causing havoc
			
			if(threshold_count > 10)	{
				Altitude_PID.outputSum = 0;
				X_PID.outputSum = 0;
				Y_PID.outputSum = 0;
				
				X_Position = 0;
				Y_Position = 0;
			}
		}
		
		else	{
			threshold_count = 0;
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
static void Periodic_Charge_Calc (void *argument) 
{	
	Charge += (float)Current / 18000.0;																																								// Runs 5 times per second -> 18000 times per hour
}																																																										// So 1mAh current for 1 hour (ie. 18000 iterations) would give 1mAh.

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
void app_main (void *argument) 
{	
	osDelay(5000);
	
	// Start some tasks
	osThreadNew(Task_Alive_LED, 		NULL, NULL);
  osThreadNew(Task_IV, 						NULL, NULL);	
	osThreadNew(Task_Ctrl_Mode, 		NULL, NULL);
	osThreadNew(Task_Pass_Through, 	NULL, NULL);
	osThreadNew(Task_Telemetry, 		NULL, NULL);
	osThreadNew(Task_Read_Px4Flow, 	NULL, NULL);
	osThreadNew(Task_Read_LIDAR, 		NULL, NULL);
	osThreadNew(Task_PID, 					NULL, NULL);

	// Start periodic timers to handle the PIDs for Altitude hold, etc
	osTimerId_t Periodic_Altitude_PID_id, Periodic_X_PID_id, Periodic_Y_PID_id, Periodic_Charge_id;
	
	Periodic_Altitude_PID_id = osTimerNew(Periodic_Altitude_PID, osTimerPeriodic, NULL, NULL);
	osTimerStart(Periodic_Altitude_PID_id, ALTITUDE_PID_SAMPLE_T);		

	Periodic_X_PID_id = osTimerNew(Periodic_X_PID, osTimerPeriodic, NULL, NULL);
	osTimerStart(Periodic_X_PID_id, X_PID_SAMPLE_T);	

	Periodic_Y_PID_id = osTimerNew(Periodic_Y_PID, osTimerPeriodic, NULL, NULL);
	osTimerStart(Periodic_Y_PID_id, Y_PID_SAMPLE_T);		
	
	Periodic_Charge_id = osTimerNew(Periodic_Charge_Calc, osTimerPeriodic, NULL, NULL);
	osTimerStart(Periodic_Charge_id, CHARGE_SAMPLE_T);
	
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
	ADC_init();																						// Initialises ADC for reading current and voltage of battery
	ppm_init();																						// Initialises for CPPM input
	TIM3_Init();																					// Initialises a 1us counter for CPPM output timings. Generates compare interrupts
	USART1_Init();																				// Initialises UART1 functions, FIFO & interrupts for serial tx through telementry
	USART3_Init();																				// Initialises UART3 functions, FIFO & interrupts for comms with TFmini Lidar module
	I2C_Init_fnc();																				// Initialises I2C functions, FIFO & interrupts

	// Initialise variables
	Altitude_Avg.Filter_Size = 20;
	X_Avg.Filter_Size = 5;
	Y_Avg.Filter_Size = 5;
	
	// OS System Initialization
	SystemCoreClockUpdate();
	
	osKernelInitialize(); 																// Initialize CMSIS-RTOS		 
	osThreadNew(app_main, NULL, NULL); 										// Create application main thread
	osKernelStart(); 																			// Start thread execution
	
	osDelay(osWaitForever);
	while(1) {}		
}
























