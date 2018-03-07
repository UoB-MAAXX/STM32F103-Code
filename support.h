#ifndef _SUPPORT_H_
#define _SUPPORT_H_

#define           	PPM_CHANNELS            8
#define           	FRAME_LENGTH            20000                                       // Set the PPM frame length in microseconds (1ms = 1000µs)
#define           	SEPARATOR_PULSE_LENGTH  400                                         // Set the pulse length

#define 						STOPPER 													0                                      /* Smaller than any datum */
#define    					MEDIAN_FILTER_SIZE    						5

extern volatile uint16_t PPM_In[PPM_CHANNELS];
extern volatile uint16_t PPM_Out[PPM_CHANNELS];

void gpio_init(void);
void ppm_init(void);
void TIM3_Init(void);

int32_t constrain(int32_t x, int32_t min, int32_t max);
uint16_t slew_limiter(uint16_t datum);
int16_t PID_Output_average(int16_t datum);
uint16_t median_filter(uint16_t datum);

#endif


