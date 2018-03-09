#include "stm32f10x.h"
#include "support.h"


volatile uint8_t PPMi = 0;
volatile uint16_t PPMValue_Prev, PPMValue;

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
void gpio_init() 
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); 

	GPIO_InitTypeDef GPIO_InitStructure;	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;      
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
void ppm_init() 
{
		GPIO_InitTypeDef gpio_cfg;	

	/* Timer TIM4, channel 4 */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
		GPIO_StructInit(&gpio_cfg);
	  gpio_cfg.GPIO_Mode = GPIO_Mode_IPU;
	  gpio_cfg.GPIO_Pin = GPIO_Pin_9;
	  GPIO_Init(GPIOB, &gpio_cfg);

	  /* Timer TIM4 enable clock */
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	  /* Timer TIM4 settings */
	  TIM_TimeBaseInitTypeDef timer_base;
	
	  TIM_TimeBaseStructInit(&timer_base);
	  timer_base.TIM_Prescaler = 72;
	  TIM_TimeBaseInit(TIM4, &timer_base);

	  /* Signal capture settings:
	   - Channel: 1
	   - Count: Up
	   - Source: Input
	   - Divider: Disable
	   - Filter: Disable */
	  TIM_ICInitTypeDef timer_ic;
		
	  timer_ic.TIM_Channel = TIM_Channel_4;
	  timer_ic.TIM_ICPolarity = TIM_ICPolarity_Rising;
	  timer_ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
	  timer_ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	  timer_ic.TIM_ICFilter = 0;
	  TIM_ICInit(TIM4, &timer_ic);

	  /* Enable Interrupt by overflow */
	  TIM_ITConfig(TIM4, TIM_IT_CC4, ENABLE);

	  TIM_Cmd(TIM4, ENABLE);

	  NVIC_EnableIRQ(TIM4_IRQn);
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
void TIM3_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  GPIO_InitTypeDef   			 GPIO_InitStructure;	
	TIM_TimeBaseInitTypeDef  TIM3_TimeBaseStructure;
	TIM_OCInitTypeDef 			 OCStructure;	
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;      
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	
	TIM_TimeBaseStructInit(&TIM3_TimeBaseStructure);
	TIM3_TimeBaseStructure.TIM_Prescaler = 72;  
	TIM3_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM3_TimeBaseStructure.TIM_Period = 0xFFFF;      
	TIM3_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(TIM3, &TIM3_TimeBaseStructure);
	TIM_Cmd(TIM3, ENABLE);
	
	TIM_OCStructInit(&OCStructure);
	OCStructure.TIM_OCMode = TIM_OCMode_Timing;
  OCStructure.TIM_Pulse = 1000;
  TIM_OC1Init(TIM3, &OCStructure);
	
	TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);

	NVIC_EnableIRQ(TIM3_IRQn);
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
void TIM4_IRQHandler(void)
{
	// Inputs CPPM over the PPM_In_Pin. Uses timer4 to do this
	volatile uint16_t PPM;

	if (TIM_GetITStatus(TIM4, TIM_IT_CC4) != RESET)
	  {
	    TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);

	    PPMValue_Prev = PPMValue;
	    PPMValue = TIM_GetCapture4(TIM4);
	    PPM = (PPMValue >= PPMValue_Prev) ? (PPMValue - PPMValue_Prev) : (UINT16_MAX - PPMValue_Prev + PPMValue);
	    if (PPM < 3000) { 
	    	PPM_In[PPMi] = PPM;
	    	PPMi++;
		    if (PPMi > 7) {
		    	PPMi = 0;
		    }
	    }
	    else {
	    	PPMi = 0;
	    }

	    if (TIM_GetFlagStatus(TIM4, TIM_FLAG_CC4OF) != RESET)
	    {
	      TIM_ClearFlag(TIM4, TIM_FLAG_CC4OF);
	    }
	  }
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
void TIM3_IRQHandler (void)
{
	// Outputs CPPM over the PPM_Out_Pin. Uses timer3 to do this
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);	
  
  static uint8_t state = 1;
  static uint8_t cur_chan_numb = 0;
  static uint16_t calc_rest = 0;  
  
  TIM_SetCounter(TIM3, 0);                                                         		// Set Counter to 0
  
  if (state) {                                                                        // start CPPM frame by pulling line low
    GPIO_ResetBits(GPIOA, GPIO_Pin_8);
    state = 0;                    
		TIM_SetCompare1(TIM3, SEPARATOR_PULSE_LENGTH);																		// Next interrupt at SEPARATOR_PULSE_LENGTH microseconds 
  } 
      
  else  {
    GPIO_SetBits(GPIOA, GPIO_Pin_8);                                                  // Pull line high
    state = 1;

    if(cur_chan_numb < PPM_CHANNELS){                                             
      TIM_SetCompare1(TIM3, (PPM_Out[cur_chan_numb] - SEPARATOR_PULSE_LENGTH));
      calc_rest += PPM_Out[cur_chan_numb];
      cur_chan_numb++;
    }
    else  {                                                                           // Just finished the last pulse so stay high until it's been 20ms since start of frame
      cur_chan_numb = 0;
      calc_rest += SEPARATOR_PULSE_LENGTH;
      TIM_SetCompare1(TIM3, (FRAME_LENGTH - calc_rest));
      calc_rest = 0;      
    }     
  }		
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
int32_t constrain(int32_t x, int32_t min, int32_t max)
{
	if(x < min)
		return(min);
	
	if(x > max)
		return(max);
	
	return(x);	
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
int16_t Average(average_filter * Avg, int16_t datum)
{	
  Avg->Buffer[Avg->Index] = datum;                                
  Avg->Index++;
    
  if(Avg->Index == Avg->Filter_Size)                                    
    Avg->Index = 0;
  
  int32_t sum = 0;
  for(int8_t i = 0; i < Avg->Filter_Size; i++)
    sum += Avg->Buffer[i];

  return(sum/Avg->Filter_Size);
}













