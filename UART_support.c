#include "stm32f10x.h"
#include "UART_support.h"
#include "cmsis_os2.h"

UART_FIFO_struct UART1_FIFO;

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
void USART1_Init(void)
{
	/* USART configuration structure for USART1 */
	USART_InitTypeDef usart1_init_struct;

	/* Bit configuration structure for GPIOA PIN9 and PIN10 */
	GPIO_InitTypeDef GPIOA_init_struct;
	 
	/* Enalbe clock for USART1, AFIO and GPIOA */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);
													
	/* GPIOA PIN9 alternative function Tx */
	GPIOA_init_struct.GPIO_Pin = GPIO_Pin_9;
	GPIOA_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOA_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIOA_init_struct);		

	/* GPIOA PIN10 alternative function Rx */
	GPIOA_init_struct.GPIO_Pin = GPIO_Pin_10;
	GPIOA_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOA_init_struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIOA_init_struct);

	/* Enable USART1 */
	USART_Cmd(USART1, ENABLE); 
	
	/* Baud rate 115200, 8-bit data, One stop bit, No parity, Do both Rx and Tx, No HW flow control */     
	usart1_init_struct.USART_BaudRate = 115200;   
	usart1_init_struct.USART_WordLength = USART_WordLength_8b;  
	usart1_init_struct.USART_StopBits = USART_StopBits_1;   
	usart1_init_struct.USART_Parity = USART_Parity_No;
	usart1_init_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usart1_init_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1, &usart1_init_struct);
	
	/* Enable RXNE interrupt */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	
	/* Enable USART1 global interrupt */
	NVIC_EnableIRQ(USART1_IRQn);		
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
void USART1_IRQHandler(void)
{
	/* RXNE handler */
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{       
		if(UART1_FIFO.rx_buffer_count < UART_RX_BUFFER_SIZE)																														// If there is space in the Rx buffer add it into the FIFO
		{
			UART1_FIFO.rx_buffer[UART1_FIFO.rx_head_index++] = USART_ReceiveData(USART1);																	// Reading the DR also clears the RXNE flag.
			UART1_FIFO.rx_buffer_count++;

			if(UART1_FIFO.rx_head_index >= UART_RX_BUFFER_SIZE)																														// Wrap head_index around when hitting the top
			{
				UART1_FIFO.rx_head_index = 0;
			}				
		}
		
		else
		{
			USART_ReceiveData(USART1);
		}
	}
	
	/* TXE handler */
	if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
	{
		if(UART1_FIFO.tx_buffer_count > 0)																																							// If there is data in the tx buffer send it	
		{
			USART_SendData(USART1, UART1_FIFO.tx_buffer[UART1_FIFO.tx_tail_index++]);
			UART1_FIFO.tx_buffer_count--;
			
			if(UART1_FIFO.tx_tail_index >= UART_TX_BUFFER_SIZE)																														// Wrap tail_index around when hitting the top
			{
				UART1_FIFO.tx_tail_index = 0;
			}				
		}
		
		else
		{
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);				
		}
	}
} 


//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################	
uint8_t UART_putc(char character)
{		
	if(UART1_FIFO.tx_buffer_count >= UART_TX_BUFFER_SIZE)																															// If FIFO full return with 0
	{
		return(0);
	}
	
	if(UART1_FIFO.tx_head_index >= UART_TX_BUFFER_SIZE)																																// Wrap head_index around when hitting the top
	{
		UART1_FIFO.tx_head_index = 0;
	}
	
	UART1_FIFO.tx_buffer[UART1_FIFO.tx_head_index++] = character;																											// Add character to buffer and increment index
	UART1_FIFO.tx_buffer_count++;																																											// Increment the total buffer count
	
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);																																			// Enable interrupt on TXE
	return(1);
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################	
char UART_getc()
{		
	if(UART1_FIFO.rx_buffer_count == 0)																																								// If FIFO empty return
	{
		return(0);
	}
	
	if(UART1_FIFO.rx_tail_index >= UART_RX_BUFFER_SIZE)																																// Wrap tail_index around when hitting the top
	{
		UART1_FIFO.rx_tail_index = 0;
	}
	
	UART1_FIFO.rx_buffer_count--;																																																										
	return(UART1_FIFO.rx_buffer[UART1_FIFO.rx_tail_index++]);
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################	
void UART_flush_TX()
{		
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
	
	UART1_FIFO.tx_buffer_count = 0;
	UART1_FIFO.tx_head_index = 0;
	UART1_FIFO.tx_tail_index = 0;
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################	
void UART_flush_RX()
{		
	UART1_FIFO.rx_buffer_count = 0;
	UART1_FIFO.rx_head_index = 0;
	UART1_FIFO.rx_tail_index = 0;
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################	
uint8_t UART_available()
{		
	return(UART1_FIFO.rx_buffer_count);	
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################	
void UART_putstr(char * str)
{
	while(*str) {
		UART_putc(*(str++));
	}	
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################	
void UART_putstrln(char * str)
{
	UART_putstr(str);
	UART_putc(10);
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################	
uint8_t UART_getstr(char * str, char stop_char)
{
	uint16_t timeout = UART_RX_STR_TIMEOUT;
	uint8_t count = 0;
	
	while(timeout) 																																																		// Do all our UART Rx within a timeframe
	{
		if(UART_available())																																														// If char available																																													
		{
			*str = UART_getc();																																														// Read the char and write it to the buffer
			
			if(*str == stop_char)	{																																												// If char equal to our stop char break from the while loop																																								
				break;
			}			
			
			str++;																																																				// otherwise increment the str pointer
			count++;																																																			// and increment the characters received count
		}
		else	{
			osDelay(1);
			timeout--;
		}
	}
	
	*str = 0;																																																					// This is the string terminator overwritting our stop char
	return(count);
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
void UART_putnum(int32_t x)
{
  char value[10]; 
  int i = 0; 
	
	if(x < 0)	{
		x *= -1;
		UART_putc('-');
	}
  
  do  {
    value[i++] = (char)(x % 10) + '0';
    x /= 10;
  } while(x);
  
  while(i) 
  {
    UART_putc(value[--i]);		
  }
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
void UART_putnumln(int32_t x)
{
  UART_putnum(x);	
	UART_putc(10);
}

//##################################################################################################################
//------------------------------------------------------------------------------------------------------------------
//##################################################################################################################
int32_t stoi(char* str)
{
	int32_t res = 0;  
	int32_t sign = 1;  
	int32_t i = 0;  
		
	// If number is negative, then update sign
	if (str[0] == '-')
	{
			sign = -1;  
			i++;  // Also update index of first digit
	}
		
	// Iterate through all digits and update the result
	for (; str[i]; ++i)	{
			res = res*10 + (str[i] - '0');
	}

	// Return result with sign
	return(sign*res);
}






