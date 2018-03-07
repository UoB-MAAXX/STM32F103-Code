#ifndef _UART_SUPPORT_H_
#define _UART_SUPPORT_H_


#define							UART_TX_BUFFER_SIZE								64
#define							UART_RX_BUFFER_SIZE								64

#define							UART_RX_STR_TIMEOUT								50

void USART1_Init(void);
uint8_t UART_putc(char character);
char UART_getc(void);
void UART_flush_TX(void);
void UART_flush_RX(void);
uint8_t UART_available(void);
void UART_putstr(char * str);	
void UART_putstrln(char * str);
uint8_t UART_getstr(char * str, char stop_char);
void UART_putnum(int32_t x);
void UART_putnumln(int32_t x);

int32_t stoi(char* s);


typedef struct UART_FIFO_struct
{
	volatile uint8_t 		tx_buffer[UART_TX_BUFFER_SIZE];
	volatile uint8_t		tx_buffer_count;	
	volatile uint8_t		tx_head_index;	
	volatile uint8_t		tx_tail_index;		
	volatile uint8_t 		rx_buffer[UART_RX_BUFFER_SIZE];
	volatile uint8_t		rx_buffer_count;
	volatile uint8_t		rx_head_index;
	volatile uint8_t		rx_tail_index;
} UART_FIFO_struct;


#endif

