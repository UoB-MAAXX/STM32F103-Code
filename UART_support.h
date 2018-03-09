#ifndef _UART_SUPPORT_H_
#define _UART_SUPPORT_H_


#define							UART_TX_BUFFER_SIZE								64
#define							UART_RX_BUFFER_SIZE								64

#define							UART_RX_STR_TIMEOUT								50

#define							UART1															0
#define							UART2															1
#define							UART3															2

void USART1_Init(void);
void USART3_Init(void);
uint8_t UART_putc(char character, int8_t UARTx);
char UART_getc(int8_t UARTx);
void UART_flush_TX(int8_t UARTx);
void UART_flush_RX(int8_t UARTx);
uint8_t UART_available(int8_t UARTx);
void UART_putstr(char * str, int8_t UARTx);	
void UART_putstrln(char * str, int8_t UARTx);
uint8_t UART_getstr(char * str, char stop_char, int8_t UARTx);
void UART_putnum(int32_t x, int8_t UARTx);
void UART_putnumln(int32_t x, int8_t UARTx);

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

