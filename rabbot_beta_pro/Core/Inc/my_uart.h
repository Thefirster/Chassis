#ifndef _MY_UART_H_
#define _MY_UART_H_
#include "stm32f4xx.h"

#define RX_LEN1 78

typedef struct
{
	uint8_t RX_flag:1;
	uint16_t RX_Size;
	uint8_t RX_pData[RX_LEN1];
}USART1_RECEIVETYPE;


void Usart6_Start(void);
void Usart4_Start(void);
void Usart5_Start(void);
void float2u8Arry(uint8_t *u8Arry, float *floatdata, int key);
void Usart6Receive_IDLE(void);

#endif
