#ifndef __MYUSART_H
#define __MYUSART_H

#include "stm32f4xx.h"
#include "usart.h"

#define BUFFSIZE 8

//接收部分
typedef struct
{
	union
  {
		char ProcessedBuff[BUFFSIZE];
		float GetFloat[2];  //转化为float型
	}Receive;
  uint8_t Rxbuffer[20]; //注意数组不要越界
}Data_Rx;
//发送部分
typedef struct
{
	union
  {
		char ProcessedBuff[BUFFSIZE];
		double SendDouble;  //转化为float型
	}Send;
  uint8_t Txbuffer[20]; //注意数组不要越界
}Data_Tx;

extern Data_Rx vision_data_receive;
extern Data_Tx vision_data_send;

void uart_start_receive(void);
void vision_send_data(double send_flag);
	
#endif
