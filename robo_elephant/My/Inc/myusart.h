#ifndef __MYUSART_H
#define __MYUSART_H

#include "stm32f4xx.h"
#include "usart.h"

#define BUFFSIZE 8

//���ղ���
typedef struct
{
	union
  {
		char ProcessedBuff[BUFFSIZE];
		float GetFloat[2];  //ת��Ϊfloat��
	}Receive;
  uint8_t Rxbuffer[20]; //ע�����鲻ҪԽ��
}Data_Rx;
//���Ͳ���
typedef struct
{
	union
  {
		char ProcessedBuff[BUFFSIZE];
		double SendDouble;  //ת��Ϊfloat��
	}Send;
  uint8_t Txbuffer[20]; //ע�����鲻ҪԽ��
}Data_Tx;

extern Data_Rx vision_data_receive;
extern Data_Tx vision_data_send;

void uart_start_receive(void);
void vision_send_data(double send_flag);
	
#endif
