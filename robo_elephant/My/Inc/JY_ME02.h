#ifndef _JY_ME02_H_
#define _JY_ME02_H_

#include "main.h"

//JY can����
typedef struct
{
	float angle;  //��ǰ�Ƕ�
	uint16_t turns;  //Ȧ��
	float w;  //���ٶ�
}JY_Can_Receive;

extern JY_Can_Receive JY_Encoder[2];

void JY_Encoder_Can_Receive(CAN_RxHeaderTypeDef *pHeader, uint8_t *RxCanData, JY_Can_Receive *JY_Encoder);

#endif
