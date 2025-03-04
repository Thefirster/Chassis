#ifndef _JY_ME02_H_
#define _JY_ME02_H_

#include "main.h"

//JY can接收
typedef struct
{
	float angle;  //当前角度
	uint16_t turns;  //圈数
	float w;  //角速度
}JY_Can_Receive;

extern JY_Can_Receive JY_Encoder[2];

void JY_Encoder_Can_Receive(CAN_RxHeaderTypeDef *pHeader, uint8_t *RxCanData, JY_Can_Receive *JY_Encoder);

#endif
