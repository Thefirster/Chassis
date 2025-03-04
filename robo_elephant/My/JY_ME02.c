#include "JY_ME02.h"
#include "math.h"
#include "yuntai.h"
/**
https://wit-motion.yuque.com/wumwnr/docs/qcgu36#AboSL
*/

JY_Can_Receive JY_Encoder[2];

float w_sum0;
void JY_Encoder_Can_Receive(CAN_RxHeaderTypeDef *pHeader, uint8_t *RxCanData, JY_Can_Receive* JY)
{
	uint8_t num = pHeader->StdId & 0x00FF;
	uint16_t angle_num = 0, w_num = 0;
	
//	if(num == 0x66){
//			if(RxCanData[0] == 0x55 && RxCanData[1] == 0x55){
//			angle_num &= 0x0000;  
//			angle_num |= (RxCanData[3]<<8)|RxCanData[2];  //回传角度为u16位
//			JY[level_encoder].angle = angle_num * 360.0000f / 32768.0000f;  //角度
//			
//			JY[level_encoder].turns &= 0x0000;
//			JY[level_encoder].turns |= (RxCanData[7]<<8)|RxCanData[6];  //转数
//			
//			w_num &= 0x0000;
//			w_num |= (RxCanData[5]<<8)|RxCanData[4];
//			w_sum0=w_num;
//			w_num=fabs((w_num>>15)?w_num^0xffff:w_num);
//			JY[level_encoder].w = w_num * 360.0000f / 32768.0000f /0.1000f;  //角速度
//	  }
//	}else 
	if(num == 0x52){
			if(RxCanData[0] == 0x55 && RxCanData[1] == 0x55){
			angle_num &= 0x0000;  
			angle_num |= (RxCanData[3]<<8)|RxCanData[2];  //回传角度为u16位
			JY[pitch_encoder].angle = angle_num * 360.0000f / 32768.0000f;  //角度
			
			JY[pitch_encoder].turns &= 0x0000;
			JY[pitch_encoder].turns |= (RxCanData[7]<<8)|RxCanData[6];  //转数
			
			w_num &= 0x0000;
			w_num |= (RxCanData[5]<<8)|RxCanData[4];
			w_sum0=w_num;
			w_num=fabs((w_num>>15)?w_num^0xffff:w_num);
			JY[pitch_encoder].w = w_num * 360.0000f / 32768.0000f /0.1000f;  //角速度
	  }
	}
	
}	
