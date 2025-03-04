#include "VESC.h"
#include "mycan.h"

//ÉèÖÃ×ªËÙ
void VESC_Set_RPM(uint32_t id, int32_t RPM, CAN_HandleTypeDef *hcan)
{
	uint8_t mes[4];
	int i=0;
	unsigned char *hex = (unsigned char *)&RPM;
	for(i=0;i<4;i++)
	{
		mes[i]=hex[3-i];
	}
	VESC_CAN_Send(id | ((uint32_t)CAN_PACKET_SET_RPM << 8),mes, 4, hcan);
}
