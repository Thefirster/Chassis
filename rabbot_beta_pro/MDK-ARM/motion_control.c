#include "motion_control.h"
#include "math.h"
#include "M3508.h"
#include "A1_motor.h"
#include "freertos.h"
#define a2 352.5/2
#define b2 310.0/2



//float RPM;
extern M3508x_STA M3508[8];
extern M3508x_STA M3508_2[8];
extern A1_data a1_data[2];
/**
  **************************************************************************
  ** -------------------------------------------------------------------- **
  ** @name          : motion_resolve
  ** @brief         : 麦轮の速度分解
  ** @param         : None
  ** @retval        : None
  ** @author        : XRaccoon
  ** -------------------------------------------------------------------- **
  ** @attention     : None
  ** -------------------------------------------------------------------- **
  **************************************************************************
**/
void motion_resolve(float vx,float vy,float vw)
{

		VESC_Set_RPM(1, -vy+vx+vw, &hcan2);
		VESC_Set_RPM(2, -vy-vx+vw, &hcan2);
		VESC_Set_RPM(3, vy-vx+vw, &hcan2);
		VESC_Set_RPM(4, vy+vx+vw, &hcan2);
	
}

float lift_move(float speed,int wei)
{
		float sppeed_set;
		sppeed_set=speed*wei;
		M3508[5].Position_S_PID_STruct.pos_target=0;
		M3508[5].Speed_P_PID_STruct.target=sppeed_set;
		
		return sppeed_set;
}

void lift_step(float step)
{
		position_set(&M3508[5],8192*step+M3508[5].dAngle_Sum,10*step);
}

void catch_down(float down_pos)
{
		position_set(&M3508[0],down_pos+M3508[0].dAngle_Sum,400);
}

void catch_up(float up_pos)
{
		position_set(&M3508[0],up_pos+M3508[0].dAngle_Sum,400);
}
/**
  **************************************************************************
  ** -------------------------------------------------------------------- **
  ** @name          : altitude_control_s1
  ** @brief         : Control the 2 wheels ahead through key S1
  ** @param         : float high,float speed
  ** @retval        : None
  ** @author        : XRaccoon
  ** -------------------------------------------------------------------- **
  ** @attention     : UP 0 # DOWN 1 #direction -1 or 1 #speed adjust to 1000
  ** -------------------------------------------------------------------- **
  **************************************************************************
**/
void altitude_control_s1(float high,float speed)
{
    M3508[1].Angle_target=-high;
    M3508[2].Angle_target=high;

    if(M3508[1].Angle_target!=M3508[1].target_last)M3508[1].direction=(M3508[1].Angle_target-M3508[1].dAngle_Sum/8192)/fabs(M3508[1].Angle_target-M3508[1].dAngle_Sum/8192);
    if(M3508[2].Angle_target!=M3508[2].target_last)M3508[2].direction=(M3508[2].Angle_target-M3508[2].dAngle_Sum/8192)/fabs(M3508[2].Angle_target-M3508[2].dAngle_Sum/8192);
    //对上一次的目标值进行保存，并和这一次进行比较，获得此次的方向

    M3508[1].target_last=M3508[1].Angle_target;
    M3508[2].target_last=M3508[2].Angle_target;//保存这次的目标值

    if((M3508[1].Angle_target-M3508[1].dAngle_Sum/8192)*M3508[1].direction>1)
         M3508[1].Position_S_PID_STruct.target=speed*(M3508[1].Angle_target-M3508[1].dAngle_Sum/8192)/fabs(M3508[1].Angle_target-M3508[1].dAngle_Sum/8192);
    else M3508[1].Position_S_PID_STruct.target=0;

    if((M3508[2].Angle_target-M3508[2].dAngle_Sum/8192)*M3508[2].direction>1)
         M3508[2].Position_S_PID_STruct.target=speed*(M3508[2].Angle_target-M3508[2].dAngle_Sum/8192)/fabs(M3508[2].Angle_target-M3508[2].dAngle_Sum/8192);
    else M3508[2].Position_S_PID_STruct.target=0;//每次至少上下一转

    M3508[1].Position_S_PID_STruct.Input=M3508[1].Speed;
    M3508[2].Position_S_PID_STruct.Input=M3508[2].Speed;

}

/**
  ************************************************************************** 
  ** -------------------------------------------------------------------- **
  ** @name          : altitude_control_s2
  ** @brief         : Control the 2 wheels ahead through key S2
  ** @param         : float high,float speed
  ** @retval        : None
  ** @author        : XRaccoon
  ** -------------------------------------------------------------------- **
  ** @attention     : None
  ** -------------------------------------------------------------------- **
  ************************************************************************** 
**/
void altitude_control_s2(float high,float speed)
{
    M3508[0].Angle_target=high;
    M3508[3].Angle_target=high;

    if(M3508[0].Angle_target!=M3508[0].target_last)M3508[0].direction=(M3508[0].Angle_target-M3508[0].dAngle_Sum/8192)/fabs(M3508[0].Angle_target-M3508[0].dAngle_Sum/8192);
    if(M3508[3].Angle_target!=M3508[3].target_last)M3508[3].direction=(M3508[3].Angle_target-M3508[3].dAngle_Sum/8192)/fabs(M3508[3].Angle_target-M3508[3].dAngle_Sum/8192);

    M3508[0].target_last=M3508[0].Angle_target;
    M3508[3].target_last=M3508[3].Angle_target;

    if((M3508[0].Angle_target-M3508[0].dAngle_Sum/8192)*M3508[0].direction>1)
        M3508[0].Position_S_PID_STruct.target=speed*(M3508[0].Angle_target-M3508[0].dAngle_Sum/8192)/fabs(M3508[0].Angle_target-M3508[0].dAngle_Sum/8192);
    else M3508[0].Position_S_PID_STruct.target=0;
    if((M3508[3].Angle_target-M3508[3].dAngle_Sum/8192)*M3508[3].direction>1)
        M3508[3].Position_S_PID_STruct.target=speed*(M3508[3].Angle_target-M3508[3].dAngle_Sum/8192)/fabs(M3508[3].Angle_target-M3508[3].dAngle_Sum/8192);
    else M3508[3].Position_S_PID_STruct.target=0;
    M3508[0].Position_S_PID_STruct.Input=M3508[0].Speed;
    M3508[3].Position_S_PID_STruct.Input=M3508[3].Speed;
}



//void VESC_CAN_Send(uint32_t id, const uint8_t *data, uint8_t len)
//{
//	int i;
//	CAN_TxHeaderTypeDef TxMessage;
//	uint32_t CANTXMAILBOX0;
//	
//	if (len > 8) len = 8;
//	
//	TxMessage.IDE = CAN_ID_EXT;
//	TxMessage.RTR = CAN_RTR_DATA; 
//	TxMessage.DLC = len;
//	TxMessage.ExtId = id;
//	
//	uint8_t mbox[8];
//	for(i=0; i<len; i++)
//		mbox[i] = data[i];
//	
///*	
//	i = 0;
//	while((HAL_CAN_AddTxMessage(&hcan, &TxMessage, mbox, (uint32_t*)CANTXMAILBOX0) != HAL_OK)&&(i < 0X7FF))
//	{
//		i++;
//	}*/
//	
//	HAL_CAN_AddTxMessage(&hcan2, &TxMessage, mbox, (uint32_t*)CANTXMAILBOX0);
//}
//void VESC_Set_RPM(uint32_t id, int32_t RPM)
//{
//	uint8_t mes[4];
//	int i=0;
//	unsigned char *hex = (unsigned char *)&RPM;
//	for(i=0;i<4;i++)
//	{
//		mes[i]=hex[3-i];
//	}
//	VESC_CAN_Send(id | ((uint32_t)CAN_PACKET_SET_RPM << 8),mes, 4);
//}
//float Scope_Max_Min(float NowSpeed, float MaxSpeed)
//{
//	if(NowSpeed > MaxSpeed)
//		NowSpeed = MaxSpeed;
//	if(NowSpeed < -MaxSpeed)
//		NowSpeed = -MaxSpeed;
//	return NowSpeed;
//}
//void Rotor_Control(float send_angle1, float send_angle2)
//{
//	//M2006
//	send_angle1 = Scope_Max_Min(send_angle1, 70);  
//	send_angle2 = Scope_Max_Min(send_angle2, 200);  //ÏÞÖµ
////	M_Calculate_CascadePID_Test(&M2006[eid1], send_angle1*8191, SpeedMax1);
////	M_Calculate_CascadePID_Test(&M2006[eid2], send_angle2*8191, SpeedMax2);  //pid¼ÆËã²¿·Ö
////	M350x_TxMes_0_3(&hcan, M2006);    //can·¢ËÍ²¿·Ö	
//	//VESC
//	VESC_Set_RPM(2, -RPM);
//	VESC_Set_RPM(3, RPM);
//	//DM
//	//DM_MotorCtrl(&hcan, DM_CANID_1, MIT_MODE , send_pos, 0,  KP, KD, 0);
//} 



void VESC_CAN_Send(uint32_t id, const uint8_t *data, uint8_t len, CAN_HandleTypeDef *hcan)
{
	int i;
	CAN_TxHeaderTypeDef TxMessage;
	uint32_t mailbox;
	if (len > 8) len = 8;
	
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.DLC = len;
	TxMessage.ExtId = id;
	
	uint8_t mbox[8];
	for(i=0; i<len; i++)
		mbox[i] = data[i];
	
	i = 0;
	while((HAL_CAN_AddTxMessage(hcan, &TxMessage, mbox, &mailbox) != HAL_OK)&&(i < 0XFFF))
	{
		i++;
	}
}

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

VESC_STA VESC[4];
void VESC_CAN_Receive(CAN_RxHeaderTypeDef RxHeader, uint8_t* CanReceiveData)
{
	int id;
	int cmd;
	id = (RxHeader.ExtId & 0xFF);  //½ØÈ¡ID
	cmd = (RxHeader.ExtId >>8);    //½ØÈ¡ÃüÁî
	if(RxHeader.IDE==CAN_ID_EXT && (id == 1 || id == 2 || id == 3))
	{
		switch (cmd) { //cmdÅÐ¶Ï
			case CAN_PACKET_STATUS_1:
				VESC[id].vesc_msg_1.rx_time =0;
				VESC[id].vesc_msg_1.rpm =(CanReceiveData[0]<<24|CanReceiveData [1]<<16|CanReceiveData [2]<<8|CanReceiveData [3]);
				VESC[id].vesc_msg_1.current =((int16_t)(CanReceiveData[4]<<8|CanReceiveData[5]))/10.0f;
				VESC[id].vesc_msg_1.duty =((int16_t)(CanReceiveData[6]<<8|CanReceiveData[7]))/1000.0f;
				break ;
			case CAN_PACKET_STATUS_2:
				VESC[id].vesc_msg_2.rx_time =0; 
				VESC[id].vesc_msg_2.amp_hours =((int32_t)(CanReceiveData[0]<<24|CanReceiveData [1]<<16|CanReceiveData [2]<<8|CanReceiveData [3]))/10000.0f;
				VESC[id].vesc_msg_2.amp_hours_charged =((int32_t)(CanReceiveData[4]<<24|CanReceiveData [5]<<16|CanReceiveData [6]<<8|CanReceiveData [7]))/10000.0f;
				break ;
			case CAN_PACKET_STATUS_3:
				VESC[id].vesc_msg_3.rx_time =0;		
				VESC[id].vesc_msg_3.watt_hours =((int32_t)(CanReceiveData[0]<<24|CanReceiveData [1]<<16|CanReceiveData [2]<<8|CanReceiveData [3]))/10000.0f; 
				VESC[id].vesc_msg_3.watt_hours_charged =((int32_t)(CanReceiveData[4]<<24|CanReceiveData [5]<<16|CanReceiveData [6]<<8|CanReceiveData [7]))/10000.0f;
				break ;
			case CAN_PACKET_STATUS_4:
				VESC[id].vesc_msg_4.rx_time =0;
				VESC[id].vesc_msg_4.temp_fet =((int16_t)(CanReceiveData[0]<<8|CanReceiveData[1]))/10.0f;
				VESC[id].vesc_msg_4.temp_motor =((int16_t)(CanReceiveData[2]<<8|CanReceiveData[3]))/10.0f;
				VESC[id].vesc_msg_4.current_in =((int16_t)(CanReceiveData[4]<<8|CanReceiveData[5]))/10.0f;
				VESC[id].vesc_msg_4.pid_pos_now =((int16_t)(CanReceiveData[6]<<8|CanReceiveData[7]))/50.0f;
				break ;
			case CAN_PACKET_STATUS_5:
				VESC[id].vesc_msg_5.rx_time =0;
				VESC[id].vesc_msg_5.tacho_value =(CanReceiveData[0]<<24|CanReceiveData [1]<<16|CanReceiveData [2]<<8|CanReceiveData [3]);
				VESC[id].vesc_msg_5.v_input =((int16_t)(CanReceiveData[4]<<8|CanReceiveData[5]))/10.0f;
				VESC[id].vesc_msg_5.reserved =(CanReceiveData[6]<<8|CanReceiveData[7]);
				break ;
			default:
				break ;
		}
	}
}

void VESC_Set_HardBreak_Current(uint32_t id, int32_t current, CAN_HandleTypeDef *hcan)//毫伏
{
	uint8_t mes[4];
	int i=0;
	unsigned char *hex = (unsigned char *)&current;
	for(i=0;i<4;i++)
	{
		mes[i]=hex[3-i];
	}
	VESC_CAN_Send(id | ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE << 8),mes, 4, hcan);	
}



