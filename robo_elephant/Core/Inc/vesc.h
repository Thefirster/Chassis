#ifndef _VESC_H_
#define _VESC_H_
#include "stm32f4xx_hal.h"
typedef enum {
CAN_PACKET_SET_DUTY=0 ,          //0	设置电机占空比	4字节	有符号整数	单位：Thousandths of percent (5000 0 –> 50%)
CAN_PACKET_SET_CURRENT,          //1	设置电机电流	4字节	有符号整数	mA
CAN_PACKET_SET_CURRENT_BRAKE,    //2	设置制动电流	4字节	有符号整数	mA
CAN_PACKET_SET_RPM,              //3	设置（电）转速	4字节	有符号整数	ERPM
CAN_PACKET_SET_POS,              //4	设置电机转角位置
CAN_PACKET_FILL_RX_BUFFER,       //5
CAN_PACKET_FILL_RX_BUFFER_LONG,  //6
CAN_PACKET_PROCESS_RX_BUFFER,    //7
CAN_PACKET_PROCESS_SHORT_BUFFER, //8
CAN_PACKET_STATUS_1,             //9
CAN_PACKET_SET_CURRENT_REL,      //10
CAN_PACKET_SET_CURRENT_BRAKE_REL,//11
CAN_PACKET_SET_CURRENT_HANDBRAKE,
CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
CAN_PACKET_STATUS_2,
CAN_PACKET_STATUS_3,
CAN_PACKET_STATUS_4,
CAN_PACKET_PING,
CAN_PACKET_PONG,
CAN_PACKET_DETECT_APPLY_ALL_FOC,
CAN_PACKET_DETECT_APPLY_ALL_FOC_RES,
CAN_PACKET_CONF_CURRENT_LIMITS,
CAN_PACKET_CONF_STORE_CURRENT_LIMITS,
CAN_PACKET_CONF_CURRENT_LIMITS_IN,
CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN,
CAN_PACKET_CONF_FOC_ERPMS,
CAN_PACKET_CONF_STORE_FOC_ERPMS,
CAN_PACKET_STATUS_5
} CAN_PACKET_ID;


//用于接受vesc发出的can数据的结构体：
typedef struct {
	uint32_t rx_time;
	int32_t rpm;
	float current;
	float duty;
} can_status_msg_1;

typedef struct {
	uint32_t rx_time;
	float amp_hours;
	float amp_hours_charged;
} can_status_msg_2;

typedef struct {	
	uint32_t rx_time;
	float watt_hours;
	float watt_hours_charged;
} can_status_msg_3;

typedef struct {
	uint32_t rx_time;
	float temp_fet;
	float temp_motor;
	float current_in;
	float pid_pos_now;
} can_status_msg_4;

typedef struct {
	uint32_t rx_time;
	int32_t tacho_value;
	float v_input;
	int16_t reserved;
} can_status_msg_5;

typedef struct {
	can_status_msg_1 vesc_msg_1;
	can_status_msg_2 vesc_msg_2;
	can_status_msg_3 vesc_msg_3;
	can_status_msg_4 vesc_msg_4;
	can_status_msg_5 vesc_msg_5;	

} VESC_STA;

void VESC_Set_RPM(uint32_t id, int32_t RPM, CAN_HandleTypeDef *hcan);    //设置电机转速（电转速）
void VESC_CAN_Send(uint32_t id, const uint8_t *data, uint8_t len, CAN_HandleTypeDef *hcan);//VESC数据发送


//VESC接收数据
//void VESC_CAN_Receive(CanRxMsg rx_message)
//{
//	int id;
//	int cmd;
//	id = (rx_message.ExtId & 0xFF);  //截取ID
//	cmd = (rx_message.ExtId >>8);    //截取命令
//	if(id==VESC_id&&rx_message.IDE==CAN_Id_Extended)
//	{
//		switch (cmd) { //cmd判断
//			case CAN_PACKET_STATUS_1:
//				VESC.vesc_msg_1.rx_time =0;
//				VESC.vesc_msg_1.rpm =(rx_message.Data[0]<<24|rx_message.Data [1]<<16|rx_message.Data [2]<<8|rx_message.Data [3]);
//				VESC.vesc_msg_1.current =((s16)(rx_message.Data[4]<<8|rx_message.Data[5]))/10.0f;
//				VESC.vesc_msg_1.duty =((s16)(rx_message.Data[6]<<8|rx_message.Data[7]))/1000.0f;
//				break ;
//			case CAN_PACKET_STATUS_2:
//				VESC.vesc_msg_2.rx_time =0; 
//				VESC.vesc_msg_2.amp_hours =((s32)(rx_message.Data[0]<<24|rx_message.Data [1]<<16|rx_message.Data [2]<<8|rx_message.Data [3]))/10000.0f;
//				VESC.vesc_msg_2.amp_hours_charged =((s32)(rx_message.Data[4]<<24|rx_message.Data [5]<<16|rx_message.Data [6]<<8|rx_message.Data [7]))/10000.0f;
//				break ;
//			case CAN_PACKET_STATUS_3:
//				VESC.vesc_msg_3.rx_time =0;		
//				VESC.vesc_msg_3.watt_hours =((s32)(rx_message.Data[0]<<24|rx_message.Data [1]<<16|rx_message.Data [2]<<8|rx_message.Data [3]))/10000.0f; 
//				VESC.vesc_msg_3.watt_hours_charged =((s32)(rx_message.Data[4]<<24|rx_message.Data [5]<<16|rx_message.Data [6]<<8|rx_message.Data [7]))/10000.0f;
//				break ;
//			case CAN_PACKET_STATUS_4:
//				VESC.vesc_msg_4.rx_time =0;
//				VESC.vesc_msg_4.temp_fet =((s16)(rx_message.Data[0]<<8|rx_message.Data[1]))/10.0f;
//				VESC.vesc_msg_4.temp_motor =((s16)(rx_message.Data[2]<<8|rx_message.Data[3]))/10.0f;
//				VESC.vesc_msg_4.current_in =((s16)(rx_message.Data[4]<<8|rx_message.Data[5]))/10.0f;
//				VESC.vesc_msg_4.pid_pos_now =((s16)(rx_message.Data[6]<<8|rx_message.Data[7]))/50.0f;
//				break ;
//			case CAN_PACKET_STATUS_5:
//				VESC.vesc_msg_5.rx_time =0;
//				VESC.vesc_msg_5.tacho_value =(rx_message.Data[0]<<24|rx_message.Data [1]<<16|rx_message.Data [2]<<8|rx_message.Data [3]);
//				VESC.vesc_msg_5.v_input =((s16)(rx_message.Data[4]<<8|rx_message.Data[5]))/10.0f;
//				VESC.vesc_msg_5.reserved =(rx_message.Data[6]<<8|rx_message.Data[7]);
//				break ;
//			default:
//				break ;
//		}
//	}
//}
#endif
