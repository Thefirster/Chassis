#include "stm32f4xx_hal.h"
#include "can.h"
#include "M3508.h"
#define u8 uint8_t
#define s32 int32_t
#define CATCH 0
#define SHOOT 1
void motion_resolve(float vx,float vy,float vw);

void altitude_control_s1(float high,float speed);
void altitude_control_s2(float high,float speed);
typedef enum {
CAN_PACKET_SET_DUTY=0 ,          //0	ÉèÖÃµç»úÕ¼¿Õ±È	4×Ö½Ú	ÓÐ·ûºÅÕûÊý	µ¥Î»£ºThousandths of percent (5000 0 ¨C> 50%)
CAN_PACKET_SET_CURRENT,          //1	ÉèÖÃµç»úµçÁ÷	4×Ö½Ú	ÓÐ·ûºÅÕûÊý	mA
CAN_PACKET_SET_CURRENT_BRAKE,    //2	ÉèÖÃÖÆ¶¯µçÁ÷	4×Ö½Ú	ÓÐ·ûºÅÕûÊý	mA
CAN_PACKET_SET_RPM,              //3	ÉèÖÃ£¨µç£×ªËÙ	4×Ö½Ú	ÓÐ·ûºÅÕûÊý	ERPM
CAN_PACKET_SET_POS,              //4	ÉèÖÃµç»ú×ª½ÇÎ»ÖÃ
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


//ÓÃÓÚ½ÓÊÜvesc·¢³öµÄcanÊý¾ÝµÄ½á¹¹Ìå£º
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

void VESC_Set_RPM(uint32_t id, int32_t RPM, CAN_HandleTypeDef *hcan);    //ÉèÖÃµç»ú×ªËÙ£¨µç×ªËÙ£
void VESC_CAN_Send(uint32_t id, const uint8_t *data, uint8_t len, CAN_HandleTypeDef *hcan);//VESCÊý¾Ý·¢ËÍ
void motion_count(M3508x_STA *M350x);

void lift_step(float step);
float lift_move(float speed,int wei);
void catch_down(float down_pos);
void catch_up(float up_pos);
void VESC_Set_HardBreak_Current(uint32_t id, int32_t current, CAN_HandleTypeDef *hcan);
	