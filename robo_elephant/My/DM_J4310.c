#include "DM_J4310.h"
#include "can.h"
#include "pid.h"
#include "M3508.h"

PID DM_SpeedPID;
PID DM_SP_Pos_PID;
PID DM_SP_Speed_PID;

Motor_Inf cmd,mtr;
#define DM_MAX_SPEED 1000
#define DM_MAX_POS 1000

int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }
    
    
float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }

void start_motor(CAN_HandleTypeDef* hcan,uint16_t id)
{
  CAN_TxHeaderTypeDef TxHeader;
  uint8_t Data[8] = {0};
  uint32_t TxMailbox;
  
	TxHeader.StdId = id;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 0x08;
	Data[0] = 0xFF;
	Data[1] = 0xFF;
	Data[2] = 0xFF;
	Data[3] = 0xFF;
	Data[4] = 0xFF;
	Data[5] = 0xFF;
	Data[6] = 0xFF;
	Data[7] = 0xFC;
	
	HAL_CAN_AddTxMessage(hcan, &TxHeader, Data, &TxMailbox);
}	

void set_zero_pos(CAN_HandleTypeDef* hcan,uint16_t id)
{
  CAN_TxHeaderTypeDef TxHeader;
  uint8_t Data[8] = {0};
  uint32_t TxMailbox;
  
	TxHeader.StdId = id;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 0x08;
	Data[0] = 0xFF;
	Data[1] = 0xFF;
	Data[2] = 0xFF;
	Data[3] = 0xFF;
	Data[4] = 0xFF;
	Data[5] = 0xFF;
	Data[6] = 0xFF;
	Data[7] = 0xFE;
	
	HAL_CAN_AddTxMessage(hcan, &TxHeader, Data, &TxMailbox);
}	

void ctrl_motor_MIT(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq)
{
  CAN_TxHeaderTypeDef TxHeader;
  uint8_t Data[8] = {0};
  uint32_t TxMailbox;
  
  uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
  vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
	kp_tmp  = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
  tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);
	
	TxHeader.StdId = id;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 0x08;
	Data[0] = (pos_tmp >> 8);
	Data[1] = pos_tmp;
	Data[2] = (vel_tmp >> 4);
	Data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	Data[4] = kp_tmp;
	Data[5] = (kd_tmp >> 4);
	Data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	Data[7] = tor_tmp;
	
	HAL_CAN_AddTxMessage(hcan, &TxHeader, Data, &TxMailbox);
}	

void ctrl_motor_PV(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel)
{
  CAN_TxHeaderTypeDef TxHeader;
  uint8_t Data[8] = {0};
  uint32_t TxMailbox;
  
	uint8_t *pbuf,*vbuf;
	pbuf=(uint8_t*)&_pos;
	vbuf=(uint8_t*)&_vel;
	
	TxHeader.StdId = id;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 0x08;
	Data[0] = *pbuf;
	Data[1] = *(pbuf+1);
	Data[2] = *(pbuf+2);
	Data[3] = *(pbuf+3);
	Data[4] = *vbuf;
	Data[5] = *(vbuf+1);
	Data[6] = *(vbuf+2);
	Data[7] = *(vbuf+3);
	
	HAL_CAN_AddTxMessage(hcan, &TxHeader, Data, &TxMailbox);
}	

void ctrl_motor_V(CAN_HandleTypeDef* hcan,uint16_t id, float _vel)
{
  CAN_TxHeaderTypeDef TxHeader;
  uint8_t Data[8] = {0};
  uint32_t TxMailbox;
  
  uint8_t *vbuf;
	vbuf=(uint8_t*)&_vel;
	
	TxHeader.StdId = id;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 0x04;
	Data[0] = *vbuf;
	Data[1] = *(vbuf+1);
	Data[2] = *(vbuf+2);
	Data[3] = *(vbuf+3);
	
	HAL_CAN_AddTxMessage(hcan, &TxHeader, Data, &TxMailbox);
}	


/**
 * @description: 电机启动函数，在控制电机前调用一次即可，可多次调用
 * @param {CAN_HandleTypeDef*} hcan
 * @param {uint16_t} id：DM电机对应的canid
 * @param {uint8_t} ctrl_mode：控制模式，@ref：MIT_MODE，PV_MODE，V_MODE
 * @return {*}
 */
void DM_MotorStart(CAN_HandleTypeDef* hcan, uint16_t id, uint8_t ctrl_mode)
{
  static uint16_t canid = 0;
  if(ctrl_mode==1) // MIT MODE
    canid = id;
  else if(ctrl_mode==2) // p_v cascade mode
    canid = id+0x100;
  else if(ctrl_mode==3)//v mode
    canid = id+0x200;
  
  start_motor(hcan,canid);
  HAL_Delay(10);
  set_zero_pos(hcan, canid);//设置当前位置为位置零点
}

/**
 * @description: 电机控制函数，循环调用这个即可控制电机
 * @param {CAN_HandleTypeDef*} hcan
 * @param {uint16_t} id：DM电机对应的canid
 * @param {uint8_t} ctrl_mode：控制模式
 *        @arg MIT_MODE：MIT 模式有三种控制方法：速度、位置、力矩
 *                       速度模式：给定速度 5r/s,KD给 1N*s/r 其余全部给 0
 *                       位置模式：位置给定 3.14rad，KP 给定 2N/r，KD 给 1N*s/r 其余全部给 0
 *                       力矩模式：转矩设定为 1N-m，其余全部给 0
 *        @arg PV_MODE：速度位置模式，_pos为目标位置，_vel为梯形最大速度，只能在上位机中调pid
 *        @arg V_MODE：速度模式，_vel为目标速度，只能在上位机中调pid
 * @param {float} _pos：MIT，PV模式下目标位置，V模式下没用
 * @param {float} _vel：MIT，PV，V模式下目标速度  
 * @param {float} _KP：仅MIT模式下有用
 * @param {float} _KD：仅MIT模式下有用
 * @param {float} _torq：力矩，仅MIT模式下有用
 * @return {*}
 */
void DM_MotorCtrl(CAN_HandleTypeDef* hcan, uint16_t id, uint8_t ctrl_mode, float _pos, float _vel, float _KP, float _KD, float _torq)
{
  if(ctrl_mode==1) // MIT MODE
   ctrl_motor_MIT(hcan, id, _pos, _vel, _KP ,_KD, _torq);
  else if(ctrl_mode==2) // p_v cascade mode
   ctrl_motor_PV(hcan, id+0x100,_pos,_vel);
  else if(ctrl_mode==3) // v_mode
   ctrl_motor_V(hcan, id+0x200,_vel);
}

//MIT模式下，以一定速度转到指定位置，速度范围0~30rad/s，空载转速最大20.9rad/s(200rpm)，位置范围正负50rad。头文件中可改位置范围
//堵塞式函数，通过位置反馈，切换MIT的三种模式（力矩模式、位置模式、速度模式），达到速度位置环的效果
//不可用
//void DM_S2P_MIT(CAN_HandleTypeDef* hcan, uint16_t id, float pos, float speed, float KP, float KD, float torq)
//{
//  while((pos - mtr.pos)>0.1)//逆时针正向转
//  {
//    DM_MotorCtrl(hcan, id, 0, speed, 0, 1, torq);
//    HAL_Delay(2);
//  }
//  DM_MotorCtrl(hcan, id, 0, 0, 0, 1, torq);
//}


/**
 * @description: 速度环，可用，MIT力矩模式
 * @param {CAN_HandleTypeDef*} hcan
 * @param {uint16_t} id：DM电机对应的canid
 * @param {float} speed：目标速度
 * @return {*}
 */
void DM_SpeedMode(CAN_HandleTypeDef* hcan, uint16_t id, float speed)
{
  PID_Calculate_Positional(&DM_SpeedPID, mtr.vel, speed);//位置式PID计算
  DM_MotorCtrl(hcan, id, DM_MODE_1, 0, 0, 0, 0, DM_SpeedPID.Output);
}


/**
 * @description: 速度位置环，可用，MIT力矩模式，PID待调整
 * @param {CAN_HandleTypeDef*} hcan
 * @param {uint16_t} id：DM电机对应的canid
 * @param {float} pos：目标位置，单位rad
 * @param {float} maxspeed，梯形速度的最大值
 * @return {*}
 */
void DM_SpeedPositionMode(CAN_HandleTypeDef* hcan, uint16_t id, float pos, float maxspeed)
{
	PID_Calculate_Positional(&DM_SP_Pos_PID, mtr.pos, pos);
	DM_SP_Pos_PID.Output = Scope_Max_Min(DM_SP_Pos_PID.Output, DM_MAX_POS);
	PID_Calculate_Incremental(&DM_SP_Speed_PID, mtr.vel, DM_SP_Pos_PID.Output);
	DM_SP_Speed_PID.Output = Scope_Max_Min(DM_SP_Speed_PID.Output, DM_MAX_SPEED);
  DM_MotorCtrl(hcan, id, DM_MODE_1, 0, 0, 0, 0, DM_SP_Speed_PID.Output);//力矩控制
}
