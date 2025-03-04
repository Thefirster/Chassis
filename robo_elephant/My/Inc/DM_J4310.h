#ifndef _DM_J4310
#define _DM_J4310

#include "main.h"

#define ABS(x)	( (x>0) ? (x) : (-x) )

//下面所有值都要与上位机中的值一致！！
//下面所有值都要与上位机中的值一致！！
//下面所有值都要与上位机中的值一致！！
#define P_MIN -5000.0f//反馈位置的最小值(rad)，初始值12.5
#define P_MAX 5000.0f
#define V_MIN -100.0f//反馈速度的最小值(rad/s)
#define V_MAX 100.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

#define MIT_MODE 1
#define PV_MODE 2
#define V_MODE 3

//电机1
//要控制的电机id与控制模式
#define DM_CANID_1 0x03 //电机1的canid，如果在上位机中更改了电机canid，记得改完把这个宏定义也改了
#define DM_MODE_1 MIT_MODE //电机1的控制模式，如果在上位机中更改了电机控制模式，记得改完把这个宏定义也改了

typedef struct{
	int id;
	int state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float pos;
	float vel;
	float toq;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
  float last_pos;
  float total_pos;
}Motor_Inf;


extern Motor_Inf cmd,mtr;

int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void DM_MotorStart(CAN_HandleTypeDef* hcan, uint16_t id, uint8_t ctrl_mode);
void DM_MotorCtrl(CAN_HandleTypeDef* hcan, uint16_t id, uint8_t ctrl_mode, float _pos, float _vel, float _KP, float _KD, float _torq);
void DM_S2P_MIT(CAN_HandleTypeDef* hcan, uint16_t id, float speed, float pos, float KP, float KD, float torq);
void DM_SpeedMode(CAN_HandleTypeDef* hcan, uint16_t id, float speed);
void DM_SpeedPositionMode(CAN_HandleTypeDef* hcan, uint16_t id, float pos, float maxspeed);

#endif
