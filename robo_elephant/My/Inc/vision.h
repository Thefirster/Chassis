#ifndef __VISION_H
#define __VISION_H

#include "timer.h"
#include "myusart.h"
#include "M3508.h"
#include "pid.h"
#include "math.h"

#define vision_process_none 0
#define vision_process_yes 1
#define vision_column_error 5 //环识别死区

//vision
enum process_flag{
	//vision_control_flag 在仿真中赋给下面值
	vision_ring_identify = 1,  //辨认圆环
	vision_column_identify = 2,  //辨认圆柱
	vision_column_identify_left = 3,  //左边圆环辨认
	vision_column_identify_right = 4,  //右边圆环辨认
	//vision_process_flag
	vision_cloumn_identify_begin = 1,  //柱子辨认开始 
	vision_cloumn_identify_process = 9,  //柱子识别过程中
	vision_cloumn_identify_complete = 2,  //柱子辨认完成
	
	vision_column_identify_left_begin = 3,  //左柱识别开始
	vision_column_identify_left_process = 10,  //左柱识别过程中
	vision_column_identify_left_complete = 4,  //左边柱识别完成
	
	vision_column_identify_right_begin = 5,  //右柱识别开始
	vision_column_identify_right_process = 11,  //右柱识别过程中
	vision_column_identify_right_complete = 6,  //右柱识别完成
	
	vision_ring_identify_begin = 7,  //环识别开始
	vision_ring_identify_process = 12,  //环识别过程中
	vision_ring_identify_complete = 8,  //环识别完成
};

//接收部分
enum vision_receive_flag{
	find_column_flag = 1,  //找到圆柱，收到第二位为水平error
	unfind_column_flag = 2,  //未找到圆柱，第二位为0
	column_distance_flag = 3,  //对准圆柱后，收到第二位为深度距离
};
//发送部分
enum vision_send_flag{
	column_begin_flag = 1,  //开启柱子识别
	column_align_flag = 2,  //柱子对齐完成
	column_left_flag = 3,  //识别左边柱子
	column_right_flag = 4,  //识别右边柱子
};
//标志位
typedef struct
{
	uint8_t vision_control_flag;  //用于开启柱子识别，左右柱子识别
	uint8_t vision_process_flag;  //用于展示当前是在哪个过程中
}vision_flag;
//接收发送信息
typedef struct
{
	double send_data;
  float receive_data_mode;
  float receive_data_error;
}rx_tx_data;

//视觉部分所用数据
typedef struct
{
	rx_tx_data data;
	vision_flag flag;
  float k_error;
}vision_init;

uint8_t vision_control(uint8_t control_mode);
void level_motor_control(uint8_t control_mode);
void pitch_motor_control(float error);

extern vision_init vision;

#endif
