#ifndef __QUHUAN_H
#define __QUHUAN_H

#include "stm32f4xx.h"

#define process_none 0
#define process_ok 1
#define pump_up 1
#define pump_down 0
#define duoji_level_state 0  //舵机水平
#define duoji_upright_state 1  //舵机竖直

//
enum shoubing_receive_data{	
	quhuan_trans_lor_close = 0,  
	quhuan_trans_lor_open = 1,  //旋转开始
	
	quhuan_trans_left = 2,  //旋转到左边
  quhuan_trans_right = 3,  //旋转到右边
	
	quhuan_put_lor_close = 0,  
	quhuan_put_lor_open = 1,  //放置开始
	
	quhuan_put_left = 2,  //左放置
	quhuan_put_right = 3,  //右放置
};

//开关1控制左右下面切换开启 旋钮两个模式
//开关2控制左放环和右放环开启 旋钮控制两个模式
//开关3控制舵机的水平
typedef struct{
  uint8_t switch_lor_trans_begin;  //左转右或者右转左开始
  uint8_t switch_lor_trans_mode;  //模式选择
	
	uint8_t switch_lor_put_begin;  //左边放或者右边开始放开始
	uint8_t switch_lor_put_mode;  //模式选择
	
	uint8_t duoji_level_upright;
}quhuan_init;

extern quhuan_init quhuan_flag;
extern uint8_t switch_now_flag;
extern float yuntai_middle_level; //中间初始位置
extern float Left_Down_Pos;

void air_pump(uint8_t pump_name, uint8_t pump_state);
void Pump_Init(void);
void duoji(uint8_t duoji_state);
uint8_t quhuan_left_up_put_down(void);
uint8_t trans_to_right(void);
uint8_t quhuan_right_up_put_down(void);
uint8_t trans_to_left(void);
void yuntai_control_all(void);

#endif
