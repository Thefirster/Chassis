#include "quhuan.h"
#include "yuntai.h"
#include "math.h"
#include "tim.h"
#include "M3508.h"
#include "Speed_decomposition.h"
#include "pid_controller.h"
//
extern M350D_STA M350x[8];
enum pump_name{
	fashe_pump = 0,
	quhuan_pump = 1,
	shangshen_pump = 2,
};
//
enum M2006_pos{
	
	Left_Up_Pos = 160000,
	Right_Down_Pos = 364500,
	Right_Up_Pos = 227700,
};
//
enum yuntai_pos{
	yuntai_pitch = 240,  //
	
	
	yuntai_left_level_1 = -20375,
	yuntai_left_level_2 = -72500,  
	
	yuntai_right_level_1 = 42600,
	yuntai_right_level_2 = 78381,
};
//
enum duoji_pos{
	duoji_level_left = 1150,
	duoji_level_right = 450,
	duoji_upright_left = 450,
	duoji_upright_right = 1200,
};

quhuan_init quhuan_flag;
float yuntai_middle_level = 0; //中间初始位置
float Left_Down_Pos = 0;
int last_mode = 0;
int last_mode1 = 0;

//M2006初始位置左边 取环气泵起始位置上 发射气泵起始位置上 上升气泵起始位置下 
//测量值：
//左侧M2006初始位置1 左侧M2006放环位置2 右侧M2006初始位置3 右侧M2006放环位置4
//云台初始位置1 云台左侧放环位置2 云台左侧旋转最终位置3 云台右侧放环位置4 云台哟测最终位置5

//气泵状态
void air_pump(uint8_t pump_name, uint8_t pump_state)
{
	if(pump_state == pump_up){
		switch(pump_name){
			case fashe_pump:  //发射up
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
				break;
			case quhuan_pump:  //取环up
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
				break;
			case shangshen_pump:  //上升up
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
				break;
			default:
				break;
		}
	}else if(pump_state == pump_down){
		switch(pump_name){
			case fashe_pump:  //发射down
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
				break;
			case quhuan_pump:  //取环down
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
				break;
			case shangshen_pump:  //上升down
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
				break;
			default:
				break;
		}
	}
}
//舵机控制
void duoji(uint8_t duoji_state)
{
	if(duoji_state == duoji_level_state){  //舵机水平
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duoji_level_left);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duoji_level_right);
	}else if(duoji_state == duoji_upright_state){  //舵机竖直
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duoji_upright_left);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duoji_upright_right);
	}
}
//只进行一次，在取环开始前
void Zhuaqu_Init(void)
{
	air_pump(fashe_pump, pump_up);
	HAL_Delay(1000);
	air_pump(quhuan_pump, pump_up);
	HAL_Delay(1000);
	air_pump(shangshen_pump, pump_down);  //气泵初始化
	HAL_Delay(1000);  
	
	yuntai.M3508_pitch = yuntai_pitch;
	yuntai.M3508_level = yuntai_middle_level;   //云台初始化
	
	yuntai.M2006_angle_send = Left_Down_Pos;  //m2006初始化
	
	duoji(duoji_level_state);  //舵机初始化
}
//
void yuntai_control_all(void)
{
	if(quhuan_flag.switch_lor_put_begin == quhuan_trans_lor_open){ //左切右 右切左
		if(quhuan_flag.switch_lor_trans_mode == quhuan_trans_left)  //切换到左
			trans_to_left();
		else if(quhuan_flag.switch_lor_trans_mode  == quhuan_trans_right)  //切换到右
			trans_to_right();
	}else if(quhuan_flag.switch_lor_trans_begin == quhuan_put_lor_open){  //放置开始
		if(quhuan_flag.switch_lor_put_mode == quhuan_put_left)  //左边放
			quhuan_left_up_put_down();
		else if(quhuan_flag.switch_lor_put_mode == quhuan_put_right)  //右边放
			quhuan_right_up_put_down();
	}
	
	if(quhuan_flag.duoji_level_upright == duoji_level_state){  //水平放置舵机
		duoji(duoji_level_state);
	}else if(quhuan_flag.duoji_level_upright == duoji_upright_state){  //竖直放置舵机
		duoji(duoji_upright_state);
	}
}
//1上升放环加下降
//开启标志：开关1开+
//上升：取环气泵下 上升气泵上 
//放环：云台位置2 M2006位置2 取环气泵上 发射气泵下 云台位置3
//下降：M2006位置1 上升气泵下 云台1
uint8_t quhuan_left_up_put_down(void)
{
	static int cnt = 0;
	
	if(last_mode == 1)
		cnt = 0;
	
	switch(cnt){
		case 0:  //夹紧环上升
			air_pump(quhuan_pump, pump_down);  
	    HAL_Delay(1000); 
	    air_pump(shangshen_pump, pump_up);  
	    HAL_Delay(2000);
		  last_mode = 2;
		  cnt++;
			break;
		case 1:  //转动云台水平到指定位置
			yuntai.M3508_level = yuntai_left_level_1;
		  if(fabs(M350x[level_eid].angleSum - yuntai.M3508_level) < 100)
			  cnt++;
			break;
		case 2:  //M2006到放环的位置
			yuntai.M2006_angle_send = Left_Up_Pos;
		  if(fabs(M350x[m2006_eid].angleSum - yuntai.M2006_angle_send) < 100)
			  cnt++;
			break;
		case 3:  //取环部分up 发射部分down
			air_pump(quhuan_pump, pump_up);  
	    HAL_Delay(1000);
		  air_pump(fashe_pump, pump_down);  
	    HAL_Delay(1000);
		  cnt++;
			break;
		case 4:  //云台继续转到与取环部分分离
			yuntai.M3508_level = yuntai_left_level_2;
		  if(fabs(M350x[level_eid].angleSum - yuntai.M3508_level) < 100)
			  cnt++;
			break;
		case 5:  //M2006回到左边初始位置
			yuntai.M2006_angle_send = Left_Down_Pos;
		  if(fabs(M350x[m2006_eid].angleSum - yuntai.M2006_angle_send) < 100)
			  cnt++;
			break;
		case 6:  //下降，发射气泵上升，舵机转到竖直
			air_pump(shangshen_pump, pump_down);  
	    HAL_Delay(1000);
		  air_pump(fashe_pump, pump_up);  
	    HAL_Delay(1000);
		  cnt++;
		  break;
		case 7:  //云台归位
			yuntai.M3508_level = yuntai_middle_level;
		  if(fabs(M350x[level_eid].angleSum - yuntai.M3508_level) < 100)
			  cnt++;
			break;
		case 8:
			return process_ok;
		default:
			break;
	}
	return process_none;
}

//2旋转到右侧
//开启标志：开关1开+
//上升气泵上 M2006位置3 上升气泵下 
uint8_t trans_to_right(void)
{
	static int cnt = 0;
	
	if(last_mode1 == 1)
		cnt = 0;
	
	switch(cnt){
		case 0:  //上升
			air_pump(shangshen_pump, pump_up);  
	    HAL_Delay(1000);
		  last_mode1 = 2;
		  cnt++;
			break;
		case 1:  //M2006旋转
			yuntai.M2006_angle_send = Right_Down_Pos;
		  if(fabs(M350x[m2006_eid].angleSum - yuntai.M2006_angle_send) < 100)
			  cnt++;
			break;
		case 2:  //下降
			air_pump(shangshen_pump, pump_down);  
	    HAL_Delay(1000);
			return process_ok;
		default:
			break;
	}
	return process_none;
}

//3右侧上升放环加下降
//开启标志：开关1开 
//上升：取环气泵下 上升气泵上
//放环：云台位置4 M2006位置4 取环气泵上 发射气泵下 云台位置5
//下降：M2006位置3 上升气泵下 云台位置1
uint8_t quhuan_right_up_put_down(void)
{
	static int cnt = 0;
	
	if(last_mode == 2)
		cnt = 0;
	
	switch(cnt){
		case 0:  //夹紧环上升
			air_pump(quhuan_pump, pump_down);  
	    HAL_Delay(1000); 
	    air_pump(shangshen_pump, pump_up);  
	    HAL_Delay(2000);
		  last_mode = 1;
		  cnt++;
			break;
		case 1:  //转动云台水平到指定位置
			yuntai.M3508_level = yuntai_right_level_1;
		  if(fabs(M350x[level_eid].angleSum - yuntai.M3508_level) < 100)
			  cnt++;
			break;
		case 2:  //M2006到放环的位置
			yuntai.M2006_angle_send = Right_Up_Pos;
		  if(fabs(M350x[m2006_eid].angleSum - yuntai.M2006_angle_send) < 100)
			  cnt++;
			break;
		case 3:  //取环部分up 发射部分down
			air_pump(quhuan_pump, pump_up);  
	    HAL_Delay(1000);
		  air_pump(fashe_pump, pump_down);  
	    HAL_Delay(1000);
		  cnt++;
			break;
		case 4:  //云台继续转到与取环部分分离
			yuntai.M3508_level = yuntai_right_level_2;
		  if(fabs(M350x[level_eid].angleSum - yuntai.M3508_level) < 100)
			  cnt++;
			break;
		case 5:  //M2006回到左边初始位置
			yuntai.M2006_angle_send = Right_Down_Pos;
		  if(fabs(M350x[m2006_eid].angleSum - yuntai.M2006_angle_send) < 100)
			  cnt++;
			break;
		case 6:  //下降，发射气泵上升，舵机转到竖直
			air_pump(shangshen_pump, pump_down);  
	    HAL_Delay(1000);
		  air_pump(fashe_pump, pump_up);  
	    HAL_Delay(1000);
		  cnt++;
		  break;
		case 7:  //云台归位
			yuntai.M3508_level = yuntai_middle_level;
		  if(fabs(M350x[level_eid].angleSum - yuntai.M3508_level) < 100)
			  cnt++;
			break;
		case 8:
			return process_ok;
		default:
			break;
	}
	return process_none;
}

//4旋转到左侧
//开启标志：开关1开
//上升气泵上 M2006位置1 上升气泵下
uint8_t trans_to_left(void)
{
	static int cnt = 0;
	
	if(last_mode1 == 2)
		cnt = 0;
	
	switch(cnt){
		case 0:  //上升
			air_pump(shangshen_pump, pump_up);  
	    HAL_Delay(1000);
		  last_mode1 = 1;
		  cnt++;
			break;
		case 1:  //M2006旋转
			yuntai.M2006_angle_send = Left_Down_Pos;
		  if(fabs(M350x[m2006_eid].angleSum - yuntai.M2006_angle_send) < 100)
			  cnt++;
			break;
		case 2:  //下降
			air_pump(shangshen_pump, pump_down);  
	    HAL_Delay(1000);
			return process_ok;
		default:
			break;
	}
	return process_none;
}

