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
float yuntai_middle_level = 0; //�м��ʼλ��
float Left_Down_Pos = 0;
int last_mode = 0;
int last_mode1 = 0;

//M2006��ʼλ����� ȡ��������ʼλ���� ����������ʼλ���� ����������ʼλ���� 
//����ֵ��
//���M2006��ʼλ��1 ���M2006�Ż�λ��2 �Ҳ�M2006��ʼλ��3 �Ҳ�M2006�Ż�λ��4
//��̨��ʼλ��1 ��̨���Ż�λ��2 ��̨�����ת����λ��3 ��̨�Ҳ�Ż�λ��4 ��̨Ӵ������λ��5

//����״̬
void air_pump(uint8_t pump_name, uint8_t pump_state)
{
	if(pump_state == pump_up){
		switch(pump_name){
			case fashe_pump:  //����up
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
				break;
			case quhuan_pump:  //ȡ��up
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
				break;
			case shangshen_pump:  //����up
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
				break;
			default:
				break;
		}
	}else if(pump_state == pump_down){
		switch(pump_name){
			case fashe_pump:  //����down
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
				break;
			case quhuan_pump:  //ȡ��down
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
				break;
			case shangshen_pump:  //����down
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
				break;
			default:
				break;
		}
	}
}
//�������
void duoji(uint8_t duoji_state)
{
	if(duoji_state == duoji_level_state){  //���ˮƽ
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duoji_level_left);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duoji_level_right);
	}else if(duoji_state == duoji_upright_state){  //�����ֱ
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duoji_upright_left);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duoji_upright_right);
	}
}
//ֻ����һ�Σ���ȡ����ʼǰ
void Zhuaqu_Init(void)
{
	air_pump(fashe_pump, pump_up);
	HAL_Delay(1000);
	air_pump(quhuan_pump, pump_up);
	HAL_Delay(1000);
	air_pump(shangshen_pump, pump_down);  //���ó�ʼ��
	HAL_Delay(1000);  
	
	yuntai.M3508_pitch = yuntai_pitch;
	yuntai.M3508_level = yuntai_middle_level;   //��̨��ʼ��
	
	yuntai.M2006_angle_send = Left_Down_Pos;  //m2006��ʼ��
	
	duoji(duoji_level_state);  //�����ʼ��
}
//
void yuntai_control_all(void)
{
	if(quhuan_flag.switch_lor_put_begin == quhuan_trans_lor_open){ //������ ������
		if(quhuan_flag.switch_lor_trans_mode == quhuan_trans_left)  //�л�����
			trans_to_left();
		else if(quhuan_flag.switch_lor_trans_mode  == quhuan_trans_right)  //�л�����
			trans_to_right();
	}else if(quhuan_flag.switch_lor_trans_begin == quhuan_put_lor_open){  //���ÿ�ʼ
		if(quhuan_flag.switch_lor_put_mode == quhuan_put_left)  //��߷�
			quhuan_left_up_put_down();
		else if(quhuan_flag.switch_lor_put_mode == quhuan_put_right)  //�ұ߷�
			quhuan_right_up_put_down();
	}
	
	if(quhuan_flag.duoji_level_upright == duoji_level_state){  //ˮƽ���ö��
		duoji(duoji_level_state);
	}else if(quhuan_flag.duoji_level_upright == duoji_upright_state){  //��ֱ���ö��
		duoji(duoji_upright_state);
	}
}
//1�����Ż����½�
//������־������1��+
//������ȡ�������� ���������� 
//�Ż�����̨λ��2 M2006λ��2 ȡ�������� ���������� ��̨λ��3
//�½���M2006λ��1 ���������� ��̨1
uint8_t quhuan_left_up_put_down(void)
{
	static int cnt = 0;
	
	if(last_mode == 1)
		cnt = 0;
	
	switch(cnt){
		case 0:  //�н�������
			air_pump(quhuan_pump, pump_down);  
	    HAL_Delay(1000); 
	    air_pump(shangshen_pump, pump_up);  
	    HAL_Delay(2000);
		  last_mode = 2;
		  cnt++;
			break;
		case 1:  //ת����̨ˮƽ��ָ��λ��
			yuntai.M3508_level = yuntai_left_level_1;
		  if(fabs(M350x[level_eid].angleSum - yuntai.M3508_level) < 100)
			  cnt++;
			break;
		case 2:  //M2006���Ż���λ��
			yuntai.M2006_angle_send = Left_Up_Pos;
		  if(fabs(M350x[m2006_eid].angleSum - yuntai.M2006_angle_send) < 100)
			  cnt++;
			break;
		case 3:  //ȡ������up ���䲿��down
			air_pump(quhuan_pump, pump_up);  
	    HAL_Delay(1000);
		  air_pump(fashe_pump, pump_down);  
	    HAL_Delay(1000);
		  cnt++;
			break;
		case 4:  //��̨����ת����ȡ�����ַ���
			yuntai.M3508_level = yuntai_left_level_2;
		  if(fabs(M350x[level_eid].angleSum - yuntai.M3508_level) < 100)
			  cnt++;
			break;
		case 5:  //M2006�ص���߳�ʼλ��
			yuntai.M2006_angle_send = Left_Down_Pos;
		  if(fabs(M350x[m2006_eid].angleSum - yuntai.M2006_angle_send) < 100)
			  cnt++;
			break;
		case 6:  //�½��������������������ת����ֱ
			air_pump(shangshen_pump, pump_down);  
	    HAL_Delay(1000);
		  air_pump(fashe_pump, pump_up);  
	    HAL_Delay(1000);
		  cnt++;
		  break;
		case 7:  //��̨��λ
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

//2��ת���Ҳ�
//������־������1��+
//���������� M2006λ��3 ���������� 
uint8_t trans_to_right(void)
{
	static int cnt = 0;
	
	if(last_mode1 == 1)
		cnt = 0;
	
	switch(cnt){
		case 0:  //����
			air_pump(shangshen_pump, pump_up);  
	    HAL_Delay(1000);
		  last_mode1 = 2;
		  cnt++;
			break;
		case 1:  //M2006��ת
			yuntai.M2006_angle_send = Right_Down_Pos;
		  if(fabs(M350x[m2006_eid].angleSum - yuntai.M2006_angle_send) < 100)
			  cnt++;
			break;
		case 2:  //�½�
			air_pump(shangshen_pump, pump_down);  
	    HAL_Delay(1000);
			return process_ok;
		default:
			break;
	}
	return process_none;
}

//3�Ҳ������Ż����½�
//������־������1�� 
//������ȡ�������� ����������
//�Ż�����̨λ��4 M2006λ��4 ȡ�������� ���������� ��̨λ��5
//�½���M2006λ��3 ���������� ��̨λ��1
uint8_t quhuan_right_up_put_down(void)
{
	static int cnt = 0;
	
	if(last_mode == 2)
		cnt = 0;
	
	switch(cnt){
		case 0:  //�н�������
			air_pump(quhuan_pump, pump_down);  
	    HAL_Delay(1000); 
	    air_pump(shangshen_pump, pump_up);  
	    HAL_Delay(2000);
		  last_mode = 1;
		  cnt++;
			break;
		case 1:  //ת����̨ˮƽ��ָ��λ��
			yuntai.M3508_level = yuntai_right_level_1;
		  if(fabs(M350x[level_eid].angleSum - yuntai.M3508_level) < 100)
			  cnt++;
			break;
		case 2:  //M2006���Ż���λ��
			yuntai.M2006_angle_send = Right_Up_Pos;
		  if(fabs(M350x[m2006_eid].angleSum - yuntai.M2006_angle_send) < 100)
			  cnt++;
			break;
		case 3:  //ȡ������up ���䲿��down
			air_pump(quhuan_pump, pump_up);  
	    HAL_Delay(1000);
		  air_pump(fashe_pump, pump_down);  
	    HAL_Delay(1000);
		  cnt++;
			break;
		case 4:  //��̨����ת����ȡ�����ַ���
			yuntai.M3508_level = yuntai_right_level_2;
		  if(fabs(M350x[level_eid].angleSum - yuntai.M3508_level) < 100)
			  cnt++;
			break;
		case 5:  //M2006�ص���߳�ʼλ��
			yuntai.M2006_angle_send = Right_Down_Pos;
		  if(fabs(M350x[m2006_eid].angleSum - yuntai.M2006_angle_send) < 100)
			  cnt++;
			break;
		case 6:  //�½��������������������ת����ֱ
			air_pump(shangshen_pump, pump_down);  
	    HAL_Delay(1000);
		  air_pump(fashe_pump, pump_up);  
	    HAL_Delay(1000);
		  cnt++;
		  break;
		case 7:  //��̨��λ
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

//4��ת�����
//������־������1��
//���������� M2006λ��1 ����������
uint8_t trans_to_left(void)
{
	static int cnt = 0;
	
	if(last_mode1 == 2)
		cnt = 0;
	
	switch(cnt){
		case 0:  //����
			air_pump(shangshen_pump, pump_up);  
	    HAL_Delay(1000);
		  last_mode1 = 1;
		  cnt++;
			break;
		case 1:  //M2006��ת
			yuntai.M2006_angle_send = Left_Down_Pos;
		  if(fabs(M350x[m2006_eid].angleSum - yuntai.M2006_angle_send) < 100)
			  cnt++;
			break;
		case 2:  //�½�
			air_pump(shangshen_pump, pump_down);  
	    HAL_Delay(1000);
			return process_ok;
		default:
			break;
	}
	return process_none;
}

