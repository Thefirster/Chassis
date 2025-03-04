#ifndef __QUHUAN_H
#define __QUHUAN_H

#include "stm32f4xx.h"

#define process_none 0
#define process_ok 1
#define pump_up 1
#define pump_down 0
#define duoji_level_state 0  //���ˮƽ
#define duoji_upright_state 1  //�����ֱ

//
enum shoubing_receive_data{	
	quhuan_trans_lor_close = 0,  
	quhuan_trans_lor_open = 1,  //��ת��ʼ
	
	quhuan_trans_left = 2,  //��ת�����
  quhuan_trans_right = 3,  //��ת���ұ�
	
	quhuan_put_lor_close = 0,  
	quhuan_put_lor_open = 1,  //���ÿ�ʼ
	
	quhuan_put_left = 2,  //�����
	quhuan_put_right = 3,  //�ҷ���
};

//����1�������������л����� ��ť����ģʽ
//����2������Ż����ҷŻ����� ��ť��������ģʽ
//����3���ƶ����ˮƽ
typedef struct{
  uint8_t switch_lor_trans_begin;  //��ת�һ�����ת��ʼ
  uint8_t switch_lor_trans_mode;  //ģʽѡ��
	
	uint8_t switch_lor_put_begin;  //��߷Ż����ұ߿�ʼ�ſ�ʼ
	uint8_t switch_lor_put_mode;  //ģʽѡ��
	
	uint8_t duoji_level_upright;
}quhuan_init;

extern quhuan_init quhuan_flag;
extern uint8_t switch_now_flag;
extern float yuntai_middle_level; //�м��ʼλ��
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
