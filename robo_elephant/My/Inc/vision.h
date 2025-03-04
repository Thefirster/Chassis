#ifndef __VISION_H
#define __VISION_H

#include "timer.h"
#include "myusart.h"
#include "M3508.h"
#include "pid.h"
#include "math.h"

#define vision_process_none 0
#define vision_process_yes 1
#define vision_column_error 5 //��ʶ������

//vision
enum process_flag{
	//vision_control_flag �ڷ����и�������ֵ
	vision_ring_identify = 1,  //����Բ��
	vision_column_identify = 2,  //����Բ��
	vision_column_identify_left = 3,  //���Բ������
	vision_column_identify_right = 4,  //�ұ�Բ������
	//vision_process_flag
	vision_cloumn_identify_begin = 1,  //���ӱ��Ͽ�ʼ 
	vision_cloumn_identify_process = 9,  //����ʶ�������
	vision_cloumn_identify_complete = 2,  //���ӱ������
	
	vision_column_identify_left_begin = 3,  //����ʶ��ʼ
	vision_column_identify_left_process = 10,  //����ʶ�������
	vision_column_identify_left_complete = 4,  //�����ʶ�����
	
	vision_column_identify_right_begin = 5,  //����ʶ��ʼ
	vision_column_identify_right_process = 11,  //����ʶ�������
	vision_column_identify_right_complete = 6,  //����ʶ�����
	
	vision_ring_identify_begin = 7,  //��ʶ��ʼ
	vision_ring_identify_process = 12,  //��ʶ�������
	vision_ring_identify_complete = 8,  //��ʶ�����
};

//���ղ���
enum vision_receive_flag{
	find_column_flag = 1,  //�ҵ�Բ�����յ��ڶ�λΪˮƽerror
	unfind_column_flag = 2,  //δ�ҵ�Բ�����ڶ�λΪ0
	column_distance_flag = 3,  //��׼Բ�����յ��ڶ�λΪ��Ⱦ���
};
//���Ͳ���
enum vision_send_flag{
	column_begin_flag = 1,  //��������ʶ��
	column_align_flag = 2,  //���Ӷ������
	column_left_flag = 3,  //ʶ���������
	column_right_flag = 4,  //ʶ���ұ�����
};
//��־λ
typedef struct
{
	uint8_t vision_control_flag;  //���ڿ�������ʶ����������ʶ��
	uint8_t vision_process_flag;  //����չʾ��ǰ�����ĸ�������
}vision_flag;
//���շ�����Ϣ
typedef struct
{
	double send_data;
  float receive_data_mode;
  float receive_data_error;
}rx_tx_data;

//�Ӿ�������������
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
