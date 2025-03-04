#include "yuntai.h"

yuntai_control yuntai = {
	.M3508_pitch = 250,
	.RPM = 10000,
};
enum yuntai_receive_data{
	//���տ���
  fashe_close = 0,
	fashe_open = 1,
	//������ť
	fashe_pos1 = 1,
	fashe_pos2 = 2,
  fashe_pos3 = 3,
	fashe_pos4 = 4,
	fashe_pos5 = 5,
	//DM����
	DM_plus = 0,  //��תһȦ
	DM_reduce = 1,  //��תһȦ
	//��5��λ��
	fashe_pos1_level = 0,
	fashe_pos1_pitch = 0,
	RPM1 = 0,
	
	fashe_pos2_level = 0,
	fashe_pos2_pitch = 0,
	RPM2 = 0,
	
	fashe_pos3_level = 0,
	fashe_pos3_pitch = 0,
	RPM3 = 0,
	
	fashe_pos4_level = 0,
	fashe_pos4_pitch = 0,
	RPM4 = 0,
	
	fashe_pos5_level = 0,
	fashe_pos5_pitch = 0,
	RPM5 = 0,
};

/*
//M3508
M3508_Data_Init M3508_Data = {
	.level_send_angle = 0,
	.pitch_send_angle = 260,
};
//VESC
float RPM = 0;
//DM
DM_Data_Init DM = {
	.KP = 3.5f,
  .KD = 0.2f,
  .send_pos = 0,
  .send_vel = 0,
  .send_pos_cnt = 0,
  .tff = 0.31f,
};
//�Ӿ�
uint8_t vision_control_flag = 0;
double send_data = 0;  //�������������*/

//��̨���俪��һ��
//��ť4+1
//�������
void Yuntai_Rotor_Control(void)
{	
	/*
//	static int cnt = 0;
	
//   	limit_a_b(&level_send_angle, level_max, level_min);
//		M_Calculate_CascadePID_Test(&M3508[level_eid], level_send_angle, 1000);
//  	M_Encoder_Cascade_PID(&M3508[level_eid], &JY_Encoder[level_encoder], level_send_angle, SpeedMaxLevel);
	
	//Pitch�ñ�����
	
//  if(//(M3508[level_eid].angleSum < M3508_Data.anglesum_level_max)&&
////			 (M3508[level_eid].angleSum > M3508_Data.anglesum_level_min)&&
// 		(M3508[pitch_eid].angleSum < yuntai.anglesum_pitch_max)&&
//		(M3508[pitch_eid].angleSum > yuntai.anglesum_pitch_min)){
//		yuntai.M3508_pitch = 260;
//  }  

	if(cnt == 0){  //VESC
		cnt++;
	}else if(cnt == 1){  //DM
		if(mtr.vel > 5)
			yuntai.DM_vel = 20;
		else
			yuntai.DM_vel = 0;
		
	  if(yuntai.DM_cnt == 1){  //��1��һȦ
		  yuntai.DM_pos += ONE_ROUND_DM;
		  yuntai.DM_cnt = 0;
	  }else if(yuntai.DM_cnt == 2){  //��2��һȦ
	    yuntai.DM_pos -= ONE_ROUND_DM;
      yuntai.DM_cnt = 0;}
		
		cnt++;
	}else if(cnt == 2){*/
	static int cnt1 = 0;
	static int last_mode = 0;
	
		if(yuntai.begin_flag == fashe_open){  //�����
			switch(yuntai.mode_flag){
//				case fashe_pos1:
//					yuntai.M3508_level = fashe_pos1_level;
//				  yuntai.M3508_pitch = fashe_pos1_pitch;
//				  yuntai.RPM =  RPM1;
//					break;
				case fashe_pos2:
					yuntai.M3508_level = fashe_pos2_level;
				  yuntai.M3508_pitch = fashe_pos2_pitch;
				  yuntai.RPM =  RPM2;
					break;
        case fashe_pos3:
					yuntai.M3508_level = fashe_pos3_level;
				  yuntai.M3508_pitch = fashe_pos3_pitch; 
		  		yuntai.RPM =  RPM3;
					break;
        case fashe_pos4:
					yuntai.M3508_level = fashe_pos4_level;
				  yuntai.M3508_pitch = fashe_pos4_pitch;
				  yuntai.RPM =  RPM4;
					break;
        case fashe_pos5:
					yuntai.M3508_level = fashe_pos5_level;
				  yuntai.M3508_pitch = fashe_pos5_pitch;
				  yuntai.RPM =  RPM5;
					break;				
				default :
					break;
			}
		}
		
		if(yuntai.DM_flag == DM_plus){  //��ת
			if(last_mode == 2)
				cnt1 = 0;
			
			if(cnt1 == 0){
		    yuntai.DM_pos += ONE_ROUND_DM;
				cnt1++;
				last_mode = 1;
			}
		}else if(yuntai.DM_flag == DM_reduce){  //��ת
			if(last_mode == 1)
				cnt1 = 0;
			
			if(cnt1 == 0){
				yuntai.DM_pos -= ONE_ROUND_DM;
				cnt1++;
				last_mode = 2;
			}
		}
		
		if(yuntai.RPM_begin == 1){
			if(yuntai.RPM_flag == 1){
				yuntai.RPM = 0;
			}else if(yuntai.RPM_flag == 2){
			  yuntai.RPM = 5000;
		  }else if(yuntai.RPM_flag == 3){
				yuntai.RPM = 10000;
			}else if(yuntai.RPM_flag == 4){
				yuntai.RPM = 15000;
			}else if(yuntai.RPM_flag == 5){
				yuntai.RPM = 17200;
			}
		}
} 

//
void motor_control(void)
{	
	//pitch
	limit_a_b(&yuntai.M3508_pitch, pitch_max, pitch_min);
  M_Encoder_Cascade_PID(&M350x[pitch_eid], &JY_Encoder[pitch_encoder], yuntai.M3508_pitch, 1000);
	
	//level
//	M_Calculate_CascadePID_Test(&M350x[level_eid], yuntai.M3508_level, 1000);
	
	M_Calculate_PositionalPID_Test(&M350x[level_eid], yuntai.M3508_level);
	
	//M2006
	M_Calculate_CascadePID_Test(&M350x[m2006_eid], yuntai.M2006_angle_send, 1000);
}

//�Ӿ�����
void Vision_Control(void)
{
//	if(vision.flag.vision_process_flag != 0){  //process_flag��Ϊ0ʱ�ſ�������
//		vision.data.receive_data_mode = vision_data_receive.Receive.GetFloat[0];  
//	  vision.data.receive_data_error = vision_data_receive.Receive.GetFloat[1];  //���µ�ǰ����
//	
//	  if(vision_control(vision.flag.vision_control_flag) == vision_process_yes)  //ֻ�е�ǰ�������֮����ܸ��¿��Ʊ�־λ
//		  vision.flag.vision_control_flag = vision_control_flag;
//  
//	  send_data = vision.data.send_data;  //����ʵ�ʷ�������
//	}
}

//
void Anglesum_Max_Min_Init(void)  //51800 -45800
{
//	M3508_Data.anglesum_level_max = M3508[level_eid].angleSum + 97500*(level_max - JY_Encoder[level_encoder].angle)/(level_max - level_min) + 4000;
//	M3508_Data.anglesum_level_min = M3508[level_eid].angleSum - 97500*(JY_Encoder[level_encoder].angle - level_min)/(level_max - level_min) - 4000;
	yuntai.anglesum_pitch_max = M350x[pitch_eid].angleSum + 69500*(pitch_max - JY_Encoder[pitch_encoder].angle)/(pitch_max - pitch_min) + 4000;
	yuntai.anglesum_pitch_min = M350x[pitch_eid].angleSum - 69500*(JY_Encoder[pitch_encoder].angle - pitch_min)/(pitch_max - pitch_min) - 4000;
	yuntai.M3508_level= M350x[level_eid].angleSum;
}

//
volatile void limit_a_b(volatile float *control_num, volatile float max_num, volatile float min_num)
{
	if(*control_num > max_num)
		*control_num = max_num;
	if(*control_num < min_num)
		*control_num = min_num;
}
