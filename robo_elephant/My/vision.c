#include "vision.h"

//vision_init vision = {
//	.k_error = 0.01,
//};

////��ˮƽ�������
//void level_motor_control(uint8_t control_mode)
//{
////	static int flag_angle_cnt = 2;  
////  static float cycle_angle_start = 0;
////	
////	if(vision.data.receive_data_mode == find_column_flag){  //�ҵ���
////		M3508_Data.level_send_angle += vision.data.receive_data_error * vision.k_error;  
////		if(ABS(vision.data.receive_data_error) < vision_column_error){  //������ɸ��Ĺ��̱�־λ
////			switch(control_mode){
////				case vision_column_identify:
////					vision.flag.vision_process_flag = vision_cloumn_identify_complete;
////				  flag_angle_cnt = 2;
////				  cycle_angle_start = 0;
////					break;
////				case vision_column_identify_left:
////					vision.flag.vision_process_flag = vision_column_identify_left_complete;
////				  flag_angle_cnt = 2;
////				  cycle_angle_start = 0;
////					break;
////				case vision_column_identify_right:
////					vision.flag.vision_process_flag = vision_column_identify_right_complete;
////				  flag_angle_cnt = 2;
////				  cycle_angle_start = 0;
////					break;
////				default:
////					break;
////			}
////		}
////	}
////	}else if(vision.data.receive_data_mode == unfind_column_flag){  //δ�ҵ���������ת
////		if(cycle_angle_start == 0) 
////			cycle_angle_start = JY_Encoder[level_encoder].angle;
////		if(flag_angle_cnt%2 == 0){    //ż���ν�����ת
////		  M3508_Data.level_send_angle = cycle_angle_start + flag_angle_cnt*12;
////		  if(fabs(JY_Encoder[level_encoder].angle - M3508_Data.level_send_angle) < 0.5)
////			  flag_angle_cnt++;
////		}else if(flag_angle_cnt%2 == 1){   //�����ν�����ת
////			M3508_Data.level_send_angle = cycle_angle_start - flag_angle_cnt*12;
////			if(fabs(JY_Encoder[level_encoder].angle - M3508_Data.level_send_angle) < 0.5)
////				flag_angle_cnt++;}
////	}
//}
////�Ը����������
//void pitch_motor_control(float error)
//{
//	
//}
////control_mode��ǰʶ��ģʽ�����һ���׶κ󷵻�yes������none
//uint8_t vision_control(uint8_t control_mode)
//{
//	//���Ʋ���
//	static uint8_t last_mode = 0;
//	
//	switch(control_mode){
//		case vision_ring_identify:  //����Բ�� 
//			break;
//		
//		case vision_column_identify:  //����Բ�� 
//			if(vision.flag.vision_process_flag == vision_cloumn_identify_complete){  //������ɷ�column_align_flag ����3
//				if(vision.data.receive_data_mode == column_distance_flag)  //������Զ�����Ⱦ���
//					pitch_motor_control(vision.data.receive_data_error);
//				else
//					vision_send_data(column_align_flag);  //δ�յ���Ⱦ�����һֱ���Ͷ�������
//			}
//			
//			if(last_mode == vision_column_identify_left_complete || 
//				 last_mode == vision_column_identify_right_complete || 
//				 last_mode == vision_ring_identify_complete){  //��һ��������ģʽ�л�����ʱ������־λ�ı�Ϊbegin ����1֮ǰ
//				   vision.flag.vision_process_flag = vision_cloumn_identify_begin;  
//				   last_mode = vision_cloumn_identify_complete;}
//		 
//			if(vision.data.receive_data_mode == find_column_flag || vision.data.receive_data_mode == unfind_column_flag)
//				if(vision.flag.vision_process_flag == vision_cloumn_identify_begin)
//					vision.flag.vision_process_flag = vision_cloumn_identify_process;  //�յ�error��ı��־λΪprocess ����1->����2
//			
//			if(vision.flag.vision_process_flag == vision_cloumn_identify_process)  //��־λΪprocess����level_control ����2
//				level_motor_control(vision_column_identify);
//				
//			if(vision.flag.vision_process_flag == vision_cloumn_identify_begin)  //��־λΪbeginʱ����column_begin_flag ����1
//				vision_send_data(column_begin_flag);
//			break;
//			
//		case vision_column_identify_left:  //���Բ������
//			if(vision.flag.vision_process_flag == vision_column_identify_left_complete){
//			  if(vision.data.receive_data_mode == column_distance_flag){
//					pitch_motor_control(vision.data.receive_data_error);
//				}else
//					vision_send_data(column_align_flag);
//			}
//			
//			if(last_mode == vision_cloumn_identify_complete || 
//				 last_mode == vision_column_identify_right_complete || 
//				 last_mode == vision_ring_identify_complete){  
//				   vision.flag.vision_process_flag = vision_column_identify_left_begin;
//				   last_mode = vision_column_identify_left_complete;}
//			
//			if(vision.data.receive_data_mode == find_column_flag || vision.data.receive_data_mode == unfind_column_flag)
//				if(vision.flag.vision_process_flag == vision_column_identify_left_begin)
//					vision.flag.vision_process_flag = vision_column_identify_left_process;  
//				
//			if(vision.flag.vision_process_flag == vision_column_identify_left_process)  
//				level_motor_control(vision_column_identify_left);
//			
//			if(vision.flag.vision_process_flag == vision_column_identify_left_begin)
//				vision_send_data(column_left_flag);
//			break;
//			
//		case vision_column_identify_right:  //�ұ�Բ������
//			if(vision.flag.vision_process_flag == vision_column_identify_right_complete){
//				if(vision.data.receive_data_mode == column_distance_flag){
//					pitch_motor_control(vision.data.receive_data_error);
//				}else
//					vision_send_data(column_align_flag);
//			}
//				
//      if(last_mode == vision_cloumn_identify_complete || 
//				 last_mode == vision_column_identify_left_complete || 
//				 last_mode == vision_ring_identify_complete){  
//				   vision.flag.vision_process_flag = vision_column_identify_right_process;
//				   last_mode = vision_column_identify_right_complete;}
//			
//			if(vision.data.receive_data_mode == find_column_flag || vision.data.receive_data_mode == unfind_column_flag)
//				if(vision.flag.vision_process_flag == vision_column_identify_right_begin)
//					vision.flag.vision_process_flag = vision_column_identify_right_process;  
//			
//			if(vision.flag.vision_process_flag == vision_column_identify_right_process)  
//				level_motor_control(vision_column_identify_right);
//			
//			if(vision.flag.vision_process_flag == vision_column_identify_right_begin)
//				vision_send_data(column_right_flag);
//			break;
//		default:
//			break;
//	}
//	
//	//���ز���
//	if(vision.flag.vision_process_flag == vision_cloumn_identify_complete ||
//		 vision.flag.vision_process_flag == vision_column_identify_left_complete ||
//     vision.flag.vision_process_flag == vision_column_identify_right_complete ||
//	   vision.flag.vision_process_flag == vision_ring_identify_complete)
//			 return vision_process_yes;
//	else
//		return vision_process_none;
//}
