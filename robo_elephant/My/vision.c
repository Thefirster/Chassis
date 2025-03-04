#include "vision.h"

//vision_init vision = {
//	.k_error = 0.01,
//};

////对水平电机控制
//void level_motor_control(uint8_t control_mode)
//{
////	static int flag_angle_cnt = 2;  
////  static float cycle_angle_start = 0;
////	
////	if(vision.data.receive_data_mode == find_column_flag){  //找到环
////		M3508_Data.level_send_angle += vision.data.receive_data_error * vision.k_error;  
////		if(ABS(vision.data.receive_data_error) < vision_column_error){  //对齐完成更改过程标志位
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
////	}else if(vision.data.receive_data_mode == unfind_column_flag){  //未找到环，自由转
////		if(cycle_angle_start == 0) 
////			cycle_angle_start = JY_Encoder[level_encoder].angle;
////		if(flag_angle_cnt%2 == 0){    //偶数次进入左转
////		  M3508_Data.level_send_angle = cycle_angle_start + flag_angle_cnt*12;
////		  if(fabs(JY_Encoder[level_encoder].angle - M3508_Data.level_send_angle) < 0.5)
////			  flag_angle_cnt++;
////		}else if(flag_angle_cnt%2 == 1){   //奇数次进入右转
////			M3508_Data.level_send_angle = cycle_angle_start - flag_angle_cnt*12;
////			if(fabs(JY_Encoder[level_encoder].angle - M3508_Data.level_send_angle) < 0.5)
////				flag_angle_cnt++;}
////	}
//}
////对俯仰电机控制
//void pitch_motor_control(float error)
//{
//	
//}
////control_mode当前识别模式，完成一个阶段后返回yes，否则none
//uint8_t vision_control(uint8_t control_mode)
//{
//	//控制部分
//	static uint8_t last_mode = 0;
//	
//	switch(control_mode){
//		case vision_ring_identify:  //辨认圆环 
//			break;
//		
//		case vision_column_identify:  //辨认圆柱 
//			if(vision.flag.vision_process_flag == vision_cloumn_identify_complete){  //对齐完成发column_align_flag 步骤3
//				if(vision.data.receive_data_mode == column_distance_flag)  //对齐后自动发深度距离
//					pitch_motor_control(vision.data.receive_data_error);
//				else
//					vision_send_data(column_align_flag);  //未收到深度距离则一直发送对齐命令
//			}
//			
//			if(last_mode == vision_column_identify_left_complete || 
//				 last_mode == vision_column_identify_right_complete || 
//				 last_mode == vision_ring_identify_complete){  //第一次由其他模式切换至此时，将标志位改变为begin 步骤1之前
//				   vision.flag.vision_process_flag = vision_cloumn_identify_begin;  
//				   last_mode = vision_cloumn_identify_complete;}
//		 
//			if(vision.data.receive_data_mode == find_column_flag || vision.data.receive_data_mode == unfind_column_flag)
//				if(vision.flag.vision_process_flag == vision_cloumn_identify_begin)
//					vision.flag.vision_process_flag = vision_cloumn_identify_process;  //收到error则改变标志位为process 步骤1->步骤2
//			
//			if(vision.flag.vision_process_flag == vision_cloumn_identify_process)  //标志位为process进行level_control 步骤2
//				level_motor_control(vision_column_identify);
//				
//			if(vision.flag.vision_process_flag == vision_cloumn_identify_begin)  //标志位为begin时发送column_begin_flag 步骤1
//				vision_send_data(column_begin_flag);
//			break;
//			
//		case vision_column_identify_left:  //左边圆柱辨认
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
//		case vision_column_identify_right:  //右边圆环辨认
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
//	//返回部分
//	if(vision.flag.vision_process_flag == vision_cloumn_identify_complete ||
//		 vision.flag.vision_process_flag == vision_column_identify_left_complete ||
//     vision.flag.vision_process_flag == vision_column_identify_right_complete ||
//	   vision.flag.vision_process_flag == vision_ring_identify_complete)
//			 return vision_process_yes;
//	else
//		return vision_process_none;
//}
