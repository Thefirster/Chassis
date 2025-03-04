#include "Speed_decomposition.h"
#include "pid_controller.h"
#include "math.h"
#include "vesc.h"
#include "usart.h"
#include "lj_plus.h"
#include "yuntai.h"


M3508x_STA M3508[8];
int k = 0;
float multiple_angle = 144/17.0f*8192/360.0f;    //角度制X这个
wheel_Struct  	wheel_rotation_v[5],        //轮子自转速度
								wheel_final_v[5];           //轮子最终速度

extern coordinate_Struct Robot_Coordinate_system_Vel;	
extern yuntai_control yuntai;

//static float ABS(float a)
//{
//	return a>0?a:-a;
//}								
//sin和cos算角度
float get_angle(float sin_num,float cos_num)
{//极坐标
	if(sin_num>=0&&cos_num>=0)//第一象限点
		return asin(sin_num);
	else if(sin_num>=0&&cos_num<=0)//第二象限
		return PI-asin(sin_num);
	else if(sin_num<=0&&cos_num<=0)//第三象限
		return PI-asin(sin_num);
	else if(sin_num<=0&&cos_num>=0)//第四象限
		return 2*PI+asin(sin_num);
	return 0;
	
}				

void gain_absolute_angle(wheel_Struct *wheel_v,int id)
{
	switch(id)
	{
		case 1:    wheel_v->absolute_angle = wheel_v->angle_sum; //角度差的积分  +    这个轮的初始角度，你摆在0就是0，摆在45就是45
					break;
		case 2:    wheel_v->absolute_angle = wheel_v->angle_sum; //角度差的积分
					break;
		case 3:    wheel_v->absolute_angle = wheel_v->angle_sum; //角度差的积分
					break;
		case 4:    wheel_v->absolute_angle = wheel_v->angle_sum; //角度差的积分
					break;
		default:	break;
	}

	while(wheel_v->absolute_angle>=360)
	wheel_v->absolute_angle -= 360;
	while(wheel_v->absolute_angle<0)
	wheel_v->absolute_angle += 360;          //将absolute_angle化为【0,360),
}

void gain_gap_angle(wheel_Struct *wheel_v)
{
	float speed;
	speed = wheel_v->resultant_v;
	wheel_v->angle_gap= wheel_v->v_angle - wheel_v->absolute_angle; //轮接下来要转的角度，目标速度角度－当前舵轮角度，两个量都已是[0,360),结果为负则舵轮顺时针转		   
	if(wheel_v->angle_gap>270)
	{
		wheel_v->angle_gap = -(360-wheel_v->angle_gap);
		speed = speed;
	}
	else if(wheel_v->angle_gap>180)
	{
		wheel_v->angle_gap = -180+wheel_v->angle_gap;
		speed = -speed;
		//wheel_v->resultant_v = -wheel_v->resultant_v;
	}
	else if(wheel_v->angle_gap>90)
	{
		wheel_v->angle_gap = -(180-wheel_v->angle_gap);
		speed = -speed; 
		//wheel_v->resultant_v = -wheel_v->resultant_v;
	} 
	
	if(wheel_v->angle_gap<-270)
	{
		wheel_v->angle_gap = (360+wheel_v->angle_gap);
		speed = speed;
	}
	else if(wheel_v->angle_gap<-180)
	{
		wheel_v->angle_gap = (180+wheel_v->angle_gap);
		speed = -speed; 
		//wheel_v->resultant_v = -wheel_v->resultant_v;
	}

	else if(wheel_v->angle_gap<-90)
	{
		wheel_v->angle_gap = (180+wheel_v->angle_gap);
		speed = -speed;  
		//wheel_v->resultant_v = -wheel_v->resultant_v;
	}
	wheel_v->resultant_v = speed;
}
extern RC_Ctl_t RC_CtrlData2;
extern  handle shark;
void Robot_Wheel_Control(void)
{

	if(1)
	{    
//		 Robot_Coordinate_system_Vel.x=30*shark.b[4];//(RC_CtrlData2.rc.ch0-1024);
//		 Robot_Coordinate_system_Vel.y=30*shark.b[3];//(RC_CtrlData2.rc.ch1-1024);
//		 Robot_Coordinate_system_Vel.w=30*shark.b[1];//(RC_CtrlData2.rc.ch3-1024);
		//3.2自转速度分配到每个轮子
		 wheel_rotation_v[1].x =  -Robot_Coordinate_system_Vel.w * HASSIS_Struct_o1_length * cos(HASSIS_Struct_o1_angle);
		 wheel_rotation_v[1].y =  -Robot_Coordinate_system_Vel.w * HASSIS_Struct_o1_length * sin(HASSIS_Struct_o1_angle);
		 wheel_rotation_v[3].x =  Robot_Coordinate_system_Vel.w * HASSIS_Struct_o2_length * cos(HASSIS_Struct_o2_angle);
		 wheel_rotation_v[3].y =  Robot_Coordinate_system_Vel.w * HASSIS_Struct_o2_length * sin(HASSIS_Struct_o2_angle);
		 wheel_rotation_v[2].x =  Robot_Coordinate_system_Vel.w * HASSIS_Struct_o3_length * cos(HASSIS_Struct_o3_angle);
		 wheel_rotation_v[2].y =  Robot_Coordinate_system_Vel.w * HASSIS_Struct_o3_length * sin(HASSIS_Struct_o3_angle);
		
		/*4  最终合速度分配*/
		wheel_final_v[1].x  =  wheel_rotation_v[1].x+Robot_Coordinate_system_Vel.x;
		wheel_final_v[1].y  =  wheel_rotation_v[1].y+Robot_Coordinate_system_Vel.y;
		wheel_final_v[1].resultant_v  = sqrt(wheel_final_v[1].x*wheel_final_v[1].x+wheel_final_v[1].y*wheel_final_v[1].y);
		wheel_final_v[1].v_angle  =  get_angle(wheel_final_v[1].y/wheel_final_v[1].resultant_v,
										   wheel_final_v[1].x/wheel_final_v[1].resultant_v)   *180/PI;	//目标速度的角度
		
		wheel_final_v[2].x = wheel_rotation_v[2].x+Robot_Coordinate_system_Vel.x;
		wheel_final_v[2].y = wheel_rotation_v[2].y+Robot_Coordinate_system_Vel.y;  
		wheel_final_v[2].resultant_v  = sqrt(wheel_final_v[2].x*wheel_final_v[2].x+wheel_final_v[2].y*wheel_final_v[2].y);
		wheel_final_v[2].v_angle = get_angle(wheel_final_v[2].y/wheel_final_v[2].resultant_v,
										   wheel_final_v[2].x/wheel_final_v[2].resultant_v)   *180/PI;	//目标速度的角度
		
		wheel_final_v[3].x = wheel_rotation_v[3].x+Robot_Coordinate_system_Vel.x;
		wheel_final_v[3].y = wheel_rotation_v[3].y+Robot_Coordinate_system_Vel.y;  
		wheel_final_v[3].resultant_v  = sqrt(wheel_final_v[3].x*wheel_final_v[3].x+wheel_final_v[3].y*wheel_final_v[3].y) ;
		wheel_final_v[3].v_angle = get_angle(wheel_final_v[3].y/wheel_final_v[3].resultant_v,
										   wheel_final_v[3].x/wheel_final_v[3].resultant_v)   *180/PI;	//目标速度的角度
		
		//如果键B在中间，并且遥控器角度很小，让车保持45度自转姿态，待命。
//		if( (tdf_handkey.Buttom_B == buttom1)&&
//			(wheel_final_v[1].resultant_v < 2)&&(wheel_final_v[2].resultant_v < 2)
//			&&(wheel_final_v[3].resultant_v < 2)&&(wheel_final_v[4].resultant_v < 2)  )   
//		{ 
//			wheel_final_v[1].v_angle = 135;
//			wheel_final_v[2].v_angle = 45;
//			wheel_final_v[3].v_angle = 315;
//			wheel_final_v[4].v_angle = 225;
//		}									 
	    //获得轮子当前在(0,360)内的绝对角度
		gain_absolute_angle(&wheel_final_v[1],1);
		gain_absolute_angle(&wheel_final_v[2],2);
		gain_absolute_angle(&wheel_final_v[3],3);
 
		//将要转的角度wheel_final_v->angle_gap，化为 90 度内的，按不同情况合速度有的要取反方向
		gain_gap_angle(&wheel_final_v[1]);
		gain_gap_angle(&wheel_final_v[2]);   
		gain_gap_angle(&wheel_final_v[3]);

		//累加积分得 轮子增量式的角度和	
		if( ! ((ABS(wheel_final_v[1].resultant_v) < 250)&&(ABS(wheel_final_v[2].resultant_v) < 250)   //防止keep_position时频繁抖动
		     &&(ABS(wheel_final_v[3].resultant_v < 250)))) //若四轮的速度都很小，那angle_sum保持上一次的位置不动 270
		{	
			wheel_final_v[1].angle_sum += wheel_final_v[1].angle_gap; 
			wheel_final_v[2].angle_sum += wheel_final_v[2].angle_gap;
			wheel_final_v[3].angle_sum += wheel_final_v[3].angle_gap;
		}		
	}
}
void Robot_Wheel_Control_3508(int8_t number)
{  
//	if(k==0)
//	{
   if(number&0x01)M3508_Speed_Position(M3508  ,(int32_t)(-multiple_angle*wheel_final_v[1].angle_sum-52.5*144/17.0f*8192/360.0f+wheel_final_v[1].Original_angle_encode));
	 if(number&0x02)M3508_Speed_Position(M3508+1,(int32_t)(-multiple_angle*wheel_final_v[2].angle_sum-52.5*144/17.0f*8192/360.0f+wheel_final_v[2].Original_angle_encode));
	 if(number&0x04)M3508_Speed_Position(M3508+2,(int32_t)(-multiple_angle*wheel_final_v[3].angle_sum-52.5*144/17.0f*8192/360.0f+wheel_final_v[3].Original_angle_encode)); 
//	}
//	 if(ABS(M3508[0].Position_S_PID_STruct.Err) < 5000 & ABS(M3508[1].Position_S_PID_STruct.Err) < 5000 & ABS(M3508[2].Position_S_PID_STruct.Err) < 5000)
//			k = 1;
//	 if(ABS(M3508[0].Position_S_PID_STruct.Err) >= 5000 & ABS(M3508[1].Position_S_PID_STruct.Err) >= 5000 & ABS(M3508[2].Position_S_PID_STruct.Err) >= 5000)
//			k = 0;
	
	CAN2_SetMotor_0_3(M3508);
}
void Robot_Wheel_speed_Control_3508(float Speed,int8_t number)
{
  if(number&0x01)M3508_Speed(M3508  ,Speed);
  if(number&0x02)M3508_Speed(M3508+1,Speed);
	if(number&0x04)M3508_Speed(M3508+2,Speed);
}
void Robot_Wheel_speed_Control_VESC(void)
{
	static int cnt = 0;
//	if( k==1 )
//	{
	if(cnt == 0){
		VESC_Set_RPM(1,wheel_final_v[1].resultant_v,&hcan2);
	  VESC_Set_RPM(2,wheel_final_v[2].resultant_v,&hcan2);
	  VESC_Set_RPM(3,wheel_final_v[3].resultant_v,&hcan2);
		cnt++;
	}else if(cnt == 1){
		VESC_Set_RPM(2, 11500, &hcan1);
	  VESC_Set_RPM(3, 11500, &hcan1);
		cnt = 0;
	}
	  
		
//	}
//	else
//	{
//		VESC_Set_RPM(1,0,&hcan2);
//	  VESC_Set_RPM(2,0,&hcan2);
//	  VESC_Set_RPM(3,0,&hcan2);
//	}
}
