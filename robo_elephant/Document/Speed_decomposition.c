#include "Speed_decomposition.h"
#include "pid_controller.h"
#include "math.h"
M3508x_STA M3508[8];
float multiple_angle = 1802.24f;    //�Ƕ���X���
wheel_Struct  	wheel_rotation_v[5],        //������ת�ٶ�
								wheel_final_v[5];           //���������ٶ�

coordinitioate_Struct Robot_Coordinate_system;								
static float ABS(float a)
{
	return a>0?a:-a;
}								
//sin��cos��Ƕ�
float get_angle(float sin_num,float cos_num)
{//������
	if(sin_num>=0&&cos_num>=0)//��һ���޵�
		return asin(sin_num);
	else if(sin_num>=0&&cos_num<=0)//�ڶ�����
		return PI-asin(sin_num);
	else if(sin_num<=0&&cos_num<=0)//��������
		return PI-asin(sin_num);
	else if(sin_num<=0&&cos_num>=0)//��������
		return 2*PI+asin(sin_num);
	return 0;
}								
void gain_absolute_angle(wheel_Struct *wheel_v,int id)
{
	switch(id)
	{
		case 1:    wheel_v->absolute_angle = wheel_v->angle_sum; //�ǶȲ�Ļ���  +    ����ֵĳ�ʼ�Ƕȣ������0����0������45����45
					break;
		case 2:    wheel_v->absolute_angle = wheel_v->angle_sum; //�ǶȲ�Ļ���
					break;
		case 3:    wheel_v->absolute_angle = wheel_v->angle_sum; //�ǶȲ�Ļ���
					break;
		case 4:    wheel_v->absolute_angle = wheel_v->angle_sum; //�ǶȲ�Ļ���
					break;
		default:	break;
	}

	while(wheel_v->absolute_angle>=360)
	wheel_v->absolute_angle -= 360;
	while(wheel_v->absolute_angle<0)
	wheel_v->absolute_angle += 360;          //��absolute_angle��Ϊ��0,360),
}

void gain_gap_angle(wheel_Struct *wheel_v)
{
	float speed;
	speed = wheel_v->resultant_v;
	wheel_v->angle_gap= wheel_v->v_angle - wheel_v->absolute_angle; //�ֽ�����Ҫת�ĽǶȣ�Ŀ���ٶȽǶȣ���ǰ���ֽǶȣ�������������[0,360),���Ϊ�������˳ʱ��ת		   
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
void Robot_Wheel_Control(void)
{

	if(1)
	{    
		//3.2��ת�ٶȷ��䵽ÿ������
		 wheel_rotation_v[1].x = -Robot_Coordinate_system.Angular_velocity * HASSIS_Struct_o1_length * cos(HASSIS_Struct_o1_angle);
		 wheel_rotation_v[1].y =  Robot_Coordinate_system.Angular_velocity * HASSIS_Struct_o1_length * sin(HASSIS_Struct_o1_angle);
		 wheel_rotation_v[2].x =  Robot_Coordinate_system.Angular_velocity * HASSIS_Struct_o2_length * cos(HASSIS_Struct_o2_angle);
		 wheel_rotation_v[2].y =  Robot_Coordinate_system.Angular_velocity * HASSIS_Struct_o2_length * sin(HASSIS_Struct_o2_angle);
		 wheel_rotation_v[3].x =  Robot_Coordinate_system.Angular_velocity * HASSIS_Struct_o3_length * cos(HASSIS_Struct_o3_angle);
		 wheel_rotation_v[3].y = -Robot_Coordinate_system.Angular_velocity * HASSIS_Struct_o3_length * sin(HASSIS_Struct_o3_angle);
		
		/*4  ���պ��ٶȷ���*/
		wheel_final_v[1].x  =  wheel_rotation_v[1].x+Robot_Coordinate_system.x;
		wheel_final_v[1].y  =  wheel_rotation_v[1].y+Robot_Coordinate_system.y;
		wheel_final_v[1].resultant_v  = sqrt(wheel_final_v[1].x*wheel_final_v[1].x+wheel_final_v[1].y*wheel_final_v[1].y);
		wheel_final_v[1].v_angle  =  get_angle(wheel_final_v[1].y/wheel_final_v[1].resultant_v,
										   wheel_final_v[1].x/wheel_final_v[1].resultant_v)   *180/PI;	//Ŀ���ٶȵĽǶ�
		
		wheel_final_v[2].x = wheel_rotation_v[2].x+Robot_Coordinate_system.x;
		wheel_final_v[2].y = wheel_rotation_v[2].y+Robot_Coordinate_system.y;  
		wheel_final_v[2].resultant_v  = sqrt(wheel_final_v[2].x*wheel_final_v[2].x+wheel_final_v[2].y*wheel_final_v[2].y);
		wheel_final_v[2].v_angle = get_angle(wheel_final_v[2].y/wheel_final_v[2].resultant_v,
										   wheel_final_v[2].x/wheel_final_v[2].resultant_v)   *180/PI;	//Ŀ���ٶȵĽǶ�
		
		wheel_final_v[3].x = wheel_rotation_v[3].x+Robot_Coordinate_system.x;
		wheel_final_v[3].y = wheel_rotation_v[3].y+Robot_Coordinate_system.y;  
		wheel_final_v[3].resultant_v  = sqrt(wheel_final_v[3].x*wheel_final_v[3].x+wheel_final_v[3].y*wheel_final_v[3].y) ;
		wheel_final_v[3].v_angle = get_angle(wheel_final_v[3].y/wheel_final_v[3].resultant_v,
										   wheel_final_v[3].x/wheel_final_v[3].resultant_v)   *180/PI;	//Ŀ���ٶȵĽǶ�
		
		//�����B���м䣬����ң�����ǶȺ�С���ó�����45����ת��̬��������
//		if( (tdf_handkey.Buttom_B == buttom1)&&
//			(wheel_final_v[1].resultant_v < 2)&&(wheel_final_v[2].resultant_v < 2)
//			&&(wheel_final_v[3].resultant_v < 2)&&(wheel_final_v[4].resultant_v < 2)  )   
//		{ 
//			wheel_final_v[1].v_angle = 135;
//			wheel_final_v[2].v_angle = 45;
//			wheel_final_v[3].v_angle = 315;
//			wheel_final_v[4].v_angle = 225;
//		}									 
	    //������ӵ�ǰ��(0,360)�ڵľ��ԽǶ�
		gain_absolute_angle(&wheel_final_v[1],1);
		gain_absolute_angle(&wheel_final_v[2],2);
		gain_absolute_angle(&wheel_final_v[3],3);
 
		//��Ҫת�ĽǶ�wheel_final_v->angle_gap����Ϊ 90 ���ڵģ�����ͬ������ٶ��е�Ҫȡ������
		gain_gap_angle(&wheel_final_v[1]);
		gain_gap_angle(&wheel_final_v[2]);   
		gain_gap_angle(&wheel_final_v[3]);

		//�ۼӻ��ֵ� ��������ʽ�ĽǶȺ�	
		if( ! ((ABS(wheel_final_v[1].resultant_v) < 10)&&(ABS(wheel_final_v[2].resultant_v) < 10)   //��ֹkeep_positionʱƵ������
		     &&(ABS(wheel_final_v[3].resultant_v < 10)))) //�����ֵ��ٶȶ���С����angle_sum������һ�ε�λ�ò��� 
		{	
			wheel_final_v[1].angle_sum += wheel_final_v[1].angle_gap; 
			wheel_final_v[2].angle_sum += wheel_final_v[2].angle_gap;
			wheel_final_v[3].angle_sum += wheel_final_v[3].angle_gap;
		}
	}
}
void Robot_Wheel_Control_3508(uint8_t number)
{
		if(number&0x01)M3508_Speed_Position(M3508  ,(int32_t)(multiple_angle*wheel_final_v[1].angle_sum+wheel_final_v[1].Original_angle_encode));
		else if(number&0x02)M3508_Speed_Position(M3508+1,(int32_t)(multiple_angle*wheel_final_v[2].angle_sum+wheel_final_v[2].Original_angle_encode));
		else if(number&0x04)M3508_Speed_Position(M3508+2,(int32_t)(multiple_angle*wheel_final_v[3].angle_sum+wheel_final_v[3].Original_angle_encode));    
		CAN1_SetMotor_0_3(M3508);
}
void Robot_Wheel_speed_Control_3508(float Speed,uint8_t number)
{
  if(number&0x01)M3508_Speed(M3508  ,Speed);
	else if(number&0x02)M3508_Speed(M3508+1,Speed);
	else if(number&0x04)M3508_Speed(M3508+2,Speed);
}
