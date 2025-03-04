#include "pid_controller.h"
#include "can.h"
#include "JY_ME02.h"
#include "DM_J4310.h"
#include "M3508.h"

extern M3508x_STA M3508[8];
extern M350D_STA M350x[8];
int PID_Complete_Flag;
int flag;
//static float ABSF(float a)
//{
//	return a>0?a:-a;
//}
/*********************************pid优化**************************************************/
	void f_Trapezoid_Intergral(PID_Struct *pid)
{
    pid->ITerm = pid->KI * ((pid->Err + pid->Last_Err) / 2);
}

 void f_Changing_Integral_Rate(PID_Struct *pid,float ScalarA,float ScalarB)
{
    if (pid->Err * pid->Iout > 0)
    {
        //Integral still increasing
        if (ABS(pid->Err) <= ScalarB)
            return; //Full integral
        if (ABS(pid->Err) <= (ScalarA + ScalarB))
            pid->ITerm *= (ScalarA - ABS(pid->Err) + ScalarB) /ScalarA;
        else
            pid->ITerm = 0;
    }
}

 void f_Integral_Limit(PID_Struct *pid,float MaxOut,float IntegralLimit)
{
    float temp_Output, temp_Iout;
    temp_Iout = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + pid->Iout + pid->Dout;
    if (ABS(temp_Output) > MaxOut)
    {
        if (pid->Err * pid->Iout > 0)
        {
            //Integral still increasing
            pid->ITerm = 0;
        }
    }

    if (temp_Iout > IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = IntegralLimit;
    }
    if (temp_Iout < -IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -IntegralLimit;
    }
}

 void f_Derivative_On_Measurement(PID_Struct *pid)
{
    pid->Dout = pid->KD * (pid->Last_Measure - pid->Measure);
}

 void f_Derivative_Filter(PID_Struct *pid,float Derivative_Filtering_Coefficient)
{
    pid->Dout = pid->Dout * Derivative_Filtering_Coefficient +
                pid->Last_Dout*(1 - Derivative_Filtering_Coefficient);
}

 void f_Output_Filter(PID_Struct *pid,float Output_Filtering_Coefficient)
{
    pid->Output = pid->Output * Output_Filtering_Coefficient +
                  pid->Last_Output * (1 -Output_Filtering_Coefficient);
}

 void f_Output_Limit(PID_Struct *pid,float MaxOut)
{
    if (pid->Output > MaxOut)
    {
        pid->Output = MaxOut;
    }
    if (pid->Output < -(MaxOut))
    {
        pid->Output = -(MaxOut);
    }
}

 void f_Proportion_Limit(PID_Struct *pid,float MaxOut)
{
    //Proportion limit is insignificant in control process
    //but it makes variable chart look better
    if (pid->Pout > MaxOut)
    {
        pid->Pout = MaxOut;
    }
    if (pid->Pout < -(MaxOut))
    {
        pid->Pout = -(MaxOut);
    }
}
/*PID计算子函数*/
 void PID_Calculate (PID_Struct *PID_Parameter,float A,float B,
	float Maxout,float IntegralLimit ,
	float Derivative_Filtering_Coefficient,float Output_Filtering_Coefficient )
{
    PID_Parameter->Err = PID_Parameter->target - PID_Parameter->Input;
    PID_Parameter->Measure = PID_Parameter->Input;
    PID_Parameter->target =  PID_Parameter->target;
    if (ABS(PID_Parameter->Err ) > PID_Parameter->DeadBand)
    {
        PID_Parameter->Pout = PID_Parameter->KP * PID_Parameter->Err;
        PID_Parameter->ITerm = PID_Parameter->KI * PID_Parameter->Err;
        PID_Parameter->Dout = PID_Parameter->KD * (PID_Parameter->Err - PID_Parameter->Last_Err);

        //Trapezoid Intergral 梯形积分
        if (PID_Parameter->youhua[0])
            f_Trapezoid_Intergral(PID_Parameter);
        //Changing Integral Rate 变速积分
        if (PID_Parameter->youhua[1])
            f_Changing_Integral_Rate(PID_Parameter,A,B);
        //Integral limit 积分限幅，积分抗饱和
        if (PID_Parameter->youhua[2])
            f_Integral_Limit(PID_Parameter,Maxout,IntegralLimit);
        //Derivative On Measurement 微分先行，防止微分冲击
        if (PID_Parameter->youhua[3])
            f_Derivative_On_Measurement(PID_Parameter);
        //Derivative filter 
        if (PID_Parameter->youhua[4])
            f_Derivative_Filter(PID_Parameter,Derivative_Filtering_Coefficient);

        PID_Parameter->Iout += PID_Parameter->ITerm;

        PID_Parameter->Output = PID_Parameter->Pout + PID_Parameter->Iout + PID_Parameter->Dout;

        //Output Filter
        if (PID_Parameter->youhua[5])
            f_Output_Filter(PID_Parameter,Output_Filtering_Coefficient);

        //Output limit
        f_Output_Limit(PID_Parameter,Maxout);

        //Proportional limit
        f_Proportion_Limit(PID_Parameter,Maxout);
    }
    PID_Parameter->Last_Measure = PID_Parameter->Measure;
    PID_Parameter->Last_Output = PID_Parameter->Output;
    PID_Parameter->Last_Dout = PID_Parameter->Dout;
    PID_Parameter->Last_Err = PID_Parameter->Err;
    if(ABS(PID_Parameter->Err)<2000)PID_Parameter->PID_Complete_Flag=1;
		else PID_Parameter->PID_Complete_Flag=0;
}


//初始化
void M3510_PID_MotorInit(M3508x_STA *M350x)
{
	M350x[0].Position_S_PID_STruct.KP=0.2;
	M350x[0].Position_S_PID_STruct.KI=0;
	M350x[0].Position_S_PID_STruct.KD=0;	
	M350x[0].Speed_P_PID_STruct.KP=10;
	M350x[0].Speed_P_PID_STruct.KI=0.8;
	M350x[0].Speed_P_PID_STruct.KD=5;
	M350x[0].Single_Speed_PID_STruct.KP=9;
	M350x[0].Single_Speed_PID_STruct.KI=0.4;
	M350x[0].Single_Speed_PID_STruct.KD=2;	
	
	M350x[1].Position_S_PID_STruct.KP=0.2;
	M350x[1].Position_S_PID_STruct.KI=0;
	M350x[1].Position_S_PID_STruct.KD=0;
	M350x[1].Speed_P_PID_STruct.KP=10;
	M350x[1].Speed_P_PID_STruct.KI=0.8;
	M350x[1].Speed_P_PID_STruct.KD=5;	
	M350x[1].Single_Speed_PID_STruct.KP=9;
	M350x[1].Single_Speed_PID_STruct.KI=0.4;
	M350x[1].Single_Speed_PID_STruct.KD=2;	
	
	M350x[2].Position_S_PID_STruct.KP=0.2;
	M350x[2].Position_S_PID_STruct.KI=0;
	M350x[2].Position_S_PID_STruct.KD=0;
	M350x[2].Speed_P_PID_STruct.KP=10;
	M350x[2].Speed_P_PID_STruct.KI=0.8;
	M350x[2].Speed_P_PID_STruct.KD=5;		
	M350x[2].Single_Speed_PID_STruct.KP=9;
	M350x[2].Single_Speed_PID_STruct.KI=0.4;
	M350x[2].Single_Speed_PID_STruct.KD=2;	
	
	M350x[3].Position_S_PID_STruct.KP=0.2;
	M350x[3].Position_S_PID_STruct.KI=0;
	M350x[3].Position_S_PID_STruct.KD=0;
	M350x[3].Speed_P_PID_STruct.KP=10;
	M350x[3].Speed_P_PID_STruct.KI=0.8;
	M350x[3].Speed_P_PID_STruct.KD=5;
	

	
	M350x[0].Angle_Old=M350x[0].Angle;
	M350x[0].dAngle_Sum=0;
	M350x[1].Angle=M350x[1].Angle_Old;
	M350x[1].dAngle_Sum=0;
	M350x[2].Angle=M350x[2].Angle_Old;
	M350x[2].dAngle_Sum=0;
	M350x[3].Angle=M350x[3].Angle_Old;
	M350x[3].dAngle_Sum=0;
	M350x[4].Angle=M350x[4].Angle_Old;
	M350x[4].dAngle_Sum=0;			
	M350x[5].Angle=M350x[5].Angle_Old;
	M350x[5].dAngle_Sum=0;
	M350x[6].Angle=M350x[6].Angle_Old;
	M350x[6].dAngle_Sum=0;
	M350x[7].Angle=M350x[7].Angle_Old;
	M350x[7].dAngle_Sum=0;	
}

void M3508_Speed_Position(M3508x_STA *M350x,int32_t Angle)
{
	M350x->Position_S_PID_STruct.target=Angle;
	M350x->Position_S_PID_STruct.Input=M350x->dAngle_Sum;
	PID_Calculate(&(M350x->Position_S_PID_STruct),50,50,
	32767,32767,
	0,0);
	
	M350x->Speed_P_PID_STruct.target=M350x->Position_S_PID_STruct.Output;
	M350x->Speed_P_PID_STruct.Input=M350x->Speed;
	PID_Calculate(&(M350x->Speed_P_PID_STruct),50,50,
	32767,32767,
	0,0);
	
	M350x->Out=(int16_t)M350x->Speed_P_PID_STruct.Output;
}

void M3508_Speed(M3508x_STA *M350x,int32_t speed)
{
  M350x->Single_Speed_PID_STruct.target=speed;
	M350x->Single_Speed_PID_STruct.Input=M350x->Speed;
	PID_Calculate(&(M350x->Single_Speed_PID_STruct),50,50,
	32767,32767,
	0,0);
	
	M350x->Out=(int16_t)M350x->Single_Speed_PID_STruct.Output;
}
void M3510_Angle_Calculate(M3508x_STA *M350x)
{
	int dAngle1=0,dAngle2=0,delta;

	/*计算累积旋转角度*/	
	
	if(M350x->Angle>M350x->Angle_Old)
	{
		dAngle1=M350x->Angle-M350x->Angle_Old;//未超过一圈
		dAngle2=M350x->Angle-8192-M350x->Angle_Old;//超过一圈
	}
	else 
	{
		dAngle1=M350x->Angle-M350x->Angle_Old;
		dAngle2=M350x->Angle+8192-M350x->Angle_Old;
	}
	if(ABS(dAngle1)<ABS(dAngle2))
	{
		delta=dAngle1;
	}
	else
	{
		delta=dAngle2;
	}
  M350x->Angle_Old=M350x->Angle;
	M350x->dAngle_Sum+=delta;
}
void Can_3508_Receive(CAN_RxHeaderTypeDef Can_Rx,uint8_t*Data,M3508x_STA *M350x)
{
			int id_num;
			if((Can_Rx.StdId&0xff0)==0x200&&Can_Rx.IDE==CAN_ID_STD)
			{
				id_num=Can_Rx.StdId&0x00f;
				M350x[id_num-1].Angle&=0x0000;
				M350x[id_num-1].Angle|=(Data[0]<<8)|Data[1];
				M350x[id_num-1].Speed&=0x0000;
				M350x[id_num-1].Speed|=(Data[2]<<8)|Data[3];
				M350x[id_num-1].Currency&=0x0000;
				M350x[id_num-1].Currency|=(Data[4]<<8)|Data[5];
				M350x[id_num-1].Temperature=Data[6];
			}	
			#if 1
			M3510_Angle_Calculate(M350x);
			#endif
			#if 1
			M3510_Angle_Calculate(M350x+1);
			#endif
			#if 1
			M3510_Angle_Calculate(M350x+2);
			#endif
			#if 1
			M3510_Angle_Calculate(M350x+3);
			#endif
			#if 0
			M3510_Angle_Calculate(M350x+4);
			#endif
			#if 0
			M3510_Angle_Calculate(M350x+5);
			#endif
			#if 0
			M3510_Angle_Calculate(M350x+6);
			#endif
			#if 0
			M3510_Angle_Calculate(M350x+7);
			#endif
		}

 //接收函数并解码
void CAN2_SetMotor_0_3(M3508x_STA *M350x)
{
  CAN_TxHeaderTypeDef  Can_Tx ;
	uint32_t  mbox;
	uint16_t k;
	uint8_t Data[8];

	Can_Tx.StdId=0X200;
  Can_Tx.IDE  =CAN_ID_STD ;
  Can_Tx.RTR  =CAN_RTR_DATA; 
	Can_Tx.TransmitGlobalTime=DISABLE;
  Can_Tx.DLC  =8;  
	#if 1	
  Data[0]=(M350x[0].Out>>8)&0xff;             
	Data[1]=(M350x[0].Out)&0xff;
	#endif
	#if 1
  Data[2]=(M350x[1].Out>>8)&0xff;             
	Data[3]=(M350x[1].Out)&0xff;
	#endif
	#if 1
  Data[4]=(M350x[2].Out>>8)&0xff;             
	Data[5]=(M350x[2].Out)&0xff;
	#endif
	#if 0
  Data[6]=(M350x[3].Out>>8)&0xff;             
	Data[7]=(M350x[3].Out)&0xff;
	#endif
//	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
  while(HAL_CAN_AddTxMessage(&hcan2,&Can_Tx,Data,&mbox)!=HAL_OK&&(k<0XFFF)){k++;}
}

void CAN2_SetMotor_4_7(M3508x_STA *M350x)
{
  CAN_TxHeaderTypeDef  Can_Tx ;
	uint32_t  mbox;
	uint16_t k;
	uint8_t Data[8];

	Can_Tx.StdId=0X1FF;
  Can_Tx.IDE  =CAN_ID_STD ;
  Can_Tx.RTR  =CAN_RTR_DATA;  
	Can_Tx.TransmitGlobalTime=DISABLE;	
  Can_Tx.DLC  =8;  
	#if 1	
  Data[0]=(M350x[4].Out>>8)&0xff;             
  Data[1]=(M350x[4].Out)&0xff;
	#endif
	#if 1
  Data[2]=(M350x[5].Out>>8)&0xff;             
	Data[3]=(M350x[5].Out)&0xff;
	#endif
	#if 1
  Data[4]=(M350x[6].Out>>8)&0xff;             
	Data[5]=(M350x[6].Out)&0xff;
	#endif
	#if 1
  Data[6]=(M350x[7].Out>>8)&0xff;             
	Data[7]=(M350x[7].Out)&0xff;
	#endif
	//while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
  while(HAL_CAN_AddTxMessage(&hcan2,&Can_Tx,Data,&mbox)!=HAL_OK&&(k<0XFFF)){k++;}
}

//CAN接收中断

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)  //接收回调函数
{
	CAN_RxHeaderTypeDef Can_Rx;
  uint8_t  Data[8];
		int id;
  HAL_StatusTypeDef	HAL_RetVal;
   if(hcan ==&hcan2){
		 HAL_RetVal=HAL_CAN_GetRxMessage(hcan,  CAN_RX_FIFO0, &Can_Rx,  Data);
    if ( HAL_OK==HAL_RetVal){                              			
      //在这里接收数据
		Can_3508_Receive(Can_Rx,Data,M3508);	
    }
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	}
	 	//CAN1接收中断
	if (hcan->Instance == hcan1.Instance)
	{
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Can_Rx, Data) == HAL_OK)		// 获得接收到的数据头和数据
		{
if((Can_Rx.StdId & 0xff00) == 0x0000){  //RxHeader.StdId == 0x 05x
	    JY_Encoder_Can_Receive(&Can_Rx, Data, JY_Encoder);
	  }else if((Can_Rx.StdId & 0x0FF0) == 0x200){  //RxHeader.StdId == 0x 20x
		  M350x_RecData(&Can_Rx, Data, M350x);
	  }
		else{
      switch(Can_Rx.StdId){  //DM接收
		  case 0: 
			// feedback of motor,Modify this value if you setting a different Master ID
				mtr.id = (Data[0])&0x0F;
		    id = mtr.id;
		    if(id == DM_CANID_1){
					mtr.state = (Data[0])>>4;
			    mtr.p_int=(Data[1]<<8)|Data[2];
				  mtr.v_int=(Data[3]<<4)|(Data[4]>>4);
				  mtr.t_int=((Data[4]&0xF)<<8)|Data[5];
				  mtr.pos = uint_to_float(mtr.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
				  mtr.vel = uint_to_float(mtr.v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
				  mtr.toq = uint_to_float(mtr.t_int, T_MIN, T_MAX, 12);  // (-18.0,18.0)
				  mtr.Tmos = (float)(Data[6]);
				  mtr.Tcoil = (float)(Data[7]);
				}
			 break;
		  case 1:
			// control command,for mit monitor bus,Modify this value if you setting a different ESC ID
			  cmd.p_int = (Data[0]<<8)|Data[1];
        cmd.v_int = (Data[2]<<4)|(Data[3]>>4);
        cmd.kp_int = ((Data[3]&0xF)<<8)|Data[4];
        cmd.kd_int = (Data[5]<<4)|(Data[6]>>4);
        cmd.t_int = ((Data[6]&0xF)<<8)|Data[7];
				cmd.pos = uint_to_float(cmd.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
				cmd.vel = uint_to_float(cmd.v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
        cmd.Kp = uint_to_float(cmd.kp_int, KP_MIN, KP_MAX, 12); // (0,500.0)
        cmd.Kd = uint_to_float(cmd.kd_int, KD_MIN, KD_MAX, 12); // (0,5.0)
				cmd.toq = uint_to_float(cmd.t_int, T_MIN, T_MAX, 12);  // (-18.0,18.0)
					break;
		  case 0x101:
			// control command monitoring,for PV mode,Modify this value if you setting a different ESC ID
				memcpy(&cmd.pos,&Data[0],4);
				memcpy(&cmd.vel,&Data[4],4);
					break;
		//case 0x201:
			// control command monitoring,for V mode,Modify this value if you setting a different ESC ID
				//memcpy(&cmd.vel,&Data[0],4);
				//	break;
		default: break;
	    }
		}
	}
 }
}

 //接收函数并解码
//void CAN2_SetMotor_0_3(M3508x_STA *M350x)
//{
//  CAN_TxHeaderTypeDef  Can_Tx ;
//	uint32_t  mbox;
//	uint16_t k;
//	uint8_t Data[8];

//	Can_Tx.StdId=0X200;
//  Can_Tx.IDE  =CAN_ID_STD ;
//  Can_Tx.RTR  =CAN_RTR_DATA; 
//	Can_Tx.TransmitGlobalTime=DISABLE;
//  Can_Tx.DLC  =8;  
//	#if 1	
//  Data[0]=(M350x[0].Out>>8)&0xff;             
//	Data[1]=(M350x[0].Out)&0xff;
//	#endif
//	#if 1
//  Data[2]=(M350x[1].Out>>8)&0xff;             
//	Data[3]=(M350x[1].Out)&0xff;
//	#endif
//	#if 1
//  Data[4]=(M350x[2].Out>>8)&0xff;             
//	Data[5]=(M350x[2].Out)&0xff;
//	#endif
//	#if 1
//  Data[6]=(M350x[3].Out>>8)&0xff;             
//	Data[7]=(M350x[3].Out)&0xff;
//	#endif
////	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
//  while(HAL_CAN_AddTxMessage(&hcan2,&Can_Tx,Data,&mbox)!=HAL_OK&&(k<0XFFF)){k++;}
//}

//void CAN2_SetMotor_4_7(M3508x_STA *M350x)
//{
//  CAN_TxHeaderTypeDef  Can_Tx ;
//	uint32_t  mbox;
//	uint16_t k;
//	uint8_t Data[8];

//	Can_Tx.StdId=0X1FF;
//  Can_Tx.IDE  =CAN_ID_STD ;
//  Can_Tx.RTR  =CAN_RTR_DATA;  
//	Can_Tx.TransmitGlobalTime=DISABLE;	
//  Can_Tx.DLC  =8;  
//	#if 1	
//  Data[0]=(M350x[4].Out>>8)&0xff;             
//  Data[1]=(M350x[4].Out)&0xff;
//	#endif
//	#if 1
//  Data[2]=(M350x[5].Out>>8)&0xff;             
//	Data[3]=(M350x[5].Out)&0xff;
//	#endif
//	#if 1
//  Data[4]=(M350x[6].Out>>8)&0xff;             
//	Data[5]=(M350x[6].Out)&0xff;
//	#endif
//	#if 1
//  Data[6]=(M350x[7].Out>>8)&0xff;             
//	Data[7]=(M350x[7].Out)&0xff;
//	#endif
//	//while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
//  while(HAL_CAN_AddTxMessage(&hcan2,&Can_Tx,Data,&mbox)!=HAL_OK&&(k<0XFFF)){k++;}
//}


