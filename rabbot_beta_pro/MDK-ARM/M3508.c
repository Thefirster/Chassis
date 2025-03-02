#include "M3508.h"
#include "can.h"
#include "math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
M3508x_STA M3508[8];
M3508x_STA M3508_2[8];
float watch[10];

int PID_Complete_Flag;
int flag;
static float ABS(float a)
{
    return a>0?a:-a;
}
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

/**
  ************************************************************************** 
  ** -------------------------------------------------------------------- **
  ** @name          : PID_Calculate
  ** @brief         : PID计算子函数
  ** @param         : None
  ** @retval        : None
  ** @author        : XRaccoon
  ** -------------------------------------------------------------------- **
  ** @attention     : 注意Err中Input的输入是当前的速度
  ** -------------------------------------------------------------------- **
  ************************************************************************** 
**/
void PID_Calculate(PID_Struct *PID_Parameter,float measure,float target)
{
		if(measure!=0||target!=0){
			PID_Parameter->target=target;
			PID_Parameter->Input=measure;
		}
    PID_Parameter->Err = PID_Parameter->target - PID_Parameter->Input;
    PID_Parameter->Measure = PID_Parameter->Input;
    PID_Parameter->target =  PID_Parameter->target;
    PID_Parameter->Pout = PID_Parameter->KP * PID_Parameter->Err;
		
		PID_Parameter->ITerm +=  PID_Parameter->Err;
		if(fabs(PID_Parameter->Err)<PID_Parameter->DeadBand&&(PID_Parameter->DeadBand!=0))PID_Parameter->ITerm=0;
    //if((fabs(PID_Parameter->Err)>0.5)&&(fabs(PID_Parameter->ITerm)<50000))PID_Parameter->ITerm +=  PID_Parameter->Err;
		if(PID_Parameter->ITerm>300000)PID_Parameter->ITerm=300000;
		else if(PID_Parameter->ITerm<-300000)PID_Parameter->ITerm=-300000;
		
    PID_Parameter->Dout = PID_Parameter->KD * (PID_Parameter->Err - PID_Parameter->Last_Err);
    PID_Parameter->Iout = PID_Parameter->KI *PID_Parameter->ITerm;

    PID_Parameter->Output = PID_Parameter->Pout + PID_Parameter->Iout + PID_Parameter->Dout+(PID_Parameter->Err > 0 ? PID_Parameter->Forward : -PID_Parameter->Forward);//QIANKUI
    PID_Parameter->Last_Measure = PID_Parameter->Measure;
    PID_Parameter->Last_Output = PID_Parameter->Output;
    PID_Parameter->Last_Dout = PID_Parameter->Dout;
    PID_Parameter->Last_Err = PID_Parameter->Err;


}

void PID_Calculate_yuntai(PID_Struct *PID_Parameter)
{
    PID_Parameter->Err = PID_Parameter->target - PID_Parameter->Input;
    PID_Parameter->Measure = PID_Parameter->Input;
    PID_Parameter->target = PID_Parameter->target;
    PID_Parameter->Pout = PID_Parameter->KP * PID_Parameter->Err;

    if (fabs(PID_Parameter->Err) > 100)
        PID_Parameter->ITerm += PID_Parameter->Err;
    else if(fabs(PID_Parameter->Err) < 100)
        PID_Parameter->ITerm = 0;
		if(PID_Parameter->Err>10000)PID_Parameter->Err=10000;
		else if(PID_Parameter->Err<-10000)PID_Parameter->Err=-10000;

    PID_Parameter->Dout = PID_Parameter->KD * (PID_Parameter->Err - PID_Parameter->Last_Err);
    PID_Parameter->Iout = PID_Parameter->KI * PID_Parameter->ITerm;

    PID_Parameter->Output = PID_Parameter->Pout + PID_Parameter->Iout + PID_Parameter->Dout;
    PID_Parameter->Last_Measure = PID_Parameter->Measure;
    PID_Parameter->Last_Output = PID_Parameter->Output;
    PID_Parameter->Last_Dout = PID_Parameter->Dout;
    PID_Parameter->Last_Err = PID_Parameter->Err;
}

void position_set(M3508x_STA *M350x,float position,float step)
{
		float target_boost,pos_target;
		target_boost=(position-M350x->dAngle_Sum)/step;
		pos_target=M350x->dAngle_Sum;
		//watch[0]=pos_target;
		//watch[1]=target_boost;
		//watch[2]=step;
		for(int i=0;i<step;i++){
		pos_target += target_boost;
			M350x->Position_S_PID_STruct.pos_target=pos_target;
			//watch[3]=i;
			osDelay(2);
		}
}

/**
  **************************************************************************
  ** -------------------------------------------------------------------- **
  ** @name          : M3510_PID_MotorInit
  ** @brief         : PID初始化
  ** @param         : None
  ** @retval        : None
  ** @author        : XRaccoon
  ** -------------------------------------------------------------------- **
  ** @attention     : None
  ** -------------------------------------------------------------------- **
  **************************************************************************
**/
void M3510_PID_MotorInit(M3508x_STA *M350x)
{
    M350x[0].Position_S_PID_STruct.KP=1;
    M350x[0].Position_S_PID_STruct.KI=0;
    M350x[0].Position_S_PID_STruct.KD=0;
	M350x[0].Position_S_PID_STruct.DeadBand=1;
    M350x[0].Speed_P_PID_STruct.KP=10;
    M350x[0].Speed_P_PID_STruct.KI=0.8;
    M350x[0].Speed_P_PID_STruct.KD=5;
    M350x[0].Single_Speed_PID_STruct.KP=9;
    M350x[0].Single_Speed_PID_STruct.KI=0.4;
    M350x[0].Single_Speed_PID_STruct.KD=2;

    M350x[1].Position_S_PID_STruct.KP=6;
    M350x[1].Position_S_PID_STruct.KI=0.01;
    M350x[1].Position_S_PID_STruct.KD=13;
	M350x[1].Position_S_PID_STruct.DeadBand=1;
    M350x[1].Speed_P_PID_STruct.KP=10;
    M350x[1].Speed_P_PID_STruct.KI=0.8;
    M350x[1].Speed_P_PID_STruct.KD=5;
    M350x[1].Single_Speed_PID_STruct.KP=9;
    M350x[1].Single_Speed_PID_STruct.KI=0.4;
    M350x[1].Single_Speed_PID_STruct.KD=2;

    M350x[2].Position_S_PID_STruct.KP=6;
    M350x[2].Position_S_PID_STruct.KI=0.01;
    M350x[2].Position_S_PID_STruct.KD=13;
    M350x[2].Speed_P_PID_STruct.KP=10;
	M350x[2].Position_S_PID_STruct.DeadBand=1;
    M350x[2].Speed_P_PID_STruct.KI=0.8;
    M350x[2].Speed_P_PID_STruct.KD=5;
    M350x[2].Single_Speed_PID_STruct.KP=9;
    M350x[2].Single_Speed_PID_STruct.KI=0.4;
    M350x[2].Single_Speed_PID_STruct.KD=2;

    M350x[3].Position_S_PID_STruct.KP=6;
    M350x[3].Position_S_PID_STruct.KI=0.01;
    M350x[3].Position_S_PID_STruct.KD=13;
	M350x[3].Position_S_PID_STruct.DeadBand=1;
    M350x[3].Speed_P_PID_STruct.KP=10;
    M350x[3].Speed_P_PID_STruct.KI=0.8;
    M350x[3].Speed_P_PID_STruct.KD=5;

    M350x[4].Position_S_PID_STruct.KP=10;
    M350x[4].Position_S_PID_STruct.KI=0;
    M350x[4].Position_S_PID_STruct.KD=5;
    M350x[4].Speed_P_PID_STruct.KP=10;
    M350x[4].Speed_P_PID_STruct.KI=0.8;
    M350x[4].Speed_P_PID_STruct.KD=5;

    M350x[5].Position_S_PID_STruct.KP=0.3;
    M350x[5].Position_S_PID_STruct.KI=0;
    M350x[5].Position_S_PID_STruct.KD=0;
    M350x[5].Speed_P_PID_STruct.KP=10;
    M350x[5].Speed_P_PID_STruct.KI=0.8;
    M350x[5].Speed_P_PID_STruct.KD=5;


    M350x[6].Position_S_PID_STruct.KP=10;
    M350x[6].Position_S_PID_STruct.KI=0;
    M350x[6].Position_S_PID_STruct.KD=5;
    M350x[6].Speed_P_PID_STruct.KP=10;
    M350x[6].Speed_P_PID_STruct.KI=0.8;
    M350x[6].Speed_P_PID_STruct.KD=5;

    M350x[7].Position_S_PID_STruct.KP=10;
    M350x[7].Position_S_PID_STruct.KI=0;
    M350x[7].Position_S_PID_STruct.KD=5;
    M350x[7].Speed_P_PID_STruct.KP=10;
    M350x[7].Speed_P_PID_STruct.KI=0.8;
    M350x[7].Speed_P_PID_STruct.KD=5;


    M350x[0].Angle_Old=M350x[0].Angle;
    M350x[0].dAngle_Sum=0;
    M350x[1].Angle_Old=M350x[1].Angle;
    M350x[1].dAngle_Sum=0;
    M350x[2].Angle_Old=M350x[2].Angle;
    M350x[2].dAngle_Sum=0;
    M350x[3].Angle_Old=M350x[3].Angle;
    M350x[3].dAngle_Sum=0;
    M350x[4].Angle_Old=M350x[4].Angle;
    M350x[4].dAngle_Sum=0;
    M350x[5].Angle_Old=M350x[5].Angle;
    M350x[5].dAngle_Sum=0;
    M350x[5].Angle_Old=M350x[5].Angle;
    M350x[6].dAngle_Sum=0;
    M350x[7].Angle_Old=M350x[7].Angle;
    M350x[7].dAngle_Sum=0;
}

void m3508_motion_control(M3508x_STA *M350x)
{
					if(M350x->Position_S_PID_STruct.pos_target!=0){
						M350x->Position_S_PID_STruct.target=M350x->Position_S_PID_STruct.pos_target;
						M350x->Position_S_PID_STruct.Input=M350x->dAngle_Sum;
						PID_Calculate_yuntai(&(M350x->Position_S_PID_STruct));
						M350x->Speed_P_PID_STruct.target=M350x->Position_S_PID_STruct.Output;
						M350x->Speed_P_PID_STruct.Input= M350x->Speed;
				}
					M350x->Speed_P_PID_STruct.Input=M350x->Speed;
					PID_Calculate_yuntai(&M350x->Speed_P_PID_STruct);
							if (M350x->Speed_P_PID_STruct.Output > 20000)
            M350x->Out = 20000;
        else if (M350x->Speed_P_PID_STruct.Output < -20000)
            M350x->Out = -20000;
        else
            M350x->Out = (int16_t)M350x->Speed_P_PID_STruct.Output;

}

//void M3508_Speed(M3508x_STA *M350x,int32_t speed)
//{
//  M350x->Single_Speed_PID_STruct.target=speed;
//	M350x->Single_Speed_PID_STruct.Input=M350x->Speed;
//	PID_Calculate(&(M350x->Single_Speed_PID_STruct),50,50,
//	32767,32767,
//	0,0);
//
//	M350x->Out=(int16_t)M350x->Single_Speed_PID_STruct.Output;
//}
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
#if 1
    M3510_Angle_Calculate(M350x+4);
#endif
#if 1
    M3510_Angle_Calculate(M350x+5);
#endif
#if 1
    M3510_Angle_Calculate(M350x+6);
#endif
#if 1
    M3510_Angle_Calculate(M350x+7);
#endif
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)  //接收回调函数
{
		uint32_t status_value;
	status_value = taskENTER_CRITICAL_FROM_ISR(); 
    CAN_RxHeaderTypeDef Can_Rx;
    uint8_t  Data[8];
    HAL_StatusTypeDef	HAL_RetVal;
	
    if(hcan ==&hcan1) {
        HAL_RetVal=HAL_CAN_GetRxMessage(hcan,  CAN_RX_FIFO0, &Can_Rx,  Data);
        if ( HAL_OK==HAL_RetVal) {
            //在这里接收数据
            Can_3508_Receive(Can_Rx,Data,M3508);
        }
        HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
    //CAN2接收中断
    if (hcan ==&hcan2)
    {
        HAL_RetVal=HAL_CAN_GetRxMessage(hcan,  CAN_RX_FIFO0, &Can_Rx,  Data);
        if ( HAL_OK==HAL_RetVal) {
            //在这里接收数据
            Can_3508_Receive(Can_Rx,Data,M3508_2);
						
        }
        HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
		taskEXIT_CRITICAL_FROM_ISR(status_value);
}

/**
  ************************************************************************** 
  ** -------------------------------------------------------------------- **
  ** @name          : CAN1_SetMotor_0_3
  ** @brief         : 接收函数并解码
  ** @param         : None
  ** @retval        : None
  ** @author        : XRaccoon
  ** -------------------------------------------------------------------- **
  ** @attention     : None
  ** -------------------------------------------------------------------- **
  ************************************************************************** 
**/
void CAN1_SetMotor_0_3(M3508x_STA *M350x)
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
#if 1
    Data[6]=(M350x[3].Out>>8)&0xff;
    Data[7]=(M350x[3].Out)&0xff;
#endif
//	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
    while(HAL_CAN_AddTxMessage(&hcan1,&Can_Tx,Data,&mbox)!=HAL_OK&&(k<0XFFF)) {
        k++;
    }
}
/**
  ************************************************************************** 
  ** -------------------------------------------------------------------- **
  ** @name          : CAN1_SetMotor_4_7
  ** @brief         : 接收函数并解码
  ** @param         : None
  ** @retval        : None
  ** @author        : XRaccoon
  ** -------------------------------------------------------------------- **
  ** @attention     : None
  ** -------------------------------------------------------------------- **
  ************************************************************************** 
**/
void CAN1_SetMotor_4_7(M3508x_STA *M350x)
{
    CAN_TxHeaderTypeDef TxMessage;
    uint16_t K;
    TxMessage.StdId=0x1ff;
    TxMessage.IDE=CAN_ID_STD;
    TxMessage.RTR=CAN_RTR_DATA;
    TxMessage.DLC=0x08;
    uint8_t mbox[8];
    mbox[0] = (M350x[4].Out >> 8)&0xff;
    mbox[1] = M350x[4].Out&0xff;
    mbox[2] = (M350x[5].Out >> 8)&0xff;
    mbox[3] = M350x[5].Out&0xff;
    mbox[4] = (M350x[6].Out >> 8)&0xff;
    mbox[5] = M350x[6].Out&0xff;
    mbox[6] = (M350x[7].Out >> 8)&0xff;
    mbox[7] = M350x[7].Out&0xff;
    while(HAL_CAN_AddTxMessage(&hcan1,&TxMessage,mbox,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK&&(K<0XFFF)) {
        K++;
    }
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
#if 1
    Data[6]=(M350x[3].Out>>8)&0xff;
    Data[7]=(M350x[3].Out)&0xff;
#endif
    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
    while(HAL_CAN_AddTxMessage(&hcan2,&Can_Tx,Data,&mbox)!=HAL_OK&&(k<0XFFF)) {
        k++;
    }
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
    while(HAL_CAN_AddTxMessage(&hcan2,&Can_Tx,Data,&mbox)!=HAL_OK&&(k<0XFFF)) {
        k++;
    }
}



