#include "M3508.h"

M350D_STA M350x[8];

/**
 * @description: CAN���պ�����������ܵ�CAN����
 * @param {CAN_RxHeaderTypeDef *pHeader} RxHeader
 * @param {uint8_t *} RxCanData CAN�����������
 * @return none
 */
void M350x_RecData(CAN_RxHeaderTypeDef *pHeader, uint8_t *RxCanData, M350D_STA *M350x)
{
	uint8_t num = pHeader->StdId & 0x000F;
	uint16_t add1 = 0, add2 = 0;       //add1��ʾ��ת���,add2��ʾ��ת������������ȡС��һ��
	
	M350x[num-1].angleNow = (RxCanData[0]<<8) | RxCanData[1];
	M350x[num-1].speed&=0x0000;
	M350x[num-1].speed |= (RxCanData[2]<<8) | RxCanData[3];
		
	if(M350x[num-1].angleNow > M350x[num-1].angleLast){ 
		add1 = M350x[num-1].angleNow - M350x[num-1].angleLast;
		add2 = 8191 - M350x[num-1].angleNow + M350x[num-1].angleLast;
		if(add1 > add2){
			M350x[num-1].angleSum += add2;
		}else{
			M350x[num-1].angleSum += add1;}
	}else{
		add1 = M350x[num-1].angleLast - M350x[num-1].angleNow;
    add2 = 8191 - M350x[num-1].angleLast + M350x[num-1].angleNow;
		if(add1 > add2){
			M350x[num-1].angleSum -= add2;
		}else{
			M350x[num-1].angleSum -= add1;}
	}
	M350x[num-1].dAngle = M350x[num-1].angleSum - M350x[num-1].angleSumLast;
	M350x[num-1].angleLast = M350x[num-1].angleNow; 
	M350x[num-1].angleSumLast = M350x[num-1].angleSum;
}
/**
 * @description: CAN���ͺ��������͵���ֵ,eid0-3
 * @param {CAN_HandleTypeDef *} hcan
 * @param {M350D_STA *M3508} �ṹ������ 
 * @return none
 */
void M350x_TxMes_0_3(CAN_HandleTypeDef *hcanx, M350D_STA *M350x)
{
	CAN_TxHeaderTypeDef TxMessage;
  uint8_t mbox[8];
  uint32_t CAN_TX_MAILBOX;
	uint16_t k = 0;
	
	TxMessage.StdId=0x200;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.RTR=CAN_RTR_DATA; 
	TxMessage.DLC=0x08;
	
	mbox[0] = (M350x[0].electric >> 8)&0xff;
	mbox[1] =  M350x[0].electric&0xff;
	mbox[2] = (M350x[1].electric >> 8)&0xff;
	mbox[3] =  M350x[1].electric&0xff;
	mbox[4] = (M350x[2].electric >> 8)&0xff;
	mbox[5] =  M350x[2].electric&0xff;
	mbox[6] = (M350x[3].electric >> 8)&0xff;
	mbox[7] =  M350x[3].electric&0xff;
	
	while(HAL_CAN_AddTxMessage(hcanx, &TxMessage, mbox, &CAN_TX_MAILBOX) != HAL_OK && (k < 0xFFF))
		k++;
}
/**
 * @description: CAN���ͺ��������͵���ֵ,eid4-7
 * @param {CAN_HandleTypeDef *} hcan
 * @param {M350D_STA *M3508} �ṹ������ 
 * @return none
 */
void M350x_TxMes_4_7(CAN_HandleTypeDef *hcanx, M350D_STA *M350x)
{
	CAN_TxHeaderTypeDef TxMessage;
  uint8_t mbox[8];
  uint32_t CAN_TX_MAILBOX;
	uint16_t k = 0;
	
	TxMessage.StdId=0x1FF;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.RTR=CAN_RTR_DATA; 
	TxMessage.DLC=0x08;
	
	mbox[0] = (M350x[4].electric >> 8)&0xff;
	mbox[1] =  M350x[4].electric&0xff;
	mbox[2] = (M350x[5].electric >> 8)&0xff;
	mbox[3] =  M350x[5].electric&0xff;
	mbox[4] = (M350x[6].electric >> 8)&0xff;
	mbox[5] =  M350x[6].electric&0xff;
	mbox[6] = (M350x[7].electric >> 8)&0xff;
	mbox[7] =  M350x[7].electric&0xff;
	
  while(HAL_CAN_AddTxMessage(hcanx, &TxMessage, mbox, &CAN_TX_MAILBOX) != HAL_OK && (k < 0xFFF))
		k++;
}
/**
* @description: ����pid��������
* @param {M350D_STA *M3508} ��Ҫ����Ľṹ������
* @return none
*/
void pid_clear(M350D_STA *M350x)
{
	M350x->M_Speed_InPID.Kp = 0;
	M350x->M_Speed_InPID.Ki = 0;
	M350x->M_Speed_InPID.Kd = 0;
	M350x->M_Speed_PoPID.Kp = 0;
	M350x->M_Speed_PoPID.Ki = 0;
	M350x->M_Speed_PoPID.Kd = 0;
}
/**
* @description: ��������һ������ֵ������
* @param {float Pk} λ��ʽPid������ֵ
* @param {float Ik} ����ʽPid������ֵ
* @return none
*/
void pid_Init(M350D_STA *M350x, float Pkp, float Pki, float Pkd, float Ikp, float Iki, float Ikd)
{
	M350x->M_Speed_PoPID.Kp = Pkp; 
	M350x->M_Speed_PoPID.Ki = Pki; 
	M350x->M_Speed_PoPID.Kd = Pkd;
	M350x->M_Speed_InPID.Kp = Ikp;
	M350x->M_Speed_InPID.Ki = Iki; 
	M350x->M_Speed_InPID.Kd = Ikd;
}
/**
* @description: ���е��pid��ʼ��
* @param  none
* @return none
*/
void pid_Init_All(void)
{
//	pid_Init(&M3508[level_eid], 30, 0, 0, 15, 4, 7);
	pid_Init(&M350x[pitch_eid], 0, 0, 0, 0, 0, 0);
//	pid_Init(&M350x[level_eid], 0.5, 0, 0.01, 0, 0, 0);
	pid_Init(&M350x[level_eid], 0.5, 0, 0.01, 0, 0, 0);
	pid_Init(&M350x[m2006_eid], 0, 0, 0, 0, 0, 0);
//	pid_Init(&M3508[pitch_eid], 0.7, 0, 0, 12, 0.5, 2);
}
/**
* @description: ��������һ������ֵ������
* @param {float NowSpeed} ���Ƶ���
* @param {float MaxSpeed} �������ֵ
* @return {float NowSpeed} ���������
*/
float Scope_Max_Min(float NowSpeed, float MaxSpeed)
{
	if(NowSpeed > MaxSpeed)
		NowSpeed = MaxSpeed;
	if(NowSpeed < -MaxSpeed)
		NowSpeed = -MaxSpeed;
	return NowSpeed;
}
/**
* @description: ����ʽpid�����ٶȣ�������ת�ٶ�
* @param {M350D_STA* M3508} ���ṹ������
* @param {float Speed} Ŀ���ٶ�
* @return none
*/
void M_Calculate_IncrementalPID_Test(M350D_STA* M350x, float Speed)
{
	PID_Calculate_Incremental(&(M350x->M_Speed_InPID), M350x->speed, Speed);
	M350x->M_Speed_InPID.Output = Scope_Max_Min(M350x->M_Speed_InPID.Output, incremenOutputMax);
	M350x->incremenOutput = M350x->M_Speed_InPID.Output;
	M350x->electric = M350x->incremenOutput;
}
/**
* @description: λ��ʽpid�����ٶȣ���ת��Ŀ��λ��
* @param {M350D_STA* M3508} ���ṹ������
* @param {float Angle} Ŀ��λ��
* @return none
*/
void M_Calculate_PositionalPID_Test(M350D_STA *M350x, float Angle)
{
	PID_Calculate_Positional(&(M350x->M_Speed_PoPID), M350x->angleSum, Angle);
	M350x->M_Speed_PoPID.Output = Scope_Max_Min(M350x->M_Speed_PoPID.Output, poitionOutputMax);
	M350x->poitionOutput = M350x->M_Speed_PoPID.Output;
	M350x->electric = M350x->poitionOutput;
}
/**
* @description: ����pid���⻷Ϊλ�û�������λ��
* @param {M350D_STA* M3508} ���ṹ������
* @param {float Angle} Ŀ��λ��
* @param {float SpeedMax} ����Ŀ��λ��;������ٶ�
* @return none
*/
void M_Calculate_CascadePID_Test(M350D_STA *M350x, float Angle, float SpeedMax)
{
	PID_Calculate_Positional(&(M350x->M_Speed_PoPID), M350x->angleSum, Angle);
	M350x->M_Speed_PoPID.Output = Scope_Max_Min(M350x->M_Speed_PoPID.Output, SpeedMax);
	
	PID_Calculate_Incremental(&(M350x->M_Speed_InPID), M350x->speed, M350x->M_Speed_PoPID.Output);
	M350x->M_Speed_InPID.Output = Scope_Max_Min(M350x->M_Speed_InPID.Output, cascadeOutputMax);
	
  M350x->cascadeOutput = M350x->M_Speed_InPID.Output;
	M350x->electric = M350x->cascadeOutput;
}
/**
* @description: ����ϵ����
* @param {M350D_STA* M3508} ���ṹ������
* @param {float *SendAngle} ���͵�Ŀ��ֵ
* @return none
*/
volatile void M_Init_Stable(M350D_STA *M350x, volatile float *SendAngle)
{
	*SendAngle = M350x->angleSum;
  HAL_Delay(500);
}
/**
* @description: ���������ٶ� �⻷λ�ã���target_vel�����룬���ƶ���Ϊd_angle
* @param {M350D_STA* M3508} ���ṹ������
* @param {float Vel} Ŀ���ٶ�
* @return none
*/
void M_Calculate_CascadePID_Vel(M350D_STA *M350x, float Vel)
{
	PID_Calculate_Positional(&(M350x->M_Speed_PoPID), M350x->dAngle, Vel);
	M350x->M_Speed_PoPID.Output = Scope_Max_Min(M350x->M_Speed_PoPID.Output, poitionOutputMax);
	
	PID_Calculate_Incremental(&(M350x->M_Speed_InPID), M350x->speed, M350x->M_Speed_PoPID.Output);
	M350x->M_Speed_InPID.Output = Scope_Max_Min(M350x->M_Speed_InPID.Output, incremenOutputMax);
	M350x->electric = M350x->M_Speed_InPID.Output;
}
//�ñ��������Ƕ�����
void M_Encoder_Cascade_PID(M350D_STA *M350x, JY_Can_Receive *JY, float Angle, float SpeedMax)
{
	PID_Calculate_Positional(&(M350x->M_Speed_PoPID), JY->angle, Angle);
	M350x->M_Speed_PoPID.Output = Scope_Max_Min(M350x->M_Speed_PoPID.Output, SpeedMax);
	
	PID_Calculate_Incremental(&(M350x->M_Speed_InPID), M350x->speed, M350x->M_Speed_PoPID.Output);
	M350x->M_Speed_InPID.Output = Scope_Max_Min(M350x->M_Speed_InPID.Output, 6000);
	
  M350x->cascadeOutput = M350x->M_Speed_InPID.Output;
	M350x->electric = M350x->cascadeOutput;
}
