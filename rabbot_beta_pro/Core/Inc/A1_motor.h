#ifndef _A1_MOTOR_H
#define _A1_MOTOR_H
#include "stdint.h"
#include "usart.h"
#define RS485_TX_EN(x)		HAL_GPIO_WritePin(RE_GPIO_Port, RE_Pin, x)	//485ģʽ����.0,����;1,����.

typedef int16_t q15_t;

#pragma pack(1)

// �����õ����������ݽṹ
typedef union{
        int32_t           L;
        uint8_t       u8[4];
       uint16_t      u16[2];
       uint32_t         u32;
          float           F;
}COMData32;

typedef struct {
	// ���� ���ݰ�ͷ
  unsigned char  start[2];     // ��ͷ
	unsigned char  motorID;      // ���ID  0,1,2,3 ...   0xBB ��ʾ�����е���㲥����ʱ�޷��أ�
	unsigned char  reserved;
}COMHead;

#pragma pack()

#pragma pack(1)

typedef struct { 
	
	   uint8_t  fan_d;       // �ؽ��ϵ�ɢ�ȷ���ת��
	   uint8_t  Fmusic;      // �������Ƶ��   /64*1000   15.625f Ƶ�ʷֶ�
	   uint8_t  Hmusic;      // �������ǿ��   �Ƽ�ֵ4  ����ǿ�� 0.1 �ֶ�  
	   uint8_t  reserved4;
	
	   uint8_t  FRGB[4];     // ���LED 
	
}LowHzMotorCmd;

typedef struct {  // �� 4���ֽ�һ������ ����Ȼ�����������
	// ���� ����
    uint8_t  mode;        // �ؽ�ģʽѡ��
    uint8_t  ModifyBit;   // ������Ʋ����޸�λ
    uint8_t  ReadBit;     // ������Ʋ�������λ
    uint8_t  reserved;

    COMData32  Modify;     // ��������޸� ������ 
    //ʵ�ʸ�FOC��ָ������Ϊ��
    //K_P*delta_Pos + K_W*delta_W + T
    q15_t     T;      // �����ؽڵ�������أ������������أ�x256, 7 + 8 ����
    
    //ԭ������*256��2��8�η�����ȡ����Ȼ��ֵ��T//
    
    q15_t     W;      // �����ؽ��ٶ� �����������ٶȣ� x128,       8 + 7����	
    int32_t   Pos;      // �����ؽ�λ�� x 16384/6.2832, 14λ������������0������������ؽڻ����Ա�����0��Ϊ׼��

    q15_t    K_P;      // �ؽڸն�ϵ�� x2048  4+11 ����
    q15_t    K_W;      // �ؽ��ٶ�ϵ�� x1024  5+10 ����

    uint8_t LowHzMotorCmdIndex;     // �����Ƶ�ʿ������������, 0-7, �ֱ����LowHzMotorCmd�е�8���ֽ�
    uint8_t LowHzMotorCmdByte;      // �����Ƶ�ʿ���������ֽ�
	
     COMData32  Res[1];    // ͨѶ �����ֽ�  ����ʵ�ֱ��һЩͨѶ����
	
}MasterComdV3;   // �������ݰ��İ�ͷ ��CRC 34�ֽ�

typedef struct {
	// ���� ��������������ݰ�	
    COMHead head;    
    MasterComdV3 Mdata;
    COMData32 CRCdata;
}MasterComdDataV3;//�������� 

typedef struct{
    uint8_t id;
    uint8_t mode;
    float kw;
    float kp;
    float pos;
    float w;
    float t;
    float angle;
    float w_now;
}A1_data;

// typedef struct {
// 	// ���� �ܵ�485 ���ݰ�
	
//   MasterComdData M1;
// 	MasterComdData M2;
// 	MasterComdData M3;
	
// }DMA485TxDataV3;

#pragma pack()

#pragma pack(1)

typedef struct {  // �� 4���ֽ�һ������ ����Ȼ�����������
    // ���� ����
    uint8_t  mode;        // ��ǰ�ؽ�ģʽ
    uint8_t  ReadBit;     // ������Ʋ����޸�     �Ƿ�ɹ�λ
    int8_t  Temp;        // �����ǰƽ���¶�   
    uint8_t  MError;      // ������� ��ʶ
 
    COMData32  Read;     // ��ȡ�ĵ�ǰ ��� �Ŀ������� 
    int16_t     T;      // ��ǰʵ�ʵ���������       7 + 8 ����

    int16_t     W;      // ��ǰʵ�ʵ���ٶȣ����٣�   8 + 7 ����
    float      LW;      // ��ǰʵ�ʵ���ٶȣ����٣�   

    int16_t     W2;      // ��ǰʵ�ʹؽ��ٶȣ����٣�   8 + 7 ����
    float      LW2;      // ��ǰʵ�ʹؽ��ٶȣ����٣�   

    int16_t    Acc;           // ���ת�Ӽ��ٶ�       15+0 ����  ������С
    int16_t    OutAcc;        // �������ٶ�         12+3 ����  �����ϴ�
		 
    int32_t   Pos;      // ��ǰ���λ�ã�����0������������ؽڻ����Ա�����0��Ϊ׼��
    int32_t   Pos2;     // �ؽڱ�����λ��(���������)

    int16_t     gyro[3];  // ���������6�ᴫ��������
    int16_t     acc[3];   

    // ��������������   
    int16_t     Fgyro[3];  //  
    int16_t     Facc[3];
    int16_t     Fmag[3];
    uint8_t     Ftemp;     // 8λ��ʾ���¶�  7λ��-28~100�ȣ�  1λ0.5�ȷֱ���
    
    int16_t     Force16;   // ����������16λ����
    int8_t      Force8;    // ����������8λ����
		
    uint8_t     FError;    //  ��˴����������ʶ
		
    int8_t      Res[1];    // ͨѶ �����ֽ�
	
}ServoComdV3;  // �������ݰ��İ�ͷ ��CRC 78�ֽڣ�4+70+4��

typedef struct {
    // ���� ��������������ݰ�	
    COMHead        head;
    ServoComdV3      Mdata;

    COMData32    CRCdata;

}ServoComdDataV3;	//��������

// typedef struct {
// 	// ���� �ܵ�485 �������ݰ�
	
//   ServoComdDataV3 M[3];
//  // uint8_t  nullbyte1;
	
// }DMA485RxDataV3;


#pragma pack()

//  00 00 00 00 00 
//  00 00 00 00 00 
//  00 00 00 00 00 
//  00 00 00
// ���ݰ�Ĭ�ϳ�ʼ�� 
// �������͵����ݰ� 
/*
                 Tx485Data[_FR][i].head.start[0] = 0xFE ;     Tx485Data[_FR][i].head.start[1] = 0xEE; // ���ݰ�ͷ					 
				 Tx485Data[_FR][i].Mdata.ModifyBit = 0xFF;    Tx485Data[_FR][i].Mdata.mode = 0;   // Ĭ�ϲ��޸����� �� �����Ĭ�Ϲ���ģʽ				
				 Tx485Data[_FR][i].head.motorID = i;    0                                          // Ŀ�������
				 Tx485Data[_FR][i].Mdata.T = 0.0f;                           // Ĭ��Ŀ��ؽ��������                      motor1.Extra_Torque = motorRxData[1].Mdata.T*0.390625f;     // N.M  ת��Ϊ N.CM   IQ8����
				 Tx485Data[_FR][i].Mdata.Pos = 0x7FE95C80;                   // Ĭ��Ŀ��ؽ�λ��  ������λ�û�          14λ�ֱ��� 
				 Tx485Data[_FR][i].Mdata.W = 16000.0f;                       // Ĭ��Ŀ��ؽ��ٶ�  �������ٶȻ�          1+8+7����     motor1.Target_Speed =  motorRxData[1].Mdata.W*0.0078125f;   // ��λ rad/s	       IQ7����
				 Tx485Data[_FR][i].Mdata.K_P = (q15_t)(0.6f*(1<<11));        // Ĭ�Ϲؽڸն�ϵ��   4+11 ����                     motor1.K_Pos = ((float)motorRxData[1].Mdata.K_P)/(1<<11);      // �����նȵ�ͨѶ���ݸ�ʽ  4+11
				 Tx485Data[_FR][i].Mdata.K_W = (q15_t)(1.0f*(1<<10));        // Ĭ�Ϲؽ��ٶ�ϵ��   5+10 ����                    motor1.K_Speed = ((float)motorRxData[1].Mdata.K_W)/(1<<10);    // ���������ͨѶ���ݸ�ʽ  5+10
*/ 

typedef struct {
	// ���� ���͸�ʽ������
    MasterComdDataV3  motor_send_data;  //����������ݽṹ�壬���motor_msg.h
	int hex_len;                    //���͵�16�����������鳤��, 34
    long long send_time;            //The time that message was sent���͸������ʱ��, ΢��(us)
    // �����͵ĸ�������
    unsigned short id;              //���ID��0����ȫ�����
    unsigned short mode;            //0:����, 5:����ת��, 10:�ջ�FOC����
    //ʵ�ʸ�FOC��ָ������Ϊ��
    //K_P*delta_Pos + K_W*delta_W + T
    float T;                        //�����ؽڵ�������أ������������أ���Nm��
    float W;                        //�����ؽ��ٶȣ����������ٶȣ�(rad/s)
    float Pos;                      //�����ؽ�λ�ã�rad��
    float K_P;                      //�ؽڸն�ϵ��
    float K_W;                      //�ؽ��ٶ�ϵ��
}MOTOR_send;

typedef struct
{
    // ���� ��������
    ServoComdDataV3 motor_recv_data;     //����������ݽṹ�壬���motor_msg.h
    int hex_len;                    //���յ�16�����������鳤��, 78
    long long resv_time;            //���ո������ʱ��, ΢��(us)
    int correct;                   //���������Ƿ�������1������0��������
    //����ó��ĵ������
    unsigned char motor_id;         //���ID
    unsigned char mode;             //0:����, 5:����ת��, 10:�ջ�FOC����
    int Temp;                       //�¶�
    unsigned char MError;           //������

    float T;                        // ��ǰʵ�ʵ���������
    float W;                        // ��ǰʵ�ʵ���ٶȣ����٣�
    float LW;                       // ��ǰʵ�ʵ���ٶȣ����٣�
    int Acc;                      // ���ת�Ӽ��ٶ�
    float Pos;                      // ��ǰ���λ�ã�����0������������ؽڻ����Ա�����0��Ϊ׼��

    float gyro[3];                  // ���������6�ᴫ��������
    float acc[3];

}MOTOR_recv;

//extern long long getSystemTime();       //��ȡ��ǰϵͳʱ�䣨΢��us��
int modify_data(MOTOR_send* motor);    //�����ݴ���Ϊstm32����ĸ�ʽ
//int extract_data(MOTOR_recv*);   //�����յ������ݽ��
uint32_t crc32_core(uint32_t*, uint32_t);    //��У������ָ�룬���鳤�ȣ�������ȥ��

void A1_Motor_Send_Data(float W, float Pos, float k_p, float k_w,int id,int mode);
void A1_Motor_Send_Data_yuntai(float W, float Pos, float k_p, float k_w, float t, int id, int mode);
void a1_init(void);
float a1_get_position(uint8_t id);
float a1_get_speed(uint8_t id);
void a1_waiting_recv(uint8_t id);
void a1_set_mode(uint8_t id, uint8_t x);
void a1_set_kp_kw(uint8_t id, float kp, float kw);
void a1_set_torque(uint8_t id, float x);
void a1_set_pos(uint8_t id, float rad);
void a1_set_w(uint8_t id, float w);
void a1_set_kw_w(uint8_t id,float kw,float w);
#endif
