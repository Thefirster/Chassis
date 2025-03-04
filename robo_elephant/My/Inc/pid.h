#ifndef _PID_H_
#define _PID_H_

#define POS_ERROR_MAX 5000 //位置式最大累计误差
//pid输出值最大限幅
enum IPC_SpeedMax{
	incremenOutputMax = 6000,
	poitionOutputMax = 6000,
	cascadeOutputMax = 6000,
};

typedef struct
{
	float Measure;
	float Target;
	
	float Kp;
	float Ki;
	float Kd;

	float Error;
	float LastError;
	float LastLastError;
	float ErrorSum;

  float p_out;
  float i_out;
  float d_out;
	float Output;
}PID;

void PID_Calculate_Incremental(PID* PID, float Measure, float Target);
void PID_Calculate_Positional(PID* PID, float Measure, float Target);
#endif
