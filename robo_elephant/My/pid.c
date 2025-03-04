#include "pid.h"

//增量式PID
void PID_Calculate_Incremental(PID* PID, float Measure, float Target) 
{
	PID->Measure = Measure;
	PID->Target = Target;
	PID->Error = Target - Measure;
	
	if((PID->Error<10)&&(PID->Error>-10))
		PID->Error = 0;
	
	PID->p_out = (PID->Kp)*(PID->Error - PID->LastError);  
  PID->i_out = (PID->Ki)*(PID->Error);	
	PID->d_out = (PID->Kd)*((PID->Error -PID->LastError)+(PID->LastLastError - PID->LastError));
	PID->Output += PID->d_out + PID->i_out + PID->p_out;
	
	PID->LastLastError = PID->LastError;
	PID->LastError = PID->Error;
}

//位置式PID
void PID_Calculate_Positional(PID* PID, float Measure, float Target)
{
	PID->Measure = Measure;
	PID->Target = Target;
	PID->Error = Target - Measure;
	
	if((PID->Error<5)&&(PID->Error>-5))
	  PID->Error = 0.0f;
	
	PID->ErrorSum += PID->Error;
	PID->d_out = (PID->Kd)*((PID->Error) - (PID->LastError));
	PID->LastError = PID->Error;
	PID->p_out = (PID->Kp)*(PID->Error);
  PID->i_out = (PID->Ki)*(PID->ErrorSum);

  PID->Output = PID->d_out + PID->i_out + PID->p_out;
}
