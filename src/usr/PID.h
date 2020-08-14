#ifndef _PID_H
#define _PID_H
#include "stm32f30x.h"  
#ifdef  PID_EXTERN_DEFINE
  #define PID_EXTERN
#else 
  #define  PID_EXTERN   extern
#endif
#include "arm_math.h"
#include "filter.h"
#define  Diflenth 4
typedef struct PID_Type
{ 	
    float	MaxLimit;
	float	MinLimit;
    float	OutMaxLimit;
	float	OutMinLimit;

	float Beta;	
	float Beta1;	
	float	inter;
	u8 StatelimitFlag;
	u8 OutlimitFlag;
	u8 DifFirst;
	u8 resetStateFlag;
	arm_pid_instance_f32 pid_instance;	
	float  PMconif;
	float LastOut[3];
	float Errtep[2];
	float LowPassout;
}PID_Type;

PID_EXTERN float PIDdeltat ;
void PID_SetDeltat(PID_Type*PID,float deltat);
float Get_PID(PID_Type*PID,float Expect ,float Actual);
float Get_IncPID(PID_Type*PID,float Expect ,float Actual);
float Get_PID_Po(PID_Type*PID,float Expect ,float Actual);
void PID_Init(PID_Type* PID,float P,float I,float D);
void PID_SetParam(PID_Type* PID,float P,float I,float D);
void PID_SetMax(PID_Type* PID,float Imax,float outmax);
#endif


