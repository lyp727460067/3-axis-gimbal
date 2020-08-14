#define PID_EXTERN_DEFINE
#include "PID.h"
#include "time.h"
#include "stdlib.h"
void PID_Init(PID_Type* PID,float P,float I,float D)
{

	
	
	PID->pid_instance.Ki = I;
	PID->pid_instance.Kd =D ;
	PID->pid_instance.Kp = P;	
	PID->StatelimitFlag = 1;
	
	//PID->DifFirst = 1;
	PID->OutlimitFlag = 1;
	PID->PMconif	 = 1;
	PID->Beta = 0.5f;	
	PID->Beta1 = 0.6f;		
	PID->MaxLimit = 3000;
	PID->MinLimit = -3000;
	PID->OutMaxLimit = 3000;
	PID->OutMinLimit = -3000;
	
	PID->inter = 0;
	arm_pid_init_f32(&PID->pid_instance,1);
}
void PID_SetParam(PID_Type* PID,float P,float I,float D)
{  
	PID->pid_instance.Ki =I ;
	PID->pid_instance.Kd = D;
	PID->pid_instance.Kp = P;	
	arm_pid_init_f32(&PID->pid_instance,0);
}
void PID_SetMax(PID_Type* PID,float Imax,float outmax)
{  
	PID->MaxLimit = Imax;
	PID->MinLimit = -Imax;
	PID->OutMaxLimit = outmax;
	PID->OutMinLimit = -outmax;
}
float Get_PID_Po(PID_Type*PID,float Expect ,float Actual)
{

    float Err = Expect- Actual;
    float Out = PID->inter+ PID->pid_instance.Ki*Err;	
    if(Out>=PID->MaxLimit)Out = PID->MaxLimit;
    if(Out<=PID->MinLimit)Out = PID->MinLimit;
    PID->inter = Out;	
    float  outtemp = 	Out+PID->pid_instance.Kp*Err ;
    if(PID->pid_instance.Kd>0.0f){
        float der ;
        if(isnan(PID->LastOut[0])){
            PID->LastOut[0] = 0;
            der = 0;
        }else {
            der = Err-PID->Errtep[0];
        }
        PID->Errtep[0] = Err; 
		
		PID->LastOut[0] = PID->LastOut[0]+PID->Beta1*(der-PID->LastOut[0]);//Beta1 >=0.5f  a = 2*pi*f*deltat ==> 6.28*0.001*100Hz
		//D项先滤波后平均2-3次再加入  
        //Err  =  update_butterworth_4_low_pass(&PID->LowPass,Err);
        outtemp	+= 	(PID->LastOut[0]*PID->pid_instance.Kd);	
    }
    
    if(PID->OutlimitFlag){
        if(outtemp>=PID->OutMaxLimit)outtemp = PID->OutMaxLimit;
        if(outtemp<=PID->OutMinLimit)outtemp = PID->OutMinLimit;
    }
    return outtemp;

}
float Get_PID(PID_Type*PID,float Expect ,float Actual)
{
	float Out=0;
	float Err = Expect- Actual;
	float Kptemp = PID->pid_instance.Ki;
	
	if(PID->DifFirst){
      Kptemp =PID->Beta*PID->pid_instance.Kp;
      Out = PID->pid_instance.state[2]
          +Kptemp*(Err -PID->pid_instance.state[0])
              +PID->pid_instance.Ki*((Err+PID->pid_instance.state[0])/2.0f);///;
      PID->inter = Out;
      //		Out += (Err -2*PID->Errtep[0]+PID->Errtep[1])*PID->pid_instance.Kd*PID->Beta1 ;	
      Out -=(Actual -PID->LastOut[0])*(PID->pid_instance.Kp-Kptemp);	 
      if(PID->pid_instance.Kd>0){
          Out-=(Actual -2*PID->LastOut[0]+PID->LastOut[1])*PID->pid_instance.Kd*(1-PID->Beta1) ;  
      }
      //		PID->LowPassout	= Actual*0.7+PID->LowPassout*0.3;
      //		Actual  = PID->LowPassout	;
      PID->LastOut[1] = PID->LastOut[0];
      PID->LastOut[0] = Actual;
      PID->Errtep[1] = PID->Errtep[0];
      PID->Errtep[0] = Err;			
      PID->pid_instance.state[2] = Out;
      PID->pid_instance.state[0] = Err;
		//	Out+=(PID->pid_instance.Kp-Kptemp)*Err;
	}else{
      Out = arm_pid_f32(&PID->pid_instance,Err);
	}
	if (isnan(PID->pid_instance.state[2])){
      PID->pid_instance.state[2] = PID->LastOut[2];
	}
	if(PID->StatelimitFlag){
      if(Out>=PID->MaxLimit)PID->pid_instance.state[2]  = PID->MaxLimit;
      if(Out<=PID->MinLimit)PID->pid_instance.state[2]  = PID->MinLimit;
	}
	Out = Out*PID->PMconif;	
	if(PID->OutlimitFlag){
      if(Out>=PID->OutMaxLimit)Out = PID->OutMaxLimit;
      if(Out<=PID->OutMinLimit)Out = PID->OutMinLimit;
	}
  
	PID->LastOut[2] = PID->pid_instance.state[2];
  return Out;
  
  
	
}
float Get_IncPID(PID_Type*PID,float Expect ,float Actual)
{
	float Out;
	float Err = Expect- Actual;;
    Out = (PID->pid_instance.A0 * Err) +
      (PID->pid_instance.A1 *PID->pid_instance.state[0]) 
		+ (PID->pid_instance.A2 * PID->pid_instance.state[1]) ;
    
	PID->pid_instance.state[1] =PID->pid_instance.state[0];
    PID->pid_instance.state[0] = Err;
    PID->pid_instance.state[2] = Out;
	return Out;
}


