#include "mpuheart.h"
#include "PID.h"
#include "device.h"

PID_Type MpuHearPID;
float fHeatpid_p   = 200.0f;
float fHeatpid_i    = 0.1201f;
float fHeatpid_d = 500.f;

//float fHeatpid_p   = 3500.0f;
//float fHeatpid_i    = 0.3f;
//float fHeatpid_d = 12000.f;
#define HEART_PWM   TIM15
//内部相机发热到67.9°
void MpuHeartInit(void)
{
    
	if(g_ucDeviceNum!=0)return ;
	/* GPIOA clock enable */ 
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM15, ENABLE); 
	
	
	GPIO_InitTypeDef  GPIO_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	/* TIM2 channel 2 pin (PA.01) configuration */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_9);
	
	
	/* Time base configuration */  
	
	TIM_TimeBaseStructure.TIM_Period =  1800 ;  //1k
	TIM_TimeBaseStructure.TIM_Prescaler =0 ;//36 M
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM15, &TIM_TimeBaseStructure);
	
	/* Channel 1, 2,3 and 4 Configuration in PWM mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState =  TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 		0;
	TIM_OCInitStructure.TIM_OCPolarity =  TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState =  TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	
	TIM_OC2Init(HEART_PWM, &TIM_OCInitStructure);
	
	TIM_OC2PreloadConfig(HEART_PWM,TIM_OCPreload_Enable);  
	TIM_CtrlPWMOutputs(HEART_PWM, ENABLE);
	TIM_Cmd(HEART_PWM, ENABLE);
	
	
	PID_Init(&MpuHearPID,fHeatpid_p,fHeatpid_i,fHeatpid_d);
	MpuHearPID.OutlimitFlag  = 1;
	MpuHearPID.DifFirst=1;
	MpuHearPID.StatelimitFlag = 1;
	PID_SetMax(&MpuHearPID,1400-1,1400-1);	
	MpuHearPID.MaxLimit = 600;
	MpuHearPID.MinLimit = -600;

	PID_SetParam(&MpuHearPID,fHeatpid_p,fHeatpid_i,fHeatpid_d); 
	MpuHearPID.Beta = 0.8f;
	MpuHearPID.Beta1 = 0.7;
    
}
float  ExpHeart = 70.f;
extern float   Temprature ;
uint8_t  TempeConstantFlag  = 0;//上电小于1
uint8_t  TempeConstantFlag1  = 0;//校准要小于0.1
uint32_t TempeConstantConter = 0;
uint32_t TempeConstantConter1 = 0;
extern uint8_t  expv[];
void MpuHeart(void)
{
 
    if(g_ucDeviceNum!=0)return ;
  
    float err = fabs(ExpHeart-Temprature);
		if(TempeConstantFlag == 0){
			if(err<=0.5f){
				TempeConstantConter++;
			}else {
				TempeConstantConter = 0;
			}
			if(TempeConstantConter>=500){
				TempeConstantFlag = 1;
			}
		}
		if(TempeConstantFlag1==0){
			if(err<=0.2f){
				TempeConstantConter1++;
			}else {
				TempeConstantConter1 =0;	
			}	
			if(TempeConstantConter1>=1000){
				TempeConstantFlag1 = 1;
			}  
		}

		float pid ;
		if(err>0.1f){
			 pid = 0+Get_PID_Po(&MpuHearPID,ExpHeart,Temprature);
		}else {
			 pid = MpuHearPID.inter;
		}
			
    
		if(pid<0.0f)pid = 0.f;
    int32_t pluse  = (int32_t)pid;
    HEART_PWM->CCR2  =  220+pluse;   
}

uint8_t GetTempeConstantFlag(void)
{
    return TempeConstantFlag;
}


