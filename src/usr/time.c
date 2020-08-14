#define DEFIN_TIME_EXTERN

#include "time.h"
#include "core_cm4.h" 
#include "device.h"








//time 6  用于时间 标志
//time 2 用于 delata
//time 3 用于usart 接受错误时候dma开始传输

void TIM_Init()
{


	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    
	/* TIM1 clock enable */  
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	/* Time base configuration */  
	TIM_DeInit(TIM6);
	TIM_TimeBaseStructure.TIM_Period =  250-1 ;  // = 1000us = 1ms
	TIM_TimeBaseStructure.TIM_Prescaler =71 ;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	
	

	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);   
  /* TIM6 enable counter */
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE ); 
	TIM_Cmd(TIM6, ENABLE);

	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_DeInit(TIM2);
	/* Time base configuration */  
	TIM_TimeBaseStructure.TIM_Period =  0xFFFFFFFF-1 ;  // = 1000us = 1ms
	TIM_TimeBaseStructure.TIM_Prescaler = 0 ;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	
//	/* Enable the TIM1 Trigger and commutation interrupt */
//	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);   
//  /* TIM3 enable counter */
//	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); 
	TIM_Cmd(TIM2, ENABLE);	
}
u16 Time100us = 0;

uint32_t  g_ul_ms_ticks = 0;
uint32_t   Get_TimReg_Couter(PrecisionTime_Type*  PrecisionTime)//<1min
{
	u32 TimeTemp;
	TimeTemp = TIM2->CNT;
	if(TimeTemp>  PrecisionTime->OldReadTime){
		PrecisionTime->Dutyime = 	PrecisionTime->OldReadTime + ~TimeTemp ;
	}else {
		PrecisionTime->Dutyime = 	PrecisionTime->OldReadTime- TimeTemp ;
	}
	return  TimeTemp;
}
float Get_Time_us(PrecisionTime_Type*  PrecisionTime)
{
	uint32_t TimeTemp  = Get_TimReg_Couter(PrecisionTime);
    PrecisionTime->OldReadTime = TimeTemp;
	return (PrecisionTime->Dutyime*0.013889f); 
  
}
float Get_Time_s(PrecisionTime_Type*  PrecisionTime)
{
	return Get_Time_us(PrecisionTime)*0.001f*0.001f; 
}

void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)){
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
		TIM2->CNT  =  0xFFFFFFFF-1;
		//PrecisionTime.TimeOverFlowCouter++;
	}
}





uint8_t time500us = 0;
uint8_t time500usflag = 0;
void TIM6_DAC_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM6,TIM_IT_Update)){
		TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
        TimeCounter100uS++;
        if(++time500us>=2){
            time500us = 0;
            time500usflag  = 1;
           Time100usFlag = 1;       
        }
          
		if(++Time100us >=4){
            
			Time100us = 0;
			Time1MsFlag = 1;
            TimeCounter1Ms ++;
            g_ul_ms_ticks++;  
		}
      
        
	}
}

uint8_t  CompareTime1ms(PrecisionTime_Type*  PrecisionTime,uint16_t CT)
{
    u32 TimeTemp;
	TimeTemp = g_ul_ms_ticks;
    if(PrecisionTime->ClearFlag==0){
        PrecisionTime->ClearFlag = 1;
        PrecisionTime->OldReadTime = TimeTemp;
        return 0;
    }
	if(TimeTemp>=  PrecisionTime->OldReadTime){
		PrecisionTime->Dutyime = 	TimeTemp  -PrecisionTime->OldReadTime ;
	}else {
		PrecisionTime->Dutyime = 	PrecisionTime->OldReadTime + ~TimeTemp ;
	}
    if(PrecisionTime->Dutyime>=CT){
        PrecisionTime->OldReadTime = TimeTemp;
        return 1;
    }
        
    return  0; 
}
uint8_t CompareTime1s(PrecisionTime_Type*  PrecisionTime,uint16_t CT)
{
    u32 TimeTemp;
	TimeTemp = g_ul_ms_ticks;
    if(PrecisionTime->ClearFlag==0){
        PrecisionTime->ClearFlag = 1;
        PrecisionTime->OldReadTime = TimeTemp;
        return 0;
    }
	if(TimeTemp>=  PrecisionTime->OldReadTime){
		PrecisionTime->Dutyime = 	TimeTemp  -PrecisionTime->OldReadTime ;
	}else {
		PrecisionTime->Dutyime = 	PrecisionTime->OldReadTime + ~TimeTemp ;
	}
    if(PrecisionTime->Dutyime>= (CT*1000)){
        PrecisionTime->OldReadTime = TimeTemp;
        return 1;
    }
    return  0;  
}
void ClearCompareTime(PrecisionTime_Type*  PrecisionTime)
{
     PrecisionTime->ClearFlag  = 0; 
}
