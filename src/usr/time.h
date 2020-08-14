#ifndef TIME_H
#define TIME_H
#include "stm32f30x.h"
#ifdef DEFIN_TIME_EXTERN
	#define TIME_EXTERN
#else
	#define TIME_EXTERN   extern 
#endif 

TIME_EXTERN  __IO  uint32_t   TimeCounter1Ms  ;
TIME_EXTERN  __IO  uint32_t   TimeCounter100uS  ;
TIME_EXTERN   uint8_t Time1MsFlag;
TIME_EXTERN   uint8_t Time100usFlag;	

typedef struct  {
	u32 OldReadTime;
	u32 ReadTime;
	uint32_t Time1msConter;
    uint8_t ClearFlag;
	u32     Dutyime;
	u8  	TimeOverFlowCouter;
}PrecisionTime_Type;




void TIM_Init(void);
float Get_Time_us(PrecisionTime_Type*  PrecisionTime);
float Get_Time_s(PrecisionTime_Type*  PrecisionTime);

uint8_t CompareTime1s(PrecisionTime_Type*  PrecisionTime,uint16_t CT);
uint8_t  CompareTime1ms(PrecisionTime_Type*  PrecisionTime,uint16_t CT);
void ClearCompareTime(PrecisionTime_Type*  PrecisionTime);


void Get_Delat(void);
#endif
