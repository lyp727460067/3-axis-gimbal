#ifndef ANGSENSOR_H_
#define ANGSENSOR_H_
#include "stm32f30x.h"
#include "arm_math.h"

#ifdef  DEFIN_ANGSENSOR_EXTERN
	#define  ANGSENSOR_EXTERN
#else 
	#define  ANGSENSOR_EXTERN extern 
#endif


ANGSENSOR_EXTERN int16_t g_hi0rgangle ;	

//Re 相对应水平位置
//g_hiOrgEMReAngle 相对应水平位置的原始角度还没有减偏移值
ANGSENSOR_EXTERN int16_t g_hiEMReAngle;	
ANGSENSOR_EXTERN float g_fEMFocangle ;	
	
void Init_AngSensor(void);
u8 Get_AngSensor(void);
void Get_AngRev(u8 Direction);
void Set_EMInitValveRe(int16_t Value);
void Set_EMInitValue(int16_t Value);
float Get_EMReangle(int16_t Value);
void set_ang_clibre(float clibrate);



/******/
//factor 要用到
void set_ang_offset(int16_t focang,int16_t reang);
void set_init_org(uint8_t index);


#endif


