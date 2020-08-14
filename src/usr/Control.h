#ifndef CONTROL_H
#define CONTROL_H

#include "commonvalue.h"

#ifdef  DEFIN_CONTROL_EXTERN
	#define  CONTROL_EXTERN
#else 
	#define  CONTROL_EXTERN extern 
#endif


/*
	g_ucUpdatePra
	FROM_DEFAULT 默认更新参数
	FROM_FLASH 从flash更新
	FORM_DEBUG 上位机更新
*/	
typedef enum {
	 FROM_DEFAULT,
	 FROM_FLASH,
	 FROM_DEBUG,
}UpdatePraState_t;

CONTROL_EXTERN uint8_t 				  g_ucDeviceMode;
CONTROL_EXTERN UpdatePraState_t 	   g_emUpdatePra;	
CONTROL_EXTERN	__IO float			     g_fAngleExp[3] ;//飞控控制角度，初始化读取flash
/***
g_hiReceptionExp[0] =  PWM1
g_hiReceptionExp[1] =  PWM2
*/
CONTROL_EXTERN  int16_t           g_hiReceptionExp[3] ;//从遥控器介绍到的控制信息
CONTROL_EXTERN	float			  g_fSpeedPiderr[3] ;

CONTROL_EXTERN  CurrentType 	 Ibase;
CONTROL_EXTERN  CurrentType		ADCCurrent;
CONTROL_EXTERN  ThrCurrent_type	 globleCurrentdebug ;	
CONTROL_EXTERN   Positon_type  	Positon;
CONTROL_EXTERN  	vector 	g_SDq;

CONTROL_EXTERN uint8_t g_InitAngleStarte ;
void control_Init(void);
void control(void);
void PWM_Circl(void);
void control_ready(void);


/****/

//factor 要用到
void set_motor_const_position(uint16_t sector);
void update_pid_pra_to_flash(void);
void update_flashclib_from_ram(void);
void set_start_control(uint8_t);
/******/

#endif
	
	