#ifndef   _PWM_H_
#define   _PWM_H_

#include "stm32f30x.h"


#ifdef  DEFIN_PWM_EXTERN
	#define  PWM_EXTERN
#else 
	#define  PWM_EXTERN extern 
#endif

#define  PWM_PERIOD   4000
PWM_EXTERN uint16_t TimerPeriod ;
PWM_EXTERN uint16_t Channel1Pulse , Channel2Pulse , Channel3Pulse , Channel4Pulse ;
PWM_EXTERN uint16_t OldChannel1Pulse , OldChannel2Pulse , OldChannel3Pulse , OldChannel4Pulse ;	
void PWM_Init(void);
void Chang_PWMPulse(void);
#endif 
