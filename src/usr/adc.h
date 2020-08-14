#ifndef _ADC_H
#define _ADC_H
#include "FIFO.h"
#include "stm32f30x.h"


#ifdef  ADC_EXTERN_DEFINE
  #define ADC_EXTERN
#else 
  #define  ADC_EXTERN   extern
#endif
  
void adc_Init(void);


ADC_EXTERN	FIFO_Type	CurrentADC;
ADC_EXTERN  FIFO_Type	BatteryADC;
#endif

