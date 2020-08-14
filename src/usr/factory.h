#ifndef _FACTORY_H
#define _FACTORY_H

#include "stm32f30x.h"

#ifdef  DEFIN_FACTORY_EXTERN
	#define  FACTORY_EXTERN
#else 
	#define  FACTORY_EXTERN extern 
#endif


void factory(void);
void factory_int(void);
void ClibrateFoc(void);

#endif