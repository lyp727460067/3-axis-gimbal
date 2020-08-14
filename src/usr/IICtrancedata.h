#ifndef 	_IICTRANCEDATA_H
#define 	_IICTRANCEDATA_H 
#include "IIC.h"
#include "stm32f30x.h"

#ifdef  DEFIN_IICTRANCEDATA_EXTERN
	#define  IICTRANCEDATA_EXTERN
#else 
	#define  IICTRANCEDATA_EXTERN extern 
#endif




#include "IObit_remap.h"	
 
 
#define  DeviceNuber  2
#define MAXDATALENT   60
IICTRANCEDATA_EXTERN        int16_t g_hiRecEMReangle[3];
IICTRANCEDATA_EXTERN        MIIC_TypeDef  MIIC_DATA[DeviceNuber*2];
IICTRANCEDATA_EXTERN u8     SendSlavebuffer[DeviceNuber*2][MAXDATALENT];


void iic_tran_data(void);
void iic_tran_init(void);
void iic_tran_redy(void);
	
	
	
#endif
	
	
	
	