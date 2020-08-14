#ifndef 	IIC_H
#define     IIC_H

#include "stm32f30x.h"

#ifdef  DEFIN_IIC_EXTERN
	#define  IIC_EXTERN
#else 
	#define  IIC_EXTERN extern 
#endif
//#define IO_IIC

typedef struct
{ 
	I2C_TypeDef* piic;	
	u8 number;
	u8 adrr;
	u8 lenth;
	u8 errcouter;
	u8 secusscouter;	
	u8 *data;
	u8 dataupdataflag;
	u8 receviefinish;
    
    /*slave*/
    uint8_t *DmaBuf;
}MIIC_TypeDef;


IIC_EXTERN	u8 g_ucWaitFinish;//just read or write need finish ones

//#define I2C_TIMING                0x00310309
//#define I2C_TIMING              0x00700623//1m  SORCE72M  err=0.045% digfilter = 10
//#define I2C_TIMING              0x00700D2E//800k  SORCE72M  err=0.04% digfilter = 10
#define I2C_TIMING              0x00700653//600k  SORCE72M  err=0.033% digfilter = 10
// 
//#define I2C_TIMING              0x0070063F //720k  SORCE72M  err=0.04% digfilter = 10
//#define I2C_TIMING              0x00D02277//400k  SORCE72M  err=0.0133% digfilter = 3

//#define I2C_TIMING               0x00700954//600k  SORCE72M  err=0.0333% digfilter = 4
//#define I2C_TIMING               0x00700E69//500k  SORCE72M  err=0.0278% digfilter = 4
#define IIC_DIGFILTER  10			
u8 iic_write_data(MIIC_TypeDef* MIIC);
u8  iic_read_data(MIIC_TypeDef* MIIC);
void iic_init(void);


#endif
