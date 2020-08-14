#ifndef CCS_H
#define CCS_H
#include "stm32f30x.h"
#include "IObit_remap.h"

#define 	STAT		0x00	//STATus	register	            
#define 	ACSTAT      0x01	//ACtivation	STATus	register    
#define 	AVAL        0x02	//Angle	VALue	register            
#define 	ASPD        0x03	//Angle	SPeeD	register            
#define 	AREV        0x04	//Angle	REVolution	register        
#define 	FSYNC       0x05	//Frame	SYNChronization	register    
#define 	MOD_1       0x06	//Interface	MODe1	register        
#define 	SIL         0x07	//SIL	register	                
#define 	MOD_2       0x08	//Interface	MODe2	register        
#define 	MOD_3       0x09	//Interface	MODe3	register        
#define 	OFFX        0x0A	//OFFset	X	                    
#define 	OFFY        0x0B	//OFFset	Y	                    
#define 	SYNCH       0x0C	//SYNCHronicity		                
#define 	IFAB        0x0D	//IFAB	register	                
#define 	MOD_4       0x0E	//Interface	MODe4	register        
#define 	TCO_Y       0x0F	//Temperature	COefficient	register
#define 	ADC_X       0x10	//ADC	X-raw	value               
#define 	ADC_Y       0x11	//ADC	Y-raw	value               
#define 	D_MAG       0x14	//Angle	vector	MAGnitude           
#define 	T_RAW       0x15	//Temperature	sensor	RAW-value   
#define 	IIF_CNT     0x20	//IIF	CouNTer	value               
#define 	T25O        0x30	//Temperature	25¡ãC	Offset value

#define CCSDATA_IN()	{GPIOC->MODER&=~(3<<(13*2));}	
#define CCSDATA_OUT()	{GPIOC->MODER|=1<<(13*2);} 

#define	CCS_SCK0()		GPIOC->BRR = GPIO_Pin_15
#define	CCS_SCK1()		GPIOC->BSRR = GPIO_Pin_15
#define	CCS_DATA0()		GPIOC->BRR = GPIO_Pin_13
#define	CCS_DATA1()		GPIOC->BSRR = GPIO_Pin_13

#define	READ_DATA()		(GPIOC->IDR&GPIO_Pin_13)//PAin(7)//GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)
#define	CCS_CSQ0()		GPIOC->BRR = GPIO_Pin_14
#define	CCS_CSQ1()		GPIOC->BSRR = GPIO_Pin_14




u8 CCSWrite_Data(u8 adrr,u16* data,u8 len);
u8 CCSRead_Data(u8 adrr,u16* data,u8 len);
u8 CCSRead_Data_ASM(u8 adrr,u16* data,u8 len);
u16 Read_16Byte_ASM(void);	
	
	
#endif

