#ifndef _FLASH_H
#define _FLASH_H

#include "stm32f30x.h"

#ifdef  DEFIN_FLASH_EXTERN
	#define  FLASH_EXTERN
#else 
	#define  FLASH_EXTERN extern 
#endif
	
#define  DEVICENUMBER   3
#define  ANGLEPOSINDEX  1
#define  PRAPOSINDEX    2


//ram 够大   flash 内容拷贝两份
typedef struct
{
	uint32_t  wCRC[4];
	/*fac*/
	struct{
		uint16_t hwResert;	
		uint8_t  ucFactorFlag;//出厂校正过标志
		uint8_t  ucDeviceMode;// 设备模式		
	}sFac;
        /**/
	struct{
		uint32_t wResert;
		int32_t  iAngExp[DEVICENUMBER];//  期望角度 pid位置环
	}sExpang;
    
	/*angleoffset*/
	struct {
		uint32_t wResert;
        int16_t hiFocInitAngValue ;//foc零偏
        int16_t hiReInitAngValue;	//相对于水平偏置
	}sAngoffset;
	/*pra*/
	struct {
		uint32_t wResert;
		uint16_t hwMotorpid[DEVICENUMBER][2][6]; //电调pid
		uint16_t hwSpeedpid[3][6];	
		uint16_t hwEulerpid[3][6];
    uint8_t  ucExpSet[4][3];

	}sPra;
	/*MPU*/
	struct {
		uint32_t wResert;
		uint8_t ucGroFsSel;
		uint8_t ucAccFsSel;//mpu  fullscal 设置
		uint8_t ucGroDlpfCfg;
		uint8_t ucAccDlpfCfg; 
        
        
    int16_t hiMaxMinAccValue[6][3];
    int16_t hiAccBiasValue[3];
    int16_t hiGryBiasValue[3];
    int16_t hiTemperature;
    uint16_t Reserve;
    
    int32_t iGroOffsetValue[10];
        
        struct
        {
            uint32_t wImuId;
            uint8_t  ucCalibConter;
            int16_t hiDefaultBias[3];
            int16_t hiDefaultSlop[3];
            struct{
                uint8_t ucFlag;// 
                int16_t hiTemp;
                struct 
                {
                    int16_t hiBias;
                    int16_t hiSlop; 
                    int16_t hiNetom;                                      
                }Bias[3];
                
            }GyrBias[15];
        }tGyroBiasList;      
	 }sMpu;	    
	/**/
    struct {
        
        uint32_t wResert;
        struct {
            float P1[2][2];
            float X1[2]; 
        }LRS[3];
        
    }sLRS; 
}Flashpra_TypeDef;

FLASH_EXTERN 	Flashpra_TypeDef  g_SFlashpra;
FLASH_EXTERN    uint8_t   g_ucFlashReadFlag;
FLASH_EXTERN    uint8_t   g_ucFlashWriteFlag;
FLASH_EXTERN    uint8_t   g_ucFlashNormal;//有数据读取正常

FLASH_EXTERN uint8_t g_ucFactorFlag ;	

FLASH_EXTERN  u8    g_FlashIsWrited;
FLASH_EXTERN  u8   g_SaveTunigflashstate;//当PC端发送参数的时候，设置1为开始写flash，写完清除
FLASH_EXTERN  u8   g_Resetparadefault;

void flash_write(void);
void update_flashpra_from_ram(void);
void update_flashpra_from_flash(void);
void flash_Init(void);





#endif
