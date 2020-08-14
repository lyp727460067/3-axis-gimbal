#define DEFIN_FLASH_EXTERN
#include "flash.h"
 #include "IICtrancedata.h"
#include <string.h>
#include  "Control.h"
#include "AngSensor.h"
#include "pctogimbal.h"
typedef struct
{
	uint32_t wStartAdrr;
	uint32_t wEndAdrr;
	uint32_t *pwData;
	uint32_t wLenth;
}Flash_write_TypeDef;

#define FLASH_PID0   (uint32_t)0X505050
#define FLASH_PID1   (uint32_t)0X505050
#define FLASH_PID3   (uint32_t)(FLASH_PID0^FLASH_PID1)


/*
					adrr		lenth
		bootloade   0x8000000   0x2000		8k
		app         0x8002000   0x10000    64K
		information 0x8012000   0x800	2K
*/
#define FLASH_PAGE_SIZE         ((uint32_t)0x00000800)   /* FLpASH Page Size */
#define FLASH_USER1_START_ADDR   ((uint32_t)0x0801F800)   /* Start @ of user Flash area */
#define FLASH_USER1_END_ADDR     ((uint32_t)0x08020000)   /* End @ of user Flash area */


#define FLASH_USER2_START_ADDR   ((uint32_t)0x08010000)   /* Start @ of user Flash area */
#define FLASH_USER2_END_ADDR     ((uint32_t)0x08010100)   /* End @ of user Flash area */
/*
*/
void flash_Init(void)
{
	memset(&g_SFlashpra,0,sizeof(Flashpra_TypeDef));
    update_flashpra_from_flash();
   // g_FlashIsWrited = 0;
    if(g_FlashIsWrited==0){
        memset(&g_SFlashpra,0,sizeof(g_SFlashpra));
        update_flashpra_from_ram();
        g_ucFlashWriteFlag = 1;
        flash_write();
    }else{
        g_ucUpdatePraFlag = 1;//更新新的参数到  ram
    }  
}
/*
*/
uint8_t  Flash_writedata(Flash_write_TypeDef* pwSdata)
{
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR); 
	
	uint32_t NbrOfPage = (pwSdata->wEndAdrr - pwSdata->wStartAdrr) / FLASH_PAGE_SIZE;
	uint32_t Address = pwSdata->wStartAdrr;
	for(uint32_t EraseCounter = 0; (EraseCounter <= NbrOfPage) ; EraseCounter++){ 
		if (FLASH_ErasePage(Address + (FLASH_PAGE_SIZE * EraseCounter))!= FLASH_COMPLETE){
			return 0;
		}
	}
	for(uint32_t i=0;i<pwSdata->wLenth;i++){
		if (FLASH_ProgramWord(Address, (*(pwSdata->pwData+i))) == FLASH_COMPLETE){
			Address = Address + 4;
		}	
	}
	FLASH_Lock(); 
	return 1;	
}

/*
*/
//写入之前 PWM全部给0 要不电机乱动
//然后关闭中断
//写入完成后开启中断
//然后给两个设备发送的是0
uint8_t flash_writePra(void)
{
	uint8_t result = 0;	
    Flash_write_TypeDef  Switedata;
    Switedata.pwData = (uint32_t*)&g_SFlashpra;
    Switedata.wStartAdrr = FLASH_USER1_START_ADDR;
    Switedata.wEndAdrr = FLASH_USER1_END_ADDR;
    Switedata.wLenth = sizeof(Flashpra_TypeDef);

	__disable_irq() ;
	
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	if(Flash_writedata(&Switedata)){
		result = 1;
	}
	__enable_irq();
	return result;
}
/*
*/
uint32_t FlashWaitConter = 0;
void flash_write(void)
{
	if(g_ucFlashWriteFlag){
        if(++FlashWaitConter>=5000){
            FlashWaitConter  = 0;
            g_ucFlashWriteFlag   = 0;     
            update_flashpra_from_ram();
            flash_writePra();
        } 
       
    }

}

extern int16_t Get_EMInitValue(void)  ;
extern int16_t Get_EMInitValveRe(void);

extern void update_gra_clic(int16_t *data,uint8_t idex);
extern void update_graoffset_clic(float *data,uint8_t idex);
extern void updateGryLrs(float *data,uint8_t idex);
void update_flashclib_from_ram(void)
{   
	g_SFlashpra.sAngoffset.wResert   = 0x5a5a5a5a;
	FlashMoterVariableEachCopy((void*)&g_SFlashpra.sAngoffset.hiFocInitAngValue,0);

	g_SFlashpra.sMpu.wResert   = 0x5a5a5a5a;	
    FlashMpuVariableEachCopy(&g_SFlashpra.sMpu.hiMaxMinAccValue[0],0);  
    
    //g_SFlashpra.sMpu.wCRC   = 0x5a5a5a5a;
    FlashGryListVariableEachCopy(&g_SFlashpra.sMpu.tGyroBiasList,0);
    
    g_SFlashpra.sLRS.wResert   = 0x5a5a5a5a;
    FlashRLSVariableEachCopy((float*)&g_SFlashpra.sLRS.LRS[0],0);
}

void update_flashpra_from_ram(void)
{
	g_SFlashpra.wCRC[0]  =FLASH_PID0;
	g_SFlashpra.wCRC[1]  =FLASH_PID1;
	g_SFlashpra.wCRC[2]  =sizeof(Flashpra_TypeDef);
	g_SFlashpra.wCRC[3]  =FLASH_PID3^g_SFlashpra.wCRC[0]^g_SFlashpra.wCRC[1]^g_SFlashpra.wCRC[2];
	/***********/
    g_SFlashpra.sFac.hwResert =0x5a5a;
	g_SFlashpra.sFac.ucDeviceMode = g_ucDeviceMode;
	/***********/
	g_SFlashpra.sPra.wResert   = 0x5a5a5a5a;
	memcpy((void*)((uint8_t*)&g_SFlashpra.sPra+4),(void*)((uint8_t*)&Tuning_data+4),sizeof(Tuning_data)-4);	
	/***********/
	g_SFlashpra.sExpang.wResert   = 0x5a5a5a5a;	
	memcpy((void*)((uint8_t*)&g_SFlashpra.sExpang+4) ,(void*)g_fAngleExp,12);	


}
extern uint8_t moterangCliflag ;
extern void UpdateOldOffSetZeroToNewStruct(void);
void update_flashpra_from_flash(void)
{
	uint32_t  data[4];
	uint32_t  startadrr  = FLASH_USER1_START_ADDR;
	memcpy((void*)&data,(void*)startadrr,4*4);
//	if( data[0]  == FLASH_PID0 &&   data[1]== FLASH_PID1 &&
//		data[2]  == (uint32_t)sizeof(Flashpra_TypeDef)  && 
//		(data[0]^data[1]^data[2]^FLASH_PID3)  == data[3]){
//		g_FlashIsWrited = 1;	
//	}else {
//		g_FlashIsWrited  = 0;
//		return ;
//	}
    
    if( data[0]  == FLASH_PID0 &&   data[1]== FLASH_PID1){
		g_FlashIsWrited = 1;	
	}else {
		g_FlashIsWrited  = 0;
		return ;
	}
	memcpy((void*)((uint8_t*)&g_SFlashpra+4),(void*)(startadrr+4),sizeof(Flashpra_TypeDef)-4);
	/***********/	
	if(g_SFlashpra.sFac.hwResert ==0x5a5a){
		g_ucDeviceMode = g_SFlashpra.sFac.ucDeviceMode;
		g_ucFactorFlag = g_SFlashpra.sFac.ucFactorFlag;
	}
	/***********/	
	if(g_SFlashpra.sAngoffset.wResert ==0x5a5a5a5a){
        FlashMoterVariableEachCopy((void*)&g_SFlashpra.sAngoffset.hiFocInitAngValue);
	}	
	/***********/	
	if(g_SFlashpra.sPra.wResert ==0x5a5a5a5a){
        memcpy((void*)((uint8_t*)&Tuning_data+4),(void*)((uint8_t*)&g_SFlashpra.sPra+4),sizeof(Tuning_data)-4);	       
	}
	/***********/
	if(g_SFlashpra.sExpang.wResert ==0x5a5a5a5a){//期望控制姿态PID的角度
		//memcpy((void*)g_fAngleExp ,(void*)((uint8_t*)&g_SFlashpra.sExpang+4),12);	
	}
    /***********/	
 	if(g_SFlashpra.sMpu.wResert ==0x5a5a5a5a){
        
         FlashMpuVariableEachCopy(&g_SFlashpra.sMpu.hiMaxMinAccValue[0],1);  
         FlashGryListVariableEachCopy(&g_SFlashpra.sMpu.tGyroBiasList,1);
	}   
    if(g_SFlashpra.sLRS.wResert  ==0x5a5a5a5a ){
         FlashRLSVariableEachCopy((float*)&g_SFlashpra.sLRS.LRS[0],1);
    }
   
    
    
}




