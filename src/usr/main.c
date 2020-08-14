#include "stm32f30x.h"
#include "time.h"
#include "led.h"
#include "AngSensor.h"
#include "pwm.h"
#include "adc.h"
#include  "Control.h"
#include "math.h"
#include "CORDIC_math.h"
#include "arm_math.h"
#include "usart.h"

#include  "datacalibrate.h"
#include "EulerQuaternion.h"
#include "IICtrancedata.h"
#include "spi.h"
#include "MPU6050.h"
#include "IObit_remap.h"
#include "IICtrancedata.h"
#include "flash.h"
#include "usart_data.h"
#include "device.h"
#include "main.h"
#include "pctogimbal.h"
#include "UartPack.h"
#include "mpuheart.h"
#include  "Control.h"
#include "factory.h"

/**/
extern void Get_SetBas(void);
extern void Start_ADC(void);
extern void IIC_Sche(void);
extern u8 Slavstart ;
extern void ComX_Init(void);
extern void ComXDebugCallBack(void);
extern void CmdProcess_DebugData(uint8_t *p_data ,uint16_t lenth);
extern void CmdProcess_Cailration(uint8_t *p_data,uint8_t lenth,uint8_t PortNum);
/**/
extern void enable_op(void);
PrecisionTime_Type  whilecircl_time;
//float circl_time;
extern u8 speederrupdataflag ;
uint8_t		UpdateReqCnt = 0;
/**/
void Get_Circltime(void)
{
  //circl_time = Get_Time_us(&whilecircl_time);
}

/**/
void ReadyToUpgrade(void)
{
  if(UpdateReqCnt < 2)
  {
    UpdateReqCnt++;
    return ;
  }
  NVIC->ICER[0] = 0xffffffff;
  NVIC->ICER[1] = 0xffffffff;
  NVIC->ICER[2] = 0xffffffff;
  NVIC->ICER[3] = 0xffffffff;
  NVIC->ICER[4] = 0xffffffff;
  NVIC->ICER[5] = 0xffffffff;
  NVIC->ICER[6] = 0xffffffff;
  NVIC->ICER[7] = 0xffffffff;
  //关闭所有外设时钟
  RCC->APB2ENR = 0;
  RCC->APB1ENR = 0;
  //复位所有外设
  RCC->APB2RSTR = 0xffffffff;
  RCC->APB1RSTR = 0xffffffff;
  RCC->APB2RSTR = 0;
  RCC->APB1RSTR = 0;
  
  __disable_fault_irq();
  
  //解锁FLASH
  FLASH_Unlock();
  //清除所有标记
  FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP|FLASH_FLAG_PGERR |FLASH_FLAG_WRPERR);	
  FLASH_ErasePage(BootloaderInfAddr);
  FLASH_Lock();
  
  //
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0);
  NVIC_SystemReset();
}



/**/
void ComXRxEvent(uint8_t *p_data,uint8_t DataLength,uint8_t PortNum)
{
    uint8_t* p_UartData;
    int32_t sa,cdl;
    switch(p_data[0]){
    case CI_SetGimbalPar:
        set_gimbalpra(p_data,DataLength);	
        CmdProcess_DebugData(p_data,DataLength);
        break;
    case  CI_CaliGimbalPar:
        CmdProcess_Cailration(p_data,DataLength,PortNum);      
        break;
    case CI_UpdataMC:
        p_UartData = &p_data[1];
        cdl = (p_UartData[3]<<8) | p_UartData[2];
        sa =  (p_UartData[cdl+5]<<24) | (p_UartData[cdl+4]<<16) | (p_UartData[1]<<8) | p_UartData[0];
        /* */
        if(cdl == 0xffff)
            cdl = 0x00;
        if(p_UartData[cdl+6] != 0x02 || cdl == 0xffff )
        {
            if(PortNum ==  0x00)
                ComX_SendPack(p_data-4,0,DataLength+4);/* 数据格式并不是串口更新格式，而是USB更新格式 所以要-4将USB数据信息加上*/
        }
        if(g_ucDeviceNum == 0x02 && PortNum == 0x01)
            USBDevice_WriteData(p_data[0],&p_data[1],DataLength-1);
        
        if(p_UartData[cdl+6] == g_ucDeviceNum )
            ReadyToUpgrade();
        
        break;
    case 0x02:/* USB转串口命令符 更新的时候用 */
        if(PortNum == 0x01)
        {/* 从串口来的USB转换命令 偏移后使用串口命令格式重新调用*/
            ComXRxEvent(&p_data[4],DataLength-4,PortNum);
        }
        break;
    case CI_UpdataDATA:
        
        break;
        
    }
  
}

/**/
int timer11 =0;
extern   uint8_t   g_InitAngleStarte  ;
extern void tuning_pid_pra(void);
extern void set_start_control(uint8_t state);
extern u8 InitStates ;
extern void get_mpudatatosen(void);
extern void control_uler(void);
extern void control_update();


extern uint8_t time500usflag;
extern void  mpu_update(void);
PrecisionTime_Type circl_time;
float time = 0;
extern void iic_tran_read(void);
extern uint8_t GetUsbConnectState(void);
uint8_t  UsbConnectStates ;
extern void UpdataMpuData(void);
void InitCalibrate(void);

int main(void)
{
//  if (FLASH_OB_GetRDP() != SET) {   
//    FLASH_Unlock();
//    FLASH_OB_Unlock();
//    
//    //FLASH_OB_WRPConfig();
//    FLASH_OB_RDPConfig(OB_RDP_Level_1);
//    //FLASH_OB_UserConfig();
//    //FLASH_OB_BORConfig();
//    
//    printf("protection done\n");
//    FLASH_OB_Launch();
//    FLASH_OB_Lock();
//    FLASH_Lock();
//  }
  //0x8004000		0x10000

  NVIC_SetVectorTable(NVIC_VectTab_FLASH, (16*1024));
  __enable_fault_irq();
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  SysTick_Config(0x00ffffff);
  //	/**/
  
  ComX_Init();
  // while(1);
  /**/
  
  //	
		
    uint16_t  conter1ms = 0,counter100us =0;
    mpu6500_init();
    TIM_Init();
    LED_Init();	
    adc_Init();	
    Init_AngSensor();	
    iic_tran_init();
    PWM_Init();
    Start_ADC(); 
    control_Init(); 
    factory_int();
#ifdef USE_MAGNETIC_SENSER
    //uart_init(115200);
#endif
    flash_Init(); 
    InitCalibrate();
    MpuHeartInit();
    TelecontrollerInit();
    if(g_ucDeviceNum==0){ 
        set_led_state(1);   
    }  
    set_start_control(1);
    while (1) 
    {	
        IIC_Sche();
        UsbConnectStates = GetUsbConnectState();
        //35us  or  15us
        ComX_Process(ComXRxEvent,ComXDebugCallBack);
        //UpdataMpuData();
        tuning_pid_pra();
        factory();    
        flash_write();    
        Get_AngSensor(); 
//        if(Time100usFlag){//500us
//            Time100usFlag = 0;
//            control();      //143us
//            iic_tran_write();   
//        }
//        
		if(Time1MsFlag){
		  Time1MsFlag = 0; 
		  control();      //143us
		  iic_tran_write(); 
		  if(++conter1ms>=5){
			conter1ms = 0;  
			MpuHeart();
		  }
		  
		  if(UsbConnectStates==0){
			//UartRec();
		  }
		  Led_Flash();
		}
    }
}

