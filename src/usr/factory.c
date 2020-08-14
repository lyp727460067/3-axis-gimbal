
#define DEFIN_FACTORY_EXTERN
#include "factory.h"
#include "flash.h"
#include "time.h"
#include "led.h"
#include "Control.h"
#include "AngSensor.h"
#include  "datacalibrate.h"
#include "UartPack.h"
#include "device.h"


static uint8_t ucFactoryState  ;
uint8_t CaStatesTopc;




void factory_int(void)
{
   ucFactoryState   = 0; 
}




typedef enum
{
    CALIBRATE_INIT = 0,
    CALIBRATE_STARI,
    //motor
    CALIBRATE_FOC,
    CALIBRATE_ME_RE,
    //mpu
    CALIBRATE_6PLANACC,
    CALIBRATE_ACC,
    CALIBRATE_GYR,
    //tlb5021  reserve
    CALIBRATE_WAIT, 
    CALIBRATE_END  = 0xff>>3  
}tCalibrateSates;


typedef enum
{

    TEST = 0X01,
    
}tCalibrateReturnSates;


tCalibrateSates cmCalibrateSates;
PrecisionTime_Type SCalibrateTime;

extern  uint8_t  MoterAngClibrate(uint8_t states);
//
void FactorySetGlobStates(uint8_t states)
{
    static  uint8_t  Oldstates = 0;
    if(Oldstates!=states){//在跳变的时候改变状态
        ucFactoryState = states;
    }
}

void  FactorySubMainPcb(void)
{
  
    switch(ucFactoryState){
    case CALIBRATE_INIT :

        break;
    case CALIBRATE_FOC:
        set_start_control(0);  
        set_motor_const_position(0x001); 
        set_led_state(1); 
        MoterAngClibrate(1);
		ClearCompareTime(&SCalibrateTime);
        break;
    case CALIBRATE_ME_RE:
        set_start_control(0);  
        set_motor_const_position(0x000); 
        set_led_state(2); 
        MoterAngClibrate(2);
		ClearCompareTime(&SCalibrateTime);
        break;
    case CALIBRATE_ACC:
        set_start_control(0);  
        set_led_state(3);
        set_motor_const_position(0x000);
		 ClearCompareTime(&SCalibrateTime);
        break;
    case CALIBRATE_6PLANACC:
        set_start_control(0);  
        set_led_state(3);
        set_motor_const_position(0x000);
		 ClearCompareTime(&SCalibrateTime);
    case CALIBRATE_GYR:
		set_start_control(0);  
		set_led_state(4);
   	     set_motor_const_position(0x000);
		 ClearCompareTime(&SCalibrateTime);
        break;     
    case CALIBRATE_WAIT:
        set_start_control(0);  
        set_motor_const_position(0x000); 
        set_led_state(5);
		ClearCompareTime(&SCalibrateTime);
       break;      
    case CALIBRATE_END:
        if(CompareTime1ms(&SCalibrateTime,10) == 0)break;
        ClearCompareTime(&SCalibrateTime);
        g_ucFlashWriteFlag = 1;
        update_flashclib_from_ram();
		set_start_control(1); 
        set_led_state(0);
        ucFactoryState = 0;
    break;     
    }
}
/*
*/

void FactoryMainSendToSub(void);
extern void SetMpuClibrate(uint8_t index);
extern  uint8_t GetMpuClibrateStates(void);

//CALIBRATE_FOC
void FactoryMainPcbCalibrateFoc(void)
{
    set_start_control(0);  
    set_motor_const_position(0x001); 
    set_led_state(1);
    if(MoterAngClibrate(1)==0xff){
        ucFactoryState =CALIBRATE_END;      
    }  
}

//CALIBRATE_ME_RE
void FactoryMainPcbCalibrateMeRe(void)
{
    set_start_control(0);  
    set_motor_const_position(0x000); 
    set_led_state(2);
    if(MoterAngClibrate(2)==0xff){
        ucFactoryState =CALIBRATE_END;
    } 
    
}
//CALIBRATE_ACC
void FactoryMainPcbCalibrateACC(void)
{
    set_start_control(0);  
    set_motor_const_position(0x000); 
    set_led_state(3);
    SetMpuClibrateStates(0x01);//0x01-0x40开始为ACC的命令
    if(GetMpuClibrateStates()==0xff){
		SetMpuClibrateStates(0xff);//设置状态和当前一样
        ucFactoryState =CALIBRATE_END;
    } 
}
//CALIBRATE_GYR
void FactoryMainPcbCalibrateGyr(void)
{
    set_start_control(0);  
    set_motor_const_position(0x000);
    set_led_state(4);
    SetMpuClibrateStates(0x41);//0x41-0xFE开始为GRY的命令
    if(GetMpuClibrateStates()==0xff){
		SetMpuClibrateStates(0xff);//设置状态和当前一样
        ucFactoryState =CALIBRATE_END;
    } 
}
//
uint8_t ACC6PlanIndex = 0;//上位机指示校准哪一个面
void FactoryMainPcbCalibrate6PlaneACC(void)
{
    set_start_control(0);  
    set_motor_const_position(0x000);
    set_led_state(4);
    SetMpuClibrateStates(ACC6PlanIndex);//0x41-0xFE开始为GRY的命令
    if(GetMpuClibrateStates()==0xff){
		SetMpuClibrateStates(0xff);//设置状态和当前一样
        ucFactoryState =CALIBRATE_END;
    } 

}

void  FactoryMainPcb(void)
{
    
    switch(ucFactoryState){
    case CALIBRATE_INIT:

        break;
    case CALIBRATE_FOC: 
        FactoryMainPcbCalibrateFoc();
        break;
    case CALIBRATE_ME_RE:
        FactoryMainPcbCalibrateMeRe();
        break;
    case CALIBRATE_ACC:
        FactoryMainPcbCalibrateACC();
        break;
    case CALIBRATE_6PLANACC:
        FactoryMainPcbCalibrate6PlaneACC();
        break;
    case CALIBRATE_GYR:
        FactoryMainPcbCalibrateGyr(); 
        break ;    
    case CALIBRATE_WAIT:
        set_start_control(0);
        set_motor_const_position(0x000);
        set_led_state(5);
        break;
    case CALIBRATE_END:
        
        if(CompareTime1ms(&SCalibrateTime,20) == 0)break;
        ClearCompareTime(&SCalibrateTime);
        g_ucFlashWriteFlag = 1;
        ucFactoryState = CALIBRATE_INIT;  
        update_flashclib_from_ram();
  		set_start_control(1);
        set_led_state(1); 
        break;
    default:break;
    }
   FactoryMainSendToSub();
}

/*
*出厂APP 校准
*/
#define   CALIBRATE_FOC_ANG_OFFSET       	 0X10
#define   CALIBRATE_RE_ANG_OFFSET         	 0X20
#define   CALIBRATE_MPU_1PLANACC_OFFSET		 0X28

#define   CALIBRATE_MPU_GR_6PLANACC_OFFSET	 0x30
#define   CALIBRATE_OFFSET_WAITE			 0x40
#define   CALIBRATE__OK                    	 0Xff
#define   CALIBRATE_FOC_ANG_End         	0X01



static  uint8_t  FactoryPcStates;
uint32_t FactoryPcCounter = 0;
void PCSetClibrateSates(uint8_t states)
{  
	static  uint8_t  Oldstates = 0;
	if(Oldstates!=states){//在跳变的时候改变状态
		FactoryPcStates = states;
	}
	Oldstates = states;
}
extern uint8_t GetACC6PlanidexStates(void);
uint8_t  FactoryAppUsart(void)
{
    static  uint8_t states = 0;
    switch(states) { 
    case 0:	
        if(FactoryPcStates){
			CaStatesTopc = 0;
            states = 1;
           // ucFactoryState = CALIBRATE_WAIT;
        }
        break;
    case 1:
		
        if(FactoryPcStates==CALIBRATE_FOC_ANG_OFFSET){//Foc
            CaStatesTopc = 0x10;
            ucFactoryState = CALIBRATE_FOC;
            states = 2;
        }
		
		if(FactoryPcStates==CALIBRATE_OFFSET_WAITE){
            CaStatesTopc = 0x10;
            ucFactoryState = CALIBRATE_WAIT;
            states = 2;
        }
		//之前是 只要是按那个按键就是进入那种校准
		//现在改成 必须点击准备无力后 再进入 校准陀螺仪
        /*if(FactoryPcStates==CALIBRATE_RE_ANG_OFFSET){//acc_gyr_re
           // CaStatesTopc = 0x20;
            ucFactoryState = CALIBRATE_ACC;
            states = 3;//1面校准acc
        }
        if(CALIBRATE_MPU_GR_6PLANACC_OFFSET == (FactoryPcStates&0xf0)){//acc_gyr_re
            ucFactoryState = CALIBRATE_6PLANACC;
            ACC6PlanIndex = 0x31;
            states = 10;//6面校准acc
        }*/
        break;
    case 2:
		//之前是整个状态连一起 中间不能断电 等待校准
		//现在 就是说 校准完FOC  写入FLASH  允许断电
		/*
        if(ucFactoryState == CALIBRATE_END){
             CaStatesTopc = 0x11;
             ucFactoryState = CALIBRATE_WAIT;
        }*/
		if(ucFactoryState == CALIBRATE_INIT){//等待FOC写入Flash完成后等待
             CaStatesTopc = 0x11;
             ucFactoryState = CALIBRATE_WAIT;
        }
		/*取消一面校准 直接添加一个一面校准按键
        if(FactoryPcStates==CALIBRATE_RE_ANG_OFFSET){
            CaStatesTopc = 0x21;
            ucFactoryState = CALIBRATE_ACC;
            states = 3;
        }*/
		//一面校准
		 if(FactoryPcStates==CALIBRATE_MPU_1PLANACC_OFFSET){
            CaStatesTopc = 0x21;
            ucFactoryState = CALIBRATE_ACC;
            states = 3;
         }
		//6面校准
        if(CALIBRATE_MPU_GR_6PLANACC_OFFSET == (FactoryPcStates&0xf0)){//acc_gyr_re
            ucFactoryState = CALIBRATE_6PLANACC;
            ACC6PlanIndex = 0x31;
            states = 10;//6面校准acc
        }
		//校准水平
       if(FactoryPcStates==CALIBRATE_RE_ANG_OFFSET){//gyr_re
            ucFactoryState = CALIBRATE_ME_RE;
            CaStatesTopc = 0x22;
            states = 4;
        } 
        
        break;
    case 3:
        if(ucFactoryState == CALIBRATE_END){
             ucFactoryState = CALIBRATE_ME_RE;
             states = 4;
        }
        break;
    case 4:
        if(ucFactoryState == CALIBRATE_END){
            CaStatesTopc = 0x22;
            ucFactoryState = CALIBRATE_GYR;  
            states = 0xff;
        }
        break;
    case 10:
        CaStatesTopc = 0x30|GetACC6PlanidexStates();
		//之前是等待一次性校准完成后写入flash
		//现在改成 校准完成就写入flash
		/*
        if(ucFactoryState == CALIBRATE_END){//6面校准acc完成
             ucFactoryState = CALIBRATE_WAIT;
             states = 11;
        }else{ 
           ACC6PlanIndex = (FactoryPcStates&0x0f)+0x10;
        }
		*/
		if(ucFactoryState == CALIBRATE_INIT){//6面校准acc完成 写入FLASH 完成
             ucFactoryState = CALIBRATE_WAIT;
             states = 11;
        }else{ 
           ACC6PlanIndex = (FactoryPcStates&0x0f)+0x10;
        }
        break;
    case 11: 
        CaStatesTopc = 0x37;//水平校准完成
        if(FactoryPcStates==CALIBRATE_RE_ANG_OFFSET){//gyr_re
            ucFactoryState = CALIBRATE_ME_RE;
            CaStatesTopc = 0x22;
            states = 4;//校准水平
        }   
    case 0xff:
		if(ucFactoryState==CALIBRATE_END){
			CaStatesTopc = 0x23;
		}
		if(ucFactoryState==0){
			CaStatesTopc = 0xFF;
			FactoryPcCounter = 0;
			states = 0;
			FactoryPcStates = 0;
		}	
        break;
    default:break;
    }
    return states;
}

/*
*遥控器VR 校准
*/
static uint8_t FactoryTeleStates = 0;
void  SetFactoryTele(uint8_t states)
{
    FactoryTeleStates  = states;
}
uint8_t  GetFactoryTele(void)
{
    return FactoryTeleStates ; 
}
uint8_t  FactoryTele(void)
{
    static  uint8_t states = 0;
    switch(states) { 
    case 0:
        if(FactoryTeleStates){
            states = 1;
            ucFactoryState = CALIBRATE_WAIT;
        }
        break;
    case 1:
        if(FactoryTeleStates==3){//acc
            ucFactoryState = CALIBRATE_ACC;
            states = 2;
        }
        if(FactoryTeleStates==1){//gry
            ucFactoryState = CALIBRATE_GYR;
            states = 2;
        }
        if(FactoryTeleStates==4){//fco
            ucFactoryState = CALIBRATE_FOC;
            states = 2;
        }

        break;
    case 2:
        
        if(ucFactoryState==0){
            states = 0;
            FactoryTeleStates = 0;
        }
        break;
 
    }
    return states;
    
}
/*
*遥控器通过usart校准
*/
static uint8_t UsartTeleClibState  =0;
void UsartSetTeleClib(uint8_t states)
{ 
    UsartTeleClibState  =  states ;
}
uint8_t  FactoryTeleUsart(void)
{
    static  uint8_t states = 0;
    switch(states) { 
    case 0:
        if(UsartTeleClibState){
            states = 1;
            ucFactoryState = CALIBRATE_WAIT;
        }
        break;
    case 1:
        if(UsartTeleClibState==3){//acc
            ucFactoryState = CALIBRATE_ACC;
            states = 2;
        }
        if(UsartTeleClibState==1){//GYR
            ucFactoryState = CALIBRATE_GYR;
            states = 2;
        }
        if(UsartTeleClibState==4){//foc
            ucFactoryState = CALIBRATE_FOC;
            states = 2;
        }
        if(UsartTeleClibState==2){//wait
            ucFactoryState = CALIBRATE_WAIT;
            states = 2;
        }
        break;
    case 2:
        if(ucFactoryState==0){
            states = 0;
            UsartTeleClibState = 0;
        }
        break;
    }
    return states;
}
/*
*/
void factory(void)
{   
    if(g_ucDeviceNum==0){
        uint8_t states = 0;
        FactoryTele();
        FactoryAppUsart();
        FactoryTeleUsart();
        FactoryMainPcb();
    }else {
        FactorySubMainPcb();
    }
}

/*
发送校准状态给其他PCB
*/
/*
*以下功能跟单向串口有冲突了
*用IIC 传输
*/

//void FactoryMainSendToSub(void)
//{
//    uint8_t pdata[4];
//    pdata[0] = 27;
//    pdata[2] = 0x1F;//F命令
//    pdata[3] = ucFactoryState;
//    ComX_SendPack((uint8_t*)pdata,0,4);
//
//}
extern void IICSendToCalibreteStates(uint8_t states);
void FactoryMainSendToSub(void)
{
    IICSendToCalibreteStates(ucFactoryState);
}
//陀螺仪板子给两小板发送校准状态
void FactorySubMainSetCalibreStates(uint8_t states)
{
   static  uint8_t Oldstates = 0;
   if(states!=Oldstates ){//狀態有跳变的时候赋值
        ucFactoryState = states;
   }
   Oldstates = states;

}



