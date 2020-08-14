
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
    CALIBRATE_ACC,
    CALIBRATE_GYR,
    //tlb5021  reserve
    
    CALIBRATE_END  = 0xff  
}tCalibrateSates;

tCalibrateSates cmCalibrateSates;






#define DELAYTIME5S 4000

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
        set_led_state(1);
        set_start_control(0);  
        set_motor_const_position(0x001); 
        set_init_org(0);
        break;
    case CALIBRATE_WAIT:
        set_led_state(5);
        set_start_control(0);  
        set_motor_const_position(0x000);
        break;
    case CALIBRATE_GYR:
        set_led_state(2);
        set_start_control(0);  
        set_motor_const_position(0x000);
        break;   
    case CALIBRATE_ACC_RE_GRY:
        set_init_org(1);
        set_led_state(3);
        set_start_control(0);  
        set_motor_const_position(0x000);
    break;   
    case CALIBRATE_END:
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
void ClearTimeCounter1Ms(void)
{
    TimeCounter1Ms = 0;
}
void FactoryMainSendToSub(void);
extern void SetMpuClibrate(uint8_t index);
extern  uint8_t GetMpuClibrateStates(void);

//CALIBRATE_FOC
void FactoryMainPcbCalibrateFoc(void)
{
    set_motor_const_position(0x001); 
    set_init_org(0);
    set_led_state(1);
    if(GetFocClibrateStates(1)){
        ucFactoryState =CALIBRATE_END;
    }  
}

//CALIBRATE_ME_RE
void FactoryMainPcbCalibrateMeRe(void)
{
    set_motor_const_position(0x000); 
    set_init_org(0);
    set_led_state(1);
    if(GetMeClibrateStates(1)){
        ucFactoryState =CALIBRATE_END;
    } 
    
}
//CALIBRATE_ACC
void FactoryMainPcbCalibrateACC(void)
{
    set_motor_const_position(0x000); 
    set_init_org(0);
    set_led_state(1);
    if(GetMpuClibrateStates(1)){
        ucFactoryState =CALIBRATE_END;
    } 
}
//CALIBRATE_GYR
void FactoryMainPcbCalibrateGyr(void)
{
    set_motor_const_position(0x000); 
    set_init_org(0);
    set_led_state(1);
    if(GetMpuClibrateStates(2)){
        ucFactoryState =CALIBRATE_END;
    } 
}


void  FactoryMainPcb(void)
{
    switch(ucFactoryState){
    case CALIBRATE_INIT:     
        break;
    case CALIBRATE_FOC: 
        FactoryMainPcbCalibrateFoc():
            break;
    case :CALIBRATE_ME_RE:
        FactoryMainPcbCalibrateMeRe();
        break;
    case :CALIBRATE_ACC:
        FactoryMainPcbCalibrateACC();
    case :CALIBRATE_GYR:
        FactoryMainPcbCalibrateGyr(); 
        break      
    case CALIBRATE_END:
        if(GetTime1ms(&SCalibrateTime,5) == 0)break;
        ClearTime1ms(&SCalibrateTime);
        ucFactoryState = CALIBRATE_INIT;  
        update_flashclib_from_ram();
        set_start_control(1);
        set_led_state(0);   
        break;
    default:break;
    }
   FactoryMainSendToSub();
}

/*
*出厂APP 校准
*/
uint8_t  FactoryAppUsart(void)
{
    static  uint8_t states = 0;
    switch(states) { 
    case 0:
        if(g_ucFactoryState){
            states = 1;
            ucFactoryState = CALIBRATE_WAIT;
        }
        break;
    case 1:
        if(g_ucFactoryState==CALIBRATE_FOC_ANG_OFFSET){//Foc
            ucFactoryState = CALIBRATE_FOC;
            ClearTimeCounter1Ms();
            states = 2;
        }
        if(g_ucFactoryState==CALIBRATE_RE_ANG_OFFSET){//acc_gyr_re
            ucFactoryState = CALIBRATE_ACC_RE_GRY;
            states = 3;
        }
        break;
    case 2:
        if(ucFactoryState == 10){
             ucFactoryState = CALIBRATE_WAIT;
        }
        if(g_ucFactoryState==CALIBRATE_RE_ANG_OFFSET){
             ucFactoryState = CALIBRATE_ACC_RE_GRY;
             states = 3;
        }
        break;
    case 3:
        if(ucFactoryState==0){
            states = 0;
            g_ucFactoryState = 0;
        }
        break;
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
            ucFactoryState = CALIBRATE_ACC_RE_GRY;
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
            ucFactoryState = CALIBRATE_ACC_RE_GRY;
            states = 2;
        }
        if(UsartTeleClibState==1){//acc
            ucFactoryState = CALIBRATE_GYR;
            states = 2;
        }
        if(UsartTeleClibState==4){//acc
            ucFactoryState = CALIBRATE_FOC;
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
        states  = FactoryTele();
        if(states==0){
          states =  FactoryAppUsart();
        }
        if(states==0){
          states = FactoryTeleUsart();
        }
        FactoryMainPcb();
    }else {
        FactorySubMainPcb();
    
    }
}

/*
发送校准状态给其他PCB
*/
/*
*一下功能跟单向串口有冲突了
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
void FactorySubMainSetCalibreStates(uint8_t states)
{
    static  uint8_t Oldstates = 0;
    if(states!=Oldstates ){//狀態有跳变的时候赋值
        ucFactoryState = states;
    }
    Oldstates = states;

}
void SetFactoryState(uint8_t states)
{
    ucFactoryState = states;
}
uint8_t  GetFactoryState(void)
{
    uint8_t states = ucFactoryState;
    return states ;
}

