#define DEFIN_ANGSENSOR_EXTERN
#include "commonvalue.h"
#include "AngSensor.h"
#include "IIC.h"
#include "CCS.H"
#include "time.h"
#include "IObit_remap.h"
#include "device.h"
#include "flash.h"
#include "filter.h"



PrecisionTime_Type SAngTime    ;
struct sMoterAng
{
    int16_t hiFocInitAngValue ;
    int16_t hiReInitAngValue;	
}*SMoterAng;
static  float calibratfactor;
/*
*/

void FlashMoterVariableEachCopy(void  *data,uint8_t index)
{
//    if(index){
//          memcpy((void*)&SMoterAng,data,sizeof(SMoterAng));
//    }else {
//          memcpy(data ,(void*)&SMoterAng,sizeof(SMoterAng));
//    }

}





static void Init_io(void)
{   
	#ifdef USE_MAGNETIC_SENSER
	GPIO_InitTypeDef GPIO_InitStructure;	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);		
	RCC_LSEConfig(RCC_LSE_OFF);
	PWR_BackupAccessCmd(ENABLE);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);  
	GPIO_SetBits(GPIOC,GPIO_Pin_13);
	GPIO_ResetBits(GPIOC,GPIO_Pin_14);
	GPIO_ResetBits(GPIOC,GPIO_Pin_15);	
	
	#endif
}


tFourOrder  FourOrderEmang;

static u8  Ang_Init(void)
{

	u16 datatemp = 0;
	datatemp = 0;
	CCSWrite_Data(STAT,&datatemp,1);
	datatemp = 0x00c0;
	CCSWrite_Data(ACSTAT,&datatemp,1);
	datatemp = 0x4000;
	CCSWrite_Data(MOD_1,&datatemp,1);	
	
	datatemp = 0x0805;
	CCSWrite_Data(MOD_2,&datatemp,1);//Prediction and auto-cal. mode 1: update every angle update cycle												//	(FIR_MD setting)
	
	
	SMoterAng =  (struct sMoterAng*)&g_SFlashpra.sAngoffset.hiFocInitAngValue;
  if(g_ucDeviceNum==2){
    IIR4OrderInitfpEmAng(&FourOrderEmang,0,0);
  }
  
  return 1;
}

/*
*/
void Init_AngSensor(void)
{
	#ifdef USE_MAGNETIC_SENSER	
	Init_io();
	while(Ang_Init()==0);
	Get_AngSensor();	
    calibratfactor    = 0.0109863f;
    #else  
    
	#endif
}
//static  uint8_t sbyMoterAngClibrateStates = 0;
//
//void SetMoterAngClibrateStates(uint8_t states)
//{
//    static  uint8_t  OldMoterAngClibrateStates = 0;
//    if(OldMoterAngClibrateStates!=states){//在跳变的时候改变状态
//        sbyMoterAngClibrateStates = states;
//    }
//}
//uint8_t  GetMoterAngClibrateStates(void)
//{
//    return sbyMoterAngClibrateStates;
//}
//为了后面校准磁传感器和增加状态
uint8_t  MoterAngClibrate(uint8_t states)
{
    switch(states){
    case 0:
        break;
    case 1://foc
        SMoterAng->hiFocInitAngValue = g_hi0rgangle;
        if(CompareTime1s(&SAngTime,5) == 0)return 0;
        ClearCompareTime(&SAngTime);
        return  0xff;
    case 2:
        SMoterAng->hiReInitAngValue = g_hi0rgangle;
        if(CompareTime1s(&SAngTime,2) == 0)return 0;
        ClearCompareTime(&SAngTime);       
        //sbyMoterAngClibrateStates = 0xff;
        return 0xff;

    case 3:
        break;
    default:break;
    }
}
/*
*/
int16_t ChageToInt16(int16_t value)
{
	if(value&0x4000){
		return value;
	}else {
		return value&0x7fff;
	}
}
/*
*/

void SetMoterAngCaliFactory(float factory)
{
	calibratfactor = factory;
}


uint16_t OffsetValve(int16_t Valve,int16_t initValve)
{
   
    uint16_t Valve1= 0 ,initValve1= 0;
    
    Valve1 = (uint16_t)0X4000 +Valve ;
    initValve1 =  initValve+(uint16_t)0X4000   ;
    if(Valve1>=initValve1) Valve1 =Valve1-initValve1 ;	
    else Valve1 = (uint16_t)0x7fff-initValve1+Valve1;
     
    return Valve1;
 
}
/*
*/
static int16_t Ang360to180(uint16_t ang)
{
    int16_t Rusult;
    if(ang<=(uint16_t)0x3fff){
        Rusult =ang;
    }else {
        Rusult =ang-(uint16_t)0x7fff;
    }  
    return Rusult;
}
extern int16_t get_org_angle(void);
u8 Get_AngSensor(void)
{
    
    //MoterAngClibrate();
#ifdef USE_MAGNETIC_SENSER	
    int16_t temp[2] ={0,0};
    if(CCSRead_Data(AVAL,(uint16_t*)temp,1)){
        if(temp[0]&0x8000){
            temp[0] =ChageToInt16(temp[0]);
            g_hi0rgangle   = temp[0];//OffsetValve(temp[0],0)
            g_fEMFocangle = calibratfactor*((uint16_t)0x7fff - OffsetValve(temp[0],SMoterAng->hiFocInitAngValue));
            uint16_t g_hiEMReAngletemp =OffsetValve(temp[0],SMoterAng->hiReInitAngValue); 
            int16_t g_hiEMReAngletemp1=Ang360to180(g_hiEMReAngletemp);
            if(g_ucDeviceNum==2){
                g_hiEMReAngletemp1 = (int16_t)IIR4Order(&FourOrderEmang,g_hiEMReAngletemp1);   
            }
            g_hiEMReAngle = g_hiEMReAngletemp1;
        }	
        return 1;
    }else {
        return 0;
    }
#else 
    g_hi0rgangle = get_org_angle();
		g_fEMFocangle = 	calibratfactor*OffsetValve(get_org_angle(),EMInitValve);
		g_hiEMReAngle = 	OffsetValve(get_org_angle(),EMInitValveRe);
    return 1;	
#endif
}
float Get_EMReangle(int16_t Value)
{
	return calibratfactor*Value;
}





