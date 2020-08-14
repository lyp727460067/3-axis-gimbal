#define DEFIN_ANGSENSOR_EXTERN
#include "commonvalue.h"
#include "AngSensor.h"
#include "IIC.h"
#include "CCS.H"
#include "time.h"
#include "IObit_remap.h"
#include "device.h"
static int16_t EMInitValve =0;		 //Electric machinery zero position value	for FOC
static int16_t EMInitValveRe=0;	 //Electric machinery zero position value	relativity  of geography Co



static float calibratfactor = 0.0f;




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
	
	return 1;
}

void Init_AngVarilbe()
{
	g_fEMFocangle =0;
	g_hiEMReAngle =0;
}
/*
*/
void Init_AngSensor(void)
{
	#ifdef USE_MAGNETIC_SENSER	
	Init_AngVarilbe();
	Init_io();
	while(Ang_Init()==0);
	Get_AngSensor();	
    calibratfactor    = 0.0109863f;
    #else  
    set_ang_clibre(0.0878121f);//Ä¬ÈÏ²â?
	#endif
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
float globeangleBC[3] = {0};
/*
*/
void Set_EMInitValue(int16_t Value)
{
	EMInitValve = Value;
}
int16_t Get_EMInitValue(void)
{
	return EMInitValve ;
}
/*
*/
void Set_EMInitValveRe(int16_t Value)
{
	EMInitValveRe = Value;
}

int16_t Get_EMInitValveRe(void)
{
	return EMInitValveRe ;
}
/*
*/
void set_ang_offset(int16_t focang,int16_t reang)
{
    EMInitValve = focang;
    EMInitValveRe = reang;
}
void set_ang_clibre(float clibrate)
{
	calibratfactor = clibrate;
}
void set_init_org(uint8_t index)
{
    if(index){
        EMInitValveRe  = g_hi0rgangle;
    }else {
        EMInitValve = g_hi0rgangle;
    }
}


uint16_t OffsetValve(int16_t Valve,int16_t initValve)
{
    
#ifdef USE_MAGNETIC_SENSER

     uint16_t Valve1= 0 ,initValve1= 0;
     
     Valve1 = (uint16_t)0X4000 +Valve ;
     initValve1 =  initValve+(uint16_t)0X4000   ;
	 if(Valve1>=initValve1) Valve1 =Valve1-initValve1 ;	
	 else Valve1 = (uint16_t)0x7fff-initValve1+Valve1;
     
    return Valve1;
#else 
	if(Valve>=initValve)Valve  =  Valve-initValve;
	else {
		Valve  =   0xfff-initValve+Valve;	
	}
    	return Valve;
#endif	

	
}
/*
*/
extern int16_t get_org_angle(void);
u8 Get_AngSensor(void)
{
	#ifdef USE_MAGNETIC_SENSER	
	int16_t temp[2] ={0,0};
	if(CCSRead_Data(AVAL,(uint16_t*)temp,1)){
		if(temp[0]&0x8000){
			temp[0] =ChageToInt16(temp[0]);
			g_hi0rgangle   = temp[0];//OffsetValve(temp[0],0)
			g_fEMFocangle = calibratfactor*((uint16_t)0x7fff - OffsetValve(temp[0],EMInitValve));
			g_hiEMReAngle = OffsetValve(temp[0],EMInitValveRe);
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




