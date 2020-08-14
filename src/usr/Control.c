#define  DEFIN_CONTROL_EXTERN
#include "IObit_remap.h"
#include  "Control.h"
#include "AngSensor.h"
#include "pwm.h"
#include "arm_math.h"
#include "FIFO.h"
#include "adc.h"
#include "svpwm.h"
#include "PID.h"
#include "time.h"
#include "kalman.h"
#include  "datacalibrate.h"
#include "EulerQuaternion.h"
#include "IICtrancedata.h"
#include "flash.h"
#include "usart_data.h"
#include "filter.h"
#include  "device.h"
#include <string.h>
#include "math.h"
#include "pctogimbal.h"
#include "factory.h"
#include "telecontroller.h"



//控制moter使能Io
PIN_TYPE  ADP3100A_OD = {GPIOB,GPIO_Pin_0};



#define   DEVICENUM  3
#define		PID_NUM		4
static PID_Type	  SPidInstance[DEVICENUM][PID_NUM];
static PID_Type  	MoterSelfPid[3];//上电先用电机角度拉到水平位置


#define	MOTORE0			0
#define	MOTORE1			1
#define	MOTORE2			2
//相机电机
#define		MasterQPi				  SPidInstance[MOTORE0][0]
#define		MasterDPi         SPidInstance[MOTORE0][1]
#define		MasterSpeedPi			SPidInstance[MOTORE0][2]  
#define		MasterPositPi			SPidInstance[MOTORE0][3]

//中间电机
#define		Slave0QPi				SPidInstance[MOTORE1][0]
#define		Slave0DPi				SPidInstance[MOTORE1][1]
#define		Slave0SpeedPi			SPidInstance[MOTORE1][2] 
#define		Slave0PositPi			SPidInstance[MOTORE1][3]   

//底座电机
#define		Slave1QPi				SPidInstance[MOTORE2][0]
#define		Slave1DPi				SPidInstance[MOTORE2][1]
#define		Slave1SpeedPi			SPidInstance[MOTORE2][2]
#define		Slave1PositPi			SPidInstance[MOTORE2][3] 





/*
sizeof(struct pid_pra) = PID_SERI_NUMBER;
*/

/*
sizeof(struct pid_pra) = PID_PRA_NUM;
*/
#define PID_PRA_NUM    6 
struct pid_pra{
	float fP;
	float fI;
	float fD;
	float fIMax;
	float fDMax;
	uint32_t  reserve;	
};
struct PidPra_t
{
	struct pid_pra  SDeviceQPra;
	struct pid_pra  SDeviceDPra;	
	struct pid_pra  SDeviceSepPra;	
	struct pid_pra  SDevicePosPra;
	
}SPidPra[DEVICENUM];
#define	PRACOFI			100.0f


/*后面要写入flash   上位机操作*/








void Set_gpio(PIN_TYPE gpio,uint8_t states)
{
	if(states)GPIO_SetBits(gpio.GPIO,gpio.Pin);
	else GPIO_ResetBits(gpio.GPIO,gpio.Pin);
}
static void Init_io(void)
{
    
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = ADP3100A_OD.Pin/*|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10 */;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(ADP3100A_OD.GPIO, &GPIO_InitStructure);		
}
void enable_op(void)
{
	Set_gpio(ADP3100A_OD,1);
}

//初始化变量
void init_control_para(void)
{
    g_InitAngleStarte  = 0;
    g_ucDeviceMode  = 0;
    g_emUpdatePra  = FROM_DEFAULT;
    
    for(uint8_t i = 0;i<3;i++){
        g_fAngleExp[i] = 0;
    }  
    for(uint8_t i = 0;i<3;i++){
        g_fSpeedPiderr[i] = 0;
    }    
     for(uint8_t i = 0;i<3;i++){
        g_hiReceptionExp[i] = 0;
    }  
    
    
    
}


/******************************/
//出厂矫正电机的时候 用到的函数
static uint8_t StartControl = 0;
void set_start_control(uint8_t state)
{
	StartControl   = state;  
}
void set_motor_const_position(uint16_t sector)
{
    float ang = 0;
    vector dq,dq1;
    float sinev,cosv;
	
    switch(sector){
    case 0x000:
        dq.x = 1;
        dq.y = 0;		
        break;
    case 0x001:
        dq.x = 3000;
        dq.y = 0;		
        break;	
    }
    SVPWM(dq);
}
/******************************/




//从默认的 参数到pidpra 变量
void update_pid_pra_from_default(void)
{
    float c_ParList [] = 
{
	/*current pid  Q  master*/
	3.1f,0.03f,0.000f,30,30,0,
	/*current pid  D*/
	3.2f,0.05f,0.000f,30,30,0,
	/* speed  pid  master*/			
	8.01f,0.8f,5.00f,6,20,0,
	/*position pid  master*/		
	50.01f,0.00f,20.f,5,5,0,
    
    
	
	/*current pid  Q  slave0*/		
	4.5f,0.05f,0.000f,30,30,0,
	/*current pid  D*/
	3.2f,0.05f,0.000f,30,30,0,	
	/* speed  pid  slave0*/			
	20.01f,1.0f,5.00f,6,23,0,	
	/*position pid  slave0*/		
	50.01f,0.00f,25.f,5,5,0,
    
	/*current pid  Q  slave1*/		
	2.21f,0.04f,0.000f,20,20,0,
	/*current pid  D*/
	2.51f,0.05f,0.000f,20,20,0,	
	/* speed  pid  slave1*/			
	20.51f,1.f,5.00f,20,20,0,	
	/*position pid  slave1*/		
	5.01f,0.00f,10.f,5,5,0,
};
	float *	pidpra  = (float*)&SPidPra[0];
	for(int i = 0;i<sizeof(c_ParList)/4;i++){
		pidpra[i] = c_ParList[i];
	}
}


/************************************************/
#define MAX_POS	3
static void init_pid_pra_from_flash(uint8_t num)
{	
	float *	pidpra  = (float*)&SPidPra[num].SDeviceQPra;
	/*current pid  Q  */
	for(uint8_t i = 0;i<PID_SERI_NUMBER;i++){
		pidpra[i]	=  g_SPidPra.motor1pid[2-num][0][i]/PRACOFI;	
	}
	for(uint8_t i = MAX_POS;i<PID_SERI_NUMBER;i++){
		pidpra[i]	= g_SPidPra.motor1pid[2-num][0][i];	
	}
	
	/*current pid  D  */
	pidpra  = (float*)&SPidPra[num].SDeviceDPra;
	for(uint8_t i = 0;i<PID_SERI_NUMBER;i++){
		pidpra[i]	= g_SPidPra.motor1pid[2-num][1][i]/PRACOFI;	
	}
	for(uint8_t i = MAX_POS;i<PID_SERI_NUMBER;i++){
		pidpra[i]	= g_SPidPra.motor1pid[2-num][1][i];	
	}
	
	/*speed pid  */	
	pidpra  = (float*)&SPidPra[num].SDeviceSepPra;
	for(uint8_t i = 0;i<PID_SERI_NUMBER;i++){
		pidpra[i]	= g_SPidPra.speedpid[num][i]/PRACOFI;	
	}	
	
	for(uint8_t i = MAX_POS;i<PID_SERI_NUMBER;i++){
		pidpra[i]	= g_SPidPra.speedpid[num][i];	
	}
	/*position pid  */	
	pidpra  = (float*)&SPidPra[num].SDevicePosPra;
	for(uint8_t i = 0;i<PID_SERI_NUMBER;i++){
		pidpra[i]	= g_SPidPra.eulerpid[num][i]/PRACOFI;	
	}	
	for(uint8_t i = MAX_POS;i<PID_SERI_NUMBER;i++){
		pidpra[i]	= g_SPidPra.eulerpid[num][i];	
	}	
}

static void init_pid_pra_to_flash(uint8_t num)
{
	float *	pidpra  = (float*)&SPidPra[num].SDeviceQPra;
	/*current pid  Q  */
	for(uint8_t i = 0;i<PID_SERI_NUMBER;i++){
		g_SPidPra.motor1pid[2-num][0][i] 	= (uint16_t)(pidpra[i]*PRACOFI);	
	}
	for(uint8_t i = MAX_POS;i<PID_SERI_NUMBER;i++){
		g_SPidPra.motor1pid[2-num][0][i] 	= (uint16_t)pidpra[i];	
	}
	
	/*current pid  D  */
	pidpra  = (float*)&SPidPra[num].SDeviceDPra;
	for(uint8_t i = 0;i<PID_SERI_NUMBER;i++){
		g_SPidPra.motor1pid[2-num][1][i]	= (uint16_t)(pidpra[i]*PRACOFI);	
	}
	for(uint8_t i = MAX_POS;i<PID_SERI_NUMBER;i++){
		g_SPidPra.motor1pid[2-num][1][i] 	= (uint16_t)pidpra[i];	
	}	
	/*speed pid  */	
	pidpra  = (float*)&SPidPra[num].SDeviceSepPra;
	for(uint8_t i = 0;i<PID_SERI_NUMBER;i++){
		g_SPidPra.speedpid[num][i]	= (uint16_t)(pidpra[i]*PRACOFI);	
	}	
	for(uint8_t i = MAX_POS;i<PID_SERI_NUMBER;i++){
		g_SPidPra.speedpid[num][i] 	= (uint16_t)pidpra[i];	
	}
	/*position pid  */	
	pidpra  = (float*)&SPidPra[num].SDevicePosPra;
	for(uint8_t i = 0;i<PID_SERI_NUMBER;i++){
		g_SPidPra.eulerpid[num][i]	= (uint16_t)(pidpra[i]*PRACOFI);	
	}	
	for(uint8_t i = MAX_POS;i<PID_SERI_NUMBER;i++){
		g_SPidPra.eulerpid[num][i] 	= (uint16_t)pidpra[i];	
	}
}
/*
init_pid_pra_from_flash and debug 
*/
void update_pid_pra_from_flash(void)
{
    init_pid_pra_from_flash(MOTORE0);
    init_pid_pra_from_flash(MOTORE1);
    init_pid_pra_from_flash(MOTORE2);	
}
/*
update_pid_to_flash and debug 
*/
void update_pid_pra_to_flash(void)
{
    init_pid_pra_to_flash(MOTORE0);
    init_pid_pra_to_flash(MOTORE1);
    init_pid_pra_to_flash(MOTORE2);	
} 





void init_pid_pra(void)
{
	switch(g_emUpdatePra){
    case FROM_DEFAULT:
        update_pid_pra_from_default();//出厂后没有写入过参数默认读
        update_pid_pra_to_flash();//写到flash的那个 变量里
        update_exp_pra_to_flash();
		break;
    case FROM_FLASH:
    case FROM_DEBUG:
        update_pid_pra_from_flash();
        update_exp_pra_from_flash();
        

		break;		
	}	
}
/***********************************************************/






/*
state == 0   初始化
state == 1   设置
*/
#define PRA_INIT	0
#define PRA_SET	 	1
#define MAX_DQ   3000
void pid_other_init()
{
	MasterSpeedPi.DifFirst = 1;
	Slave1SpeedPi.DifFirst = 1;
	Slave0SpeedPi.DifFirst = 1;
	//QPi.DifFirst = 1;
	MasterPositPi.StatelimitFlag  = 1;
	Slave0PositPi.StatelimitFlag  = 1;		
	Slave1PositPi.StatelimitFlag  = 1;
	
	MasterSpeedPi.OutlimitFlag  = 1;
	Slave1SpeedPi.OutlimitFlag  = 1;		
	Slave0SpeedPi.OutlimitFlag  = 1;
	
  for(int i =0;i<3;i++){
      PID_Init(&MoterSelfPid[i],0.8,0.02,0);	
      
  }
  
}
void set_pid_pra(uint8_t state)
{
	float *pra =  0;
	for(uint8_t i = 0;i<DEVICENUM;i++){
		pra = ((float*)&SPidPra[i]);
		for(uint8_t j = 0;j<PID_NUM;j++){		
			if(state){
				PID_SetParam(&SPidInstance[i][j],pra[0],pra[1],pra[2]);	
			}else {			
        
				PID_Init(&SPidInstance[i][j],pra[0],pra[1],pra[2]);	
				pid_other_init();
			}
			PID_SetMax(&SPidInstance[i][j],pra[MAX_POS],pra[MAX_POS+1]);
			pra+=PID_PRA_NUM;
		}	
	}	
}
//
/*************************************************/
// 当上位机下来的数据后调用这个函数写入PID参数和更新变量

void tuning_pid_pra(void)
{
	if(g_ucUpdatePraFlag){
      g_ucUpdatePraFlag = 0;
      update_pid_pra_from_flash();
      update_exp_pra_from_flash();
      set_pid_pra(PRA_SET);
      UpdateExpAngPid();
	}
}
tFourOrder RecEMreang[3];
//控制初始化
void control_Init(void)
{	
	Init_io();
  init_control_para();
	init_pid_pra();
	set_pid_pra(PRA_INIT);
	enable_op();  
  for(int  i = 0;i<3;i++){
      IIR4OrderInit(&RecEMreang[i],0,0);
  }
  
}

 float fEmReAngle[3]= {30.f,30,30} ;//得到相对位置
static void Eul_T(float *in,float* out);
static void Gra_T(float*err, float *out);
static void Get_EM3();





//SVMPW 每个电机控制周期调用
void PWM_Circl(void)
{
    vector dq,dq1;
    vector_Q  vectortemp ;
    static uint16_t  oldI_PhaseA = 0;
    static uint16_t  oldI_PhaseB = 0;
    float	sinev,cosv;
    if(StartControl){       
        
        dq1.x = Ibase.I_PhaseA-ADCCurrent.I_PhaseA ;
        dq1.y = 0.57735026919f*(dq1.x+(Ibase.I_PhaseB-ADCCurrent.I_PhaseB )*2);
        arm_sin_cos_f32((g_fEMFocangle)*7,&sinev,&cosv);
        dq.x = cosv*dq1.x+sinev*dq1.y;
        dq.y = -sinev*dq1.x+cosv*dq1.y;		
        g_SDq  =dq;    
        dq.y =Get_PID_Po(&SPidInstance[g_ucDeviceNum][0],g_fSpeedPiderr[g_ucDeviceNum],dq.y); 
        dq.x =Get_PID_Po(&SPidInstance[g_ucDeviceNum][1],0,dq.x);
        
        
        dq1.x = (cosv*dq.x-sinev*dq.y);
        dq1.y =  (sinev*dq.x+cosv*dq.y);
        SVPWM(dq1);	
// 	vectortemp.x =  (q31_t)((cosv*dq.x-sinev*dq.y)*8);
//	vectortemp.y =  (q31_t)((sinev*dq.x+cosv*dq.y)*8);
//	SVPWMq(vectortemp);	
//       
    }else {
        g_fSpeedPiderr[g_ucDeviceNum] = 0;
    }
}



//平滑滤波
#define smothdatelenth  30
#define motercouter   3
static float smothdata[3][smothdatelenth];
static  uint16_t index[motercouter] = {0};
float  smoth(float data,uint8_t c)
{
    double sum = 0;
    smothdata[c][index[c]]  = data;
    if(++index[c]>smothdatelenth){
        index[c] = 0;
    }
    for(uint16_t i  =0;i<smothdatelenth;i++){
           sum  += smothdata[c][i];
    }
    return sum/smothdatelenth;
}




//电机相对于一个角度得到0-360°得数值
float get_offset_t(float value,float offvalue)
{
    if(value>=offvalue){
        value = value- offvalue;
    }else{
        value  =   360-offvalue+value;
    }
    if(value>180  && value<360){
		value =  value-360;
	}
    return value;
}
void Gra_T1(float*err, float *out);
static void Eul_T1(float *in,float* out);
uint8_t AngMode = 0;
uint8_t GetlowPassRecEMReangle(float * ang);
extern uint8_t  expv[];

//三种位置控制方式，0-航向跟谁  1航向锁定 2电机锁定
#define SLAVE2_INIT_MEEXPANG  30.f
void control_angle(uint8_t  Mode,float *piderr)
{
    static  uint8_t Conter = 0;
    static  float OldPidErr = 0;
    Get_EM3();   
    switch(Mode){
    case  0:	
        AngMode = 0;
        float EmTemp[3] = {fEmReAngle[0],fEmReAngle[1],fEmReAngle[2]};
        // uint8_t states =GetlowPassRecEMReangle(EmTemp);

       // Gra_T1(EmTemp,EmTemp);
       // Eul_T1(EmTemp,EmTemp); 
        
        float AngTemp[3] = {-Sensordata.roll,Sensordata.pitch,EmTemp[2]};
       // Gra_T(AngTemp,AngTemp);
       // Eul_T1(AngTemp,AngTemp); 
        
        piderr[0] = Get_PID_Po(&MasterPositPi,g_fAngleExp[1],AngTemp[0]);
        piderr[1] = Get_PID_Po(&Slave0PositPi,0,AngTemp[1]);  
        piderr[2] = Get_PID_Po(&Slave1PositPi,g_fAngleExp[0],AngTemp[2]);//Pidyawerr	     
        break;
    case 1:     
        AngMode = 1;
        piderr[0] = Get_PID_Po(&MasterPositPi,g_fAngleExp[1],-Sensordata.roll);
        piderr[1] = Get_PID_Po(&Slave0PositPi,g_fAngleExp[0],Sensordata.pitch);
        piderr[2] = Get_PID_Po(&Slave1PositPi,g_fAngleExp[2],Sensordata.yaw);
        
        break;
    case 2: 
        AngMode = 2;
        piderr[0] = Get_PID_Po(&MasterPositPi,0,fEmReAngle[0]); 
        piderr[1] = Get_PID_Po(&Slave0PositPi,0,fEmReAngle[1]);  
        piderr[2] = Get_PID_Po(&Slave1PositPi,SLAVE2_INIT_MEEXPANG,fEmReAngle[2]);  
        break;
           
    }
    

}

//void control_initang_start(float *piderr)
//{
//    static  uint16_t counter    =0; 
//    uint8_t errthro[3] = {0};
//    counter++;
//    control_angle(2,piderr);
//    
//    for(int i=0;i<3;i++){
//        if(smoth(fabs(fEmReAngle[i]),i) <=1.5f){
//            if(counter>=smothdatelenth){
//                counter = smothdatelenth+1;
//                errthro[i] = 1;
//            }  
//        }       
//    }
//    if((errthro[0]&errthro[1]&errthro[2]) == 0x01){
//        if(g_InitAngleStarte==0){
//            g_InitAngleStarte =1;                
//        }
//    }
//}

#define EM_ZERO_POSITION_HOLD   1000
void control_initang_start(float *piderr)
{  
    static  uint16_t ZeroConter[3]={0};
    control_angle(2,piderr);
    for(int i =0;i<3;i++){
        float HoldErr = 1.0f;
        if(i==2){
             HoldErr = SLAVE2_INIT_MEEXPANG; 
        }
        if(fabs(fEmReAngle[i])<HoldErr){
            ZeroConter[i]++;
        }else {
            ZeroConter[i] = 0;
        }
    }
	if(g_InitAngleStarte==0){
	  g_InitAngleStarte =1;                
	}
    if(ZeroConter[0]>=EM_ZERO_POSITION_HOLD && ZeroConter[1]>=EM_ZERO_POSITION_HOLD &&
         ZeroConter[2]>=EM_ZERO_POSITION_HOLD ){       
        if(g_InitAngleStarte==0){
            g_InitAngleStarte =1;                
        }
    }
    
}

extern uint8_t GetTempeConstantFlag(void);

uint8_t  conronl_ready(float *piderr)
{
    static  uint32_t counter   = 0;
    
    TeleExpAng();
    TeleCalibrere(); 
    
    
    if(GetEuler()==0)return 0;    
    if(g_InitAngleStarte!=3){ 
        control_initang_start(piderr);
    }else {
        control_angle(g_ucDeviceMode,piderr);  
    } 
    switch(g_InitAngleStarte)
    {
    case 0: 
         set_led_state(3);
        break;
    case 1:  
        if(GetTempeConstantFlag() ){
            g_InitAngleStarte =4;   
        }
         break;
    case 4:
        
        if(Init_ulerquaternion()){
          g_InitAngleStarte =2; 
         }
        break;
    case 2:
        if(++counter>=1000){
            g_InitAngleStarte =3;
            SetEulerFixPare(1);  
            set_led_state(4);    
            counter =0;
        }
        break;
    case 3:
         
         break;
    default:break;
    } 
   return 1;
}


void control_restart(void)
{
    static uint8_t states = 0;
    static uint16_t coutern  =0; 
    uint8_t SuperFlag = 0;
    switch(states ){
    case 0:
        if(fabs(MasterSpeedPi.inter)>=(MasterSpeedPi.MaxLimit-50)){
            SuperFlag =1;
        }
        if(fabs(Slave0SpeedPi.inter)>=(Slave0SpeedPi.MaxLimit-50)){
            SuperFlag =1;
        }
        if(fabs(Slave1SpeedPi.inter)>=(Slave1SpeedPi.MaxLimit-50)){
            SuperFlag =1;
        }
        if(SuperFlag ){
            if(++coutern >=500){
                states  =1;
                coutern  = 0;
            }
        }else {
            coutern  =0;
        }
        
        break;
    case 1:
        if(++coutern >=1500){
            states  =0;
            coutern  = 0;
        }
        g_fSpeedPiderr[0] = 0;
        g_fSpeedPiderr[1] = 0;
        g_fSpeedPiderr[2] = 0; 
        break;
    case 2:   
        break;
    }
 

}
void  TraceE2B(float *in ,float* out);
void EelAllTra(float *err,float* out);

int16_t hiRecEMReangle[3];


void control(void)
{	
    float Piderr[3];
    static  u8 reserve = 0;
   // Piderr[1] = Get_EMReangle(IIR4Orderfp(&RecEMreang[1],hiRecEMReangle[1]));
    if(g_ucDeviceNum!=0)return ;

    
    float Gro_Trac[3] = {Sensordata.gx,-Sensordata.gz,Sensordata.gy};
    if(conronl_ready(Piderr)==0)return ;//IIC 读到的 电机角度刚好收到
    // Gra_T1(Piderr,Piderr);
    if(AngMode!=2){
       //TraceE2B(Piderr,Piderr);
        //Gra_T(Piderr,Piderr); 
               //Gra_T(Piderr,Piderr);
               //Eul_T(Piderr,Piderr); 
    }
    Gra_T(Gro_Trac,Gro_Trac);
    
    float  g_fSpeedPiderrtemp[3];
    
    g_fSpeedPiderrtemp[0] =  -Get_PID_Po(&MasterSpeedPi,Piderr[0],Gro_Trac[0]);	//gy roll
    g_fSpeedPiderrtemp[1] =  -Get_PID_Po(&Slave0SpeedPi,Piderr[1],Gro_Trac[1]);	//gx PITCH
    g_fSpeedPiderrtemp[2]  = -Get_PID_Po(&Slave1SpeedPi,Piderr[2],Gro_Trac[2]);	//gz yaw
    //Gra_T(g_fSpeedPiderrtemp,g_fSpeedPiderrtemp);
    //Eul_T(g_fSpeedPiderrtemp,g_fSpeedPiderrtemp);
    
    for(int i=0;i<3;i++){
        
        g_fSpeedPiderr[i] = g_fSpeedPiderrtemp[i];//Get_PID_Po(&MoterSelfPid[i],g_fSpeedPiderrtemp[i],g_fSpeedPiderr[i]);
    }
    
    //uint8_t in = TeleCalibrere();  
    //    in = UsartTeleClib();
    //    if(g_InitAngleStarte==3){
    //        if(in==0){
    //           // control_restart();
    //        }  
    //    }
   
  
}



uint8_t GetlowPassRecEMReangle(float * ang)
{
    static  uint16_t Conter = 0;
    Conter ++ ;
    uint8_t rusult = 0;
    for(int i = 0;i<3;i++){
        ang[i] = Get_EMReangle(IIR4Order(&RecEMreang[i],g_hiRecEMReangle[i]));
    }
  return rusult;

}
static void Get_EM3()
{
    u8 i = 0;
    int16_t temp;
    g_hiRecEMReangle[0] = g_hiEMReAngle;
    for(i=0;i<3;i++){
//        if(g_hiRecEMReangle[i]<=(uint16_t)0x3fff){
//            hiRecEMReangle[i] =g_hiRecEMReangle[i];
//        }else {
//            hiRecEMReangle[i] =g_hiRecEMReangle[i]-(uint16_t)0x7fff;
//        }  
        fEmReAngle[i] =  Get_EMReangle(g_hiRecEMReangle[i]);
    }
}
//y--> pitch x-->roll  z -yaw
static void EelAllTra(float *err,float* out)
{
    float	sine1,cosv1;	//x 
    float	sine2,cosv2;	//y
    float	sine3,cosv3;	//y
    arm_sin_cos_f32((Sensordata.roll),&sine1,&cosv1);
    arm_sin_cos_f32(Sensordata.pitch,&sine2,&cosv2);
    arm_sin_cos_f32(Sensordata.yaw,&sine3,&cosv3);
    
    float gxt,gyt,gzt;
    
    gxt =	 cosv2*cosv3*err[0] +(-cosv1*sine3+sine1*sine2*cosv3)*err[1]+(sine1*sine3+cosv1*sine2*cosv3)*err[2];
    gyt = sine2*sine3*err[0]+  (cosv1*cosv3+sine1*sine2*sine3)*err[1] +(-sine1*cosv3+cosv1*sine2*sine3)*err[2];  
    gzt = 	-sine2*err[0] +sine1*cosv2*err[1]+cosv1*cosv2*err[2];
   
//    gxt =	  cosv2*cosv3*err[0] + sine2*sine3*err[1]-sine2*err[2]  ;	
//    gyt =   (-cosv1*sine3+sine1*sine2*cosv3)*err[0] +(cosv1*cosv3+sine1*sine2*sine3)*err[1] +sine1*cosv2*err[2];
//    gzt = 	(sine1*sine3+cosv1*sine2*cosv3)*err[0]  +(-sine1*cosv3+cosv1*sine2*sine3)*err[1]+cosv1*cosv2*err[2];  
//    
    *(out) = gxt;	
    *(out+1) = gyt;
    *(out+2) = gzt;//-sine2*gxt;
}
static void Eul_T(float *in,float* out)
{
  float	sinex,cosvx;	//x 
	float	siney,cosvy;	//y                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
	float	gxt,gyt,gzt;
	float	rolltemp,pitchtemp;
	
	float gx =	*(in);
	float gy = 	*(in+1);
	float gz = 	*(in+2);
    
	pitchtemp =  Sensordata.pitch;//y
	rolltemp =   -Sensordata.roll;//x 
	

	arm_sin_cos_f32(rolltemp,&sinex,&cosvx);	
	arm_sin_cos_f32(pitchtemp,&siney,&cosvy);		
	
	gxt = gx				;//	-siney*(gz);
	gyt =		cosvx*(gy) +	sinex* (gz); 
	gzt = 		-cosvx*sinex*(gy) +   cosvx*cosvy* (gz);	
	*out 	 = gxt;
	*(out+1) = gyt;
	*(out+2) = gzt;	
    
}
//[                  cos1*cos2,                 -cos1*sin2,      -sin1]
//[ cos2*sin0*sin1 - cos0*sin2, cos0*cos2 - sin0*sin1*sin2, -cos1*sin0]
//[ sin0*sin2 - cos0*cos2*sin1, cos0*sin1*sin2 - cos2*sin0,  cos0*cos1]
 void Gra_T(float*in, float *out)
{ 
    float	sine1,cosv1;	//x 
    float	sine2,cosv2;	//y
    float	sine3,cosv3;	//y
    
    arm_sin_cos_f32(fEmReAngle[0],&sine1,&cosv1);
	  arm_sin_cos_f32(fEmReAngle[1],&sine2,&cosv2);
   // arm_sin_cos_f32(fEmReAngle[1],&sine3,&cosv3);
    
	float gxt,gyt,gzt;
	gxt =	*in;
	gyt = 	 *(in+1);
	gzt = 	*(in+2);
	
	*(out) = gxt;	
//	*(out+1) = cosv1*cosv2*gyt-sine1*cosv2*gzt;
//	*(out+2) = sine1*gyt*cosv2+cosv1*cosv2*gzt;//-sine2*gxt;	
	*(out+1) = cosv1*gyt-sine1*gzt;
	*(out+2) = sine1*cosv2*gyt+cosv2*cosv1*gzt+sine2*gxt;	
  
//  	*(out) = gxt;	
//	*(out+1) = cosv1*gyt-sine1*cosv2*gzt;
//	*(out+2) = (cosv3*sine1+ cosv1*sine2*sine3)*gyt   +cosv2*cosv1*gzt;//-sine2*gxt;	
}




void Gra_T1(float*err, float *out)
{ 

	// 	 z-axis |  /   x-axis     elec-me   coodi  	
	//          | /
	//    ______|/
	//	  y-axis 	  	
	// 		   	
	//   
	// 		   /   x-axis     mpu6500   coodi  	
	//     	  /
	//  ____ /	 z-axis
	//      | 	  	
	//	    |
	//      | y-axis 
    float	sine1,cosv1;	//x 
    float	sine2,cosv2;	//y
    arm_sin_cos_f32(fEmReAngle[0],&sine1,&cosv1);
	//arm_sin_cos_f32(fEmReAngle[1],&sine2,&cosv2);
	float gxt,gyt,gzt;
	gxt =	*err;
	gyt = 	 *(err+1);
	gzt = 	*(err+2);
	
	*(out) = gxt;	
	*(out+1) = cosv1*gyt+sine1*gzt;
	*(out+2) = -sine1*gyt+cosv1*gzt;//-sine2*gxt;	
}



static void Eul_T1(float *in,float* out)
{
	float	sinex,cosvx;	//x 
	float	siney,cosvy;	//y                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
	float	gxt,gyt,gzt;
	float	rolltemp,pitchtemp;
	
	float gx =	*(in);
	float gy = 	*(in+1);
	float gz = 	*(in+2);
    
	pitchtemp =  Sensordata.pitch;//y
	rolltemp =   -Sensordata.roll;//x 
	
  
	arm_sin_cos_f32(rolltemp,&sinex,&cosvx);	
	arm_sin_cos_f32(pitchtemp,&siney,&cosvy);		

    
	gxt = gx					;
	gyt =		cosvx*(gy) -	sinex* (gz); 
	gzt = 		sinex*(gy) +    cosvx* (gz);	
    
	*out 	 = gxt;
	*(out+1) = gyt;
	*(out+2) = gzt;	
    
}
void  TraceE2B(float *in ,float* out)
{
    float gxt = in[0];
    float gyt = in[1];
    float gzt = in[2];
    
    
    float  pitchtemp =   -Sensordata.pitch;//y
    float  rolltemp =    -Sensordata.roll;//x 
    float	sine1,cosv1;	//x 
    float	sine2,cosv2;	//y
    
    arm_sin_cos_f32(rolltemp,&sine1,&cosv1);	
    arm_sin_cos_f32(pitchtemp,&sine2,&cosv2);	
    
    
    out[0] = gxt;
    out[1] =  cosv1*gyt+sine1*gzt; 
    out[2] = -sine1*gyt+cosv1*gzt;
    
        
//    out[0] = cosv2*gxt-sine2 *gzt;
//    out[1] = sine1*sine2*gxt +cosv1*gyt+sine1*cosv2*gzt; 
//    out[2] = cosv1*sine2*gxt-sine1*gyt+cosv1*cosv2*gzt;
//    
//    out[0] =  (0.5 - q2*q2-q3*q3)*gxt+ (q1*q2+q0*q3)*gyt+ (q1*q3 - q0*q2) *gzt;
//    out[1] = (q1*q2-q0*q3)*gxt+ (0.5f-q1*q1-q3*q3)*gyt+ (q2*q3+q0*q1) *gzt;  
//    out[2] =  (q1*q3+q0*q2) *gxt+(q2*q3-q0*q1)*gyt+(0.5f-q1*q1-q2*q2)*gzt;
    
    
//    
//    (0.5 - q2*q2-*q3*q3)*gxt+(q1*q2-q0*q3)*gyt+*(q1*q3+q0*q2)*gzt; 
//    (q1*q2+q0*q3)       *gxt+(0.5f-q1*q1-q3*q3)*gyt+(q2*q3-q0*q1)*gzt;
//    (q1*q3 - q0*q2)     *gxt+(q2*q3+q0*q1)*gyt+(0.5f-q1*q1-q2*q2)*gzt;

}