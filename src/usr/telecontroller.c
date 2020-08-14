#include "telecontroller.h"
#include  "Control.h"
#include  "datacalibrate.h"
#include "PID.h"
#include "pctogimbal.h"
#include "math.h"


#define EXP_DEAD_WITH       100
#define TIMECONTER_CLIBRARE 5000
#define  PWMINPUTTIMER  TIM4
uint16_t Capture1  = 1500;
uint16_t Capture2  = 1500;

static uint8_t g_ExpPra[4][3] = 
{
    40,0,0,   //最大限制角度
    40,90,0,//最小限制角度
    1,1,1, //灵敏度 
    100,100,100//死取大小
};
#define  g_ucEexLimitMax        ((uint8_t*)(&g_ExpPra[0][0]))
#define  g_ucEexLimitMin        ((uint8_t*)(&g_ExpPra[1][0]))
#define  g_ucRoatFactor          ((uint8_t*)(&g_ExpPra[2][0]))
#define  g_ucEexDeadReagion      ((uint8_t*)(&g_ExpPra[3][0]))


/***********************************************************/
//更新期望角度的设置  到flash  和flash到变量
void update_exp_pra_from_flash(void)
{
    uint8_t temp[12];     
    memcpy((void*)&temp[0],(void*)&g_SPidPra.ucExpSet[0][0],12);
    for(uint8_t i =0;i<3;i++){//之前写好了，数据格式不一样转换一下
        g_ExpPra[0][i]= temp[i*2];
        g_ExpPra[1][i]= temp[i*2+1];
        g_ExpPra[2][i]= temp[6+i];
        g_ExpPra[3][i]= temp[9+i];
    }
     
}
void update_exp_pra_to_flash(void)
{
    uint8_t temp[12];  
    for(uint8_t i =0;i<3;i++){
        temp[i*2]  =g_ExpPra[0][i];
        temp[i*2+1]=g_ExpPra[1][i];
        temp[6+i]  =g_ExpPra[2][i];
        temp[9+i]  =g_ExpPra[3][i];
    }
    memcpy((void*)&g_SPidPra.ucExpSet[0][0],(void*)&temp[0],12);	
}
/***********************************************************/






void InitExpAngPidPara(void);

void TelecontrollerInit(void)
{
/***************************/
#ifdef IS_MAIN_CONTROLLOR
    NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
     //PWM1--->TIM3_CH2 -->PB5  2axis
    //PWM2--->TIM3_CH1-->PB4
    
    
    //PWM1--->TIM4_CH4 -->PB9   3axis
    //PWM2--->TIM4_CH3-->PB8
    
    GPIO_InitTypeDef  GPIO_InitStructure;
    TIM_ICInitTypeDef   TIM_ICInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 
	/* Time base configuration */  
	TIM_TimeBaseStructure.TIM_Period =  0xFFFF-1 ;  
	TIM_TimeBaseStructure.TIM_Prescaler =71 ;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(PWMINPUTTIMER, &TIM_TimeBaseStructure);
	
    /* GPIOA clock enable */ 
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    /* TIM2 channel 2 pin (PA.01) configuration */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8|GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
    GPIO_Init(GPIOB, &GPIO_InitStructure);
  
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_2);
    
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0F;
    TIM_ICInit(PWMINPUTTIMER, &TIM_ICInitStructure);
    
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
    TIM_ICInit(PWMINPUTTIMER, &TIM_ICInitStructure);   
    /* TIM enable counter */
    TIM_Cmd(PWMINPUTTIMER, ENABLE);

    

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);   
  
	//TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); 
	

    /* Enable the CC2 Interrupt Request */
    TIM_ITConfig(PWMINPUTTIMER, TIM_IT_CC3, ENABLE);	
    TIM_ITConfig(PWMINPUTTIMER, TIM_IT_CC4, ENABLE);		
#else 
    InitExpAngPidPara();
#endif

}
#define  CaliPidPos_p    0.001f
#define  CaliPidPos_i    0.0001
#define  CaliPidPos_d    0
PID_Type ExpAng[3];


//SD2 的时候取消屏蔽
void UpdateExpAngPid(void)
{
    for(int i =0;i<3;i++){     
        PID_SetParam(&ExpAng[i],CaliPidPos_p*g_ucRoatFactor[i],\
            CaliPidPos_i*g_ucRoatFactor[i],CaliPidPos_d*g_ucRoatFactor[i]);	
    } 
}

void InitExpAngPidPara(void)
{
    for(int i =0;i<3;i++){
        PID_Init(&ExpAng[i],CaliPidPos_p,CaliPidPos_i,CaliPidPos_d);
		PID_SetMax(&ExpAng[i],300,300);
    }
}
extern void  SetFactoryTele(uint8_t states);
extern uint8_t   GetFactoryTele(void);
static   float g_fAngleExpTemp[3] = {0};
void ClearExpAngPid();
uint8_t  TeleCalibrere(void)
{
    static  u8 states = 0;
	static  uint16_t oldg_hiReceptionExp[2]={0};
    static  uint16_t calibconter  = 0;
    switch(states )
    {
    case 0:
        if(g_hiReceptionExp[0]>=300   &&  g_hiReceptionExp[1]>=300 ){
            states = 2;
        }
        if((g_hiReceptionExp[0]<=-300)   &&  (g_hiReceptionExp[1]<=-300) )  {
            states = 1;
        }
        oldg_hiReceptionExp[0] = 0;
        oldg_hiReceptionExp[1] = 0;
        calibconter = 0;
        break;
    case 1:
    case 2:
        if(abs(g_hiReceptionExp[0])<=300 || abs(g_hiReceptionExp[1])<=300){
            calibconter  = 0;
            states = 0;
        }
        int x = abs(g_hiReceptionExp[0]) - oldg_hiReceptionExp[0];
        int y = abs(g_hiReceptionExp[1]) - oldg_hiReceptionExp[1];
        if(abs(x)<=2  &&  abs(y)<=2){
            calibconter  ++;
        }else {
            calibconter  = 0;
        }
        if(calibconter>=TIMECONTER_CLIBRARE){
            calibconter = 0;
            if(states==2){
                SetFactoryTele(1);//gry
            }else {
                SetFactoryTele(2);//ACC
            }
            states+=2;
        }
        break;
    case 3://Acc
        if(g_hiReceptionExp[1]>= 300){ 
            if(++calibconter>=100){
                SetFactoryTele(3);//ACC
                states = 5;
            }
        }else {
            calibconter = 0;
        }
        break;
    case 5:
    case 4://gyro
        g_fAngleExp[0] = 0;
        g_fAngleExp[1] = 0;
		g_fAngleExpTemp[0] = 0;
		g_fAngleExpTemp[1] = 0;
		ClearExpAngPid();	
        if(GetFactoryTele()==0){ 
            states  =0;
        }
        break;  
    }
    for(int i = 0;i<2;i++){
        oldg_hiReceptionExp[i]  =abs(g_hiReceptionExp[i]);
    } 
    return states;

}






uint8_t  TeleCalibrereSD2(void)
{
    static  u8 states = 0;
    static  uint16_t oldg_hiReceptionExp[2]={0};
    static  uint16_t calibconter[2]  = {0};
    static  uint8_t  xordir[2] = {0};
    static  uint8_t  oldxordir[2] = {0};
    static  uint8_t  xordirconter[2] = {0};
    
    switch(states )
    {
    case 0:
        for(int i = 0;i<2;i++){
            if(g_hiReceptionExp[i]<= -300){
                xordir[i] = 0;
            }else if(g_hiReceptionExp[i]>=300){
                xordir[i] = 1;
            }
            if(abs(g_hiReceptionExp[1-i]<100) ){
                if(xordir[i]^oldxordir[i]){
                    if(++calibconter[i] <500){
                        calibconter[i] = 0;
                        xordirconter[i]++;
                        if(xordirconter[i] >=10){   
                            calibconter[i] = 0;
                            if(i){
                                SetFactoryTele(2);//gry
                                states  = 4;
                            }else {
                                SetFactoryTele(2);//无力
                                states  = 3;
                            }
                            
                        }
                    }else {
                        calibconter[i] = 0;
                        xordirconter[i] = 0;
                    }
                }else {
                    calibconter[i] ++;
                }
            }else{
                calibconter[i] = 0;
            }
            oldxordir[i]  = xordir[i];  
        }
        break;
    case 1:
    case 2:
        break;
    case 3://Acc
        if(g_hiReceptionExp[1]>= 300){ 
            if(++calibconter[1]>=100){
               SetFactoryTele(3);//acc
                states = 5;
            }
        }else {
            calibconter[1] = 0;
        }
        break;
    case 5:
    case 4://gyro
        g_fAngleExp[0] = 0;
        g_fAngleExp[1] = 0;	
        if(GetFactoryTele()==0){ 
            states  =0;
        }
        for(int i = 0;i<2;i++){
            oldxordir[i]  = xordir[i] = 0;
            calibconter[i] = 0;
            xordirconter[i] = 0;
        }
        break;  
    }
    return states;
}

extern uint8_t  expv[];
void ClearExpAngPid(void)
{
	InitExpAngPidPara();
	UpdateExpAngPid();

}
void TeleExpAng(void)
{
static  uint8_t    oldexpdirection[3] ={0,0};
#define  exptemp    g_fAngleExpTemp
    uint8_t  expdirection[3] ={0,0};
    int16_t  golobaexp[3];
    float  attangle[3];
	
    for(uint8_t i = 0;i<3;i++){  
        g_ucEexDeadReagion[i] = EXP_DEAD_WITH;
		golobaexp[i]  = g_hiReceptionExp[i];
		
		
        if(abs(golobaexp[i])>g_ucEexDeadReagion[i]){	
            if(golobaexp[i] >0){
                expdirection[i] = 1;
            }else {
                expdirection[i] = 0;
            }
			/*20180531 SD4客户反应SD3控制太硬
            if(expdirection[i]){
				g_fAngleExp[i] +=   ((golobaexp[i]-g_ucEexDeadReagion[i])*g_ucRoatFactor[i]/10000.f) ; 
			}else {
				g_fAngleExp[i] +=   ((golobaexp[i]+g_ucEexDeadReagion[i])*g_ucRoatFactor[i]/10000.f) ; 
			}
			 oldexpdirection[i] = expdirection[i]; 
			
			 if(g_fAngleExp[i] <= -g_ucEexLimitMax[i]){
                g_fAngleExp[i] =  -g_ucEexLimitMax[i];
            }
            if(g_fAngleExp[i] >= g_ucEexLimitMin[i]){
                g_fAngleExp[i] = g_ucEexLimitMin[i];
            } 
			*/		
			
			//
			//1的是俯仰
			if(i==1){
				if(expdirection[i]){
					exptemp[i] +=  ((golobaexp[i]-g_ucEexDeadReagion[i])/10000.f) ; 
				}else {
					exptemp[i] +=  ((golobaexp[i]+g_ucEexDeadReagion[i])/10000.f) ; 
				}
			}else{
				
				if(expdirection[i]){
					exptemp[i] +=   ((golobaexp[i]-g_ucEexDeadReagion[i])*g_ucRoatFactor[i]/10000.f) ; 
				}else {
					exptemp[i] +=   ((golobaexp[i]+g_ucEexDeadReagion[i])*g_ucRoatFactor[i]/10000.f) ; 
				}
			}
			if(exptemp[i] <= -g_ucEexLimitMax[i]){
                exptemp[i] =  -g_ucEexLimitMax[i];
            }
            if(exptemp[i] >= g_ucEexLimitMin[i]){
                exptemp[i] = g_ucEexLimitMin[i];
            } 
			if(i!=1){	
				g_fAngleExp[i] = exptemp[i];	
			}
 	

	
			//
        }    
    }
	
	
	float err = exptemp[1]-g_fAngleExp[1];	
	if(fabs(err)>0.1f){
		 g_fAngleExp[1] = Get_PID_Po(&ExpAng[1],exptemp[1],g_fAngleExp[1]);
	}		
	
	if(g_fAngleExp[1] <= -g_ucEexLimitMax[1]){
		g_fAngleExp[1] =  -g_ucEexLimitMax[1];
	}
	if(g_fAngleExp[1] >= g_ucEexLimitMin[1]){
		g_fAngleExp[1] = g_ucEexLimitMin[1];
	} 
	
}
//distance of travel
void TeleExpAngSD2(void)
{
static  uint32_t   couter1m = 0;
static  uint8_t oldexpdirection[3] ={0,0};
static  uint8_t states =0;
static  uint16_t conter_angcontrl = 0;
static  int16_t oldgolobaexp[3] = {0};

  float exptemp[3] = {0};
    uint8_t expdirection[3] ={0,0};
    int16_t  golobaexp[3];
    float  attangle[3];

    golobaexp[0] =  g_hiReceptionExp[0];
    golobaexp[1] =  g_hiReceptionExp[1];
    couter1m++;
    if(couter1m<2000)return ;   
    couter1m= 2001;
    if(states==0){
        if(abs(golobaexp[0])<=100 && abs(golobaexp[1]) <=100){
            conter_angcontrl++;  
        }else {
            conter_angcontrl = 0;  
        }
        if(conter_angcontrl>=2000){
            states = 1;
            conter_angcontrl  =0;
        }
       states=1;
    }else if(states==1){
        
        for(uint8_t i = 0;i<2;i++){  
            g_ucEexDeadReagion[i]=100;
            if(abs(abs(golobaexp[i])-abs(oldgolobaexp[i]))<4){
                golobaexp[i]  = oldgolobaexp[i];
            }else {
                oldgolobaexp[i] = golobaexp[i];
            
            }
            if(abs(golobaexp[i])>g_ucEexDeadReagion[i]){
                if(golobaexp[i]<0){
                    exptemp[i] = -g_ucEexLimitMax[i]*(abs(golobaexp[i])-g_ucEexDeadReagion[i])/(400-g_ucEexDeadReagion[i]);
                }else {
                    exptemp[i] =   g_ucEexLimitMin[i]*(abs(golobaexp[i])-g_ucEexDeadReagion[i])/(400-g_ucEexDeadReagion[i]);
                }
            }else {
                exptemp[i] = 0;
            }  
            if(i==1){
                golobaexp[i] =  golobaexp[i]+400;
                 exptemp[i] =   (g_ucEexLimitMin[i]+g_ucEexLimitMax[i])
                                *(abs(golobaexp[i]))/(800);
            
            }      
            float err = exptemp[i]-g_fAngleExp[i];
            if(fabs(err)>0.1f){
                g_fAngleExp[i] = Get_PID_Po(&ExpAng[i],exptemp[i],g_fAngleExp[i]);
            }
            if(g_fAngleExp[i] <= -g_ucEexLimitMax[i]){
                g_fAngleExp[i] =  -g_ucEexLimitMax[i];
            }
            if(g_fAngleExp[i] >= g_ucEexLimitMin[i]){
                g_fAngleExp[i] = g_ucEexLimitMin[i];
            } 
            
        }       
    }
    
    
    
}

void get_capture(uint8_t cc)
{
static   uint16_t IC2ReadValue11 = 0,IC2ReadValue12 = 0;
static  uint16_t IC2ReadValue21 = 0,IC2ReadValue22 = 0;
static uint8_t CaptureNumber1 = 0,CaptureNumber2 = 0;
static uint8_t cc1state= 0;
static uint8_t cc2state= 0;
    if(cc&0x01){
        switch(cc1state){
        case 0:
            if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8)){
                /* Get the Input Capture value */
                IC2ReadValue11 = TIM_GetCapture3(PWMINPUTTIMER);
                cc1state =1;
            }
        case 1:
            if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8)==0){               
                /* Get the Input Capture value */
                IC2ReadValue12 = TIM_GetCapture3(PWMINPUTTIMER);   
                CaptureNumber1 =0;
                /* Capture computation */
                if (IC2ReadValue12 > IC2ReadValue11) {
                    Capture1 = (IC2ReadValue12 - IC2ReadValue11); 
                }
                else if (IC2ReadValue12 < IC2ReadValue11){
                    Capture1= ((0xFFFF - IC2ReadValue11) + IC2ReadValue12); 
                }
                else{
                    Capture1= 0;
                }
                 cc1state = 0;
            }      
        }  
    }
    if(cc&0x02){   
        switch(cc2state){
        case 0:        
            if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9)){
                    /* Get the Input Capture value */
                IC2ReadValue21 = TIM_GetCapture4(PWMINPUTTIMER);
                cc2state = 1;
            }   
        case 1:
            if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9)==0){
                /* Get the Input Capture value */
                IC2ReadValue22 = TIM_GetCapture4(PWMINPUTTIMER);   
                /* Capture computation */
                if (IC2ReadValue22 > IC2ReadValue21) {
                    Capture2 = (IC2ReadValue22 - IC2ReadValue21); 
                }
                else if (IC2ReadValue22 < IC2ReadValue21){
                    Capture2 = ((0xFFFF - IC2ReadValue21) + IC2ReadValue22); 
                }
                else{
                    Capture2= 0;
                }
                cc2state = 0;
            }
        }      
   
    }
}
/*

*/
void TIM4_IRQHandler(void)
{ 
    uint8_t cc = 0;
    if(TIM_GetITStatus(PWMINPUTTIMER, TIM_IT_CC3) == SET){ 
        /* Clear TIM1 Capture compare interrupt pending bit */
        TIM_ClearITPendingBit(PWMINPUTTIMER, TIM_IT_CC3);  
        cc |=0x01;
    }
    if(TIM_GetITStatus(PWMINPUTTIMER, TIM_IT_CC4) == SET){ 
        /* Clear TIM1 Capture compare interrupt pending bit */
        TIM_ClearITPendingBit(PWMINPUTTIMER, TIM_IT_CC4);
         cc |=0x02;
    }
    get_capture(cc);
    
}