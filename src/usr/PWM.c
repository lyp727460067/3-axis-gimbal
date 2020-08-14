#define  DEFIN_PWM_EXTERN
#include "PWM.h"



void TIM_Config(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA , ENABLE);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9| GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_6);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_6);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_6);
   
}
//#define  halltime  TIM4

void timeHallinit(void)
{
	

	GPIO_InitTypeDef GPIO_InitStructure;
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA , ENABLE);	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB , ENABLE);	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_11| GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    
//	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_8;	
//	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
//	
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_10);
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_10);
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_2);
//	
//	TIM_TimeBaseStructure.TIM_Prescaler = 71;       
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_TimeBaseStructure.TIM_Period =65535;
//	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
//	TIM_TimeBaseInit(halltime,&TIM_TimeBaseStructure);   
//	
//	TIM_ICInitTypeDef        TIM_ICInitStructure;
//	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;       
//	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising|TIM_ICPolarity_Falling;    
//	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_TRC; 
//	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;    
//	TIM_ICInitStructure.TIM_ICFilter = 0x03;     
//	TIM_ICInit(halltime, &TIM_ICInitStructure);


//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;                     
//	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;             
//	TIM_OCInitStructure.TIM_Pulse =1023; 
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;      
//	TIM_OC2Init(halltime,&TIM_OCInitStructure);

//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;                     
//	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;             
//	TIM_OCInitStructure.TIM_Pulse =65535; 
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;      
//	TIM_OC4Init(halltime,&TIM_OCInitStructure);
//		
//		
//	TIM_SelectHallSensor(halltime, ENABLE);   
//	TIM_SelectInputTrigger(halltime, TIM_TS_TI1F_ED);   
//	TIM_SelectSlaveMode(halltime, TIM_SlaveMode_Reset);
//	TIM_SelectMasterSlaveMode(halltime, TIM_MasterSlaveMode_Enable); 
//	TIM_SelectOutputTrigger(halltime, TIM_TRGOSource_OC2Ref);
//	TIM_SetCompare1(halltime, 0); 
//	TIM_SelectOutputTrigger(halltime, TIM_TRGOSource_OC2Ref);
//	
//	NVIC_InitStructure.NVIC_IRQChannel = 30;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);  
//	
//	/* TIM4 enable counter */
//	TIM_ITConfig(halltime, TIM_IT_COM, ENABLE); 
//	/* TIM4 enable counter */
//	TIM_ARRPreloadConfig(halltime, ENABLE); 
//	TIM_Cmd(halltime, ENABLE);    


//	halltime->CR1|=0x0001;                
//	halltime->DIER|=0x0050;                  	


}
//PB8  -->A-->white;PA12-->B-->blue ;PA11-->C-->yellow  
u8 Get_hallsection(void)
{
	u8 temp=0;
	temp |=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8);	
	temp<<=1;
	temp |=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12);
	temp<<=1;	
	temp |=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11);
	return temp;
}
extern void en_pwm(void);
void PWM_TIMEINIT(void)//18K
{
	  
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
//	Channel1Pulse = 4000-2000;//right
//	Channel2Pulse = 4000-1000;
//	Channel3Pulse = 4000-500;//left
	Channel4Pulse = 1;
	/* TIM1 clock enable */
	//TIM_DeInit(TIM1);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
	RCC_TIMCLKConfig(RCC_CFGR3_TIM1SW);//144 MHz
	/* Time Base configuration */

	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned2;
	TIM_TimeBaseStructure.TIM_Period  = PWM_PERIOD+3;//18K
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	/* Channel 1, 2,3 and 4 Configuration in PWM mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState =  TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 		Channel1Pulse;
	TIM_OCInitStructure.TIM_OCPolarity =  TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState =  TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	
	TIM_OCInitStructure.TIM_Pulse = Channel2Pulse;
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = Channel3Pulse;
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = Channel4Pulse;
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	
	/* Enable the TIM1 Trigger and commutation interrupt */
//	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM16_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);   
//  /* TIM3 enable counter */
//	TIM_ITConfig(TIM1,TIM1_UP_TIM16_IRQn,ENABLE ); 

	/* TIM1 Main Output Enable */

	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);  
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);	 
	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	
	TIM_Cmd(TIM1, ENABLE);
	TIM_ClearFlag(TIM1,TIM_IT_Update);
	TIM1->RCR = 1;

}
void PWM_Init(void)
{
	TIM_Config();
	PWM_TIMEINIT();
	//timeHallinit();
}


void Chang_PWMPulse(void)
{
	TIM1->CCR1 = Channel1Pulse+2;
	TIM1->CCR2 = Channel2Pulse+2;
	TIM1->CCR3 = Channel3Pulse+2;
}
u8 couterhall  = 0;
//void TIM4_IRQHandler(void)
//{
//	if(TIM_GetITStatus(TIM4,TIM_IT_COM)){
//		TIM_ClearITPendingBit(TIM4,TIM_IT_COM);		
//		couterhall++;
//	}
//}


void TIM1_UP_TIM16_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM1,TIM_IT_Update)){
		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);		
	}
	if(TIM_GetITStatus(TIM1,TIM_IT_COM)){
		TIM_ClearITPendingBit(TIM1,TIM_IT_COM);		
		couterhall++;
	}
}

