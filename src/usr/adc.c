#define  ADC_EXTERN_DEFINE
#include  "adc.h"
#include "commonvalue.h"
#include  "Control.h"

#define  ADC_COUTER		64  
 u16 Adc_BaterVa[ADC_COUTER];
 u16 Adc_PositVa[ADC_COUTER];
 u16 ADC_Value[ADC_COUTER];

void DMA_Configuration(void)
{

	DMA_InitTypeDef DMA_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_DeInit(DMA1_Channel1); 
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR; 
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_Value;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = ADC_COUTER;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; 
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; 
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 
	DMA_Init(DMA1_Channel1, &DMA_InitStructure); 
}
void ADC_DMA_Configer(void)
{
	ADC_DMAConfig(ADC1, ADC_DMAMode_Circular);
	ADC_DMACmd(ADC1, ENABLE);
	DMA_Configuration();	
	DMA_Cmd(DMA1_Channel1, ENABLE); //
	ADC_StartConversion(ADC1); 		
}
/*
	GPIO_Pin_0¼ì²âµç³ØµçÑ¹
	GPIO_Pin_4¼ì²â±ä×èÆ÷ ½Ç¶È
*/
static void Init_io(void)
{
	GPIO_InitTypeDef    GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);		
}
__IO uint32_t  calibration_value1 = 0;
__IO uint32_t  calibration_value2 = 0;
u16 Get_ADCValue(ADC_TypeDef* ADCx)
{
	u32 temp1=0 ;;
	for(u16 i=0;i<1024;i++){
		while(ADC_GetFlagStatus(ADCx, ADC_FLAG_EOC) == RESET);
        if(ADCx==ADC1){
            temp1 +=ADC_GetConversionValue(ADCx)-calibration_value1;
        }else {
            temp1 +=ADC_GetConversionValue(ADCx)-calibration_value2;
        }
		
	}	
	return temp1>>10;
}
extern void Init_OPAMP(void);
ADC_InitTypeDef       ADC_InitStructure;
ADC_CommonInitTypeDef ADC_CommonInitStructure;
void adc2_Init(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);	
	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div1);
	
	ADC_StructInit(&ADC_InitStructure);
	ADC_VoltageRegulatorCmd(ADC2, ENABLE);
	ADC_SelectCalibrationMode(ADC2, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC2);   
	while(ADC_GetCalibrationStatus(ADC2) != RESET );
	calibration_value2 = ADC_GetCalibrationValue(ADC2);	
/*
get base adc
*/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;                                                                    
	ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;                    
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;             
	ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;                  
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = 20;          
	ADC_CommonInit(ADC2, &ADC_CommonInitStructure);

	ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 
	ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;         
	ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;   
	ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;  
	ADC_InitStructure.ADC_NbrOfRegChannel = 1;
	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_3, 1, ADC_SampleTime_1Cycles5);
	
	/* Enable ADC1 */
	ADC_Cmd(ADC2, ENABLE);
	/* wait for ADRDY */
	while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_RDY));	
	ADC_StartConversion(ADC2); 	
	Ibase.I_PhaseB = Get_ADCValue(ADC2);
}
void adc1_Init(void)
{


	Init_io();
	//ADC_DeInit(ADC1);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);	
	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div1);
	
	ADC_StructInit(&ADC_InitStructure);
	ADC_VoltageRegulatorCmd(ADC1, ENABLE);
	ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC1);   
	while(ADC_GetCalibrationStatus(ADC1) != RESET );
	calibration_value1 = ADC_GetCalibrationValue(ADC1);	
/*
get base adc
*/	
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;                                                                    
	ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;                    
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;             
	ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;                 
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = 20;          
	ADC_CommonInit(ADC1, &ADC_CommonInitStructure);

	ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 
	ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;         
	ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;   
	ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;  
	ADC_InitStructure.ADC_NbrOfRegChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_1Cycles5);
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	/* wait for ADRDY */
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));
	ADC_StartConversion(ADC1);  	
	Ibase.I_PhaseA = Get_ADCValue(ADC1);
}

void adc_Init(void)
{
	ADC_InjectedInitTypeDef ADC_InjectedInitStructure;
	NVIC_InitTypeDef    NVIC_InitStructure;
	Init_OPAMP();
	adc1_Init();
	adc2_Init();
	
	/*
normal start adc se
*/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;                                                                    
	ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;                    
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;             
	ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_Circular;                        
	ADC_CommonInit(ADC1, &ADC_CommonInitStructure);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_1Cycles5);
	ADC_RegularChannelSequencerLengthConfig(ADC1, 1);
	ADC_DMA_Configer();//channel 1 is  battery  input;

	ADC_InjectedInitStructure.ADC_ExternalTrigInjecConvEvent = ADC_ExternalTrigInjecConvEvent_1;
	ADC_InjectedInitStructure.ADC_ExternalTrigInjecEventEdge = ADC_ExternalTrigInjecEventEdge_FallingEdge;
	ADC_InjectedInitStructure.ADC_NbrOfInjecChannel = 1;
	ADC_InjectedInitStructure.ADC_InjecSequence1 = ADC_Channel_3;
	ADC_InjectedInit(ADC1,&ADC_InjectedInitStructure);//adc1
	
	ADC_InjectedInitStructure.ADC_NbrOfInjecChannel = 2;
	ADC_InjectedInitStructure.ADC_InjecSequence2 = ADC_Channel_1;//±äÎ»Æ÷½Ç¶È
	ADC_InjectedInit(ADC2,&ADC_InjectedInitStructure);//adc2
	ADC_InjectedChannelSampleTimeConfig(ADC1, ADC_Channel_3, ADC_SampleTime_1Cycles5);
	ADC_InjectedChannelSampleTimeConfig(ADC2, ADC_Channel_3, ADC_SampleTime_1Cycles5);	
	ADC_InjectedChannelSampleTimeConfig(ADC2, ADC_Channel_1, ADC_SampleTime_1Cycles5);	
	
	
	/* Configure and enable ADC1 interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;   	 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	/* Enable AWD interrupt */
	ADC_ClearFlag(ADC2, ADC_IT_JEOC);
	ADC_ITConfig(ADC2, ADC_IT_JEOC, ENABLE);
}
void Start_ADC()
{
	ADC_StartInjectedConversion(ADC1);
	ADC_StartInjectedConversion(ADC2);
}
void init_Caliadvalue()
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);	
	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div1);
	
	ADC_StructInit(&ADC_InitStructure);
	ADC_VoltageRegulatorCmd(ADC2, ENABLE);
	ADC_SelectCalibrationMode(ADC2, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC2);   
	while(ADC_GetCalibrationStatus(ADC2) != RESET );
	calibration_value2 = ADC_GetCalibrationValue(ADC2);	
/*
get base adc
*/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;                                                                    
	ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;                    
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;             
	ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;                  
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = 20;          
	ADC_CommonInit(ADC2, &ADC_CommonInitStructure);

	ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 
	ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;         
	ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;   
	ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;  
	ADC_InitStructure.ADC_NbrOfRegChannel = 1;
	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_1Cycles5);
	
	/* Enable ADC1 */
	ADC_Cmd(ADC2, ENABLE);
	/* wait for ADRDY */
	while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_RDY));	
	ADC_StartConversion(ADC2); 	

}

u16  Get_Caliadvalue(void)
{
	while(ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == RESET);
	return ADC_GetConversionValue(ADC2);
}


uint16_t a_Positon ;
extern void ADC1_IRQHandler(void);



void ADC1_IRQHandler(void)
{
	if(ADC_GetITStatus(ADC2,ADC_IT_JEOC)){
		ADC_ClearITPendingBit(ADC2,ADC_IT_JEOC);
		ADCCurrent.I_PhaseA = ADC_GetInjectedConversionValue(ADC1,ADC_InjectedSequence_1);
        ADCCurrent.I_PhaseA -=calibration_value1;
        
		ADCCurrent.I_PhaseB = ADC_GetInjectedConversionValue(ADC2,ADC_InjectedSequence_1);		
		ADCCurrent.I_PhaseB -=calibration_value2;
        //a_Positon = 		  ADC_GetInjectedConversionValue(ADC2,ADC_InjectedSequence_2);
		PWM_Circl();
        
	}
}


int16_t  get_org_angle(void)
{
	return   0XFFF - a_Positon ;
//	uint32_t AdcPositionVal  = 0;
//	for(uint16_t i = 0;i<ADC_COUTER;i++){
//		AdcPositionVal+=Adc_PositVa[i];
//	}
//	return AdcPositionVal/ADC_COUTER;
}
int16_t get_bater_value(void)
{
	uint32_t AdcPositionVal  = 0;
	for(uint16_t i = 0;i<ADC_COUTER;i++){
		AdcPositionVal+=Adc_BaterVa[i];
	}
	return AdcPositionVal/ADC_COUTER;
} 


void DMA1_Channel1_IRQHandler(void)//TEX
{
	
	
}
void DMA2_Channel1_IRQHandler(void)//RX
{

}


