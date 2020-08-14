#define DEFIN_USART_EXTERN
#include "usart.h"	
#include <string.h>
#include "IObit_remap.h"




#define		USARTx			USART2
//#define		USARTx_TX_DMA		DMA1_Channel2
//#define		USARTx_TX_DMA_GL   	DMA1_IT_GL2

//#define		USARTx_RX_DMA		DMA1_Channel3
//#define		USARTx_RX_DMA_GL   	DMA1_IT_GL3

#define USART_TEX_LEN  			50  

static u8  USART_TX_BUF[USART_TEX_LEN]; 


#define USART_REC_LEN  			50  

  u8  USART_RX_BUF[USART_REC_LEN]; 



static void NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
static void Init_io(void)
{
	GPIO_InitTypeDef    GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 |GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_7);	
}


/*

void uart_DMA_init(void)
{
	DMA_InitTypeDef  DMA_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_DeInit(USARTx_TX_DMA);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USART_DMA_TX_BUF;	
   	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USARTx->TDR; 
    DMA_InitStructure.DMA_BufferSize = (uint16_t)USART_DMA_TX_BUF_LEN;	
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; 
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 
	DMA_Init(USARTx_TX_DMA, &DMA_InitStructure); 

    DMA_DeInit(USARTx_RX_DMA);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USART_DMA_RX_BUF;	
   	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USARTx->RDR; 
    DMA_InitStructure.DMA_BufferSize = (uint16_t)4;	
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 
	DMA_Init(USARTx_RX_DMA, &DMA_InitStructure);
	USART_RX_str.data = 	USART_DMA_RX_BUF;
	DMA_Cmd(USARTx_RX_DMA,ENABLE);
}
void DMA_InterInit(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;   	 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig(USARTx_TX_DMA,DMA_IT_TC,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel3_IRQn;   	 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig(USARTx_RX_DMA,DMA_IT_TC,ENABLE);
}
*/
void uart_init(u32 bound)
{
  

	USART_InitTypeDef USART_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	
	Init_io();

    
	USART_DeInit(USARTx);
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode =  USART_Mode_Tx|USART_Mode_Rx;	
	USART_Init(USARTx, &USART_InitStructure); 
    
	USART_Cmd(USARTx, ENABLE);  
    USART_ReceiverTimeOutCmd(USARTx, ENABLE); 
    
    NVIC_Config();
        
        
        
	USART_ClearFlag(USARTx, USART_FLAG_TC);
//	USART_ITConfig(USARTx, USART_IT_TXE, ENABLE);
	USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
 
    


}





static uint8_t SendFinishFlag= 1;
static uint16_t SendLenth = 0;




u8 USART1_send_data(u8* data,u8 lenth)//>5ms
{
	if(SendFinishFlag){
		SendFinishFlag= 0;
		SendLenth   =  lenth;
		memcpy((void*)USART_TX_BUF,(void*)data,SendLenth);
		USART_ITConfig(USARTx, USART_IT_TXE, ENABLE);
		return 1;
	}
	return 0;
}

extern 	 void UartRecCallBak(u8* data,u8 lenth);


/*
*/
void USART2_IRQHandler(void)
{
	static u8 rxptr = 0;  
    static  uint8_t sendPtr = 0;
	if(USART_GetITStatus(USARTx, USART_IT_TXE)){
        USART_ClearITPendingBit(USARTx, USART_IT_TXE);
        if(sendPtr < SendLenth){
			USART_ClearITPendingBit(USARTx, USART_IT_TXE);	
			USART_SendData(USARTx,USART_TX_BUF[sendPtr++]);				
		}else {
			USART_ITConfig(USARTx, USART_IT_TXE, DISABLE);
			SendFinishFlag = 1;
			sendPtr   = 0;
		}	
	}
    
	if(USART_GetITStatus(USARTx, USART_IT_RXNE)){     
		USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
		USART_RX_BUF[rxptr++] = USART_ReceiveData(USARTx);
        USART_SetReceiverTimeOut(USARTx,16);      
       // USART_ITConfig(USARTx, USART_IT_RTO, ENABLE);     
        USARTx->CR1 |=  0x04000000;   
        USART_ClearITPendingBit(USARTx, USART_IT_RTO);
	}	
	if(USART_GetITStatus(USARTx, USART_IT_RTO)){
		USART_ClearITPendingBit(USARTx, USART_IT_RTO);     
        USARTx->CR1 &=  ~0x04000000;
		if(USART_RX_BUF[0]==0xa5  && USART_RX_BUF[rxptr-1]== 0x5a){
			UartRecCallBak(&USART_RX_BUF[0],rxptr);			
		}
		rxptr = 0;	 
	}
}














