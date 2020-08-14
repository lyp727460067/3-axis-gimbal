#define DEFIN_SPI_EXTERN
#include "spi.h"
#include <string.h>


#define  	 SPI_TEX_LEN 	36
#define 	 SPI_RX_LEN		36
uint8_t 	TxBuffer[SPI_TEX_LEN];
uint8_t 	RxBuffer[SPI_RX_LEN];
static  uint32_t g_Spierr;
#define 	SPIx	SPI2
#define 	SPI_DR_ADDRESS			0x4000380C
#define  	SPI_RX_DMA_CHANNEL		DMA1_Channel4
#define		SPI_TX_DMA_CHANNEL		DMA1_Channel5




#define  	SPI_MOSI_PIN			GPIO_Pin_13
#define  	SPI_MISO_PIN			GPIO_Pin_14
#define  	SPI_SCK_PIN				GPIO_Pin_15

#define  	SPI_NSS_PIN				GPIO_Pin_12

#define  	SPI_MOSI_SOR			GPIO_PinSource13
#define  	SPI_MISO_SOR			GPIO_PinSource14
#define  	SPI_SCK_SOR				GPIO_PinSource15
#define  	SPI_NSS_SOR				GPIO_PinSource12

#define 	SPI_GPIO_AF			GPIO_AF_5
#define     SPI_GPIO			GPIOB



static void SPI_DMA_Config(void)
{
	DMA_InitTypeDef  DMA_InitStructure;

	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_DeInit(SPI_RX_DMA_CHANNEL);
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RxBuffer;	
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SPI_DR_ADDRESS; 
  DMA_InitStructure.DMA_BufferSize = (uint16_t)SPI_RX_LEN;	
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; 
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 
	DMA_Init(SPI_RX_DMA_CHANNEL, &DMA_InitStructure); 
	
  
	DMA_DeInit(SPI_TX_DMA_CHANNEL);
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)TxBuffer;	
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SPI_DR_ADDRESS; 
  DMA_InitStructure.DMA_BufferSize = (uint16_t)SPI_TEX_LEN;	
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; 
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 
	DMA_Init(SPI_TX_DMA_CHANNEL, &DMA_InitStructure); 	
    
    
    	/* Configure and enable ADC1 interrupt */
//	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;   	 
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
    

    
    
}
void spi_config(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
	/* Enable the SPI periph */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);//36M

	/* Enable SCK, MOSI, MISO and NSS GPIO clocks */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);


	
	//GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_5);//TI mode
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = SPI_MOSI_PIN|SPI_MISO_PIN|SPI_SCK_PIN;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);
    
	/* SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	
	GPIO_InitStructure.GPIO_Pin = SPI_NSS_PIN;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);
	GPIO_SetBits(SPI_GPIO,SPI_NSS_PIN);
    
	GPIO_PinAFConfig(SPI_GPIO, SPI_MOSI_SOR, SPI_GPIO_AF);
	GPIO_PinAFConfig(SPI_GPIO, SPI_MISO_SOR, SPI_GPIO_AF);
	GPIO_PinAFConfig(SPI_GPIO, SPI_SCK_SOR,	 SPI_GPIO_AF);    
    
	/* SPI configuration -------------------------------------------------------*/
	SPI_I2S_DeInit(SPIx);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;//48/32*8*15  = 120us
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_Init(SPIx, &SPI_InitStructure);


	SPI_RxFIFOThresholdConfig(SPIx, SPI_RxFIFOThreshold_QF);
	SPI_I2S_ITConfig(SPIx, SPI_I2S_IT_ERR, ENABLE);
	
	for(uint16_t i=0;i<SPI_TEX_LEN;i++){
		TxBuffer[i] = 0xff;
	}
	SPI_Cmd(SPIx, ENABLE);    
	SPI_DMA_Config();
	
}


#define TIME_OUT	0XFFFF
/**/
uint8_t  spi_write( uint8_t reg, uint8_t* data,uint16_t len)
{	
	uint8_t temp = 0;
	GPIO_ResetBits(GPIOB,SPI_NSS_PIN);
    while (SPI_GetTransmissionFIFOStatus(SPIx) != SPI_TransmissionFIFOStatus_Empty);
    while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET);
    while (SPI_GetReceptionFIFOStatus(SPIx) != SPI_ReceptionFIFOStatus_Empty){
        RxBuffer[0] = SPI_ReceiveData8(SPIx);
    }
    
	SPI_SendData8(SPIx,(reg&0x7f));
    while (SPI_GetTransmissionFIFOStatus(SPIx) != SPI_TransmissionFIFOStatus_Empty);   
    while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET);
    
    while (SPI_GetReceptionFIFOStatus(SPIx) != SPI_ReceptionFIFOStatus_Empty){
		RxBuffer[0] = SPI_ReceiveData8(SPIx);	
	}
    for(uint16_t i  = 0;i<len;i++){
		SPI_SendData8(SPIx,(data[i]));
        while (SPI_GetTransmissionFIFOStatus(SPIx) != SPI_TransmissionFIFOStatus_Empty);
        while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET);	
		while (SPI_GetReceptionFIFOStatus(SPIx) != SPI_ReceptionFIFOStatus_Empty){
			temp = SPI_ReceiveData8(SPIx);
		}
	}
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
	return 1;
}
/**/
uint8_t spi_read( uint8_t reg, uint8_t *buf, uint16_t len)
{
	GPIO_ResetBits(SPI_GPIO, SPI_NSS_PIN);	
    while (SPI_GetTransmissionFIFOStatus(SPIx) != SPI_TransmissionFIFOStatus_Empty);
    while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET);
    while (SPI_GetReceptionFIFOStatus(SPIx) != SPI_ReceptionFIFOStatus_Empty){
        RxBuffer[0] = SPI_ReceiveData8(SPIx);
    }
    
	SPI_SendData8(SPIx,(reg)|0x80);
	while (SPI_GetTransmissionFIFOStatus(SPIx) != SPI_TransmissionFIFOStatus_Empty);
	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET);
	while (SPI_GetReceptionFIFOStatus(SPIx) != SPI_ReceptionFIFOStatus_Empty){
		RxBuffer[0] = SPI_ReceiveData8(SPIx);	 	
	}	
	for(uint16_t i  = 0;i<len;i++){
		SPI_SendData8(SPIx,0XFF);
		while (SPI_GetTransmissionFIFOStatus(SPIx) != SPI_TransmissionFIFOStatus_Empty);
		while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET);		
		while (SPI_GetReceptionFIFOStatus(SPIx) != SPI_ReceptionFIFOStatus_Empty){
			buf[i] = SPI_ReceiveData8(SPIx);
		}
	}
	GPIO_SetBits(SPI_GPIO, SPI_NSS_PIN);
	return 1;
}
/**/
u8 spi_dma_read(u8 * data,u8 lenth)
{
    
    while (SPI_GetTransmissionFIFOStatus(SPIx) != SPI_TransmissionFIFOStatus_Empty);
    while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET);
    while (SPI_GetReceptionFIFOStatus(SPIx) != SPI_ReceptionFIFOStatus_Empty){
        SPI_ReceiveData8(SPIx);
    }   
    
    DMA_ClearFlag(DMA1_FLAG_GL4);
    DMA_ClearFlag(DMA1_FLAG_GL5);
    DMA_Cmd(SPI_TX_DMA_CHANNEL,DISABLE);
    DMA_Cmd(SPI_RX_DMA_CHANNEL,DISABLE);
    
    SPI_RX_DMA_CHANNEL->CNDTR = lenth;;
    SPI_RX_DMA_CHANNEL->CMAR =(uint32_t)data;
    
    SPI_TX_DMA_CHANNEL->CNDTR = lenth;
    SPI_TX_DMA_CHANNEL->CMAR =(uint32_t)TxBuffer;
    
    GPIO_ResetBits(SPI_GPIO, SPI_NSS_PIN);
    DMA_Cmd(SPI_RX_DMA_CHANNEL,ENABLE);	
    DMA_Cmd(SPI_TX_DMA_CHANNEL,ENABLE);	
    while(DMA_GetITStatus(DMA1_IT_TC4)==0);
    GPIO_SetBits(SPI_GPIO,SPI_NSS_PIN); 
    return 1;
}
uint8_t MpuFIFOConter[3];
void SpiDmaReadMpu(void)
{
  SPI_TX_DMA_CHANNEL->CNDTR = 3;
  SPI_RX_DMA_CHANNEL->CNDTR = 3;//读取FIFO长度 
  
  SPI_TX_DMA_CHANNEL->CMAR =(uint32_t)MpuFIFOConter;
  SPI_RX_DMA_CHANNEL->CMAR =(uint32_t)MpuFIFOConter;
  DMA_Cmd(SPI_RX_DMA_CHANNEL,ENABLE);	
	DMA_Cmd(SPI_TX_DMA_CHANNEL,ENABLE);	
  GPIO_ResetBits(SPI_GPIO, SPI_NSS_PIN);
}
//u8 spi_dma_read(u8 * data,u8 lenth)
//{
//    if(SPI_GetTransmissionFIFOStatus(SPIx) != SPI_TransmissionFIFOStatus_Empty)return 0;
//    if(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == SET)return 0;
//    if(SPI_GetReceptionFIFOStatus(SPIx) != SPI_ReceptionFIFOStatus_Empty)return 0;			
//    GPIO_SetBits(SPI_GPIO,SPI_NSS_PIN);  
//    DMA_ClearFlag(DMA1_FLAG_GL4);
//    DMA_ClearFlag(DMA1_FLAG_GL5);
//    DMA_Cmd(SPI_TX_DMA_CHANNEL,DISABLE);
//    DMA_Cmd(SPI_RX_DMA_CHANNEL,DISABLE); 
//    SPI_RX_DMA_CHANNEL->CNDTR = lenth;
//    SPI_TX_DMA_CHANNEL->CNDTR = lenth;
//    memcpy((void*)data ,(void*)&RxBuffer[0],lenth);
//    GPIO_ResetBits(SPI_GPIO, SPI_NSS_PIN);
//    DMA_Cmd(SPI_RX_DMA_CHANNEL,ENABLE);	
//    DMA_Cmd(SPI_TX_DMA_CHANNEL,ENABLE);	
//    return 1;
//}


void spi_select_mode(uint8_t mode)
{
	switch(mode){
		case 0:
			SPI_I2S_DMACmd(SPIx, SPI_I2S_DMAReq_Rx, ENABLE);
			SPI_I2S_DMACmd(SPIx, SPI_I2S_DMAReq_Tx, ENABLE);
			DMA_Cmd(SPI_RX_DMA_CHANNEL,ENABLE);	
			DMA_Cmd(SPI_TX_DMA_CHANNEL,ENABLE);	
			TxBuffer[0] = SPI_DMA_READADRR|0x80;
            //DMA_ITConfig(SPI_RX_DMA_CHANNEL,DMA_IT_TC);
		break;
		case 1:
			SPI_I2S_DMACmd(SPIx, SPI_I2S_DMAReq_Rx, DISABLE);
			SPI_I2S_DMACmd(SPIx, SPI_I2S_DMAReq_Tx, DISABLE);
			DMA_Cmd(SPI_RX_DMA_CHANNEL,DISABLE);	
			DMA_Cmd(SPI_TX_DMA_CHANNEL,DISABLE);
		
		break;
		default:break;
	}
}

uint8_t MpuDataFinish = 0;
void GetMpuFIFOData()
{

}
void DMA1_Channel4_IRQHandler(void)//TEX
{
  static  uint8_t spi_pack_couter=0;
  static uint8_t states = 0;
	if(DMA_GetITStatus(DMA1_IT_TC4)){
        DMA_ClearITPendingBit(DMA1_IT_TC4);
        GPIO_SetBits(SPI_GPIO,SPI_NSS_PIN);
        if(states==0){
            DMA_Cmd(SPI_TX_DMA_CHANNEL,DISABLE);
            DMA_Cmd(SPI_RX_DMA_CHANNEL,DISABLE);
            SPI_TX_DMA_CHANNEL->CMAR =(uint32_t)TxBuffer;
            SPI_RX_DMA_CHANNEL->CMAR =(uint32_t)RxBuffer;
            SPI_TX_DMA_CHANNEL->CNDTR = ((MpuFIFOConter[1]<<8) |MpuFIFOConter[2] )+1;
            SPI_RX_DMA_CHANNEL->CNDTR = ((MpuFIFOConter[1]<<8) |MpuFIFOConter[2]) +1;//读取FIFO长度 
            DMA_Cmd(SPI_RX_DMA_CHANNEL,ENABLE);	
            DMA_Cmd(SPI_TX_DMA_CHANNEL,ENABLE);	
            GPIO_ResetBits(SPI_GPIO, SPI_NSS_PIN);
            states = 1;
        }else {
            DMA_Cmd(SPI_RX_DMA_CHANNEL,DISABLE);	
            DMA_Cmd(SPI_TX_DMA_CHANNEL,DISABLE);	
            GPIO_SetBits(SPI_GPIO, SPI_NSS_PIN);   
            MpuDataFinish = 1;
            states = 0;
        }    
    }
}


void SPI2_IRQHandler(void)
{
  /* SPI Error interrupt--------------------------------------- */
	g_Spierr++;
	SPI_I2S_ClearFlag(SPIx,SPI_I2S_IT_OVR|I2S_IT_UDR|SPI_IT_MODF|SPI_I2S_IT_FRE);
}
