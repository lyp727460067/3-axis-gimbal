#define DEFIN_IIC_EXTERN
#include "IIC.h"
#include <string.h>
#include "fifo.h"
 #include "IICtrancedata.h"
#include "device.h"




#define IIC_DMA_TX_BUF_LEN  			MAXDATALENT  		
#define IIC_DMA_RX_BUF_LEN  			MAXDATALENT  	


#define			IIC_TX_DMA      DMA1_Channel6
#define 		IIC_RX_DMA      DMA1_Channel7
#define 		IICx            I2C1
#define 		IIC_EV_IRQn     I2C1_EV_IRQn
#define 		IIC_ER_IRQn     I2C1_ER_IRQn
#define 		IIC_GPIO        GPIOB
#define 		IIC_SCL         GPIO_Pin_6
#define 		IIC_SDA         GPIO_Pin_7
#define 		IIC_SCL_AF_SORRCE       GPIO_PinSource6
#define 		IIC_SDA_AF_SORRCE       GPIO_PinSource7
#define 		IIC_GPIO_AF             GPIO_AF_4




#define IIC_TEX_LEN  			10*8   	
uint8_t  IIC_TX_BUF[IIC_TEX_LEN]; 
FIFO_Type     IIC_TX_FIFO;


//
typedef struct 
{
   uint8_t  *pdata;//数据地址
   uint16_t lenth;//数据长度
   uint8_t  adrr;//IIC设备地址
   uint8_t  reser;//IIC设备地址  
}iic_fifo_t;

typedef enum 
{
	COMM_IDL  = 0,  
	COMM_MATCH = 1,
    COMM_TX = 2,
    COMM_TX_Done = 3,
	COMM_RX = 4 ,
	COMM_RX_Done	 =	5
}I2C_STATE;
I2C_STATE  I2C_state= COMM_IDL;

//一次写入不要超过FIFO缓冲区，
//等到完成一次后再进行第二次操作
void IIC_fifo_init(void)
{
	FIFO_DATA_Str DATA_temp ;
	DATA_temp.data = 	IIC_TX_BUF;
	DATA_temp.DATAlenth = sizeof(iic_fifo_t);
    DATA_temp.lenth = 10;
	fifo_init(&IIC_TX_FIFO,&DATA_temp);	
}
//
void iic_dma_init(void)
{

	DMA_InitTypeDef  DMA_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	
	DMA_DeInit(IIC_TX_DMA);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&I2C1->TXDR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)0;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; 
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte; 
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 
	DMA_Init(IIC_TX_DMA, &DMA_InitStructure);
	  


	DMA_DeInit(IIC_RX_DMA);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&I2C1->RXDR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)0; 
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; 
	DMA_InitStructure.DMA_BufferSize = 0; 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte; 
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; 
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;  
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(IIC_RX_DMA, &DMA_InitStructure);	
}
//
void iic_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
	I2C_InitTypeDef  I2C_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);//
	
	RCC_I2CCLKConfig(RCC_I2C1CLK_SYSCLK);
	GPIO_InitStructure.GPIO_Pin =  IIC_SCL | IIC_SDA;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
//	
//	#ifdef	IS_MAIN_CONTROLLOR
//	
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
//#else
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 	
//#endif
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_Init(IIC_GPIO, &GPIO_InitStructure);
	GPIO_PinAFConfig(IIC_GPIO, IIC_SDA_AF_SORRCE, IIC_GPIO_AF);
	GPIO_PinAFConfig(IIC_GPIO, IIC_SCL_AF_SORRCE, IIC_GPIO_AF);
	//I2C_DeInit(IICx);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DigitalFilter       = IIC_DIGFILTER;
	I2C_InitStructure.I2C_Timing = I2C_TIMING;//fclk default is 8m. also can switch to sysclc(72M)
	I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Disable;
	I2C_InitStructure.I2C_OwnAddress1 = MIIC_DATA[0].adrr<<1;
	
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(IICx, &I2C_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = IIC_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = IIC_ER_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	
	I2C_ITConfig(IICx, I2C_IT_ERRI,ENABLE);	
	I2C_ClockTimeoutCmd(IICx,ENABLE);
	I2C_ExtendedClockTimeoutCmd(IICx,ENABLE);
	I2C_TimeoutAConfig(IICx,10);
	I2C_TimeoutBConfig(IICx,10);
	I2C_StretchClockCmd(IICx,ENABLE);
	if(g_ucDeviceNum==0){
		I2C_ITConfig(IICx, I2C_IT_NACKF, ENABLE);
		I2C_ITConfig(IICx, I2C_IT_TC, ENABLE);
		I2C_AutoEndCmd(IICx,DISABLE);
		
		IIC_fifo_init();
	}else {
		I2C_ITConfig(IICx, I2C_IT_ADDR, ENABLE);
		I2C_ITConfig(IICx, I2C_IT_STOPF, ENABLE);
		
		//I2C_ITConfig(IICx, I2C_IT_OVR, ENABLE);	
		//I2C_ITConfig(IICx, I2C_IT_BERR, ENABLE);
	}	
	iic_dma_init();
	I2C_Cmd(IICx, ENABLE);
}

void disable_dma(void)
{

	I2C_Cmd(IICx, DISABLE);
	while( IICx->CR1&I2C_CR1_PE){
		I2C_Cmd(IICx, DISABLE);
	}
	I2C_Cmd(IICx, ENABLE);
	g_ucWaitFinish = 0;
	
	DMA_Cmd(IIC_TX_DMA, DISABLE);
	DMA_Cmd(IIC_RX_DMA, DISABLE);
	I2C_DMACmd(IICx,I2C_DMAReq_Rx, DISABLE);
	I2C_DMACmd(IICx,I2C_DMAReq_Tx, DISABLE);
	
	I2C_state =  COMM_IDL;
}

u8 get_iic_bus_state()
{
	if(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY)){
		return 1;
	}
	return 0;
}
/**/	
void iic_write(iic_fifo_t* iic_data)
{
	I2C_state = COMM_TX;
	//
	I2C_DMACmd(IICx,I2C_DMAReq_Tx, DISABLE);
	DMA_Cmd(IIC_TX_DMA, DISABLE);
	IIC_TX_DMA->CNDTR = iic_data->lenth;
	IIC_TX_DMA->CMAR =  (uint32_t)iic_data->pdata;	
	//
    I2C_NumberOfBytesConfig(IICx,iic_data->lenth);
	I2C_SlaveAddressConfig(IICx,iic_data->adrr);
    
	I2C_DMACmd(IICx,I2C_DMAReq_Tx, ENABLE);
	DMA_Cmd(IIC_TX_DMA, ENABLE);	
	I2C_MasterRequestConfig(IICx,I2C_Direction_Transmitter);
	I2C_GenerateSTART(IICx, ENABLE);
	
}
void iic_read(iic_fifo_t* iic_data)
{
	I2C_state = COMM_RX;
    
    //
    I2C_DMACmd(IICx,I2C_DMAReq_Rx, DISABLE);
	DMA_Cmd(IIC_RX_DMA, DISABLE);
	IIC_RX_DMA->CNDTR = iic_data->lenth;
	IIC_RX_DMA->CMAR = (uint32_t)iic_data->pdata;	
	//
    I2C_NumberOfBytesConfig(IICx,iic_data->lenth);
	I2C_SlaveAddressConfig(IICx,iic_data->adrr);

	I2C_DMACmd(IICx,I2C_DMAReq_Rx, ENABLE);
	DMA_Cmd(IIC_RX_DMA, ENABLE);
    
	I2C_MasterRequestConfig(IICx,I2C_Direction_Receiver);				
	I2C_GenerateSTART(IICx, ENABLE);
    
}
extern void iic_rev_cabck();
uint8_t iic_read_write_to_dma(void)
{ 
    iic_fifo_t  iic_data;
    if(FIFO_POP(&IIC_TX_FIFO,(uint8_t*)&iic_data)){
        if(iic_data.adrr&0x01){
            iic_read(&iic_data);	
        }else {
            iic_write(&iic_data);
        }
        return 1;
    }else {
        g_ucWaitFinish = 0;
        return 0;
    }
}
uint32_t IICSdaTimeOutCnt = 0;
void IIC_Sche(void)
{
	//if(g_ucDeviceNum!=0)return ;
	//if(get_iic_bus_state())return ;	
	//if(I2C_state!=COMM_IDL)return ;	
	//iic_read_write_to_dma();
	//水中有短路的话，现在始终跑的慢，然后一个周期后必然IIC结束
	//如果结束不了强行复位
  	if(g_ucDeviceNum!=0)return ;
	if(get_iic_bus_state()){
	  if(++IICSdaTimeOutCnt>=100){
		IICSdaTimeOutCnt = 0;
	  	disable_dma();
	  }	
	}	
	if(I2C_state!=COMM_IDL)return ;	
	iic_read_write_to_dma();
    
}



uint8_t iic_write_data(MIIC_TypeDef* MIIC)
{
	iic_fifo_t  Strtemp;
    //if(g_ucWaitFinish&get_fifo_states(&IIC_TX_FIFO))return 0 ;	
    Strtemp.pdata = MIIC->data;
    Strtemp.lenth=  MIIC->lenth;
    Strtemp.adrr =  MIIC->adrr;	
	if(FIFO_PUSH(&IIC_TX_FIFO,(uint8_t*)&Strtemp)){
		return 1;			
	}else {
		return 0;			
	}
	
}

void iic_master_irq(void)
{
  	IICSdaTimeOutCnt = 0;
	if(I2C_GetITStatus(I2C1,I2C_IT_TC)){
		I2C_GenerateSTOP(I2C1, ENABLE);	
		if(I2C_state==COMM_RX){
			iic_rev_cabck();
		}
		if(iic_read_write_to_dma()==0){
			I2C_state=COMM_IDL;	
			g_ucWaitFinish = 0;
		}		
		I2C_ClearITPendingBit(I2C1,I2C_IT_TC);
	}
	if((I2C_GetITStatus(I2C1,I2C_IT_NACKF))){
		disable_dma();		
		I2C_state = COMM_IDL;
		I2C_ClearITPendingBit(I2C1,I2C_IT_NACKF);
	}	
}


/*slave*/
//

uint8_t *slave_send_data_adr ;
uint8_t *slave_rev_data_adr ;
void iic_slave_data_init(uint8_t *sed,uint8_t *rev)
{
    slave_send_data_adr   = sed;
    slave_rev_data_adr = rev;
}

#define DMA_DEFAULT_LENTH   IIC_TEX_LEN

I2C_STATE SlaveStates = COMM_IDL;
extern void iic_slave_send_cabck(void);
extern void iic_slave_rev_cabck();
void iic_slave_write(void)
{
    SlaveStates = COMM_TX;	
    I2C1->CR1 &= (uint32_t)~I2C_DMAReq_Tx;
    IIC_TX_DMA->CCR &= (~DMA_CCR_EN);	
    IIC_TX_DMA->CNDTR = DMA_DEFAULT_LENTH;
    IIC_TX_DMA->CMAR =(uint32_t)&slave_send_data_adr[1];	
    I2C1->CR1 |= (uint32_t)I2C_DMAReq_Tx;
    iic_slave_send_cabck();  
    I2C1->ISR |=0X01;
    I2C1->TXDR =slave_send_data_adr[0];
    IIC_TX_DMA->CCR |= (DMA_CCR_EN);	  

}
void iic_slave_read(void)
{
    SlaveStates = COMM_RX;	
    
    I2C1->CR1 &= (uint32_t)~I2C_DMAReq_Rx;
    IIC_RX_DMA->CCR &= (~DMA_CCR_EN);
    IIC_RX_DMA->CNDTR = DMA_DEFAULT_LENTH;
    IIC_RX_DMA->CMAR =(uint32_t)slave_rev_data_adr;
    I2C1->CR1 |= (uint32_t)I2C_DMAReq_Rx;
    IIC_RX_DMA->CCR |= DMA_CCR_EN;	   


}

void iic_slave_finish(void)
{
    if(SlaveStates==COMM_TX){//slave 发送完成。主机接收完成  
    }else if(SlaveStates==COMM_RX){     
        iic_slave_rev_cabck();
	}
	SlaveStates = COMM_IDL;
}

void iic_slave_read_write_to_dma(uint8_t direct)
{
    if(direct ==0){
        iic_slave_read();
    }else {
        iic_slave_write();
    }    

}
void iic_slave_irq(void)
{

	if(I2C_GetITStatus(I2C1,I2C_IT_STOPF)){
	    iic_slave_finish();
		I2C_ClearITPendingBit(I2C1,I2C_IT_STOPF);	  
	}
	if(I2C_GetITStatus(I2C1,I2C_IT_ADDR)){
        iic_slave_read_write_to_dma(((uint32_t)(I2C1->ISR & I2C_ISR_DIR))>>16);
		I2C_ClearITPendingBit(I2C1,I2C_IT_ADDR);
	}
	
}

void iic_slave_erirq(void)
{
	I2C_DMACmd(I2C1,I2C_DMAReq_Tx, DISABLE);
	SlaveStates = COMM_IDL;
	disable_dma();
	I2C_ClearFlag(I2C1,I2C_FLAG_ARLO|I2C_IT_OVR|I2C_IT_BERR|I2C_IT_NACKF); 
}
void iic_master_erirq(void)
{
	I2C_ClearFlag(I2C1,I2C_IT_TIMEOUT|I2C_FLAG_ARLO|I2C_IT_OVR|I2C_IT_BERR);
	I2C_GenerateSTOP(I2C1, ENABLE);	
	disable_dma();
	I2C_state = COMM_IDL;
}
void I2C1_ER_IRQHandler(void)
{
	if(g_ucDeviceNum==0){
		iic_master_erirq();
	}else {
		iic_slave_erirq();
	}
}
void I2C1_EV_IRQHandler(void)
{
	
	if(g_ucDeviceNum==0){
		iic_master_irq();
	}else {
		iic_slave_irq();
	}
	
}
u16 sendcountdone = 0;
void DMA1_Channel6_IRQHandler(void)//TEX
{
	DMA_ClearITPendingBit(DMA1_IT_TC6);
	I2C_state = COMM_IDL;		
}
void DMA1_Channel7_IRQHandler(void)//RX
{
	DMA_ClearITPendingBit(DMA1_IT_TC7);
	I2C_state = COMM_IDL;
}
