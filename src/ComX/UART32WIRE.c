
/**/
#include "main.h"
#include "string.h"
/**/
#define COM_USART                       USART3
#define COM_USART_CLK_EN()              RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE)

#define COM_TX_PORT_CLK_EN()            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE)
#define COM_RX_PORT_CLK_EN()            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE)

#define COM_TX_PORT                     GPIOB  
#define COM_RX_PORT                     GPIOB
#define COM_TX_PIN_SOURCE               GPIO_PinSource10
#define COM_RX_PIN_SOURCE               GPIO_PinSource11
#define COM_TX_AF                       GPIO_AF_7
#define COM_RX_AF                       GPIO_AF_7
#define COM_TX_PIN                      GPIO_Pin_10
#define COM_RX_PIN                      GPIO_Pin_11
#define COM_IRQn                        USART3_IRQn
#define COM_IRQHandler                  USART3_IRQHandler
/**/
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
static uint8_t Com_Buff_Rx[256];
static volatile uint8_t Com_Buff_Rx_Write = 0;
static uint8_t Com_Buff_Rx_Read = 0;

static uint8_t Com_Buff_Tx[256];
static uint8_t Com_Buff_Tx_Write = 0;
static volatile uint8_t Com_Buff_Tx_Read = 0;
/**/
void COM_IRQHandler(void)
{
  /* USART in mode Tramitter -------------------------------------------------*/
  if ((COM_USART->ISR&(USART_ISR_TC)) != 0)
  { 
    if(Com_Buff_Tx_Read != Com_Buff_Tx_Write)
      COM_USART->TDR = Com_Buff_Tx[Com_Buff_Tx_Read++];
    else
      COM_USART->CR1 |= USART_CR1_RE;
    COM_USART->ICR = USART_ICR_TCCF;
  }
  
  /* USART in mode Receiver --------------------------------------------------*/
  if ((COM_USART->ISR&(USART_ISR_RXNE|USART_ISR_ORE)) != 0)
  {
     COM_USART->ICR = USART_ICR_ORECF;
    //COM_USART->ICR &= ~USART_ISR_ORE;
    Com_Buff_Rx[Com_Buff_Rx_Write++] = COM_USART->RDR;
  }
  
  
}

/**/
int COM_1WIRE_until_tx(int time_out)
{
  uint32_t str_time;
  uint32_t time_go;
  str_time = Timer_GetCount();
  while(Com_Buff_Tx_Write != Com_Buff_Tx_Read)
  {
    time_go = Timer_ToMs(Timer_CalDiff(str_time));
    if(time_go > time_out)
      return -1;
  }
  return time_go;
}

/**/
int32_t COM_1WIRE_Write(void *pbuff, int32_t StrLoc, int32_t wLen)
{
  uint8_t TxIsIdle = 0;
  if(Com_Buff_Tx_Write == Com_Buff_Tx_Read)
  {
    Com_Buff_Tx_Write = Com_Buff_Tx_Read = 0;
    TxIsIdle = 1;
  }
  
  if((wLen + Com_Buff_Tx_Write) > sizeof(Com_Buff_Tx))
    return -1;
  
  int k = Com_Buff_Tx_Write;
  memcpy(&Com_Buff_Tx[Com_Buff_Tx_Write],pbuff,wLen); 
  Com_Buff_Tx_Write = k + wLen;
  if(TxIsIdle == 1)
  {
    COM_USART->CR1&= ~USART_CR1_RE;
    COM_USART->TDR = Com_Buff_Tx[Com_Buff_Tx_Read++];
  }
  
  return wLen;
}

/* override */
uint32_t COM_1WIREBytesToRead(void)
{
  if(Com_Buff_Rx_Read > Com_Buff_Rx_Write)
      return  (0xff-Com_Buff_Rx_Read) + Com_Buff_Rx_Read;
  
  return (Com_Buff_Rx_Write - Com_Buff_Rx_Read);
}

/**/
int32_t COM_1WIRE_ReadByte(void)
{
  return Com_Buff_Rx[Com_Buff_Rx_Read++];
}

/**/
int COM_1WIRE_GetByte(uint8_t *p_out)
{
  int dx = COM_1WIREBytesToRead();
  if (dx > 0)
  {
    *p_out = COM_1WIRE_ReadByte();
    return (dx);
  }
  return (0);
}

/**/
int32_t COM_1WIRE_Read(void *pbuff, int32_t StrLoc, int32_t wLen)
{
  int32_t plen = COM_1WIREBytesToRead();
  if(plen > 0)
  {
    if(plen > wLen)
      plen = wLen;
    for(StrLoc=0;StrLoc<plen;StrLoc++)
    {
      ((uint8_t*)pbuff)[StrLoc] = Com_Buff_Rx[Com_Buff_Rx_Read++];
    }
  }
  /**/
  return plen;
}
/**/
void COM_1WIRE_Init(void)
{
  /**/
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Enable GPIO clock */
  COM_TX_PORT_CLK_EN();
  COM_RX_PORT_CLK_EN();
  
  /* Enable USART clock */
  COM_USART_CLK_EN();
  
  /* Connect PXx to COM_USART_Tx */
  GPIO_PinAFConfig(COM_TX_PORT, COM_TX_PIN_SOURCE, COM_TX_AF);
  
  /* Connect PXx to COM_USART_Rx */
  GPIO_PinAFConfig(COM_RX_PORT, COM_RX_PIN_SOURCE, COM_RX_AF);
  
  /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = COM_TX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(COM_TX_PORT, &GPIO_InitStructure);
  
  /* Configure USART Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = COM_RX_PIN;
  GPIO_Init(COM_RX_PORT, &GPIO_InitStructure);
  
  /* COM_USART configured as follow:
  - BaudRate = 115200 baud  
  - Word Length = 8 Bits
  - One Stop Bit
  - No parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 230400;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  /* USART configuration */
  USART_Init(COM_USART, &USART_InitStructure);
  
  /* Enable the COM_USART Interrupt */
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = COM_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = IRQ_Priority_LOW;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable USART */
  USART_Cmd(COM_USART, ENABLE);
  
  USART_ITConfig(COM_USART, USART_IT_RXNE, ENABLE);
  USART_ITConfig(COM_USART, USART_IT_TC, ENABLE);
}

///**
//* @brief  Retargets the C library printf function to the USART.
//* @param  None
//* @retval None
//*/
//PUTCHAR_PROTOTYPE
//{
//  /* Place your implementation of fputc here */
//  /* e.g. write a character to the USART */
//  USART_SendData(COM_USART, (uint8_t) ch);
//  
//  /* Loop until the end of transmission */
//  while (USART_GetFlagStatus(COM_USART , USART_FLAG_TC) == RESET)
//  {}
//  
//  return ch;
//}