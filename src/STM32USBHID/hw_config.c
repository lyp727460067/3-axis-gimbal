/**
  ******************************************************************************
  * @file    USB_Example/hw_config.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    20-September-2012
  * @brief   Hardware Configuration & Setup.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F3_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup USB_Example
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ErrorStatus HSEStartUpStatus;
EXTI_InitTypeDef EXTI_InitStructure;

/* Extern variables ----------------------------------------------------------*/
extern __IO uint8_t PrevXferComplete;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/



/**
  * @brief  Configures USB Clock input (48MHz).
  * @param  None
  * @retval None
  */
void Set_USBClock(void)
{
  /* USBCLK = PLLCLK = 48 MHz */
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
  /* Enable USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
}

/**
  * @brief  Power-off system clocks and power while entering suspend mode.
  * @param  None
  * @retval None
  */
void Enter_LowPowerMode(void)
{
  /* Set the device state to suspend */
  bDeviceState = SUSPENDED;
} 

/**
  * @brief  Restores system clocks and power while exiting suspend mode.
  * @param  None
  * @retval None
  */
void Leave_LowPowerMode(void)
{
  DEVICE_INFO *pInfo = &Device_Info;
//
//  /* Enable HSE */
//  RCC_HSEConfig(RCC_HSE_ON);
//
//  /* Wait till HSE is ready */
//  HSEStartUpStatus = RCC_WaitForHSEStartUp();
//
//
//  /* Wait till HSE is ready */
//  while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)
//  {}
//   
//  /* Enable PLL1 */
//  RCC_PLLCmd(ENABLE);
//
//  /* Wait till PLL1 is ready */
//  while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
//  {}
//
//  /* Select PLL as system clock source */
//  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
//
//  /* Wait till PLL is used as system clock source */ 
//  while (RCC_GetSYSCLKSource() != 0x08)
//  {}  

 /* Set the device state to the correct state */
  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
  }
  else
  {
    bDeviceState = ATTACHED;
  }
}

/**/
void USB_Cable_Config (FunctionalState NewState)
{
}

/**
  * @brief  Configures the USB interrupts.
  * @param  None
  * @retval None
  */
void USB_Interrupts_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* 2 bit for pre-emption priority, 2 bits for subpriority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  /* Enable the USB interrupt */
#if defined (USB_INT_DEFAULT)
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
#endif
#if defined (USB_INT_REMAP)
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_IRQn;
#endif 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable the USB Wake-up interrupt */
#if defined (USB_INT_DEFAULT)
  NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
#endif
#if defined (USB_INT_REMAP)  
  NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_RMP_IRQn;
#endif  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);      

//  /* Enable the Key EXTI line Interrupt */
//  NVIC_InitStructure.NVIC_IRQChannel = USER_BUTTON_EXTI_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//  NVIC_Init(&NVIC_InitStructure);
}


/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/ 
