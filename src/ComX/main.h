/**
******************************************************************************
* @file    USB_Example/main.h 
* @author  MCD Application Team
* @version V1.1.0
* @date    20-September-2012
* @brief   Header for main.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
/**/
#ifdef __cplusplus
extern "C" {
#endif
  
  /* Includes ------------------------------------------------------------------*/
#include "stm32f30x.h"
#include <stdio.h>
#include "usb_lib.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "platform_config.h"
#include "math.h"
#include "usb_istr.h"
#include "stm32f30x_it.h"
#include "usb_desc.h"
  
#include <stdio.h>
  
  /* Exported types ------------------------------------------------------------*/
  /* ���� ����ָ�� ��CMD��ʼ */
  typedef void (*pComXRxEvent)(uint8_t *p_data,uint8_t DataLength,uint8_t PortNum);
  typedef void (*pComXDebugEvent)(void);
  /* Exported constants --------------------------------------------------------*/
  /* Exported macro ------------------------------------------------------------*/
  #define BootloaderInfAddr       (12*1024)
  
#define IRQ_Priority_LOW        6
#define CodeAddrOffset          (16*1024)
#define ProductString           "SharkGimbal"
#define MakeVersion             100
  

  
  typedef struct{
	uint8_t Cmd;
	uint8_t Len;
	uint8_t *p_data;
  }t_ComRecData;
  
  /* Exported functions ------------------------------------------------------- */
  extern uint32_t Timer_GetCount(void);
  extern uint32_t Timer_CalDiff(uint32_t OldTime);
  extern uint32_t Timer_ToUs(uint32_t tx);
  extern uint32_t Timer_FromUs(uint32_t us);
  extern void Timer_Delay_ms(uint32_t x);
  extern void Timer_Delay_us(uint32_t x);
  extern uint32_t Timer_ToMs(uint32_t tx);
  extern uint32_t Timer_FromMs(uint32_t ms);
  
  extern int32_t USBDevice_Init(void);
  extern uint8_t* USBDevice_Read(int32_t* pOut_Length);
  extern void USBDevice_WriteData(uint8_t Cmd, uint8_t *DataPoint, int32_t DataLen);
  
  extern void COM_1WIRE_Init(void);
  extern int COM_1WIRE_until_tx(int time_out);
  extern int32_t COM_1WIRE_Write(void *pbuff, int32_t StrLoc, int32_t wLen);  
  extern int32_t COM_1WIRE_Read(void *pbuff, int32_t StrLoc, int32_t wLen);
  extern int COM_1WIRE_GetByte(uint8_t *p_out);
  extern int32_t COM_1WIRE_ReadByte(void);
  extern uint32_t COM_1WIREBytesToRead(void);
  
  
  extern void ComX_uPack_Init(void);
  extern uint8_t *ComX_CheckData(int32_t *pOut_Length);
  extern int32_t ComX_SendPack(uint8_t *p_Data,uint32_t Pos_Str,uint32_t DataLength);
  
  extern void ComX_Process(pComXRxEvent ComXRxEventCallback,pComXDebugEvent pComXDebugEventCallBack);
  
  
#define LED1_CLK_EN()                   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE)
#define LED1_PORT                       GPIOB  
#define LED1_PIN                        GPIO_Pin_2
#define LED1_OPEN()                     GPIOB->BSRR = LED1_PIN;
#define LED1_CLS()                      GPIOB->BRR = LED1_PIN;
  
  
  
  
  
  
  
  
  
  /**/
#define BUILD_YEAR_CH0 (__DATE__[ 7])
#define BUILD_YEAR_CH1 (__DATE__[ 8])
#define BUILD_YEAR_CH2 (__DATE__[ 9])
#define BUILD_YEAR_CH3 (__DATE__[10])
  
#define BUILD_MONTH_IS_JAN (__DATE__[0] == 'J' && __DATE__[1] == 'a' && __DATE__[2] == 'n')
#define BUILD_MONTH_IS_FEB (__DATE__[0] == 'F')
#define BUILD_MONTH_IS_MAR (__DATE__[0] == 'M' && __DATE__[1] == 'a' && __DATE__[2] == 'r')
#define BUILD_MONTH_IS_APR (__DATE__[0] == 'A' && __DATE__[1] == 'p')
#define BUILD_MONTH_IS_MAY (__DATE__[0] == 'M' && __DATE__[1] == 'a' && __DATE__[2] == 'y')
#define BUILD_MONTH_IS_JUN (__DATE__[0] == 'J' && __DATE__[1] == 'u' && __DATE__[2] == 'n')
#define BUILD_MONTH_IS_JUL (__DATE__[0] == 'J' && __DATE__[1] == 'u' && __DATE__[2] == 'l')
#define BUILD_MONTH_IS_AUG (__DATE__[0] == 'A' && __DATE__[1] == 'u')
#define BUILD_MONTH_IS_SEP (__DATE__[0] == 'S')
#define BUILD_MONTH_IS_OCT (__DATE__[0] == 'O')
#define BUILD_MONTH_IS_NOV (__DATE__[0] == 'N')
#define BUILD_MONTH_IS_DEC (__DATE__[0] == 'D')
  
  
#define BUILD_MONTH_CH0 \
  ((BUILD_MONTH_IS_OCT || BUILD_MONTH_IS_NOV || BUILD_MONTH_IS_DEC) ? '1' : '0')
	
#define BUILD_MONTH_CH1 \
	( \
	  (BUILD_MONTH_IS_JAN) ? '1' : \
		(BUILD_MONTH_IS_FEB) ? '2' : \
		  (BUILD_MONTH_IS_MAR) ? '3' : \
			(BUILD_MONTH_IS_APR) ? '4' : \
			  (BUILD_MONTH_IS_MAY) ? '5' : \
				(BUILD_MONTH_IS_JUN) ? '6' : \
				  (BUILD_MONTH_IS_JUL) ? '7' : \
					(BUILD_MONTH_IS_AUG) ? '8' : \
					  (BUILD_MONTH_IS_SEP) ? '9' : \
						(BUILD_MONTH_IS_OCT) ? '0' : \
						  (BUILD_MONTH_IS_NOV) ? '1' : \
							(BUILD_MONTH_IS_DEC) ? '2' : \
							  /* error default */    '?' \
								)
	  
#define BUILD_DAY_CH0 ((__DATE__[4] >= '0') ? (__DATE__[4]) : '0')
#define BUILD_DAY_CH1 (__DATE__[ 5])
	  
	  
	  // Example of __TIME__ string: "21:06:19"
	  //                              01234567
	  
#define BUILD_HOUR_CH0 (__TIME__[0])
#define BUILD_HOUR_CH1 (__TIME__[1])
	  
#define BUILD_MIN_CH0 (__TIME__[3])
#define BUILD_MIN_CH1 (__TIME__[4])
	  
#define BUILD_SEC_CH0 (__TIME__[6])
#define BUILD_SEC_CH1 (__TIME__[7])
	  
	  
	  
	  /**/
	  
#define CI_Debug_String         2       /* ?????       */
#define CI_TRst                 3       /* ????         */
#define CI_RStat                4       /* ????         */
#define CI_SetBLDC              5       /* ?????       */ 
#define CI_Calibrate            6       /* ????         */
#define CI_SetLevel             7       /* ?????       */
#define CI_Debug_1              9       /* ????1        */
#define CI_SetSysPram1          10      /* ??????1    */
#define CI_SetSysPram2          11      /* ????2        */
#define CI_GetAdcStr            12      /* ??ADC???     */
#define CI_GetMacDesc           13      /* ???????    */
#define CI_DebugButton1         14      /* ????1         */
#define CI_DataPack             15      /* ??????      */
#define CI_RCVal                16      /* RC??             */
#define CI_RestSenser           17      /* ?????          */
#define CI_ToolsErr             18      /* ????????     */
#define CI_SysInf               19      /* ????                 */
#define CI_GetRecData           20      /* ????????             */
#define CI_RecoverSys           21      /* ?????????    */
#define CI_SetahrsPram          22      /* ????ahrs             */
#define CI_CalibrationRc        23      /* ?????                */ 
#define CI_SimplePram1          24      /*  ????                */
#define CI_UpdateOsd            25      /*  ??OSD               */
#define CI_RequestLog           26      /*  ????                */
#define CI_SetGimbalPar         27      /*  ??????              */
#define CI_CaliGimbalPar        39
#define CI_SetRcSys             28      /*  ???????             */
#define CI_FlyData_V2           29      /* ????V2 */
  

	  
      
      
    #define CI_UpdataMC             197 	/* ���� */
	#define CI_UpdataDATA             198 	/* ���� */  
	  /**/
	  
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
