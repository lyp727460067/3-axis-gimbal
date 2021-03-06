/**
  ******************************************************************************
  * @file    usb_desc.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    20-September-2012
  * @brief   Descriptors Header for Demo
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
#ifndef __USB_DESC_H
#define __USB_DESC_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
#define USB_DEVICE_DESCRIPTOR_TYPE              0x01
#define USB_CONFIGURATION_DESCRIPTOR_TYPE       0x02
#define USB_STRING_DESCRIPTOR_TYPE              0x03
#define USB_INTERFACE_DESCRIPTOR_TYPE           0x04
#define USB_ENDPOINT_DESCRIPTOR_TYPE            0x05

#define HID_DESCRIPTOR_TYPE                     0x21
#define HID_Comm_SIZ_HID_DESC                   0x09
#define HID_Comm_OFF_HID_DESC                   0x12

#define HID_Comm_SIZ_DEVICE_DESC                18
#define HID_Comm_SIZ_CONFIG_DESC                41
#define HID_Comm_SIZ_REPORT_DESC                35
#define HID_Comm_SIZ_STRING_LANGID              4
#define HID_Comm_SIZ_STRING_VENDOR              38
#define HID_Comm_SIZ_STRING_PRODUCT             30
#define HID_Comm_SIZ_STRING_SERIAL              26

#define STANDARD_ENDPOINT_DESC_SIZE             0x09

/* Exported functions ------------------------------------------------------- */
extern const uint8_t HID_Comm_DeviceDescriptor[HID_Comm_SIZ_DEVICE_DESC];
extern const uint8_t HID_Comm_ConfigDescriptor[HID_Comm_SIZ_CONFIG_DESC];
extern const uint8_t HID_Comm_ReportDescriptor[HID_Comm_SIZ_REPORT_DESC];
extern const uint8_t HID_Comm_StringLangID[HID_Comm_SIZ_STRING_LANGID];
extern const uint8_t HID_Comm_StringVendor[HID_Comm_SIZ_STRING_VENDOR];
extern const uint8_t HID_Comm_StringProduct[HID_Comm_SIZ_STRING_PRODUCT];
extern uint8_t HID_Comm_StringSerial[HID_Comm_SIZ_STRING_SERIAL];

#endif /* __USB_DESC_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/ 
