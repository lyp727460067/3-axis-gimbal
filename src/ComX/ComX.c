/**/

#include "main.h"
#include "UartPack.h"
#include "stdlib.h"
#include "device.h"
/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  uint8_t   HWVer;
  uint8_t   MVer;
  uint8_t   fwver;
  uint32_t  ChipId;
  uint32_t  FlyTime;
  uint32_t  BuildTime;
  uint8_t  DesStr[128];
}tFW_Information;

/* Private function prototypes -----------------------------------------------*/
static void SendDescribeString(void);

/**/
static uint8_t  Com1_DataArray[256];
static UartPack UartPack_Com1;
static int32_t uPackRxNumber;
static uint8_t *p_uPackData;
static uint32_t	OldTime1;
static uint8_t  GetOscDataReq; 
static uint8_t  TimerToSendOscData;
/* Private variables ---------------------------------------------------------*/
static const char DataName[] =
"GyroX,GyroY,GyroZ,AccX,AccY,AccZ,AttX,AttY,AttZ,M1IA,\
M1IB,M2IA,M2IB,M3IA,M3IB,M1OrgAng,M2OrgAng,M3OrgAng,M1AngRe,M2AngRe,\
  M3AngRe,M1ID,M1IQ,M2ID,M2IQ,M3ID,M3IQ,M1FocAng,M2FocAng,M3FocAng,\
	GSX,GSY,CPUz,bAccX,bAccY,bAccZ,x1,x2,x3,x4,\
	  x5,x6,x7,x8,x9,x10,x11,x12,x13,x14";
		
		static const char App_BuildDate_Str[] =  {
		  /**/
		  BUILD_YEAR_CH2, BUILD_YEAR_CH3,
		  //'.',
		  BUILD_MONTH_CH0, BUILD_MONTH_CH1,
		  //'.',
		  BUILD_DAY_CH0, BUILD_DAY_CH1,
		  BUILD_HOUR_CH0, BUILD_HOUR_CH1,
		  //'.',
		  BUILD_MIN_CH0, BUILD_MIN_CH1,
		  //'.',
		  BUILD_SEC_CH0, BUILD_SEC_CH1,
		  0
		};

/* Public variables ---------------------------------------------------------*/
int32_t DeBug_OscData[50];

/**/
void ComX_uPack_Init(void)
{
  UartPack_Init(&UartPack_Com1,Com1_DataArray,sizeof(Com1_DataArray));
}

/**/
uint8_t *ComX_CheckData(int32_t *pOut_Length)
{
  UartPack_Com1.p_fun_GetByte = &COM_1WIRE_GetByte;
  UartPack_Com1.p_BytesToRead = &COM_1WIREBytesToRead;
  UartPack_Com1.p_ReadBytes = (int32_t(*)(uint8_t*,int32_t,int32_t))&COM_1WIRE_Read;
  return UartPack_CheckData(&UartPack_Com1,pOut_Length);
}

/**/
int32_t ComX_SendPack(uint8_t *p_Data,uint32_t Pos_Str,uint32_t DataLength)
{
  UartPack_Com1.p_BytesToRead = &COM_1WIREBytesToRead;
  UartPack_Com1.p_WriteBytes = (int32_t(*)(uint8_t*,int32_t,int32_t))&COM_1WIRE_Write;
  int32_t ret_res = UartPack_WritePack(&UartPack_Com1,p_Data,DataLength);
  
  return ret_res;
}


/**/
void ComX_Init(void)
{
  COM_1WIRE_Init();
#ifdef IS_MAIN_CONTROLLOR
  USBDevice_Init();
#endif
  ComX_uPack_Init();
  OldTime1 = Timer_GetCount();
}


/**/
void  ComX_Process(pComXRxEvent ComXRxEventCallback,pComXDebugEvent pComXDebugEventCallBack)
{
  if(Timer_ToMs(Timer_CalDiff(OldTime1)) < 2)
	return ;
  
  OldTime1 = Timer_GetCount();
  
  TimerToSendOscData++;
  
  if(TimerToSendOscData>9)
  {
	TimerToSendOscData = 0;
	if(GetOscDataReq < 50)
	{
	  GetOscDataReq++;
	  pComXDebugEventCallBack();
	  USBDevice_WriteData(CI_Debug_1, (uint8_t*)DeBug_OscData, sizeof(DeBug_OscData));
	}
  }
 
  p_uPackData = ComX_CheckData(&uPackRxNumber);
  if(uPackRxNumber > 0)
  {
	ComXRxEventCallback(&p_uPackData[0],uPackRxNumber,1);
	//USBDevice_WriteData(p_uPackData[0],&p_uPackData[1],uPackRxNumber-1);
  }
  
  p_uPackData = USBDevice_Read(&uPackRxNumber);
  if(uPackRxNumber > 0)
  {
	if(p_uPackData[4] == CI_GetMacDesc)
	  SendDescribeString();
	if(p_uPackData[4] == CI_SetBLDC)
	  GetOscDataReq = 0;
	
	ComXRxEventCallback(&p_uPackData[4],uPackRxNumber-4,0);
  }
}

/*
����:��ȡоƬID
����:void CSP_GetChipID(uint8_t *datap)
����:ID ����λ�� ,��ȡ����λ ID ����12λ
����ֵ:��
*/
uint32_t HWC_GetChipID(uint8_t *datap, uint8_t ReadLen) {
  
  uint8_t tpci;
  uint8_t * tpcpi,*tpcpa;
  uint32_t Db[4];
  tpcpi = (uint8_t*)&Db[0];
  tpcpa = (uint8_t*)0x1FFFF7E8;   //0x1FFFF7E8 ~ 0x1FFFF7F4
  for (tpci = 0; tpci < 12; tpci++)
	tpcpi[tpci] = tpcpa[tpci];
  for (tpci = 0; tpci < ReadLen; tpci++)
	datap[tpci] = tpcpi[tpci];
  /**/
  Db[0] = (Db[0] * Db[1] * Db[2]);
  return Db[0];
}

/**/
static void SendDescribeString(void)
{
  uint32_t                         fwBuildTime;
  fwBuildTime = atoi(&App_BuildDate_Str[2]);
  tFW_Information FW_Inf;
  
  FW_Inf.HWVer      = 10;
  FW_Inf.MVer       = 10;
  FW_Inf.fwver      = MakeVersion;
  FW_Inf.ChipId     = HWC_GetChipID(0, 0);  /* оƬID */
  FW_Inf.FlyTime    = 0;
  FW_Inf.BuildTime  = fwBuildTime;
  /**/
  FW_Inf.DesStr[0] = 0;
  
  strcat((char*)&FW_Inf.DesStr[0], (char*)ProductString);
  
  uint8_t IsIdLock = 0;
  if (IsIdLock != 0) 
	strcat((char*)&FW_Inf.DesStr[0], (char*)"DevKeyError");
  strcat((char*)&FW_Inf.DesStr[0], (char*)",Build:");
  strcat((char*)&FW_Inf.DesStr[0], (char*)App_BuildDate_Str);
  strcat((char*)&FW_Inf.DesStr[0], (char*)"\r\nwww.fly-shark.com");
  /**/
  USBDevice_WriteData(CI_GetMacDesc, (uint8_t*)&FW_Inf,  sizeof(tFW_Information));
  /**/
  Timer_Delay_ms(500);
  /*����ADC�ַ�*/
  USBDevice_WriteData(CI_GetAdcStr, (uint8_t*)DataName, sizeof(DataName));
  Timer_Delay_ms(100);
  /*����ADC�ַ�*/
  USBDevice_WriteData(CI_GetAdcStr, (uint8_t*)DataName, sizeof(DataName));
}

