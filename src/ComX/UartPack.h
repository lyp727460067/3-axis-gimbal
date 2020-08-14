/**/
#pragma once
/**/

typedef struct  {
  
  // public:
  uint16_t   PackErrCount;
  uint16_t   RecPackCnt,SendPackCnt;
  /**/
  int(*p_fun_GetByte)(uint8_t *p_out);
  uint32_t(*p_BytesToRead)(void);
  int32_t(*p_ReadBytes)(uint8_t *pbuff, int32_t StrLoc, int32_t wLen);
  int32_t(*p_WriteBytes)(uint8_t *pbuff, int32_t StrLoc, int32_t wLen);
  

  // private:
  int16_t      wPos_RxBuff;
  int16_t      rPos_RxBuff;
  uint8_t      *p_raw_data;
  int16_t      Buff_MaxLength;
  
  
  uint8_t       DataLength;
  uint8_t       State;
  uint8_t       PackLen;
  uint8_t       PackCheck;
} UartPack ;


/**/
void UartPack_Init(UartPack *this,uint8_t *p_DataBuff, uint16_t DataMaxNum);
void UartPack_ResetRx(UartPack *this); /* 复位内部接收状态 */
int32_t UartPack_WritePack(UartPack *this,uint8_t *pData, int32_t wLen);
int32_t UartPack_WritePack_Cmd(UartPack *this,uint8_t Cmd,uint8_t *pData, int32_t wLen);
/*
    0xa5,num,data...,chk,0x5a 
   参数: 返回数据长度
   返回: 数据指针  指向data第一个字节 
*/
uint8_t* UartPack_CheckData(UartPack *this,int32_t *pOut_Length); 
uint8_t* UartPack_CheckData1(UartPack *this,int32_t *pOut_Length); 
