/**/
#include "main.h"
#include "UartPack.h"

/**/
#define IF_StrMarker 0xa5
#define IF_EndMarker 0x5a

uint8_t* UartPack_prvCheckData(UartPack *this,int32_t *pOut_Length);

/**/
void UartPack_Init(UartPack *this,uint8_t *p_DataBuff,uint16_t DataMaxNum)
{
  this->RecPackCnt=0;
  this->PackErrCount=0;
  this->p_raw_data = p_DataBuff;
  this->Buff_MaxLength = DataMaxNum;
}

/**/
void UartPack_ResetRx(UartPack *this)
{
  this->wPos_RxBuff     = 0;
  this->rPos_RxBuff     = 0;
}

/* */
uint8_t* UartPack_CheckData(UartPack *this,int32_t *pOut_Length)
{
  uint8_t data_ret;
  while (this->p_fun_GetByte(&data_ret) > 0) 
  {
    switch(this->State)
    {
    case 2:
      this->PackCheck ^= data_ret;
      this->p_raw_data[this->wPos_RxBuff++] = data_ret; 
      this->PackLen--;
      if(this->PackLen == 4)
        this->State = 3;
      break;
      
    case 0:
      if(data_ret == 0xa5)
        this->State = 1;
      this->wPos_RxBuff = 0; 
      this->PackCheck = 0;
      break;
      
    case 1:
      this->DataLength = data_ret - 4;
      this->PackLen = data_ret;
      this->State = 2;
      this->p_raw_data[this->wPos_RxBuff++] = 0xa5;
      this->p_raw_data[this->wPos_RxBuff++] = data_ret;
      break;

    case 3:
      if(data_ret == this->PackCheck)
      {
        this->State = 4;
        this->p_raw_data[this->wPos_RxBuff++] = data_ret;
      }
      else
      {
        this->PackErrCount++;
        this->State = 0;
      }
      break;
      
    case 4:
      this->State = 0;
      if(data_ret == 0x5a)
      {
        this->p_raw_data[this->wPos_RxBuff++] = data_ret;
        this->RecPackCnt++;
        *pOut_Length = this->DataLength;
        return (&this->p_raw_data[2]); 
      }
      this->PackErrCount++;
      break;
    }
  }
  *pOut_Length = 0;
  return 0;
}

/**/
uint8_t* UartPack_CheckData1(UartPack *this,int32_t *pOut_Length)
{
  uint8_t *p_ret; 
  *pOut_Length = 0;
  do
  {
    p_ret = UartPack_prvCheckData(this,pOut_Length); 
  }
  while (*pOut_Length == -10 || *pOut_Length == -9); 
  
  return (p_ret);
}

/**/
uint8_t* UartPack_prvCheckData(UartPack *this,int32_t *pOut_Length)
{
  int16_t str;
  int16_t len;
  int16_t dstr;
  /* 读取数据 */
  int32_t d_num = this->p_BytesToRead();
  if((d_num+this->wPos_RxBuff) >= this->Buff_MaxLength)
    d_num = this->Buff_MaxLength - this->wPos_RxBuff;
  this->p_ReadBytes(&this->p_raw_data[this->wPos_RxBuff],0,d_num);
  this->wPos_RxBuff += d_num;
  
  /* 寻找开始标记 */
  for(;this->rPos_RxBuff < this->wPos_RxBuff;)
  {
    if (this->p_raw_data[this->rPos_RxBuff] == IF_StrMarker)
      break;
    this->rPos_RxBuff++;
  }
  /* 预处理 */
  str = this->rPos_RxBuff;
  len = this->wPos_RxBuff - this->rPos_RxBuff;
  if(len < 1)
  {
    this->wPos_RxBuff = this->rPos_RxBuff = 0;
    *pOut_Length = -1;
    return 0;
  }  
  /* 长度不够 */
  if (len < 2)    {
    *pOut_Length = -2;
    return 0;
  }
  
  dstr = str;
  /* 检查长度 */
  int plen = this->p_raw_data[str+1];
  
  /* 永远等不到的长度 */
  if((plen+str) > this->Buff_MaxLength)
  {
    this->rPos_RxBuff++;
    //this->wPos_RxBuff = this->rPos_RxBuff = 0;
    *pOut_Length = -10;
    return 0;
  }
  /* 如果数据量还不够 */
  if (len < (plen - 2))
  {
    *pOut_Length = -3;
    return 0;
  }
  

  /* 结束符 */
  if (this->p_raw_data[str + plen - 1] == IF_EndMarker)
  {
    /* 计算校验 */
    uint8_t chk = 0;
    int a, k = plen - 4;
    str += 2;
    for (a = 0; a < k; a++)
      chk ^= this->p_raw_data[str++];
    /**/
    if (this->p_raw_data[str] == chk)
    {
      this->rPos_RxBuff = (str + 2); 
      this->RecPackCnt ++;
      *pOut_Length = k;
      return &this->p_raw_data[dstr+2];
    }
    else
    {
      this->rPos_RxBuff++;
      this->PackErrCount++;
    }
  }
  else
  {
    this->rPos_RxBuff ++;
  }
  
  *pOut_Length = -9;
  
  return 0;
}

/* */
int32_t UartPack_WritePack(UartPack *this,uint8_t *pData, int32_t wLen) {
  
  uint8_t list1[]={IF_StrMarker,wLen + 4};
  uint8_t chk;
  int wcnt;
  
  this->p_WriteBytes(list1,0,2);
  
  chk = 0;
  for (wcnt = 0; wcnt < wLen; wcnt++) 
    chk ^= pData[wcnt]; 
  this->p_WriteBytes(pData,0,wLen);
  list1[0] = chk;
  list1[1] = IF_EndMarker;
  this->p_WriteBytes(list1,0,2);
  
  this->SendPackCnt++;
  /**/
  return 0;
}

/* */
int32_t UartPack_WritePack_Cmd(UartPack *this,uint8_t Cmd,uint8_t *pData, int32_t wLen) {
  
  uint8_t list1[4]={IF_StrMarker,wLen + 6,Cmd,wLen};
  uint8_t chk;
  int wcnt;
  
  this->p_WriteBytes(list1,0,4);
  
  chk = 0;
  chk ^= Cmd;
  chk ^= wLen;
  for (wcnt = 0; wcnt < wLen; wcnt++) 
    chk ^= pData[wcnt]; 
  this->p_WriteBytes(pData,0,wLen);
  
  list1[0] = chk;
  list1[1] = IF_EndMarker;
  this->p_WriteBytes(list1,0,2);
  
  this->SendPackCnt++;
  /**/
  return 0;
}
