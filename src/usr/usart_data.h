#ifndef URART_DATA_H
#define URART_DATA_H
#include "stm32f30x.h"

#ifdef  DEFIN_URARTDATA_EXTERN
	#define  URARTDATA_EXTERN
#else 
	#define  URARTDATA_EXTERN extern 
#endif

typedef struct{
    
  int16_t iCurrent[2];
  int16_t SensorAngle[2];
  int16_t i_qd_data[2];
  int16_t FocAngle;
  
}t_MotoDebugData1;

typedef struct {
  uint8_t CMD;
  uint8_t dLen;
  uint8_t Flag;
  uint8_t Flag1;  
  
  t_MotoDebugData1 MotoData[3];

  int16_t Imu_Gyro[3];
  int16_t Imu_Acc[3];
  int16_t Imu_EurAngle[3];
  
}t_MotorDebugData;





/*每10MS 发送一次
当发送命令下来后 云台再回复数据上去
要控制云台，直接将 tGimbalCtrData 的数据填充到 data 区域通过 串口发送。
*/


/* 
  串口硬件协议 115200，1停止位，无校验 

  软件数据包格式 ，每一个符号占用一个字节
  [0xa5][length][CMD][dLength][..data..][checksum][0x5a]
  0xa5    :
  0x5a    :
  length  : 从0xa5到0x5a的长度
  checksum: data区的checksum
  CMD     : 命令编号 
  dLength : CMD 到 data 的长度 
  checksum = 0x00^CMD^dLength^data[0]....^data[n]
*/


/* 云台控制数据 ，由飞控发送到云台 CMD=192 */
/* CMD=193，当飞控发下来的数据会发上去 */

  typedef struct
  {
    /*控制模式 0:航向跟随模式 1:航行锁定模式  */
    uint8_t ctr_Mode;
    /* 校准命令 0:无请求 1:校准陀螺仪  2:准备校准水平(开始无力,准备云台放水平完毕后发送 3) 
    3:开始校准水平  4:校准FOC
    校准的时候保持一个命令一秒以上 ，然后跳变到0后，校准此时开始生效*/
    uint8_t Calibrate;
    /* 飞控当前的姿态角，单位 =  角度 * 100 */
    int16_t Pitch;
    int16_t Roll;
    int16_t Yaw; 
    /* 云台目标姿态角，单位 =  角度 * 100 */ 
    int16_t ctr_Pitch;
    int16_t ctr_Roll; 
    int16_t ctr_Yaw; 
    
    int16_t gps[2];//航向呵速度
  }
  tGimbalCtrData;

  /* 云台反馈数据 CMD=191 */
  typedef struct
  {
    /* 校准状态 */
    uint16_t Calibrate;
    /* 云台当前的姿态角，单位 =  角度 * 100 */
    int16_t Pitch;
    int16_t Roll;
    int16_t Yaw;
  }
  tGimbalRetData; 

  
  

void UartSend(void);
void UartRec(void);//5ms

#endif