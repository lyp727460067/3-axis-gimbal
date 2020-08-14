/**/

#define DEFIN_PCTOGIMBAL_EXTERN
#include "pctogimbal.h"
#include "UartPack.h"
#include "flash.h"
#include "device.h"
#include <string.h>
#include "main.h"
#include "usart_data.h"
#include "Control.h"
#include "AngSensor.h"
#include  "datacalibrate.h"
#include  "factory.h"
#include "pwm.h"

#define ComDataFlag		p_data[2]



t_MotorDebugData  iMotorDebugData;
uint8_t	  ReqMotoDataState = 0x02;
extern int32_t DeBug_OscData[];

/**/
void set_gimbalpra(uint8_t *p_data ,uint16_t lenth)
{
    if((ComDataFlag&0x0f) > 1)
        return ;
    
    if(g_ucDeviceNum == 0X00 && (ComDataFlag&0xf0) == 0x00){
        if((ComDataFlag&0x0f) == 0x02){
            //g_SaveTunigflashstate = 1;	
            //save to flash
        }else if((ComDataFlag&0x0f) == 0x01){//write
            
            memcpy((void*)&Tuning_data,(void*)(p_data),sizeof(Tuning_data));
            ComX_SendPack(p_data,0,sizeof(Tuning_data));
            
            g_SaveTunigflashstate = 1;
            g_ucUpdatePraFlag = 1;	
            g_ucFlashWriteFlag = 1;
        }else if((ComDataFlag&0x0f) == 0x00){
            Tuning_data.CMD = 27;
            Tuning_data.dLen = sizeof(Tuning_data);
            Tuning_data.Flag = 0x00;
            ComX_SendPack((uint8_t*)&Tuning_data,0,sizeof(Tuning_data));		
        }
    }
    
    if(g_ucDeviceNum == 0X01 && (ComDataFlag&0xf0) == 0x10){
        if((ComDataFlag&0x0f) == 0x02){
            //g_SaveTunigflashstate = 1;	
            //save to flash
        }else if((ComDataFlag&0x0f) == 0x01){//write
            
            memcpy((void*)&Tuning_data,(void*)(p_data),sizeof(Tuning_data));
            ComX_SendPack(p_data,0,sizeof(Tuning_data));
            g_ucFlashWriteFlag = 1;
            g_SaveTunigflashstate = 1;
            g_ucUpdatePraFlag = 1;	
        }else if((ComDataFlag&0x0f) == 0x00){
            Tuning_data.CMD = 27;
            Tuning_data.dLen = sizeof(Tuning_data);
            Tuning_data.Flag = 0x10;
            ComX_SendPack((uint8_t*)&Tuning_data,0,sizeof(Tuning_data));		
        }
    }
    
    /*-----------------------------------------*/
    if(g_ucDeviceNum == 0X2)
    {	
        switch(ComDataFlag&0xf0){
        case 0x00://设备0回复 设置成功
            memcpy((void*)motor1pidptr,(void*)(p_data+slave1paralen+slave2paralen+4),masterparalen);
            if((ComDataFlag&0x0f) == 0x00){				
                Tuning_data.CMD = 27;
                Tuning_data.dLen = sizeof(Tuning_data);
                Tuning_data.Flag = 0x10|0x00;	
                ComX_SendPack((uint8_t*)&Tuning_data,0,sizeof(Tuning_data));
            }else if((ComDataFlag&0x0f) == 0x01){
                Tuning_data.CMD = 27;
                Tuning_data.Flag = 0x10|0x01;
                Tuning_data.dLen = sizeof(Tuning_data);
                ComX_SendPack((uint8_t*)&Tuning_data,0,sizeof(Tuning_data));
            }
            g_usart2pc_state = USART2PC_TUN_2;
            break;
        case 0x10:
            memcpy((void*)motor2pidptr,(void*)(p_data+slave1paralen+4),slave2paralen);			
            Tuning_data.Flag = (Tuning_data.Flag&0x0f)|0x20;         
            USBDevice_WriteData(CI_SetGimbalPar,(uint8_t*)&Tuning_data,sizeof(Tuning_data));	
            g_usart2pc_state = USART2PC_TUN_0;
            break;
        case 0x20://PC 下来的数据
            if((ComDataFlag&0x0f)==0x01){		//write		
                memset(&Tuning_data,0,sizeof(Tuning_data));
                memcpy((void*)&Tuning_data,(void*)(p_data),sizeof(Tuning_data));
                Tuning_data.CMD = 27;
                Tuning_data.Flag = 0x00|0x01;
                Tuning_data.dLen = sizeof(Tuning_data);
                ComX_SendPack((uint8_t*)&Tuning_data,0,sizeof(Tuning_data));					
                g_ucUpdatePraFlag = 1;	
                   g_ucFlashWriteFlag = 1;
                //					g_usart2pc_state = USART2PC_DEFAULT;
                //					g_ucUpdatePraFlag = 1;
                //					g_SaveTunigflashstate = 1;//save to flash
                //					data[1] = sizeof(Tuning_data);
                //					ComX_SendPack(data,0,sizeof(Tuning_data));
                
            }else if((ComDataFlag&0x0f)==0x00){						
                Tuning_data.CMD = 27;
                Tuning_data.dLen = sizeof(Tuning_data);
                Tuning_data.Flag = 0x00;	
                ComX_SendPack((uint8_t*)&Tuning_data,0,sizeof(Tuning_data));	
                g_usart2pc_state = USART2PC_TUN_1;					
                //					if(g_usart2pc_state == USART2PC_TUNINOK){
                //						Tuning_data.CMD = 27;
                //						Tuning_data.Flag = sizeof(Tuning_data)-1;	
                //						USBDevice_WriteData(CI_SetGimbalPar,(uint8_t*)&Tuning_data,sizeof(Tuning_data));
                //					}else{
                //						Tuning_data.CMD = 27;
                //						Tuning_data.dLen = 4;
                //						Tuning_data.Flag = 0x00;	
                //						ComX_SendPack((uint8_t*)&Tuning_data,0,sizeof(Tuning_data));	
                //					} 
                
            }
            break;	
        }
    }
    
}
extern uint16_t variance[7];
extern uint16_t Capture1 ;;
extern uint16_t Capture2 ;;
/**/
void ComXDebugCallBack(void)
{
    if(g_ucDeviceNum == 0x02 )
    {
        ReqMotoDataState = 0x00;
        iMotorDebugData.CMD = 27;
        iMotorDebugData.Flag = 0x02;
        iMotorDebugData.dLen = sizeof(iMotorDebugData);
        
        
        
        iMotorDebugData.MotoData[2].iCurrent[0] =ADCCurrent.I_PhaseA;
        iMotorDebugData.MotoData[2].iCurrent[1] =ADCCurrent.I_PhaseB;
        
        iMotorDebugData.MotoData[2].SensorAngle[0] = g_hi0rgangle;
        iMotorDebugData.MotoData[2].SensorAngle[1] = g_hiEMReAngle;
        
        iMotorDebugData.MotoData[2].i_qd_data[0] = (int16_t)g_SDq.x;
        iMotorDebugData.MotoData[2].i_qd_data[1] = (int16_t)g_SDq.y;	
        
        iMotorDebugData.MotoData[2].FocAngle = (int16_t)(g_fEMFocangle*10);
        
        
        ComX_SendPack((uint8_t*)&iMotorDebugData,0,sizeof(iMotorDebugData));
        /**/
        int k1 = 0;
        DeBug_OscData[k1++] = iMotorDebugData.Imu_Gyro[0];
        DeBug_OscData[k1++] = iMotorDebugData.Imu_Gyro[1];
        DeBug_OscData[k1++] = iMotorDebugData.Imu_Gyro[2];
        DeBug_OscData[k1++] = iMotorDebugData.Imu_Acc[0];
        DeBug_OscData[k1++] = iMotorDebugData.Imu_Acc[1];
        DeBug_OscData[k1++] = iMotorDebugData.Imu_Acc[2];
        DeBug_OscData[k1++] = iMotorDebugData.Imu_EurAngle[0];
        DeBug_OscData[k1++] = iMotorDebugData.Imu_EurAngle[1];
        DeBug_OscData[k1++] = iMotorDebugData.Imu_EurAngle[2];
        
        DeBug_OscData[k1++] = iMotorDebugData.MotoData[0].iCurrent[0];
        DeBug_OscData[k1++] = iMotorDebugData.MotoData[0].iCurrent[1];
        
        DeBug_OscData[k1++] = iMotorDebugData.MotoData[1].iCurrent[0];
        DeBug_OscData[k1++] = iMotorDebugData.MotoData[1].iCurrent[1];
        
        DeBug_OscData[k1++] = iMotorDebugData.MotoData[2].iCurrent[0];
        DeBug_OscData[k1++] = iMotorDebugData.MotoData[2].iCurrent[1];
        
        
        DeBug_OscData[k1++] = iMotorDebugData.MotoData[0].SensorAngle[0];
        DeBug_OscData[k1++] = iMotorDebugData.MotoData[1].SensorAngle[0];   
        DeBug_OscData[k1++] = iMotorDebugData.MotoData[2].SensorAngle[0];	//ORG ANG
        
        DeBug_OscData[k1++] = iMotorDebugData.MotoData[0].SensorAngle[1];   //reang
        DeBug_OscData[k1++] = iMotorDebugData.MotoData[1].SensorAngle[1];      
        DeBug_OscData[k1++] = iMotorDebugData.MotoData[2].SensorAngle[1];
        
        
        DeBug_OscData[k1++] = iMotorDebugData.MotoData[0].i_qd_data[0];	         
        DeBug_OscData[k1++] = iMotorDebugData.MotoData[0].i_qd_data[1];  
        
        DeBug_OscData[k1++] = iMotorDebugData.MotoData[1].i_qd_data[0];	         
        DeBug_OscData[k1++] = iMotorDebugData.MotoData[1].i_qd_data[1];  
        
        DeBug_OscData[k1++] = iMotorDebugData.MotoData[2].i_qd_data[0];	         
        DeBug_OscData[k1++] = iMotorDebugData.MotoData[2].i_qd_data[1];  
        
        
        
        DeBug_OscData[k1++] = iMotorDebugData.MotoData[0].FocAngle;
        DeBug_OscData[k1++] = iMotorDebugData.MotoData[1].FocAngle;
        DeBug_OscData[k1++] = iMotorDebugData.MotoData[2].FocAngle;
        
    }
}

extern void MemUsartErerDataToRam(uint8_t* sData,uint8_t lenth);
extern void MemUsart0pcbRecDataToRam(uint8_t*data,uint8_t lenth);
extern  void MemFactory0pcbRecDataToRam(uint8_t*pdata,uint8_t lenth);
extern void MemFactory1pcbRecDataToRam(uint8_t*pdata,uint8_t lenth);
extern void MemFactory2pcbRecDataToRam(uint8_t*pdata,uint8_t lenth);
extern uint8_t CaStatesTopc ;
extern uint8_t  TempeConstantFlag1;
/**/
extern void PCSetClibrateSates(uint8_t states);

/*
  pc-->云台                      
  cmd  data                           
  0x24(2) 0x10校准foc                  
          0x20 校准水平 gry 一面ACC
          0X31 六面ACC 第一面
          0X32 六面ACC 第二面
          0X33六面ACC 第3面
          0X34六面ACC 第4面
          0X35六面ACC 第5面
          0X36 六面ACC 第6面

		  0x40 没力
          
 云台-->pc     
cmd       data
          温度  校准状态
0x25(2)   0x01  0x10校准FOC
                0x11校准FOC完成
                0x20校准一面ACC
                0x21一面ACC完成 开始校准水平
                0x22水平完成 GRY开始
                0x23完成
                0x30六面ACC 开始
                0x31六面ACC第一面完成
                0x32六面ACC第2面完成
                0x33六面ACC第3面完成
                0x34六面ACC第4面完成
                0x35六面ACC第5面完成
                0x36六面ACC第6面完成
                0x37完成
                0xff  没用校准都完成
*/
void CmdProcess_Cailration(uint8_t *p_data,uint8_t lenth,uint8_t PortNum)
{
    
    if((ComDataFlag&0x0f) == 0x04){
        if(g_ucDeviceNum==0x02 && (ComDataFlag&0xf0) == 0x20){
            if(PortNum == 0){/* 来自USB接口 */
                ComDataFlag = 0x04;//发往0号
                ComX_SendPack((uint8_t*)p_data,0,lenth);
            }else {
                USBDevice_WriteData(CI_CaliGimbalPar,p_data,6);	                   
            }
        }
        if(g_ucDeviceNum==0x00 && (ComDataFlag&0xf0) == 0x00){
            PCSetClibrateSates(*(p_data+4)); //设置校准状态0    
              ComDataFlag = 0x24;//发往0号
            *(p_data+4) = TempeConstantFlag1;    
            *(p_data+5)  =CaStatesTopc;
            ComX_SendPack((uint8_t*)p_data,0,6);
        }
    }  
}

 extern float fEmReAngle[3];
 extern  float groffset[3];;
extern uint8_t   g_InitAngleStarte ;
extern float testg[3];
extern float halfex, halfey, halfez;   
extern float  temperature1;
extern float  whachtflashtemp;


extern float   Temprature ;
extern float Get_EMReangle(int16_t Value);
extern    int32_t avgindex ;
extern void GetGryErr(float* gryerr);
extern float   CofinDynamicGryOffset[3];
extern float  OrigNetom[4];
extern uint16_t Capture1  ;
extern uint16_t Capture2 ;
extern int16_t AccData[3];
extern  float   Temprature ;
extern float beta;
extern float Mgdgki;
/**/
void CmdProcess_DebugData(uint8_t *p_data ,uint16_t lenth)
{
    
    if((ComDataFlag&0x0f) != 0x02)/* 字处理02命令 */
        return ;
    
    memcpy((void*)&iMotorDebugData,(void*)(p_data),sizeof(iMotorDebugData));
    
    if(g_ucDeviceNum == 0X00 && (ComDataFlag&0xf0) == 0x00)
    {
        /* imu */
        ReqMotoDataState = 0x01;
        iMotorDebugData.Flag = 0x12;
        iMotorDebugData.dLen = sizeof(iMotorDebugData);
        
        iMotorDebugData.MotoData[0].iCurrent[0] = beta*10000;
        iMotorDebugData.MotoData[0].iCurrent[1] = Mgdgki*10000;;
        
        iMotorDebugData.MotoData[0].SensorAngle[0] = g_hi0rgangle;
        iMotorDebugData.MotoData[0].SensorAngle[1] = g_hiEMReAngle;
        
        
        iMotorDebugData.MotoData[0].i_qd_data[0] =  (int16_t)g_SDq.x;
        iMotorDebugData.MotoData[0].i_qd_data[1] =  (int16_t)g_SDq.y;	 
        iMotorDebugData.MotoData[0].FocAngle = (int16_t)(Temprature*100);
        
        
        
        
        iMotorDebugData.Imu_Gyro[0] =  (int16_t)(Sensordata.gx*10);
        iMotorDebugData.Imu_Gyro[1] =  (int16_t)(Sensordata.gy*10);
        iMotorDebugData.Imu_Gyro[2] =  (int16_t)(Sensordata.gz*10); 
        
//      
        iMotorDebugData.Imu_Acc[0] = (int16_t)(Sensordata.ax*100);
        iMotorDebugData.Imu_Acc[1] = (int16_t)(Sensordata.ay*100);
        iMotorDebugData.Imu_Acc[2] = (int16_t)(Sensordata.az*100); 
////        
//        iMotorDebugData.Imu_Acc[0] = AccData[0];
//        iMotorDebugData.Imu_Acc[1] = AccData[1];
//        iMotorDebugData.Imu_Acc[2] = AccData[2]; 
    
        
//          iMotorDebugData.Imu_Gyro[0] = g_fSpeedPiderr[0];
//          iMotorDebugData.Imu_Gyro[1] = g_fSpeedPiderr[1]; 
//          iMotorDebugData.Imu_Gyro[2] = g_fSpeedPiderr[2];
        
//        
//        iMotorDebugData.Imu_Acc[0] = (int16_t)(halfex*10000);
//        iMotorDebugData.Imu_Acc[1] = (int16_t)(halfey*10000);
//        iMotorDebugData.Imu_Acc[2] = (int16_t)(halfez*10000);
        //float Err[4];
       // GetGryErr(Err);
//        iMotorDebugData.Imu_Acc[0] = (int16_t)(CofinDynamicGryOffset[0]*100);
//        iMotorDebugData.Imu_Acc[1] = (int16_t)(CofinDynamicGryOffset[1]*100);
//        iMotorDebugData.Imu_Acc[2] = (int16_t)(CofinDynamicGryOffset[2]*100);      
//        iMotorDebugData.Imu_Acc[0] = (int16_t)(Sensordata.ax*4096);
//        iMotorDebugData.Imu_Acc[1] = (int16_t)(Sensordata.ay*4096);
//        iMotorDebugData.Imu_Acc[2] = (int16_t)(Sensordata.az*4096);        

//        
//        iMotorDebugData.Imu_Gyro[0] = testg[0]*100;
//          iMotorDebugData.Imu_Gyro[1] = testg[1]*100;
//          iMotorDebugData.Imu_Gyro[2] = testg[2]*100;
        
        
        
       // iMotorDebugData.Imu_Acc[2] = (int16_t)(halfez*100);       
        
//         iMotorDebugData.Imu_Acc[0] = (int16_t)(Sensordata.ax*100);
//        iMotorDebugData.Imu_Acc[1] = (int16_t)(Sensordata.ay*100);
//        iMotorDebugData.Imu_Acc[2] = (int16_t)(Sensordata.az*100);       
        iMotorDebugData.Imu_EurAngle[0] = (int16_t)(Sensordata.pitch*100);
        iMotorDebugData.Imu_EurAngle[1] = (int16_t)(Sensordata.roll*100);
        iMotorDebugData.Imu_EurAngle[2] = (int16_t)(Temprature*100);
        
        
        ComX_SendPack((uint8_t*)&iMotorDebugData,0,sizeof(iMotorDebugData));
    }
    
    if(g_ucDeviceNum == 0X01 && (ComDataFlag&0xf0) == 0x10)
    {
        ReqMotoDataState = 0x02;
        iMotorDebugData.Flag = 0x22;
        iMotorDebugData.dLen = sizeof(iMotorDebugData);
        iMotorDebugData.MotoData[1].iCurrent[0] = ADCCurrent.I_PhaseA;
        iMotorDebugData.MotoData[1].iCurrent[1] = ADCCurrent.I_PhaseB;
        iMotorDebugData.MotoData[1].SensorAngle[0] = g_hi0rgangle;
        iMotorDebugData.MotoData[1].SensorAngle[1] = g_hiEMReAngle;
        iMotorDebugData.MotoData[1].i_qd_data[0] = (int16_t)g_SDq.x;
        iMotorDebugData.MotoData[1].i_qd_data[1] = (int16_t)g_SDq.y;		
        iMotorDebugData.MotoData[1].FocAngle =(int16_t)(g_fEMFocangle*10);      
        ComX_SendPack((uint8_t*)&iMotorDebugData,0,sizeof(iMotorDebugData));
    }
}





