#define DEFIN_CALIB_EXTERN
#include "arm_math.h"
#include  "datacalibrate.h"
#include "filter.h"
#include "mpu6050.h"
#include  "RLS.h"
#include "time.h"
#include "flash.h"
#include "arm_math.h"
#include "device.h"
/**/
struct sMpuBias
{
    int16_t hiMaxMinAccValue[6][3];
    int16_t hiAccBiasValue[3];
    int16_t hiGryBiasValue[3];
    int16_t hiTemperature;
    uint16_t Reserve;
}*SMpuBias;

static float AccBiasValue[3];
static float AccBiasSlopeValue[3];
static int16_t GryBiasValue[3];



//如果加了温度控制可取消下面
/*
*温度界定用模糊集
*相同去用递推最小二乘
*/
#define MAX_GYR_BIAS_NUM    15
#define MINUS__TEMP_ZERO_NUM    0
#define TEMP_PITCH_SIZE         3
#define UNTRIMMED_SENS  3 //333.87 LSB/°C
typedef struct {
    uint32_t wImuId;
    uint8_t  ucCalibConter;
    int16_t hiDefaultBias[3];
    int16_t hiDefaultSlop[3];
    struct{
        uint8_t ucFlag;// 
        int16_t hiTemp;
        struct 
        {
            int16_t hiBias;
            int16_t hiSlop; 
            int16_t hiNetom;     
        }Bias[3];
        
    }GyrBias[MAX_GYR_BIAS_NUM];
}tGyroBiasList;


uint32_t wImuId = 0x5a5a5a55;
tGyroBiasList*  SGyroBiasList;
tLRS* tLRGry[3];

/*
*/
float  Temprature = 0;
#define ACCELEROMETER_AFS_SEL  4096.f
#define GYROSCOPE_AFS_SEL   16.4f
/*-----------------------------------------------*/

void UpdateGyroBiasList(tGyroBiasList* GyroBiasList ,int16_t *netom);
#define  GROAVERGE_COUNT   5000.0f

  /*@初始化局部数据
*/

//uint32_t HWC_GetChipID(void) {
//
//  uint8_t tpci;
//  uint8_t * tpcpi,*tpcpa;
//  uint32_t Db[4];
//  tpcpi = (uint8_t*)&Db[0];
//  tpcpa = (uint8_t*)0x1FFFF7E8;   //0x1FFFF7E8 ~ 0x1FFFF7F4
//  for (tpci = 0; tpci < 12; tpci++)
//    tpcpi[tpci] = tpcpa[tpci];
//  /**/
//  Db[0] = (Db[0] * Db[1] * Db[2]);
//  return Db[0];
//}

void MpuDefaultVariable(void)
{
    //memset((void*)&SMpuBias,0,sizeof(SMpuBias));
}

void FlashMpuVariableEachCopy(void  *data,uint8_t index)
{
//    if(index){
//        memcpy((void*)&SMpuBias,data,sizeof(SMpuBias));
//    }else {
//        memcpy(data,(void*)&SMpuBias,sizeof(SMpuBias));
//    }

}
void FlashGryListVariableEachCopy(void  *data,uint8_t index)
{
//    if(index){
//        memcpy((void*)&SGyroBiasList,(void*)data,sizeof(tGyroBiasList));
//    }else {
//        memcpy((void*)data,(void*)&SGyroBiasList,sizeof(tGyroBiasList));  
//    }

}
void FlashRLSVariableEachCopy(void  *data,uint8_t index)
{
//    if(index){
//         memcpy((void*)&tLRGry[0],(void*)data,sizeof(tLRS)*3);
//    }else {
//          memcpy((void*)data,(void*)&tLRGry[0],sizeof(tLRS)*3);
//    }
}


static uint8_t sbyMpuClibStates = 0;
/*
*/  
static  uint8_t  OldMpuClibStates = 0;
void SetMpuClibrateStates(uint8_t states)
{
 
    if(OldMpuClibStates!=states){//在跳变的时候改变状态
        sbyMpuClibStates = states;
    }
    
  OldMpuClibStates = states;
}

/*
*/
uint8_t GetMpuClibrateStates(void)
{
    return sbyMpuClibStates;
}

uint8_t MotionState = 0;

/*
一面校准
*/
PrecisionTime_Type SMpuTime;
uint8_t  MpuClibrateAccBias(int16_t * netom)
{
    
    static int32_t  sumAcc[4]={0};
    static int16_t  avgAcc[4]={0}; 
    static int32_t clibreConter = 0;
    clibreConter++;
    if(MotionState==0){
        memset(sumAcc,0,sizeof(sumAcc));
        ClearCompareTime(&SMpuTime);
        clibreConter = 0;
        return 0;
    }
    for(uint8_t i=0;i<4;i++){ 
        sumAcc[i]+= i==1?(ACCELEROMETER_AFS_SEL+netom[i]):netom[i];
       // sumAcc[i]+= i==2?(netom[i]-ACCELEROMETER_AFS_SEL):netom[i];
    }
    if(CompareTime1s(&SMpuTime,2)){
         ClearCompareTime(&SMpuTime);
        for(int i = 0;i<4;i++){
            avgAcc[i] = (int16_t)(sumAcc[i]/(float)clibreConter);
            
        }
        clibreConter = 0;
        memset(sumAcc,0,sizeof(sumAcc));
        memcpy((void*)&SMpuBias->hiAccBiasValue[0],(void*)avgAcc,6);
        SMpuBias->hiTemperature = avgAcc[3];
        SMpuBias->Reserve &= 0xf0ff;
        SMpuBias->Reserve |= 0x0100;
        SMpuBias->Reserve |= 0x1000;
        return  1;
    }   
    return 0;
}
/*
6面校准
*/
static uint8_t ACC6Planidex = 0;
uint8_t GetACC6PlanidexStates(void)
{
  return ACC6Planidex;
}

uint8_t  MpuClibrateAcc6Plane(int16_t * netom,uint8_t idex)
{
    static  int32_t AccSum[3]= {0};
    static  int32_t clibreConter = 0;
    static  uint8_t clibStates = 0;
    static  uint8_t states = 0;
    static  uint8_t OldIdex = 0;
    if(OldIdex !=idex){
        states  = 1;
    }
    if(states ==0)return clibStates;
    if(MotionState==0){
        memset(AccSum,0,sizeof(AccSum));
        ClearCompareTime(&SMpuTime);
        clibreConter = 0;
        return clibStates;
    }
    for(int i = 0;i<3;i++){
      AccSum[i]+=netom[i];
    }
    clibreConter ++;
    int16_t accOffSet[3];
    if(clibreConter >=4096){
        clibreConter = 0;
        for(int i = 0;i<3;i++){
            accOffSet[i] =AccSum[i]>>12;
            AccSum[i] = 0;
        }
    }else {
       return clibStates;
    }
    
    
    
    for(int i = 0;i<3;i++){
        SMpuBias->hiMaxMinAccValue[idex-1][i] = accOffSet[i];
    }
    clibStates =idex;//|= (0x01<<(idex));
    OldIdex  = idex;
    ACC6Planidex =clibStates; 
    if(clibStates==0x6){
        SMpuBias->Reserve &= 0xf0ff;
        SMpuBias->Reserve |= 0x0200;
        SMpuBias->Reserve |= 0x2000;
        clibStates = 0; 
        return 0xff;
    }
    states = 0;
    
    return clibStates;
}
/*
//6面校准完的数值计算出 偏移和slope
*/ 
void GetAccOffset(float * offset,float * slop)
{
    
    float MaxMinAcc[6][3];
    for(int i = 0;i<3;i++){
        MaxMinAcc[0][i] = SMpuBias->hiMaxMinAccValue[4][i];
    }
    for(int i = 0;i<3;i++){
        MaxMinAcc[1][i] = SMpuBias->hiMaxMinAccValue[5][i];
    }
    for(int i = 0;i<3;i++){
        MaxMinAcc[2][i] = SMpuBias->hiMaxMinAccValue[2][i];
    }
    for(int i = 0;i<3;i++){
        MaxMinAcc[3][i] = SMpuBias->hiMaxMinAccValue[0][i];
    }
    for(int i = 0;i<3;i++){
        MaxMinAcc[4][i] = SMpuBias->hiMaxMinAccValue[1][i];
    }
    for(int i = 0;i<3;i++){
        MaxMinAcc[5][i] = SMpuBias->hiMaxMinAccValue[3][i];
    }
    float MaxMinAccRef[6][3] ={
        4096, 0,  0,
        -4096,0,  0,   
        0,  4096,  0,  
        0, -4096, 0,  
        0 , 0,   4096,
        0,  0,  -4096 ,
    };
    
    
    float DMatAccMT[3][6];
    float DMatAccMMInv[3][3];
    float DMatAccMTXMatAccR[3][3];
    float tEMP[3][3];
    float Offset[3] = {(MaxMinAcc[0][0]+MaxMinAcc[1][0])/2.0f,
    (MaxMinAcc[2][1]+MaxMinAcc[3][1])/2.0f,
    (MaxMinAcc[4][2]+MaxMinAcc[5][2])/2.0f};    

    for(int i = 0;i<6;i++){
        for(int j = 0;j<3;j++){
            MaxMinAcc[i][j] = MaxMinAcc[i][j]-Offset[j];
        }
    }             
             
    arm_matrix_instance_f32 MatAccM;
    arm_matrix_instance_f32 MatAccR;
    arm_matrix_instance_f32 MatAccMMInv;
    arm_matrix_instance_f32 MatAccMT;
    arm_matrix_instance_f32 MatAccMTXMatAccR;
    arm_matrix_instance_f32 MattEMP;
    
    arm_mat_init_f32(&MatAccM,6,3,(float*)MaxMinAcc);
    arm_mat_init_f32(&MatAccR,6,3,(float*)MaxMinAccRef);
    arm_mat_init_f32(&MatAccMMInv,3,3,(float*)DMatAccMMInv);   
    arm_mat_init_f32(&MatAccMT,3,6,(float*)DMatAccMT);
    arm_mat_init_f32(&MatAccMTXMatAccR,3,3,(float*)DMatAccMTXMatAccR);
    arm_mat_init_f32(&MattEMP,3,3,(float*)tEMP);
    
    arm_mat_trans_f32(&MatAccM,&MatAccMT);
    arm_mat_mult_f32(&MatAccMT,&MatAccM,&MattEMP);
    arm_mat_inverse_f32(&MattEMP,&MatAccMMInv);
    arm_mat_mult_f32(&MatAccMT,&MatAccR,&MattEMP);  
    arm_mat_mult_f32(&MatAccMMInv,&MattEMP,&MatAccMTXMatAccR);
    
    for(int i =0;i<3;i++){
      offset[i] = Offset[i];
    }
    slop[0] = DMatAccMTXMatAccR[0][0];
    slop[1] = DMatAccMTXMatAccR[1][1];
    slop[2] = DMatAccMTXMatAccR[2][2];
}
/*
*/

static float  GyrNoise[4]={0};
uint8_t  MpuClibrateGyrBias(int16_t * netom)
{
    static  int32_t    sumGyr[4]={0};
    static  int16_t    avgGyr[4]={0};
    static  int32_t    sumGyrNoise[4] = {0};
    static  int32_t    clibreConter = 0;
    static  uint8_t    states = 0;
    
    clibreConter++;
    if(MotionState==0){
        clibreConter = 0;
        memset(sumGyr,0,sizeof(sumGyr));
        memset(sumGyrNoise,0,sizeof(sumGyrNoise));
        ClearCompareTime(&SMpuTime);
        states  = 0;
        return 0;
    } 
    if(states==0){
        for(uint8_t i=0;i<4;i++){ 
            sumGyr[i] +=netom[i+3]; 
        }
        if(CompareTime1s(&SMpuTime,2)){
            ClearCompareTime(&SMpuTime);
            
            for(int i = 0;i<4;i++){
                avgGyr[i] = (int16_t)(sumGyr[i]/(float)clibreConter);
            }
            memset(sumGyr,0,sizeof(sumGyr));
            memset(sumGyrNoise,0,sizeof(sumGyrNoise));
            clibreConter = 0;
            states = 1;
          
        }
    }else {
        for(uint8_t i=0;i<4;i++){ 
            sumGyr[i] +=netom[i+3];  
            sumGyrNoise[i] += (netom[i+3]-avgGyr[i]);
        }
        if(CompareTime1s(&SMpuTime,2)){
            ClearCompareTime(&SMpuTime);
            for(int i = 0;i<4;i++){
                avgGyr[i] = (avgGyr[i]>>1) +((sumGyr[i]/clibreConter)>>1);
                GyrNoise[i] = sumGyrNoise[i]/(float)clibreConter;
            }
       
            memcpy((void*)&SMpuBias->hiGryBiasValue[0],(void*)&avgGyr[1],6);
            SMpuBias->hiTemperature = avgGyr[0];
            
            int32_t temp = (avgGyr[0]+7011);
            int16_t avgGyrTmep[4] = {avgGyr[1],avgGyr[2],avgGyr[3],temp>>8};
            
            UpdateGyroBiasList(SGyroBiasList,avgGyrTmep); 
            memset(sumGyrNoise,0,sizeof(sumGyrNoise));

            clibreConter = 0;
            states  = 0;
			return  1;
        }
    } 
    return 0;
        
}


void UpdateOffset(uint8_t index)
{
    if(index&0x01){
        for(int i = 0;i<3;i++){
            AccBiasSlopeValue[i] = 1.0f;
        }
        if(SMpuBias->Reserve&0x2000){
            GetAccOffset(AccBiasValue,AccBiasSlopeValue);  
        }
        for(int i = 0;i<3;i++){
            AccBiasValue[i] = SMpuBias->hiAccBiasValue[i];
        }
    }
    if(index&0x02){
        GetAccOffset(AccBiasValue,AccBiasSlopeValue);    
    }
    if(index&0x04){
        for(int i = 0;i<3;i++){
          GryBiasValue[i] = SMpuBias->hiGryBiasValue[i];
        }
    }
}

uint8_t MpuClibrate(int16_t * netom)
{
    switch(sbyMpuClibStates){
    case 0:
        break;
    case 0x01:
        if(MpuClibrateAccBias(netom)){
            sbyMpuClibStates = 0xff;
            UpdateOffset(0x01);
        }
        break;
    case 0x11: 
    case 0x12:
    case 0x13:
    case 0x14:
    case 0x15:
    case 0x16:  
        uint8_t r = MpuClibrateAcc6Plane(netom,sbyMpuClibStates&0x0f);
        if(r==0xff){
            sbyMpuClibStates = 0xff;
            UpdateOffset(0x02);
        }
        break;
    case 0x41: 
        if(MpuClibrateGyrBias(netom)){
            sbyMpuClibStates = 0xff;
            UpdateOffset(0x04);
        }
        break;
    case 0xff:
        
        break;   
    }
	
}

/*
*/
float  OrigNetom[4];
void UpdateGryOffset(int16_t *netom);
void GetUpdateGryOffset(int16_t *offset,int16_t* netom);
int16_t OldTemp = 0;
extern float    DynamicGryOffset[3];
extern  float   CofinDynamicGryOffset[3];
uint8_t PowerOnGryOffsetOK =0;

void MpuUpdate(float * mpu,int16_t* netom)
{
    
    int16_t rawgyro[3];
    for(int i = 0;i<3;i++){
        rawgyro[i] = netom[4+i];
    }
    
    netom[3]+=7011;//
    OldTemp  += (netom[3]>>3) - (OldTemp>>3); //  0.04°
    Temprature = OldTemp/333.87f;
    netom[3]>>=8;//0.76°
   // GetUpdateGryOffset(&rawgyro[0],netom);
    
   	for(uint8_t i=0;i<3;i++){
        mpu[i] =    (netom[i] -AccBiasValue[i])*AccBiasSlopeValue[i] /ACCELEROMETER_AFS_SEL;  
        //mpu[i] =    (netom[i] - SMpuBias.hiAccBiasValue[i])/ACCELEROMETER_AFS_SEL; 
        mpu[i+3] = (netom[4+i]- GryBiasValue[i])/GYROSCOPE_AFS_SEL;//-DynamicGryOffset[i]*(1-PowerOnGryOffsetOK);// gro data offset at 4 of netom[]
        OrigNetom[i] = (netom[4+i]- GryBiasValue[i])/GYROSCOPE_AFS_SEL;
       // mpu[i+3] = (netom[i+4]-rawgyro[i])/16.4f;// gro data offset at 4 of netom[]
	}
}


#define g_theshold  100
#define a_theshold  400

float MpuDataAvg[7];
uint8_t FirstNetomPtr[7];
#define VARIANCECONTER 256.f
int16_t OldNetom[7][256];
int16_t OldNetomFliter[7];


void GetMpuVariance(int16_t* netom ,uint16_t *variance)
{
    for(int i = 0;i<7;i++){   
        MpuDataAvg[i] += netom[i]/VARIANCECONTER;
        MpuDataAvg[i] -= OldNetom[i][FirstNetomPtr[i]]/VARIANCECONTER;
        OldNetom[i][FirstNetomPtr[i]++] = netom[i];       
    }
    
    for(int i = 0;i<7;i++){
        OldNetomFliter[i] = ((netom[i]*16)+(OldNetomFliter[i]*4))/20;
        //OldNetomFliter[i] += (netom[i]>>2)-(OldNetomFliter[i]>>2);
        variance[i] = abs(OldNetomFliter[i]-(int16_t)MpuDataAvg[i]);
    } 
}

uint16_t variance[7];
void GetMpuVari(uint16_t ** vari )
{
    vari[0] = variance;
}
uint8_t  DetectNoMotionStaes(int16_t* netom)
{
  
    GetMpuVariance(netom,variance); 
    for(int i = 0;i<3;i++){
        if(variance[i]>=a_theshold){
            return 0;
        }
    }   
    for(int i = 0;i<3;i++){
        if(variance[i+4]>=g_theshold){
            return 0;
        }
    }     
    return 1;
}

uint8_t DetectSensorNoMotion(int16_t* netom)
{
    static  uint8_t ThrolConter = 0;
    static  uint32_t noMotionConter = 0;
    if(DetectNoMotionStaes(netom)){
        noMotionConter ++;
    }else{
        if(++ThrolConter>=10){
            ThrolConter = 0;
            noMotionConter = 0;
        }
        
    }
    
   if(noMotionConter >=1000){
        noMotionConter  = 1000;
        return 1;
   }
   return 0;
}
/**/
//flash 更新完毕后 更新 
void InitCalibrate(void)
{
    memset(AccBiasValue,0,sizeof(AccBiasValue));
    memset(AccBiasSlopeValue,0,sizeof(AccBiasSlopeValue));
    memset(GryBiasValue,0,sizeof(GryBiasValue));
    
    SMpuBias = (struct sMpuBias*)&g_SFlashpra.sMpu.hiMaxMinAccValue[0][0];
    SGyroBiasList = (tGyroBiasList*)&g_SFlashpra.sMpu.tGyroBiasList;
    for(int i = 0;i<3;i++){
        tLRGry[i] = (tLRS*)&g_SFlashpra.sLRS.LRS[i];
    }
    if((SMpuBias->Reserve&0x0f00) == 0x0200){
        UpdateOffset(0x06);
    }else {
        UpdateOffset(0x05);
    }


}

/*
*/

/*

*/

uint8_t GetOffSetAve(int16_t* netom)
{
    static int32_t OffSetConter = 0;
    static uint32_t OffSetHoldTime = 0;
    static uint8_t Sates = 0;
    static  int32_t GryOffSum[3];
    if(Sates==0){
        if(MotionState==0){
            OffSetConter = 0;
            memset(GryOffSum,0,12);
        }else {
            OffSetConter++;
            for(int i = 0;i<3;i++){
                GryOffSum[i] += netom[i+4];
            }
        }
        if(OffSetConter>=2048){
            for(int i = 0;i<3;i++){
              SMpuBias->hiGryBiasValue[i] =  GryOffSum[i]>>11;
            }
            PowerOnGryOffsetOK = 1;
            g_ucFlashWriteFlag = 1;
            UpdateOffset(0x04);
            Sates = 1;
        }
        if(OffSetHoldTime++>=10000){//15s
            Sates = 1;
            
        }
    }
    return Sates;

}
/*
*/
uint8_t DetectSensorNoMotion(int16_t* netom);

int32_t MpuDataConter = 0;
int32_t MpuDataSum[7];
void UpdataMpuData(void)
{   
    if(g_ucDeviceNum!=0)return ;
    int16_t netom[7];
    if(get_mpu_dmadata(netom)){
        MpuDataConter++;
        for(int i=0;i<7;i++){
          MpuDataSum[i]+= netom[i];
        }
    }
}
int16_t AccData[3];
uint8_t GetMpuData(float *sen)
{  
    int16_t netom[7];  
//    if(MpuDataConter==0){
//    return 0;
//    }
//    for(int i = 0;i<7;i++){
//        netom[i] = MpuDataSum[i]/MpuDataConter;
//        MpuDataSum[i] = 0;
//    }    
    if(get_mpu_dmadata(netom)==0)return 0 ; 
    MpuDataConter = 0;
    MotionState  =DetectSensorNoMotion(netom);
    MpuClibrate(netom); 
    //uint8_t states = GetOffSetAve(netom)+1;
    MpuUpdate(sen,netom);  
    return 2;
}



/*

*/
#define  NewTemperature  netom[3]
#define K_ZOOM  1024.f
/*
*/

void FirstUpdateBiasList(tGyroBiasList* GyroBiasList ,int16_t *netom)
 {
    int8_t temp = (NewTemperature>>TEMP_PITCH_SIZE) +MINUS__TEMP_ZERO_NUM;
    if(temp>=MAX_GYR_BIAS_NUM)temp = MAX_GYR_BIAS_NUM-1;
    if(temp<=0)temp = 0;
    for(int i = 0;i<3;i++){
        GyroBiasList->GyrBias[temp].Bias[i].hiNetom =  netom[i];
        GyroBiasList->GyrBias[temp].Bias[i].hiSlop = 0;//188;//MPU6500默认  ±0.24o/s/°C  -》K = 333.87/0.24 = 1389
        float  b = netom[i];//- 0.184*NewTemperature ;
        GyroBiasList->GyrBias[temp].Bias[i].hiBias = b*128;//(int16_t)(b>>7);
        GyroBiasList->hiDefaultBias[i]  = b*128; //(int16_t)(b>>7);
        GyroBiasList->hiDefaultSlop[i]  =0 ;//188;
        
        
        float xx[2] = {b,0};
        RLSInit(tLRGry[i],xx,1);
    }
    GyroBiasList->GyrBias[temp].hiTemp =  netom[3];
    GyroBiasList->GyrBias[temp].ucFlag = 1 ; //高位末尾 表示赋值了。但是斜率事默认值    
}
/*
*/



void  CalculateBias(tGyroBiasList* GyroBiasList ,int16_t *netom,uint8_t index)
{
    int8_t temp = (NewTemperature>>TEMP_PITCH_SIZE) +MINUS__TEMP_ZERO_NUM;
    if(temp>=MAX_GYR_BIAS_NUM)temp = MAX_GYR_BIAS_NUM-1;
    if(temp<=0)temp = 0;
    int16_t MinusRusult =   NewTemperature - GyroBiasList->GyrBias[index].hiTemp;                       
    for(int i = 0;i<3;i++){
        float    k  = 0;
        float  ktemp = netom[i]- GyroBiasList->GyrBias[index].Bias[i].hiNetom;
        k =  ktemp/MinusRusult;     
        float b =  netom[i]-k*NewTemperature;
        GyroBiasList->GyrBias[temp].Bias[i].hiNetom =   netom[i];
        GyroBiasList->GyrBias[temp].Bias[i].hiBias =    (int16_t )(b*128.f);//(b>>7);;
        GyroBiasList->GyrBias[temp].Bias[i].hiSlop =     (int16_t )(k*K_ZOOM);

    } 
   
    
    GyroBiasList->GyrBias[temp].ucFlag = 1 ;
    GyroBiasList->GyrBias[temp].hiTemp = NewTemperature;
   
}

/*

*/
uint8_t GetGyroBiasListWasCalibredIdex(tGyroBiasList* GyroBiasList ,int16_t *netom)
{
    int8_t temp = (NewTemperature>>TEMP_PITCH_SIZE) +MINUS__TEMP_ZERO_NUM;
    if(temp>=MAX_GYR_BIAS_NUM)temp = MAX_GYR_BIAS_NUM-1;
    if(temp<=0)temp = 0;
    int minusIdex = MAX_GYR_BIAS_NUM;
    uint8_t result  = 0;
    for(int i = 0;i<MAX_GYR_BIAS_NUM;i++){  
        if(GyroBiasList->GyrBias[i].ucFlag){
            if(minusIdex>=abs(temp-i)){
                 minusIdex = abs(temp-i);
                 result = i;
            }
        }  
    }
    return result;
}
/*
*/
//跨区校准
void GetGyroBiasListNotCalibred(tGyroBiasList* GyroBiasList ,int16_t *netom)
{    
     uint8_t  index  = GetGyroBiasListWasCalibredIdex(GyroBiasList,netom);
     CalculateBias(GyroBiasList,netom,index); 
}
/*
*/
//相同区域
void GetGyroBiasListWasCalibred(tGyroBiasList* GyroBiasList ,int16_t *netom)
{
    int8_t temp = (NewTemperature>>TEMP_PITCH_SIZE) +MINUS__TEMP_ZERO_NUM;
    if(temp>=MAX_GYR_BIAS_NUM)temp = MAX_GYR_BIAS_NUM;
    if(temp<=0)temp = 0;
    
    int16_t MinusRusult = NewTemperature - GyroBiasList->GyrBias[temp].hiTemp; 
    if(abs(MinusRusult)>=UNTRIMMED_SENS){//大于1度   
#ifdef RLS  //递推最小二乘计算零偏和斜率
        
#else   
        CalculateBias(GyroBiasList,netom,temp);//直接覆盖以前的数据     
         //   if(GyroBiasList->ucCalibConter==2){

          //  }  
#endif
    }else {
        for(int i = 0;i<3;i++){
            float  b = netom[i];//- 0.184*NewTemperature ;
            GyroBiasList->GyrBias[temp].Bias[i].hiBias = b*128;//(int16_t)(b>>7);
        }
    }  
}

/*
*/
uint8_t  NormalUpdateBiasList(tGyroBiasList* GyroBiasList ,int16_t *netom)
{
     
    int8_t temp = (NewTemperature>>TEMP_PITCH_SIZE) +MINUS__TEMP_ZERO_NUM;
    if(temp>=MAX_GYR_BIAS_NUM)temp = MAX_GYR_BIAS_NUM;
    if(temp<=0)temp = 0;
     if(GyroBiasList->GyrBias[temp].ucFlag){ 
        GetGyroBiasListWasCalibred(GyroBiasList,netom);
     }else{
        GetGyroBiasListNotCalibred(GyroBiasList,netom);     
     }    
     for(int i = 0;i<3;i++){  
         LRS(tLRGry[i],netom[3],netom[i],GyrNoise[i]);  
         GyroBiasList->hiDefaultBias[i]  = tLRGry[i]->X1[0]*128.f;
         GyroBiasList->hiDefaultSlop[i]  = tLRGry[i]->X1[1]*K_ZOOM;
     }  
     
}


//每约等于6°一个挡位
//  temp = Rbias*slop +bias
// 大于0°的十八个  小于0°的7个

void UpdateGyroBiasList(tGyroBiasList* GyroBiasList ,int16_t *netom)
{
    if(wImuId != GyroBiasList->wImuId){
        memset(GyroBiasList,0,sizeof(tGyroBiasList));
    }
    GyroBiasList->ucCalibConter++;
    if(GyroBiasList->ucCalibConter==1){
        FirstUpdateBiasList(GyroBiasList,netom);
    }else{
        NormalUpdateBiasList(GyroBiasList,netom);
    }
    
}

void GetGyroBiasList(tGyroBiasList* GyroBiasList ,int16_t *netom,int16_t temp1)
{
    int8_t temp = (temp1>>TEMP_PITCH_SIZE) +MINUS__TEMP_ZERO_NUM;
    if(temp>=MAX_GYR_BIAS_NUM)temp = MAX_GYR_BIAS_NUM-1;
    if(temp<=0)temp = 0;
    
    if(GyroBiasList->GyrBias[temp].ucFlag){
      
        for(int i=0;i<3;i++){
            int16_t  bias =  GyroBiasList->GyrBias[temp].Bias[i].hiBias;
            int16_t  slop =  GyroBiasList->GyrBias[temp].Bias[i].hiSlop;
           // if(slop==0)slop=1;
            netom[i] =  (slop/K_ZOOM)*temp1 + (bias/128.f);          
        }

    }else {
        int16_t* bias =  (int16_t*)&GyroBiasList->hiDefaultBias[0];
        int16_t* slop =  (int16_t*)&GyroBiasList->hiDefaultSlop[0];
        for(int i=0;i<3;i++){
            netom[i] =  (slop[i]/K_ZOOM)*temp1 + (bias[i]/128.f); 
          //  if(slop[i]==0)slop[i]=1;
           
        }
    }


}

//uint8_t UpdateGryOffsetTestFlag = 0;
//uint8_t GetUpdateGryOffsetTestFlag = 0;
//int16_t UpdateGryOffsetTestValue[3]; 
//int32_t SumNeto[4];
//uint16_t  SumNetoNum = 0;
//int16_t   netomi[4];
//void UpdateGryOffset(int16_t *netom)
//{
//    static uint8_t states = 0;
//  
//    if(UpdateGryOffsetTestFlag){
//            SumNetoNum ++;
//        if(states==0){
//            for(int i = 0;i<4;i++){
//                SumNeto[i]+=netom[i];
//            }
//            if(SumNetoNum<1000)return ;
//            for(int i = 0;i<4;i++){
//                netomi[i] = SumNeto[i]/1000;
//                SumNeto[i] = 0;
//                RNoise[i] = 0;
//            }
//            SumNetoNum = 0;
//            states = 1;
//        }else{
//            for(int i = 0;i<4;i++){
//                SumNeto[i]+=netom[i];
//                RNoise[i] += fabs((netom[i]-netomi[i]))/1000.f;
//            }            
//            if(SumNetoNum<1000)return ; 
//            for(int i = 0;i<4;i++){
//                netomi[i] = netomi[i]/2+SumNeto[i]/2000;
//                SumNeto[i] = 0;
//            }
//            SumNetoNum = 0;
//            states = 0;
//            
//            UpdateGryOffsetTestFlag = 0;
//            UpdateGyroBiasList(&SGyroBiasList,netomi);   
//            memset(netomi,0,8);   
//            
//        }
//
//    }
//    
//}
void GetUpdateGryOffset(int16_t *offset,int16_t* netom)
{
   // if(GetUpdateGryOffsetTestFlag){
        GetGyroBiasList(SGyroBiasList,offset,netom[3]);
   // }
 
}