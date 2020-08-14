#define DEFIN_EULER_EXTERN
#include "EulerQuaternion.h"
#include  "datacalibrate.h"
#include "arm_math.h"
#include "kalman.h"
#include "CORDIC_math.h"
#include "time.h"
#include "math.h"
tKalman KalmanRoll;
tKalman KalmanPitch;
uint8_t  expv[]= {
255	,251,246,241,236,232,227,223,218,	
214	,210,205,201,197,193,190,186,182,	
179	,175,172,168,165,162,158,155,152,	
149	,146,143,140,138,135,132,130,127,	
125	,122,120,117,115,113,111,108,106,	
104	,102,100,98	,96, 94	,92	,90	,89	,
87	,85	,84	,82	,80	,79,77	,76	,74	,
73	,71	,70	,68	,67	,66,64	,63	,62	,
61	,59	,58	,57	,56	,55,54	,53	,52 ,
51	,50	,49	,48	,47	,46,45	,44	,43	,
42	,41	,41	,40	,39	,38,38	,37	,36	,
35	,35	,34	,33	,33	,32,31	,31	,30	,
30	,29	,28	,28	,27	,27,26	,26	,25	,
25	,24	,24	,23	,23	,22,22	,21	,21	,
21	,20	,20	,19	,19	,19,18	,18	,18	,
17	,17	,17	,16	,16	,16,15	,15	,15	,
14	,14	,14	,14	,13	,13,13	,12	,12	,
12	,12	,12	,11	,11	,11,11	,10	,10	,
10	,10	,10	,   9   ,9,   9  , 9 ,  9 , 	 
8  ,8   ,8  , 8 ,  8 ,  8   ,7  , 7  , 7,		
7  ,7   ,7  , 7 ,  6 ,  6   ,6  , 6  , 6,		
6	,6	,6	,6  , 5	 ,5	,5	,5	,5	,
5	,5	,5	,5	,5	 ,4	,4	,4	,4  ,
4	,4	,4	,4	,4	 ,4	,4	,4	,3	,
3	,3	,3	,3	,3	 ,3	,3	,3	,3	,
3	,3	,3	,3	,3   ,3	,3	,2	,2	,
2,2	,2	,2	,2	,2   ,2	,2	,2	
,2	,2	,2	,1	,1	,1	 ,1	,1	,1	
,1	,0	,0	,0
};

u8 InitStates = 0;
float InitYaw = 0.0f;
void AhrsFixPra(uint8_t states ,float*  Mpu);

extern void Get_Delat(void);
PrecisionTime_Type Delta_time;

static float roll,pitch,yaw;

/*
*/
void qtoe(void)
{
	roll   = -CORDIC_atan2(2.0f * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 +q3 * q3);   
	pitch  = -CORDIC_asin(-2.0f * (q1 * q3 -q0 * q2));
	yaw    = -CORDIC_atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 -q3 * q3);
}

/*
*/

void SetEulerFixPare(uint8_t index)
{
    InitStates  = index;
}
/*
>=70US 总共耗时 <=92US
*/
float  KalmanFilterEuler(float * mpu);
extern uint8_t   GetMpuData(float* tem);

Senseordata_Type   Sensordatatemp;  
uint8_t  GetEuler(void)
{   


    uint8_t states = GetMpuData(&Sensordata.ax);
	if(states==0)return 0;

//    Sensordatatemp.gx =  Sensordata.gx *(0.0174536f);
//    Sensordatatemp.gy =	 Sensordata.gz *(0.0174536f);
//    Sensordatatemp.gz =  -Sensordata.gy *(0.0174536f);
//    Sensordatatemp.ax =  Sensordata.ax;
//    Sensordatatemp.ay =  Sensordata.az;
//    Sensordatatemp.az =  -Sensordata.ay;	
    
    Sensordatatemp.gx = Sensordatatemp.gx+ 0.5f*( Sensordata.gx *(0.0174536f) -Sensordatatemp.gx);
    Sensordatatemp.gy =	Sensordatatemp.gy+ 0.5f*( Sensordata.gz *(0.0174536f) -Sensordatatemp.gy);
    Sensordatatemp.gz = Sensordatatemp.gz+ 0.5f*( -Sensordata.gy *(0.0174536f)-Sensordatatemp.gz);
  
    Sensordatatemp.ax =  Sensordatatemp.ax + 0.08f*(Sensordata.ax  -Sensordatatemp.ax)  ;
    Sensordatatemp.ay =  Sensordatatemp.ay + 0.08f* (Sensordata.az -Sensordatatemp.ay)   ;
    Sensordatatemp.az =  Sensordatatemp.az + 0.08f*(-Sensordata.ay - Sensordatatemp.az) ;	   
  
    deltat  = Get_Time_s(&Delta_time);
		if(deltat>0.002f)deltat = 0.001f;
		
#ifndef MadgwickAHRS
    MahonyAHRSupdateIMU(&Sensordatatemp);
#else  
    MadgwickAHRSupdateIMU(&Sensordatatemp);
#endif	
	if(InitStates){
		AhrsFixPra(1,(void*)&Sensordata);
	} 
    qtoe();
   
    Sensordata.yaw =  yaw;	
    Sensordata.pitch = pitch;	 
    Sensordata.roll =  roll;
    return states;
}
/*
*beta = 0.05 的时候 零偏偏掉0-3°可拉回来
*/
float  rooli = 0.0005;
float  roolp = 0.003;
float  KalmanFilterEuler(float * mpu)
{
    
    float ax = mpu[0];
    float ay = mpu[1];
    float az = mpu[2];
    static  float rolln;
    static  float InterRoll = 0;
  
    float recipNorm = sqrt(ax * ax + ay * ay + az * az );
    ax /= recipNorm;
    ay /= recipNorm;
    az /= recipNorm; 
    
    float rolltemp = CORDIC_atan2(az ,-ay);
    float  In = deltat* mpu[3];
    float rollnn =  rolln+ In;
    float err = rolltemp-rollnn;
    if(rooli>0.f)
    InterRoll +=err*rooli;
    else InterRoll = 0;
    
    rolln = rollnn+err*roolp+InterRoll*deltat;
    return rolln;
       
    //pitch = CORDIC_asin(ax);

   // return  KalmanFilter(&KalmanRoll,rolltemp,In);
  
}
#define GRA_THROHOLD    25   //1/10*g


uint32_t OldMpuGra[256];
uint32_t OldMpuGraFliter;
uint8_t FirstMpuGraPtr;
float  MpuGraAvg ;
uint32_t graAvg;
uint32_t avgindex = 0;
extern void GetGryErr(float* gryerr);

//在上电的时候用
float   DynamicGryOffset[3];
float   CofinDynamicGryOffset[3];

float   OldCofinDynamicGryOffset[3];
float   CofinDynamicGryOffsetSum[3];

void GetDynaGyrOffset(float* offset,float* coif)
{
    for(int i  = 0;i<3;i++){
        offset[i] = DynamicGryOffset[i];
        coif[i] = CofinDynamicGryOffset[i];
    }

}

uint32_t DynamicGryOffsetConter[3];
extern void GetGryDynOffset(float* gryerr);
void extern SetGryInterZero(uint8_t index,float trust );




#define MaxTOLERATE (0.3*57.3f)
extern float Mgdgki;
float CofinDynamicGrySum[3] = {0};
extern void GetMpuVari(uint16_t ** vari );
uint32_t sqrtgra = 4096;
void GetAccConfidence(float*   Mpu,float * Confidence)
{
    float graSum = 0;
    for(int i=0 ;i<3;i++){
        graSum +=Mpu[i]*Mpu[i];
    }
    
    float Gra;
    arm_sqrt_f32(graSum,&Gra);
    uint32_t  sqrtgra1 = (uint32_t)(Gra*4096.f);
	sqrtgra+= (sqrtgra1>>2)-(sqrtgra>>2);
    
	//取重力加速的平均值
	// MpuGraAvg += sqrtgra/256.f;
	//MpuGraAvg -= OldMpuGra[FirstMpuGraPtr]/256.f;
	//  OldMpuGra[FirstMpuGraPtr++] =sqrtgra;     
    //graAvg =(uint32_t)MpuGraAvg;
    avgindex = (abs(4096-sqrtgra));
    
    if(avgindex>=250)avgindex = 250; 
    Confidence[0]  =(expv[avgindex]/256.f);
   
}

void AhrsFixPra(uint8_t statse ,float*   Mpu)
{
    static float ErrSum = 0.0f;
    float Cofiavgindex = 0;
    GetAccConfidence(Mpu,&Cofiavgindex);
    float dynamicGryOffseterr[3] ;
    float GryOffse[3] ;
    GetGryErr(dynamicGryOffseterr);
    float  ErrSum1 =  fabs(dynamicGryOffseterr[0])+
                        fabs(dynamicGryOffseterr[1])+
                        fabs(dynamicGryOffseterr[2]);
    ErrSum = ErrSum+0.005f*(ErrSum1-ErrSum);
    GetGryDynOffset(GryOffse);
	
	
    uint16_t *mpuVari = 0;
    GetMpuVari(&mpuVari);
	if(statse==0){
	  if(mpuVari[0]+mpuVari[1]+mpuVari[2] <=96){
		for(int i = 0;i<3;i++){
		  OldCofinDynamicGryOffset[i] = OldCofinDynamicGryOffset[i]*0.2f+dynamicGryOffseterr[i]*0.8f;
		  if(fabs(OldCofinDynamicGryOffset[i])<=MaxTOLERATE){
			DynamicGryOffsetConter[i]++;
			CofinDynamicGryOffsetSum[i]+=fabs(OldCofinDynamicGryOffset[i]);
			// CofinDynamicGrySum[i] +=fabs(Mpu[3+i]);
		  }else {
			CofinDynamicGryOffsetSum[i] = 0;
			DynamicGryOffsetConter[i] = 0;
		  }
		  float CoifGryoffset = 0.0f;
		  float CoifGry = 0.0f;
		  if(DynamicGryOffsetConter[i]>=1000){
			
			CoifGryoffset =     CofinDynamicGryOffsetSum[i]/1000.f;
			
			CoifGry = mpuVari[4+i]/32.f;//2du
			if(CoifGry>=1.0f)CoifGry = 1.0f;
			CoifGry = 1.0f-CoifGry;
			
			DynamicGryOffsetConter[i] = 0;
			CofinDynamicGryOffsetSum[i] = 0;
			CofinDynamicGrySum[i] = 0;
			
			if(CoifGryoffset>=MaxTOLERATE)CoifGryoffset = MaxTOLERATE;
			CoifGryoffset = 1.f-CoifGryoffset/MaxTOLERATE;
			
			CofinDynamicGryOffset[i] = CoifGryoffset*Cofiavgindex*CoifGry; 
			DynamicGryOffset[i] += GryOffse[i]*CofinDynamicGryOffset[i];
			// SetGryInterZero(i,1.0f-CofinDynamicGryOffset[i]);
		  }  
		}
	  }
	}
	
    if(statse){
        //if(ErrSum>=MaxTOLERATE)ErrSum = MaxTOLERATE;
        float  ErrSumNormal = (ErrSum/MaxTOLERATE);
		if(ErrSumNormal>=1.0f)ErrSumNormal = 1.0f;
				
            beta = 0.012f*Cofiavgindex*ErrSumNormal;
            if(beta<=0.001f)beta = 0.001f;
            Mgdgki =  0.0001f*Cofiavgindex;
           // if(Mgdgki<=0.0001f)Mgdgki =0.0001f; 
            twoKp =  0.2f*Cofiavgindex;
            //if(twoKp<=0.05f)twoKp = 0.05f;
            twoKi = 0.001f*Cofiavgindex;
            //if(twoKi<=0.0001f)twoKi = 0.0001f;
            
            
    }
  // beta = 0.003f;
   //Mgdgki = -4;
    
}
/*
*/
uint32_t Conter = 0;
float RollCov = 0;
float PitchCov = 0;
double SumRollCov = 0;
double SumPichCov = 0;
float  ArgRoll;
float  ArgPitch;


uint8_t Init_ulerquaternion() //y-->pitch   x-->roll z -yaw
{
	static  	float ax = 0,ay = 0,az = 0;
	static   uint32_t InitConter = 0;
	float sineroll,cosroll,sinepitch,cospitch;  
	float Mpu[6];
	
	//	get_mpu_avgrt(Mpu);	
	//	ax =	Mpu[0];
	//	ay = 	Mpu[1];
	//	az = 	Mpu[2];
	
	ax  += Sensordata.ax;
	ay  += Sensordata.ay;
	az  += Sensordata.az;
	beta = 0.3f;
	twoKp =  0.8f;
	twoKi = 0.00f;
	
	if(++InitConter<256)return 0;
	
	ax = ax/256.f;
	ay = ay/256.f;
	az = az/256.f;
	float recipNorm = sqrt(ax * ax + ay * ay + az * az );
	ax /= recipNorm;
	ay /= recipNorm;
	az /= recipNorm;   
	roll = CORDIC_atan2(az,-ay );
	pitch = CORDIC_asin(-ax);
	arm_sin_cos_f32(0.5f*roll,&sineroll,&cosroll);
	arm_sin_cos_f32(0.5f*pitch,&sinepitch,&cospitch);
	
	q0 = 	cosroll *cospitch;  //w//   
	q1 =	sineroll*cospitch; //x 
	q2 =   	cosroll*sinepitch;  //z  
	q3 = 	-sineroll*sinepitch; //y    
	
	recipNorm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 /= recipNorm;  
	q1 /= recipNorm;
	q2 /= recipNorm;
	q3 /= recipNorm;	
	
	return 1;
    
}


