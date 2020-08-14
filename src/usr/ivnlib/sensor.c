#include <stdio.h>
#include <stdlib.h>

#include <stdint.h>
#include <string.h>
#include <math.h>
#include "CORDIC_math.h"

#include "inv_mpu_lib/inv_mpu.h"
#include "inv_mpu_lib/inv_mpu_dmp_motion_driver.h"
#include "sensor.h"

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))

#include "EulerQuaternion.h"
// MPU control/status vars
uint8_t devStatus;      // return status after each device operation
//(0 = success, !0 = error)
uint8_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

int16_t a[4];              // [x, y, z]            accel vector
int16_t g[4];              // [x, y, z]            gyro vector
int32_t _q[4];
int32_t t;
int16_t c[3];



int r;
int initialized = 0;
int dmpReady = 0;
float lastval[3];
int16_t sensors;

float ypr[3];

float temp;
float gyro[3];
float accel[3];
float compass[3];
extern void delay_ms(uint32_t n);
uint8_t rate = 1000;
static signed char gyro_orientation[9] = {1, 0, 0,
                                           0,1, 0,
                                           0, 0, 1};
static  unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}
static  unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}
#define DIM  2

int ms_open() {
	dmpReady=1;
	initialized = 0;
	for (int i=0;i<DIM;i++){
		lastval[i]=10;
	}

	// initialize device
	printf("Initializing MPU...\n");
	if (mpu_init(NULL) != 0) {
		printf("MPU init failed!\n");
		return -1;
	}
	printf("Setting MPU sensors...\n");
	if (mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL)!=0) {
		printf("Failed to set sensors!\n");
		return -1;
	}
	printf("Setting GYRO sensitivity...\n");
	if (mpu_set_gyro_fsr(2000)!=0) {
		printf("Failed to set gyro sensitivity!\n");
		return -1;
	}
	printf("Setting ACCEL sensitivity...\n");
	if (mpu_set_accel_fsr(8)!=0) {
		printf("Failed to set accel sensitivity!\n");
		return -1;
	}
	// verify connection
	printf("Powering up MPU...\n");
	mpu_get_power_state(&devStatus);
	printf(devStatus ? "MPU6050 connection successful\n" : "MPU6050 connection failed %u\n",devStatus);

	//fifo config
	printf("Setting MPU fifo...\n");
	if (mpu_configure_fifo(INV_XYZ_GYRO|INV_XYZ_ACCEL)!=0) {
		printf("Failed to initialize MPU fifo!\n");
		return -1;
	}

	// load and configure the DMP
	printf("Loading DMP firmware...\n");
	if (dmp_load_motion_driver_firmware()!=0) {
		printf("Failed to enable DMP!\n");
		return -1;
	}
    if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation))){
    }
	printf("Activating DMP...\n");
	if (mpu_set_dmp_state(1)!=0) {
		printf("Failed to enable DMP!\n");
		return -1;
	}

	//dmp_set_orientation()
	//if (dmp_enable_feature(DMP_FEATURE_LP_QUAT|DMP_FEATURE_SEND_RAW_GYRO)!=0) {
	printf("Configuring DMP...\n");
	if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|DMP_FEATURE_GYRO_CAL)!=0) {
		printf("Failed to enable DMP features!\n");
		return -1;
	}


	printf("Setting DMP fifo rate...\n");
	if (dmp_set_fifo_rate(rate)!=0) {
		printf("Failed to set dmp fifo rate!\n");
		return -1;
	}
	printf("Resetting fifo queue...\n");
	if (mpu_reset_fifo()!=0) {
		printf("Failed to reset fifo!\n");
		return -1;
	}
    int timesamp;
	printf("Checking... ");
	do {
		delay_ms(1000/rate);  //dmp will habve 4 (5-1) packets based on the fifo_rate
		r=dmp_read_fifo(g,a,_q,&timesamp,&sensors,&fifoCount);
	} while (r!=0 || fifoCount<5); //packtets!!!
	printf("Done.\n");

	initialized = 1;
	return 0;
}
float roll,pitch,yaw;
int ms_update() {
        int timesamp;
	if (!dmpReady) {
		printf("Error: DMP not ready!!\n");
		return -1;
	}
    int32_t _q[4];
	while (dmp_read_fifo(g,a,_q,&timesamp,&sensors,&fifoCount)!=0); //gyro and accel can be null because of being disabled in the efeatures
    float q[4];
     float _g[4];
       float _a[4];
    for(int i  = 0;i<4;i++){
        q[i] = _q[i]/1073741824.0f;
        _g[i]  = g[i]/16.4;
        _a[i] = a[i]/4096.f;
    }
#define q0 q[0] 
#define q1 q[1] 
#define q2 q[2] 
#define q3 q[3]     
    float recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
    
    roll   = atan2(2.0f * (q[2] *  q[3] + q[0] * q[1]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] +q[3] * q[3]);   
	pitch  = -asin(2.0f * (-q[1] * q[3] +q[0] * q[2]));
	yaw    = -atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] -q[3] * q[3]);

    Sensordata.ax =  _a[0];	
	Sensordata.ay =  _a[1];	 
	Sensordata.az =  _a[2];  
    
    Sensordata.gx =  _g[0];	
	Sensordata.gy =  _g[1];	 
	Sensordata.gz =  _g[2];
}
void get_orller(float* ang)
{
    ang[0] =  roll*57.32  ;
    ang[1] =  pitch*57.32 ;
    ang[2] =   yaw*57.32   ;
}

int ms_close() {
	return 0;
}




