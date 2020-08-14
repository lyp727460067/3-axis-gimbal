#include "dmpinterface.h"


/**
 *   @defgroup  eMPL
 *   @brief     Embedded Motion Processing Library
 *
 *   @{
 *       @file      main.c
 *       @brief     Test app for eMPL using the Motion Driver DMP image.
 */
 
/* Includes ------------------------------------------------------------------*/

#include "stdio.h"

#include  "datacalibrate.h"



    
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
#include "packet.h"
/* Private typedef -----------------------------------------------------------*/
/* Data read from MPL. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define PRINT_COMPASS   (0x08)
#define PRINT_EULER     (0x10)
#define PRINT_ROT_MAT   (0x20)
#define PRINT_HEADING   (0x40)
#define PRINT_PEDO      (0x80)
#define PRINT_LINEAR_ACCEL (0x100)
#define PRINT_GRAVITY_VECTOR (0x200)

volatile uint32_t hal_timestamp = 0;
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (1000)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)

#define PEDO_READ_MS    (1000)
#define TEMP_READ_MS    (500)
#define COMPASS_READ_MS (100)

extern int get_tick_count(unsigned long *count);

/* USB RX binary semaphore. Actually, it's just a flag. Not included in struct
 * because it's declared extern elsewhere.
 */
volatile unsigned char rx_new;

unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";

/* Platform-specific information. Kinda like a boardfile. */
struct platform_data_s {
    signed char orientation[9];
};

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the
 * driver(s).
 * TODO: The following matrices refer to the configuration on internal test
 * boards at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
//static struct platform_data_s gyro_pdata = {
//    .orientation = { -1, 0, 0,
//                     0, 0, -1,
//                     0, -1, 0}
//};
static struct platform_data_s gyro_pdata = {
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};

extern  void update_graoffset_clic(float *data,uint8_t idex);
extern void update_gra_clic(int16_t *data,uint8_t idex);
extern void inv_set_temp(long data);
int16_t pctogambal[6];
float  whachtflashtemp;
void set_offset_gra()
{
    long gyro[3], accel[3]; 
    int16_t acce[3];
    float gro[4];
    
    update_gra_clic(acce,1);
    update_graoffset_clic(gro,1);
    
    for(int i=0;i<3;i++){
      accel[i] =     acce[i]*65536L;   
      pctogambal[i] = acce[i];
    }
    for(int i=0;i<3;i++){
      gyro[i] =     (gro[i])*65536L;               
      pctogambal[i+3] = gro[i];
    }
    whachtflashtemp = (35 + (gro[3] /321.f));

    inv_set_accel_bias(accel, 3);
    inv_set_gyro_bias(gyro, 3);
    
    inv_set_temp((long)(whachtflashtemp* 65536L));    
    
}
/*******************************************************************************/

/**
  * @brief main entry point.
  * @par Parameters None
  * @retval void None
  * @par Required preconditions: None
  */

int mpu_interface_init(void)
{ 
  
  inv_error_t result;
    unsigned char accel_fsr,  new_temp = 0;
    unsigned short gyro_rate, gyro_fsr;
    unsigned long timestamp;
    struct int_param_s int_param;

#ifdef COMPASS_ENABLED
    unsigned char new_compass = 0;
    unsigned short compass_fsr;
#endif
  
 
 // result = mpu_init(&int_param);
//  if (result) {
//      MPL_LOGE("Could not initialize gyro.\n");
//  }
  

    /* If you're not using an MPU9150 AND you're not using DMP features, this
     * function will place all slaves on the primary bus.
     * mpu_set_bypass(1);
     */
  result = inv_init_mpl();
  if (result) {
      MPL_LOGE("Could not initialize MPL.\n");
  }

    /* Compute 6-axis and 9-axis quaternions. */
    inv_enable_quaternion();
   // inv_enable_9x_sensor_fusion();
    /* The MPL expects compass data at a constant rate (matching the rate
     * passed to inv_set_compass_sample_rate). If this is an issue for your
     * application, call this function, and the MPL will depend on the
     * timestamps passed to inv_build_compass instead.
     *
     * inv_9x_fusion_use_timestamps(1);
     */

    /* This function has been deprecated.
     * inv_enable_no_gyro_fusion();
     */

    /* Update gyro biases when not in motion.
     * WARNING: These algorithms are mutually exclusive.
     */
   // inv_enable_fast_nomot();
 // inv_enable_motion_no_motion();
 // inv_set_no_motion_time(1000); 
    /* inv_enable_motion_no_motion(); */
    /* inv_set_no_motion_time(1000); */

    /* Update gyro biases when temperature changes. */
    inv_enable_gyro_tc();
    //inv_start_gyro_tc();
    /* This algorithm updates the accel biases when in motion. A more accurate
     * bias measurement can be made when running the self-test (see case 't' in
     * handle_input), but this algorithm can be enabled if the self-test can't
     * be executed in your application.
     *
     * inv_enable_in_use_auto_calibration();
     */
   // inv_enable_in_use_auto_calibration();
    //inv_start_in_use_auto_calibration();
    /* If you need to estimate your heading before the compass is calibrated,
     * enable this algorithm. It becomes useless after a good figure-eight is
     * detected, so we'll just leave it out to save memory.
     * inv_enable_heading_from_gyro()
     */

    /* Allows use of the MPL APIs in read_from_mpl. */
    inv_enable_eMPL_outputs();

  result = inv_start_mpl();
  if (result == INV_ERROR_NOT_AUTHORIZED) {
      while (1) {
          MPL_LOGE("Not authorized.\n");
      }
  }
  if (result) {
      MPL_LOGE("Could not start the MPL.\n");
  }

    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    /* Push both gyro and accel data into the FIFO. */
//    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
//    mpu_set_sample_rate(DEFAULT_MPU_HZ);

    /* Read back configuration in case it was set improperly. */
   // mpu_get_sample_rate(&gyro_rate);
    gyro_fsr = 2000;
    accel_fsr = 2;
   // mpu_get_gyro_fsr(&gyro_fsr);
    //mpu_get_accel_fsr(&accel_fsr);

    /* Sync driver configuration with MPL. */
    /* Sample rate expected in microseconds. */
    inv_set_gyro_sample_rate(1000);
    inv_set_accel_sample_rate(1000);

    /* Set chip-to-body orientation matrix.
     * Set hardware units to dps/g's/degrees scaling factor.
     */
    inv_set_gyro_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)gyro_fsr<<15);
    inv_set_accel_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)accel_fsr<<15);
  /* Compass reads are handled by scheduler. */
  get_tick_count(&timestamp);

    /* To initialize the DMP:
     * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
     *    inv_mpu_dmp_motion_driver.h into the MPU memory.
     * 2. Push the gyro and accel orientation matrix to the DMP.
     * 3. Register gesture callbacks. Don't worry, these callbacks won't be
     *    executed unless the corresponding feature is enabled.
     * 4. Call dmp_enable_feature(mask) to enable different features.
     * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
     * 6. Call any feature-specific control functions.
     *
     * To enable the DMP, just call mpu_set_dmp_state(1). This function can
     * be called repeatedly to enable and disable the DMP at runtime.
     *
     * The following is a short summary of the features supported in the DMP
     * image provided in inv_mpu_dmp_motion_driver.c:
     * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
     * 200Hz. Integrating the gyro data at higher rates reduces numerical
     * errors (compared to integration on the MCU at a lower sampling rate).
     * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
     * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
     * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
     * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
     * an event at the four orientations where the screen should rotate.
     * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
     * no motion.
     * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
     * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
     * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
     * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
     */
//    dmp_load_motion_driver_firmware();
//    dmp_set_orientation(
//        inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
//    dmp_register_tap_cb(tap_cb);
//    dmp_register_android_orient_cb(android_orient_cb);
    /*
     * Known Bug -
     * DMP when enabled will sample sensor data at 200Hz and output to FIFO at the rate
     * specified in the dmp_set_fifo_rate API. The DMP will then sent an interrupt once
     * a sample has been put into the FIFO. Therefore if the dmp_set_fifo_rate is at 25Hz
     * there will be a 25Hz interrupt from the MPU device.
     *
     * There is a known issue in which if you do not enable DMP_FEATURE_TAP
     * then the interrupts will be at 200Hz even if fifo rate
     * is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP
     *
     * DMP sensor fusion works only with gyro at +-2000dps and accel +-2G
     */
//    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
//        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
//        DMP_FEATURE_GYRO_CAL;
//    dmp_enable_feature(hal.dmp_features);
    set_offset_gra();
    //dmp_set_fifo_rate(DEFAULT_MPU_HZ);
   // mpu_set_dmp_state(1);
  //  hal.dmp_on = 1;
    
 
}

extern uint8_t  get_mpu_avgrt(float *sen);
float temperature1;
extern int16_t get_mpu_tempratureorg(void);
void get_mpu_interface_data()
{
    unsigned long timestamp;
  static    unsigned long sensor_timestamp=0;
    int8_t accuracy;
    short gyro[3], accel_short[3];
    unsigned char sensors, more;
    long accel[3], temperature;
again:
  
    sensor_timestamp++;
   // get_tick_count(&sensor_timestamp);
    float data1[6];
    if(get_mpu_avgrt(data1)==0)return ;
    for(int i =0;i< 3;i++){
        gyro[i] = data1[3+i];
    }
    inv_build_gyro(gyro, sensor_timestamp);
    //mpu_get_temperature(&temperature, &sensor_timestamp);
    temperature= (long)((35 + ((get_mpu_tempratureorg() ) / 321)) * 65536L);
    temperature1  = temperature/65536L; 
    inv_build_temp(temperature, sensor_timestamp);
    for(int i =0;i< 3;i++){
        accel_short[i] = data1[i];
    }
    accel[0] = (long)accel_short[0];
    accel[1] = (long)accel_short[1];
    accel[2] = (long)accel_short[2];
    inv_build_accel(accel, 3, sensor_timestamp);            
    inv_execute_on_data();
    long data[3];
    if (inv_get_sensor_type_accel(data, &accuracy, (inv_time_t*)&timestamp)) {    
        //goto again;
        Sensordata.ax   =data[0]/65536.f ;
        Sensordata.ay = data[1]/65536.f;
        Sensordata.az = data[2]/65536.f;
    }
    if(inv_get_sensor_type_gyro(data, &accuracy,(inv_time_t*)&timestamp)){
        //  goto again;
        Sensordata.gx   =data[0]/65536.f ;
        Sensordata.gy = data[1]/65536.f;
        Sensordata.gz = data[2]/65536.f;
    } 

    //inv_stop_gyro_tc();
//        if (inv_get_sensor_type_heading(data, &accuracy,
//            (inv_time_t*)&timestamp)){
//          //  goto again;
//            Sensordata.pitch = data[0]/65536.f ;
//            Sensordata.roll = data[1]/65536.f;
//            Sensordata.yaw = data[2]/65536.f;
//        }        
//    Sensordata.ax = data1[0] ;
//    Sensordata.ay = data1[1];
//    Sensordata.az = data1[2];  
//    
//    
//    Sensordata.gx = data1[3] ;
//    Sensordata.gy = data1[4];
//    Sensordata.gz = data1[5];  
}