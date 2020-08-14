#ifndef _MPU6500_H
#define _MPU6500_H
#include "stm32f30x.h"

#define MPU_ADDR 0x68

uint8_t  mpu6500_init(void);
uint8_t get_mpu_data(int16_t* mpu);
uint8_t get_mpu_dmadata(int16_t* mpu);

#endif

