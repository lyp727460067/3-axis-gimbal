#include "mpu6050.h"
#include "SPI.h"
#include "device.h"

void delay_us1(u32 n)
{
	u32 j;
	while(n--)
	for(j=0;j<40;j++);
}
void delay_ms(u32 n)
{
	while(n--)
	delay_us1(1000);
}

uint8_t spi_dmp_read(uint8_t adrr,uint8_t reg,uint16_t lenth,uint8_t* date)
{
    
    spi_read(reg,date,lenth);
      return 0;
}
uint8_t spi_dmp_write(uint8_t adrr,uint8_t reg,uint16_t lenth,uint8_t* date)
{
    spi_write(reg,date,lenth);
    return 0;
}
        
    
uint8_t  mpu6500_init(void)
{	
    spi_config();
    
    
    //delay_ms(200);
    uint8_t temp = 0x80;
    spi_write(0x6B,&temp,1);
    delay_ms(100);
    temp =0;
    spi_write(0x6B,&temp,1);; //wake the mpu	
    temp = 0x00;
    spi_write(26,&temp,1); 
    temp = 0x18;
    spi_write(27,&temp,1); //Gyro Full Scale Select: 10  = 2000dps
    temp =0x10;
    spi_write(28,&temp,1); //Accel Full Scale Select:  = 8g
    temp =0x00;
    spi_write(29,&temp,1); 
    uint8_t mpuid = 0;
    spi_read(117,&mpuid,1);
    if(mpuid == 0x70){
        g_ucDeviceNum  =  0;
        return 1;	
    }
#ifdef	IS_MAIN_CONTROLLOR
	g_ucDeviceNum   =   2;
    
#else 
	g_ucDeviceNum   =   1;
#endif
	return 1;
}
u8 get_mpu_data(int16_t* mpu)
{
	u8 i ;
	u8 tempdata[14];
	if(spi_read(0x3b,tempdata,14)==0)return 0;
	for(i=0;i<7;i++){
		mpu[i] = tempdata[i*2]<<8 | tempdata[i*2 +1]; 
	}
	return 1;
}

u8 get_mpu_dmadata(int16_t* mpu)
{
	u8 i ;
	static u8 state = 0;
    uint8_t temp_data[15];
    uint8_t *tempdata = &temp_data[1];
	switch(state){
    case 0:
        spi_select_mode(0);
        state   = 1;
    case 1:
        if(spi_dma_read(temp_data,15)==0)return 0;
        for(i=0;i<7;i++){
            mpu[i] = tempdata[i*2]<<8 | tempdata[i*2 +1]; 
        }
        return 1;
	}
    
}




