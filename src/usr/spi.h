#ifndef _SPI_H_
#define _SPI_H_
#ifdef  DEFIN_SPI_EXTERN
	#define  SPI_EXTERN
#else 
	#define  SPI_EXTERN extern 
#endif
#include "stm32f30x.h"

#define SPI_DMA_READADRR  0x3B


SPI_EXTERN uint8_t g_SpiTCFlag;
void spi_config(void);
u8 spi_dma_read(u8 * data,u8 lenth);
void spi_select_mode(uint8_t mode);
uint8_t spi_read( uint8_t reg, uint8_t *buf, uint16_t len);
uint8_t  spi_write( uint8_t reg, uint8_t* data,uint16_t len);
	
#endif
	
	
	