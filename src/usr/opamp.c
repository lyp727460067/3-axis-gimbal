#include "opamp.h"
/*
	Õ‚≤ør1 = 1K,r2=10k r3=2k

*/
void Init_OPAMP(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;
	OPAMP_InitTypeDef OPAMP_InitStructure;

	/* GPIOA Peripheral clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/*PA1 PA7 NON  inverting ,pa3 pa5 inverting*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_7;//|GPIO_Pin_3 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Select Vout as inverting input (internal follower) for OPAMP1 */
	OPAMP_InitStructure.OPAMP_InvertingInput = OPAMP_InvertingInput_PGA;
	/* Select IO1 (PA7) as non inverting input for OPAMP1 */
	OPAMP_InitStructure.OPAMP_NonInvertingInput = OPAMP_NonInvertingInput_IO4;
	OPAMP_Init(OPAMP_Selection_OPAMP1, &OPAMP_InitStructure);
	OPAMP_PGAConfig(OPAMP_Selection_OPAMP1, OPAMP_OPAMP_PGAGain_8, OPAMP_PGAConnect_No);
	/* Enable OPAMP1 */
	OPAMP_Cmd(OPAMP_Selection_OPAMP1, ENABLE);
	OPAMP_Init(OPAMP_Selection_OPAMP2, &OPAMP_InitStructure);
	OPAMP_PGAConfig(OPAMP_Selection_OPAMP2, OPAMP_OPAMP_PGAGain_8, OPAMP_PGAConnect_No);
	/* Enable OPAMP1 */
	OPAMP_Cmd(OPAMP_Selection_OPAMP2, ENABLE);
}

void OPAMP_Calibrate(void)
{




}