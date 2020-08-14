#ifndef _TELECONTROLLER_H
#define _TELECONTROLLER_H
#include "stm32f30x.h"



uint8_t  TeleCalibrere(void);
uint8_t  TeleCalibrereSD2(void);
void TeleExpAng(void);
void TeleExpAngSD2(void);
void update_exp_pra_to_flash(void);
void update_exp_pra_from_flash(void);
#endif