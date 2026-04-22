#ifndef __DELAY_H
#define __DELAY_H
#include "ti_msp_dl_config.h"

void delay_ms(unsigned int ms);
extern volatile uint32_t g_SystemTick;//1ms SYSCLK
#endif