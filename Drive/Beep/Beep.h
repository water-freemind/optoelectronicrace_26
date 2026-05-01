#ifndef BEEP_H
#define BEEP_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    BEEP_MODE_OFF = 0,
    BEEP_MODE_SINGLE,
    BEEP_MODE_DOUBLE,
    BEEP_MODE_TRIPLE,
    BEEP_MODE_LONG,
    BEEP_MODE_CONTINUOUS,
    BEEP_MODE_ERROR
} Beep_Mode_t;

void Beep_Init(void);
void Beep_Trigger(Beep_Mode_t mode);
void Beep_Tick_1ms(void);
void Beep_Stop(void);

#endif
