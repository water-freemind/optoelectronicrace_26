#include "Drive/Knob/Knob_drv.h"
#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "ti_msp_dl_config.h"
#include "delay.h"
#include "OLED.h"
#include "Easy_Menu.h"
#include "Knob_drv.h"
#include <string.h>
extern volatile unsigned int delay_times;
extern volatile uint32_t g_SystemTick;

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    Knob_Init();
    Easy_Menu_Init(Display_Char, Display_Char_Line, NULL, NULL);

    while (1) {
        Knob_get();
        Easy_Menu_Display(g_SystemTick);
    }
}
