#include "ti_msp_dl_config.h"
#include "delay.h"
#include "OLED.h"
#include "Easy_Menu.h"
#include "Drive/Knob/Knob_drv.h"
#include "Drive/Motor/Motor.h"
#include "APP/Trackline.h"
#include "Beep.h"
int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    Knob_Init();
    Easy_Menu_Init(Display_Char, Display_Char_Line, NULL, NULL);
    Motor_Init();
    Trackline_Init();

    while (1) {
        Knob_get();
        Easy_Menu_Display(g_SystemTick);
        Trackline_Task();
    }
}
