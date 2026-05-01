#include "ti_msp_dl_config.h"
#include "delay.h"
#include "OLED.h"
#include "Easy_Menu.h"
#include "Drive/Knob/Knob_drv.h"
#include "Drive/Motor/Motor.h"
#include "APP/Trackline.h"

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    Knob_Init();
    Easy_Menu_Init(Display_Char, Display_Char_Line, NULL, NULL);
    Motor_Init();
    Trackline_Init();

    int32_t a = 0;
    int32_t b = 0;
    while (1) {
        Knob_get();
        // Easy_Menu_Display(g_SystemTick);
        Trackline_Task();
        a = Motor_A_GetEncoderCnt();
        b = Motor_B_GetEncoderCnt();
        OLED_ShowNum(0, 0, a, 5, 8);
        OLED_ShowNum(0, 1, b, 5, 8);
    }
}
