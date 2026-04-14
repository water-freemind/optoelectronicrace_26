#include "ti_msp_dl_config.h"
#include "delay.h"
#include "OLED.h"
#include "Easy_Menu.h"
#include <string.h>
extern volatile unsigned int delay_times;

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    // OLED_ShowString(0, 0, "123456789", 8);
     Easy_Menu_Init(NULL, NULL, NULL, NULL);
    // Easy_Menu_Display_Char(0, 0, 'U');
    //Easy_Menu_Display_String(0, 3, "123456789");
    while (1) {
        Easy_Menu_Display(delay_times);
    }
}