#include "ti_msp_dl_config.h"
#include "delay.h"
#include "OLED.h"
int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    OLED_ShowString(0,0,(uint8_t *)"abcdefghijk",8);
    while (1) {
        
    }
}