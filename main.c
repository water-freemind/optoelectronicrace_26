#include "ti_msp_dl_config.h"
#include "delay.h"
int main(void)
{
    SYSCFG_DL_init();

    while (1) {
        //LED引脚输出高电平
        DL_GPIO_setPins(LED_PORT, LED_PIN_0_PIN);
        delay_ms(1000);
        //LED引脚输出低电平
        DL_GPIO_clearPins(LED_PORT, LED_PIN_0_PIN);
        delay_ms(1000);
    }
}
