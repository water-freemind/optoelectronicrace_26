#include "delay.h"
#include "Knob_drv.h"
#include "Beep.h"
volatile unsigned int delay_times = 0;
volatile uint32_t g_SystemTick = 0;
//搭配滴答定时器实现的精确ms延时
void delay_ms(unsigned int ms)
{
    delay_times = ms;
    while( delay_times != 0 );
}

//滴答定时器中断服务函数,注意优先级配置，目前是最低优先级
void SysTick_Handler(void)
{
    g_SystemTick++;
    Knob_Tick_1ms();
    Beep_Tick_1ms();
    if( delay_times != 0 )
    {
        delay_times--;
    }
}
