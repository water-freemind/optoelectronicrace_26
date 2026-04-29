#include "Knob_drv.h"
#include "Easy_Menu.h"
#include "OLED.h"
#include "Motor.h"
// 全局事件变量
volatile Knob_Event_t g_KnobEvent = KNOB_EVENT_NONE;

// 按键计时器内部变量
static uint16_t s_press_cnt = 0;
static bool s_long_pressed = false;
Easy_Menu_Input_TYPE menu_move = EASY_MENU_NONE;
// 初始化
void Knob_Init(void) {
    // 开启对应的 GPIO 中断 (MSPM0G3507 的 PORTB 通常在 GROUP1)
    NVIC_EnableIRQ(GPIO_KNOB_INT_IRQN);
}

// 外部中断处理函数 (处理旋转，极简逻辑)
void Knob_EXTI_Handler(void) {
    // 获取 A 相中断状态
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(GPIO_KNOB_PORT, GPIO_KNOB_A_PIN);

    // 只要 A 相发生下降沿中断
    if (status & GPIO_KNOB_A_PIN) {
        
        // 直接读取 B 相的电平，判断方向 (无需任何延时防抖)
        if (DL_GPIO_readPins(GPIO_KNOB_PORT, GPIO_KNOB_B_PIN) != 0) {
            g_KnobEvent = KNOB_EVENT_CCW;  // B为高电平，顺时针
        } else {
            g_KnobEvent = KNOB_EVENT_CW; // B为低电平，逆时针
        }
        
        // 清除中断标志位
        DL_GPIO_clearInterruptStatus(GPIO_KNOB_PORT, GPIO_KNOB_A_PIN);
    }
}

// 1ms 滴答函数 (仅用于区分长短按)
void Knob_Tick_1ms(void) {
    // 直接读取 S 相按键状态 (按下为低电平 0)
    if (DL_GPIO_readPins(GPIO_KNOB_PORT, GPIO_KNOB_S_PIN) == 0) {
        s_press_cnt++; // 按下时直接开始累加时间
        
        if (s_press_cnt >= 400 && !s_long_pressed) {
            g_KnobEvent = KNOB_EVENT_LONG_PRESS;
            s_long_pressed = true; // 标记已触发长按
        }
    } else {
        // 松开按键时
        if (s_press_cnt > 0 && !s_long_pressed) {
            // 如果按下了，且没达到长按标准，直接判定为短按
            g_KnobEvent = KNOB_EVENT_SHORT_PRESS;
        }
        
        // 状态清零
        s_press_cnt = 0;
        s_long_pressed = false;
    }
}
void Knob_get(void)
{
    if (g_KnobEvent != KNOB_EVENT_NONE) {
            
            // 缓存一下事件，立刻把全局变量清空，防止处理太慢漏掉下一个事件
            Knob_Event_t current_event = g_KnobEvent;
            g_KnobEvent = KNOB_EVENT_NONE; 
            menu_move = EASY_MENU_NONE;
            Easy_Menu_Input(menu_move);
            // 在主循环里刷屏幕，绝对安全！
            switch (current_event) {
                case KNOB_EVENT_CW:
                    // 顺时针代码
                    menu_move = EASY_MENU_DOWN;
                    Easy_Menu_Input(menu_move);
                    break;
                case KNOB_EVENT_CCW:
                    // 逆时针代码
                    menu_move = EASY_MENU_UP;
                    Easy_Menu_Input(menu_move);
                    break;
                case KNOB_EVENT_SHORT_PRESS:
                    // 短按代码
                    menu_move = EASY_MENU_RIGHT;
                    Easy_Menu_Input(menu_move);
                    break;
                case KNOB_EVENT_LONG_PRESS:
                    menu_move = EASY_MENU_LEFT;
                    Easy_Menu_Input(menu_move);
                    break;
                default:
                    menu_move = EASY_MENU_UP;
                    Easy_Menu_Input(menu_move);
                    break;
            }
    }
}
void GROUP1_IRQHandler(void)//Group1的中断服务函数
{
    //读取Group1的中断寄存器并清除中断标志位
    switch( DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1) )
    {
        case GPIO_KNOB_INT_IIDX:
            Knob_EXTI_Handler();
        break;
        case GPIO_ENCODER_B_INT_IIDX:
            Motor_B_EncoderIRQHandler();
        break;
    }
}
