#ifndef KNOB_DRV_H
#define KNOB_DRV_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

// 旋钮事件枚举
typedef enum {
    KNOB_EVENT_NONE = 0,    // 无事件
    KNOB_EVENT_CW,          // 顺时针旋转 (Clockwise)
    KNOB_EVENT_CCW,         // 逆时针旋转 (Counter-Clockwise)
    KNOB_EVENT_SHORT_PRESS, // 短按
    KNOB_EVENT_LONG_PRESS   // 长按
} Knob_Event_t;

// 暴露给主函数的全局事件变量
extern volatile Knob_Event_t g_KnobEvent;

// 驱动接口
void Knob_Init(void);
void Knob_Tick_1ms(void);         // 放入 1ms 定时器中断 (处理长短按时间判定)
void Knob_EXTI_Handler(void);     // 放入 GPIO 中断服务函数 (处理旋转)
void Knob_get(void);
#endif // KNOB_DRV_H