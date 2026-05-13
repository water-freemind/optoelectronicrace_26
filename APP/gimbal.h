#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>
#include <stdbool.h>

/* 电机UART地址 */
#define GIMBAL_ADDR_X      0x01
#define GIMBAL_ADDR_Y      0x02

/* 张大头57电机指令集 */
#define GIMBAL_CMD_ENABLE    0xF3
#define GIMBAL_CMD_ZERO      0x93
#define GIMBAL_CMD_POS_MOVE  0xFD
#define GIMBAL_CMD_SYSTEM    0xFE
#define GIMBAL_SUB_STOP      0x98
#define GIMBAL_CMD_GOZERO    0x9A

/* 帧结尾 */
#define GIMBAL_FRAME_END     0x6B

/* 脉冲参数 (256细分, 1.8°步进电机: 200步/圈 × 256 = 51200脉冲/圈) */
#define PULSE_PER_REV      51200.0f
#define PULSE_PER_MM_XY    (PULSE_PER_REV / 84.0f)   /* 丝杆导程84mm */
#define ZDT_STRETCH_MAX_PULSE 133200

/* 药柜尺寸 */
#define CABINET_HEIGHT  420
#define CABINET_WIDTH   420
#define CABINET_FIRST_FLOOR   0
#define CABINET_SECOND_FLOOR  160
#define CABINET_THIRD_FLOOR   320

/* DMA通道 (UART0 RX, 使用通道1, 通道0已被ADC占用) */
#define GIMBAL_UART_RX_DMA_CHAN  1

/* 电机完成标志 */
extern volatile bool g_gimbal_x_done;
extern volatile bool g_gimbal_y_done;

/* ========== 初始化 ========== */
void Gimbal_Init(void);

/* ========== 驱动层接口 ========== */
void Gimbal_Enable(uint8_t addr, bool enable);
void Gimbal_SetZero(uint8_t addr);
void Gimbal_Gozero(uint8_t addr, bool sync);
void Gimbal_MovePosition(uint8_t addr, int32_t pos, uint16_t speed, uint8_t acc, bool sync);
void Gimbal_Stop(uint8_t addr);

/* ========== 应用层接口 ========== */
void Gimbal_Enable_All(void);
void Gimbal_Gozero_All(void);
void Gimbal_MoveXY_To_mm(float x_mm, float y_mm, uint16_t speed, uint8_t acc, bool sync);

#endif
