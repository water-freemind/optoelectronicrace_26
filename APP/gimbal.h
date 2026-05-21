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

/* 位置模式 */
#define GIMBAL_MODE_REL      0x00
#define GIMBAL_MODE_ABS      0x01

/* 帧结尾 */
#define GIMBAL_FRAME_END     0x6B

/* 脉冲参数 (32细分, 1.8°步进电机: 200步/圈 × 32 = 6400脉冲/圈) */
#define PULSE_PER_REV      6400.0f

/* ========== 初始化 ========== */
void Gimbal_Init(void);

/* ========== 驱动层接口 ========== */
void Gimbal_Enable(uint8_t addr, bool enable);
void Gimbal_SetZero(uint8_t addr);
void Gimbal_Gozero(uint8_t addr, bool sync);
void Gimbal_MovePosition(uint8_t addr, int32_t pos, uint16_t speed, uint8_t acc, bool sync, uint8_t mode);
void Gimbal_Stop(uint8_t addr);

/* ========== 应用层接口 ========== */
void Gimbal_Enable_All(void);
void Gimbal_Gozero_All(void);

#endif
