#ifndef MOTOR_H
#define MOTOR_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

/* TB6612 方向控制引脚宏 */
#define AIN1_OUT(X)  ( (X) ? (DL_GPIO_setPins(GPIO_MOTOR_PORT,GPIO_MOTOR_AIN1_PIN)) : (DL_GPIO_clearPins(GPIO_MOTOR_PORT,GPIO_MOTOR_AIN1_PIN)) )
#define AIN2_OUT(X)  ( (X) ? (DL_GPIO_setPins(GPIO_MOTOR_PORT,GPIO_MOTOR_AIN2_PIN)) : (DL_GPIO_clearPins(GPIO_MOTOR_PORT,GPIO_MOTOR_AIN2_PIN)) )
#define BIN1_OUT(X)  ( (X) ? (DL_GPIO_setPins(GPIO_MOTOR_PORT,GPIO_MOTOR_BIN1_PIN)) : (DL_GPIO_clearPins(GPIO_MOTOR_PORT,GPIO_MOTOR_BIN1_PIN)) )
#define BIN2_OUT(X)  ( (X) ? (DL_GPIO_setPins(GPIO_MOTOR_PORT,GPIO_MOTOR_BIN2_PIN)) : (DL_GPIO_clearPins(GPIO_MOTOR_PORT,GPIO_MOTOR_BIN2_PIN)) )

/* PWM 周期（与 SysConfig 中 TIMG12 period = 3200 一致）*/
#define MOTOR_PWM_PERIOD 3200

/* ---------- 电机控制 ---------- */
void Motor_Init(void);
void Motor_A_SetSpeed(int16_t speed);
void Motor_B_SetSpeed(int16_t speed);
void Motor_SetSpeed(int16_t speedA, int16_t speedB);
void Motor_Brake(void);
void Motor_Coast(void);

/* ---------- 编码器 A：QEI (TIMG8, PB6/PB7) ---------- */
void    Motor_A_EncoderInit(void);
int32_t Motor_A_GetEncoderCnt(void);
void    Motor_A_ResetEncoder(void);

/* ---------- 编码器 B：GPIO 中断计数 (PA12:AE, PA13:BE) ---------- */
void    Motor_B_EncoderInit(void);
int32_t Motor_B_GetEncoderCnt(void);
void    Motor_B_ResetEncoder(void);
void    Motor_B_EncoderIRQHandler(void);   /* 放入 GPIOA 所在 GROUP 的 IRQHandler */

#endif
