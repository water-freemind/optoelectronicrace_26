#ifndef TRACKLINE_H
#define TRACKLINE_H

#include <stdint.h>
#include "Motor.h"

/* 转弯参数 */
//   Motor A QEI 返回 520 脉冲/轮圈（13PPR×4÷2 × 20 减速比）
//   轮周长 150.8mm → 0.29 mm/脉冲
#define APPROACH_DIST_PULSES  620   /* 直行靠近 ≈180mm（传感器到轮轴距离） */
#define PIVOT_MAX_SPEED       550   /* 原地旋转最高速 */

/* 转弯航向保持参数 */
#define YAW_HOLD_KP           0.3f  /* 靠近阶段航向保持比例系数 */
#define YAW_HOLD_DEADBAND     1.0f  /* yaw死区(度)，避免微小抖动 */
#define STRAIGHT_YAW_THRESH  30     /* 认为直线行驶的误差阈值 */

typedef struct {
    float Kp;
    float Ki;
    float Kd;
} PID_Params_t;

typedef struct {
    int16_t base_speed;
    int16_t max_correction;
    PID_Params_t pid;
    int16_t last_error;
    int32_t integral;
} Trackline_Controller_t;

extern Trackline_Controller_t g_Trackline;

void Trackline_Init(void);
void Trackline_Reset(void);
void Trackline_Task(void);
void Trackline_Sensor_Test(void);
void Trackline_Calibrate_White(void);
void Trackline_Calibrate_Black(void);

#endif
