#ifndef K230_TRACK_H
#define K230_TRACK_H

#include <stdint.h>
#include <stdbool.h>

/* K230 数据包结构 (13字节帧) */
typedef struct {
    volatile uint8_t flag;       // 0xBB=锁定, 0xCC=丢失
    volatile uint8_t s_dx, dx;   // 中心X: 符号(0=右1=左), 偏差(px)
    volatile uint8_t s_dy, dy;   // 中心Y: 符号(0=下1=上), 偏差(px)
    volatile uint8_t idx;        // 圆点索引 0~15
    volatile uint8_t s_dx_b, dx_b; // 圆点X
    volatile uint8_t s_dy_b, dy_b; // 圆点Y
} K230_TrackData_t;

extern volatile K230_TrackData_t g_k230_data;
extern volatile uint8_t g_k230_new_frame;

/* 像素→脉冲 (6400pulse/rev, FOV 86°×55°, QVGA 320×240) */
#define PULSE_PER_PX_X  5
#define PULSE_PER_PX_Y  4

/* 方向极性: 1=正脉冲右/下转, -1=反 */
#define GIMBAL_X_POLARITY  1
#define GIMBAL_Y_POLARITY  1

/* 追踪死区 (脉冲) */
#define AIM_DEADBAND_X  20
#define AIM_DEADBAND_Y  15

/* 指令更新间隔 (帧数, 每N帧发一次) */
#define UPDATE_INTERVAL  6

/* 追踪增益 (百分比, 越小越不超调) */
#define X_TRACK_GAIN  30
#define Y_TRACK_GAIN  35

/* 追踪 PID (pulse空间, 定标×10) */
#define TRACK_KP  5
#define TRACK_KI  0
#define TRACK_KD  80
#define TRACK_I_MAX  200

/* 追踪速度/加速度 */
#define AIM_SPEED  400
#define AIM_ACC    25

/* Y轴限位 (脉冲) */
#define Y_MIN_PULSE  -190
#define Y_MAX_PULSE   380

void K230_Track_Init(void);
void K230_Aim_Task(void);
void K230_Aim_Reset(void);

#endif
