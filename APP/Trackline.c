#include "Trackline.h"
#include "ti_msp_dl_config.h"
#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "Uart.h"
#include <stdio.h>
#include <string.h>
#include "delay.h"
#include "Beep.h"
#include "JY62.h"

extern volatile uint8_t digtals[8];

/* ==================== 传感器原始数据 ==================== */
static unsigned short Anolog[8] = {0};
static unsigned short white[8] = {1800, 1800, 1800, 1800, 1800, 1800, 1800, 1800};
static unsigned short black[8] = {350, 350, 350, 350, 350, 350, 350, 350};
static unsigned short Normal[8];
static unsigned char rx_buff[256] = {0};

No_MCU_Sensor sensor;
static unsigned char Digtal;

/* 加权系数: 左负右正, 用于PID循迹误差计算 */
static const int16_t WEIGHTS[8] = {-300, -200, -150, -50, 50, 150, 200, 300};

/* ==================== 参数定义 ==================== */
/* 旋转 */
#define TURN_SPEED           600
#define APPROACH_TIME_MS     220
#define APPROACH_PULSES      320     /* 越过弯口的编码器脉冲数(约93mm) */
#define APPROACH_SPEED       80      /* 越过弯口速度环目标速度 */ //延时靠经时间

/* 直行(YAW_HOLD) */
#define YAW_HOLD_SPEED       160//100
#define YAW_HOLD_KP          25.0f
#define YAW_HOLD_KD          10.0f
#define YAW_HOLD_DEADBAND    0.2f
#define HOLD_BLIND_MS        2000   /* 直行开始后屏蔽检线(ms) */

/* 直角弯检测 */
#define CORNER_DETECT_CNT    1      /* 满足条件几帧触发(1=立即) */

/* 切回循迹 */
#define LINE_CONFIRM_THRESH  2      /* 连续N帧有线才切回TRACK */
#define REDUCED_SPEED        550    /* 奇数弯后恢复循迹降速 */
#define NORMAL_SPEED         650    /* 正常循迹速度 */

/* 坏传感器配置: 设为-1表示全部传感器正常, 设为0~7表示跳过对应索引 */
#define BAD_SENSOR_IDX       -1   /* -1=全部正常, 4=跳过索引4 */

/* ==================== 控制器实例 ==================== */
Trackline_Controller_t g_Trackline = {
    .base_speed = NORMAL_SPEED,
    .max_correction = 500,
    .pid = { .Kp = 3.3f, .Ki = 0.02f, .Kd = 0.36f },
    .last_error = 0,
    .integral = 0
};

/* ==================== 状态机 ==================== */
enum { PHASE_TRACK = 0, PHASE_TURN, PHASE_YAW_HOLD, PHASE_DONE };

static uint8_t  g_phase = PHASE_TRACK;
static float    g_turnTargetYaw = 0.0f;
static float    g_yawHoldLastErr = 0.0f;
static uint8_t  g_cornerCount = 0;
static uint8_t  g_lineConfirmCnt = 0;
static uint32_t g_holdStartTick = 0;
static uint8_t  g_hadLine = 1;         /* 上一帧是否有线(用于丢线检测) */
static uint8_t  g_lostLineCnt = 0;     /* 连续丢线帧数 */
static uint8_t  g_lastLineLeft = 0;    /* 丢线前线是否在左半边 */
#define LOST_LINE_THRESH  3            /* 连续N帧丢线判为直角 */
uint8_t g_laps = 1;

/* ==================== 辅助函数 ==================== */

/* 读取有效传感器数量(跳过坏的) */
static uint8_t count_active_lines(void)
{
    uint8_t cnt = 0;
    for (int i = 0; i < 8; i++) {
        if (i == BAD_SENSOR_IDX) continue;
        cnt += digtals[i];
    }
    return cnt;
}

/* 判断是否有任意有效传感器检测到线 */
static uint8_t any_line_detected(void)
{
    for (int i = 0; i < 8; i++) {
        if (i == BAD_SENSOR_IDX) continue;
        if (digtals[i]) return 1;
    }
    return 0;
}

/* 更新传感器(调用一次同时刷新digtals[]和Normal[]) */
static void sensor_update(void)
{
    No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
}

/* 直角弯检测: 三条件
   条件A: 左边(0或1)有线 且 总共>=3路 (横线从左扫)
   条件B: 从有线变为连续丢线且丢线前线在左半边 (冲过弯口)
   条件C: 大面积覆盖>=5路同时有线 (整条横线) */
static uint8_t is_corner_detected(void)
{
    uint8_t active = count_active_lines();

    /* 条件C: 大面积横线 — 5路以上同时有线必定是横线 */
    if (active >= 5)
        return 1;

    /* 条件A: 横线特征 — 最左边两路任一亮 + 至少3路同时有线 */
    if ((digtals[0] || digtals[1]) && active >= 3)
        return 1;

    /* 条件B: 丢线检测 — 仅当丢线前线在左半边才算直角 */
    if (active == 0) {
        if (g_hadLine && g_lastLineLeft) {
            g_lostLineCnt++;
            if (g_lostLineCnt >= LOST_LINE_THRESH)
                return 1;
        }
    } else {
        g_hadLine = 1;
        g_lostLineCnt = 0;
        /* 记录线是否在左半边 */
        g_lastLineLeft = (digtals[0] || digtals[1] || digtals[2] || digtals[3]);
    }

    return 0;
}

/* ==================== 初始化/校准 ==================== */

void Trackline_Init(void)
{
    No_MCU_Ganv_Sensor_Init(&sensor, white, black);
    DL_DMA_setSrcAddr(DMA, DMA_detector_out_CHAN_ID, (uint32_t)&ADC0->ULLMEM.MEMRES[0]);
    DL_DMA_setDestAddr(DMA, DMA_detector_out_CHAN_ID, (uint32_t)&ADC_VALUE[0]);
    DL_DMA_enableChannel(DMA, DMA_detector_out_CHAN_ID);
    DL_ADC12_startConversion(ADC_line_detector_INST);
}

void Trackline_Sensor_Test(void)
{
    sensor_update();
    Digtal = Get_Digtal_For_User(&sensor);
    sprintf((char *)rx_buff, "Digtal %d-%d-%d-%d-%d-%d-%d-%d\r\n",
            (Digtal >> 0) & 0x01, (Digtal >> 1) & 0x01, (Digtal >> 2) & 0x01,
            (Digtal >> 3) & 0x01, (Digtal >> 4) & 0x01, (Digtal >> 5) & 0x01,
            (Digtal >> 6) & 0x01, (Digtal >> 7) & 0x01);
    uart0_send_string((char *)rx_buff);
    memset(rx_buff, 0, 256);

    if (Get_Anolog_Value(&sensor, Anolog)) {
        sprintf((char *)rx_buff, "Anolog %d-%d-%d-%d-%d-%d-%d-%d\r\n",
                Anolog[0], Anolog[1], Anolog[2], Anolog[3],
                Anolog[4], Anolog[5], Anolog[6], Anolog[7]);
        uart0_send_string((char *)rx_buff);
        memset(rx_buff, 0, 256);
    }

    if (Get_Normalize_For_User(&sensor, Normal)) {
        sprintf((char *)rx_buff, "Normalize %d-%d-%d-%d-%d-%d-%d-%d\r\n",
                Normal[0], Normal[1], Normal[2], Normal[3],
                Normal[4], Normal[5], Normal[6], Normal[7]);
        uart0_send_string((char *)rx_buff);
        memset(rx_buff, 0, 256);
    }
    delay_ms(1);
}

void Trackline_Calibrate_White(void)
{
    Beep_Trigger(BEEP_MODE_TRIPLE);
    delay_ms(500);
    sensor_update();
    Get_Anolog_Value(&sensor, Anolog);
    for (int i = 0; i < 8; i++) white[i] = Anolog[i];
}

void Trackline_Calibrate_Black(void)
{
    Beep_Trigger(BEEP_MODE_TRIPLE);
    delay_ms(500);
    sensor_update();
    Get_Anolog_Value(&sensor, Anolog);
    for (int i = 0; i < 8; i++) black[i] = Anolog[i];
    No_MCU_Ganv_Sensor_Init(&sensor, white, black);
    Beep_Trigger(BEEP_MODE_LONG);
}

/* ==================== Reset ==================== */

void Trackline_Reset(void)
{
    g_phase = PHASE_TRACK;
    g_Trackline.integral = 0;
    g_Trackline.last_error = 0;
    g_Trackline.base_speed = NORMAL_SPEED;
    g_turnTargetYaw = 0.0f;
    g_yawHoldLastErr = 0.0f;
    g_cornerCount = 0;
    g_lineConfirmCnt = 0;
    g_holdStartTick = 0;
    g_hadLine = 1;
    g_lostLineCnt = 0;
    g_lastLineLeft = 0;
    Motor_SetSpeed(0, 0);
}

/* ==================== corner_action ==================== */
/* 检测到直角弯后: 停车→计数→判停车→设目标→冲一小段→进PHASE_TURN */

static void corner_action(void)
{
    Motor_SetSpeed(0, 0);
    delay_ms(50);
    g_cornerCount++;

    /* 到达目标弯道数: 直接停车不转 */
    if (g_cornerCount >= 4 * g_laps + 1) {
        Beep_Trigger(BEEP_MODE_LONG);
        g_phase = PHASE_DONE;
        return;
    }

    /* 记录目标角度(当前+90) */
    g_turnTargetYaw = yaw_angle + 90.0f;
    if (g_turnTargetYaw > 180.0f)  g_turnTargetYaw -= 360.0f;
    if (g_turnTargetYaw < -180.0f) g_turnTargetYaw += 360.0f;
    Motor_YawSetTarget(g_turnTargetYaw);
    Motor_YawReset();

    /* 速度环闭环 + 编码器计距离越过弯口 */
    Motor_A_ResetEncoder();
    Motor_ResetSpeedControl();
    while (Motor_A_GetEncoderCnt() < APPROACH_PULSES) {
        Motor_SpeedControl(APPROACH_SPEED, APPROACH_SPEED);
        delay_ms(5);
    }
    Motor_SetSpeed(0, 0);
    delay_ms(30);

    Beep_Trigger(BEEP_MODE_SINGLE);
    g_lineConfirmCnt = 0;
    g_hadLine = 0;
    g_lostLineCnt = 0;
    g_phase = PHASE_TURN;
}

/* ==================== 主状态机 ==================== */

void Trackline_Task(void)
{
    /* 每次进入都先刷新传感器 */
    sensor_update();

    switch (g_phase) {

    /* ---------- 停车 ---------- */
    case PHASE_DONE:
        Motor_SetSpeed(0, 0);
        return;

    /* ---------- 旋转中 ---------- */
    case PHASE_TURN: {
        Motor_YawControl(TURN_SPEED);

        /* 弯道类型: corner_mod 0=弯1, 1=弯2, 2=弯3, 3=弯4 (每圈循环) */
        uint8_t corner_mod = (g_cornerCount - 1) % 4;

        /* 偶数弯(弯2/4): 旋转中检测到digtals[3]有线 或 角度到位 就切回循迹 */
        if (corner_mod == 1 || corner_mod == 3) {
            uint8_t line_hit = digtals[3];
            uint8_t angle_done = Motor_YawIsAtTarget();

            if (line_hit || angle_done) {
                if (line_hit) g_lineConfirmCnt++;
                if (g_lineConfirmCnt >= LINE_CONFIRM_THRESH || angle_done) {
                    g_lineConfirmCnt = 0;
                    Motor_Brake();
                    delay_ms(30);
                    Beep_Trigger(BEEP_MODE_LONG);
                    g_Trackline.integral = 0;
                    g_Trackline.last_error = 0;
                    g_Trackline.base_speed = NORMAL_SPEED;
                    g_phase = PHASE_TRACK;
                }
            } else {
                g_lineConfirmCnt = 0;
            }
            return;
        }

        /* 奇数弯(弯1/3): 到达目标角度后进入直行 */
        if (Motor_YawIsAtTarget()) {
            Motor_Brake();
            delay_ms(30);
            Beep_Trigger(BEEP_MODE_LONG);
            Motor_A_ResetEncoder();
            Motor_B_ResetEncoder();
            g_yawHoldLastErr = 0.0f;
            g_lineConfirmCnt = 0;
            g_holdStartTick = g_SystemTick;
            Motor_ResetSpeedControl();
            g_phase = PHASE_YAW_HOLD;
        }
        return;
    }

    /* ---------- 直行找线 ---------- */
    case PHASE_YAW_HOLD: {
        /* 2s屏蔽期后检测到任意有效传感器有线 → 切回循迹 */
        if ((g_SystemTick - g_holdStartTick) >= HOLD_BLIND_MS) {
            if (any_line_detected()) {
                g_lineConfirmCnt++;
                if (g_lineConfirmCnt >= LINE_CONFIRM_THRESH) {
                    g_lineConfirmCnt = 0;
                    g_Trackline.integral = 0;
                    g_Trackline.last_error = 0;
                    Motor_ResetSpeedControl();
                    Beep_Trigger(BEEP_MODE_SINGLE);
                    g_Trackline.base_speed = REDUCED_SPEED;
                    g_phase = PHASE_TRACK;
                    return;
                }
            } else {
                g_lineConfirmCnt = 0;
            }
        }

        /* 陀螺仪角度环PD修正 + 速度环驱动 */
        float yawErr = g_turnTargetYaw - yaw_angle;
        if (yawErr > 180.0f)  yawErr -= 360.0f;
        if (yawErr < -180.0f) yawErr += 360.0f;

        float dErr = yawErr - g_yawHoldLastErr;
        g_yawHoldLastErr = yawErr;

        int16_t yawCorr = 0;
        if (yawErr > YAW_HOLD_DEADBAND || yawErr < -YAW_HOLD_DEADBAND) {
            yawCorr = (int16_t)(yawErr * YAW_HOLD_KP + dErr * YAW_HOLD_KD);
        }

        int16_t tgtL = YAW_HOLD_SPEED - yawCorr;
        int16_t tgtR = YAW_HOLD_SPEED + yawCorr;
        Motor_SpeedControl(tgtL, tgtR);
        return;
    }

    /* ---------- 正常PID循迹 ---------- */
    case PHASE_TRACK:
    default:
        break;
    }

    /* === PHASE_TRACK 逻辑 === */

    /* 直角弯检测 — 最优先判断，传感器刚刷新完立刻检测 */
    if (is_corner_detected()) {
        corner_action();
        return;
    }

    /* PID循迹 */
    if (!Get_Normalize_For_User(&sensor, Normal))
        return;

    int32_t numerator = 0, denominator = 0;
    for (int i = 0; i < 8; i++) {
        if (i == BAD_SENSOR_IDX) continue;
        numerator   += (int32_t)Normal[i] * WEIGHTS[i];
        denominator += Normal[i];
    }

    int16_t error = (denominator != 0)
        ? (int16_t)(numerator / denominator)
        : g_Trackline.last_error;

    int16_t P = (int16_t)(g_Trackline.pid.Kp * error);
    g_Trackline.integral += error;
    if (g_Trackline.integral > 5000)  g_Trackline.integral = 5000;
    if (g_Trackline.integral < -5000) g_Trackline.integral = -5000;
    int16_t I = (int16_t)(g_Trackline.pid.Ki * g_Trackline.integral);
    int16_t D = (int16_t)(g_Trackline.pid.Kd * (error - g_Trackline.last_error));
    g_Trackline.last_error = error;

    int16_t correction = P + I + D;
    if (correction > g_Trackline.max_correction)  correction = g_Trackline.max_correction;
    if (correction < -g_Trackline.max_correction) correction = -g_Trackline.max_correction;

    int16_t speedL = g_Trackline.base_speed - correction;
    int16_t speedR = g_Trackline.base_speed + correction;
    if (speedL > MOTOR_PWM_PERIOD)   speedL = MOTOR_PWM_PERIOD;
    if (speedL < -MOTOR_PWM_PERIOD)  speedL = -MOTOR_PWM_PERIOD;
    if (speedR > MOTOR_PWM_PERIOD)   speedR = MOTOR_PWM_PERIOD;
    if (speedR < -MOTOR_PWM_PERIOD)  speedR = -MOTOR_PWM_PERIOD;

    Motor_SetSpeed(speedL, speedR);
}
