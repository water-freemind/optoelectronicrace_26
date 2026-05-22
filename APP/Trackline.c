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

static unsigned short Anolog[8] = {0};
static unsigned short white[8] = {1800, 1800, 1800, 1800, 1800, 1800, 1800, 1800};
static unsigned short black[8] = {350, 350, 350, 350, 350, 350, 350, 350};
static unsigned short Normal[8];
static unsigned char rx_buff[256] = {0};


static No_MCU_Sensor sensor;
static unsigned char Digtal;

static const int16_t WEIGHTS[8] = {-300, -200, -150, -50, 50, 150, 200, 300};

/* ---------- 用户可调参数（直角转弯） ---------- */
#define APPROACH_PULSES   340    /* 直行靠近脉冲数  115mm ÷ 0.29mm/pulse */
#define APPROACH_SPEED    200    /* 直行靠近速度 */
#define APPROACH_RAMP_DOWN 120   /* 减速起始脉冲 */
#define APPROACH_MIN_SPEED  60   /* 减速最低速度 */
#define PIVOT_SPEED          1700   /* 原地旋转最高速度（角度环maxSpeed） */
#define PIVOT_GUIDE_SPEED     800   /* 传感器引导旋转速度（2/4号弯） */
#define LINE_NORM_TRESHOLD   2400     /* 归一化中线判断阈值 0~4096，<此值视为黑线 */
#define YAW_STRAIGHT_MIN_ENC  170   /* 进入YAW_STRAIGHT后最小距离(脉冲)，防假角 */

uint8_t g_laps = 1;

static uint8_t     g_turnDir   = 0;     /* 1=左转, 2=右转 */
static float       g_approachTargetYaw = 0.0f;/* 靠近阶段目标yaw */

Trackline_Controller_t g_Trackline = {
    .base_speed = 700,//负载750，空载650
    .max_correction = 500,
    .pid = { .Kp = 3.3f, .Ki = 0.02f, .Kd = 0.36f },
    .last_error = 0,   
    .integral = 0
};

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
    No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);

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
    No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
    Get_Anolog_Value(&sensor, Anolog);
    for (int i = 0; i < 8; i++)
        white[i] = Anolog[i];
}

void Trackline_Calibrate_Black(void)
{
    Beep_Trigger(BEEP_MODE_TRIPLE);
    delay_ms(500);
    No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
    Get_Anolog_Value(&sensor, Anolog);
    for (int i = 0; i < 8; i++)
        black[i] = Anolog[i];
    No_MCU_Ganv_Sensor_Init(&sensor, white, black);
    Beep_Trigger(BEEP_MODE_LONG);
}

enum { PHASE_TRACK = 0, PHASE_STOP, PHASE_APPROACH, PHASE_PIVOT, PHASE_YAW_STRAIGHT, PHASE_DONE };

static uint8_t  g_phase             = PHASE_TRACK;
static int16_t  g_approachPulses    = 0;
static float    g_initialYaw        = 0.0f;
static float    g_straightTargetYaw = 0.0f;
static uint8_t  g_cornerCount       = 0;
static uint8_t  g_lineDebounce      = 0;

void Trackline_Reset(void)
{
    g_phase             = PHASE_TRACK;
    g_approachPulses    = 0;
    g_turnDir           = 0;
    g_Trackline.integral     = 0;
    g_Trackline.last_error   = 0;
    g_initialYaw        = yaw_angle;
    g_straightTargetYaw = g_initialYaw;
    g_cornerCount       = 0;
    g_lineDebounce      = 0;
    Motor_SetSpeed(0, 0);
}

static uint8_t line_mask_from_normal(unsigned short *normal)
{
    uint8_t mask = 0;
    int i;
    for (i = 0; i < 8; i++) {
        if (normal[i] < LINE_NORM_TRESHOLD)
            mask |= (1 << i);
    }
    return mask;
}

void Trackline_Task(void)
{
    /* ==================== 阶段5：完成停车 ==================== */
    if (g_phase == PHASE_DONE) {
        Motor_SetSpeed(0, 0);
        return;
    }

    /* ==================== 阶段4：yaw-hold直行过空白段 ==================== */
    if (g_phase == PHASE_YAW_STRAIGHT) {
        No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
        if (Get_Normalize_For_User(&sensor, Normal)) {
            uint8_t line_mask = line_mask_from_normal(Normal);

            float yawError = g_straightTargetYaw - yaw_angle;
            while (yawError > 180.0f) yawError -= 360.0f;
            while (yawError < -180.0f) yawError += 360.0f;
            if (yawError > -YAW_HOLD_DEADBAND && yawError < YAW_HOLD_DEADBAND)
                yawError = 0.0f;
            int16_t yawCorr = (int16_t)(yawError * 11.0f);
            int16_t speedL = APPROACH_SPEED - yawCorr;
            int16_t speedR = APPROACH_SPEED + yawCorr;
            if (speedL > MOTOR_PWM_PERIOD)  speedL = MOTOR_PWM_PERIOD;
            if (speedL < -MOTOR_PWM_PERIOD) speedL = -MOTOR_PWM_PERIOD;
            if (speedR > MOTOR_PWM_PERIOD)  speedR = MOTOR_PWM_PERIOD;
            if (speedR < -MOTOR_PWM_PERIOD) speedR = -MOTOR_PWM_PERIOD;
            Motor_SpeedControl(speedL, speedR);

            /* 检测下一个弯（需走过最小距离防假角） */
            if (Motor_A_GetEncoderCnt() >= YAW_STRAIGHT_MIN_ENC) {
                if ((line_mask & 0x01) && (line_mask & 0x18)) {
                    if (g_cornerCount >= g_laps * 4) {
                        Motor_SetSpeed(0, 0);
                        Beep_Trigger(BEEP_MODE_LONG);
                        g_phase = PHASE_DONE;
                        return;
                    }
                    g_turnDir = 1; Beep_Trigger(BEEP_MODE_SINGLE);
                    g_phase = PHASE_STOP; return;
                }
            }
            /* 中间传感器见线 → 切回PID */
            if (line_mask & 0x18) {
                if (++g_lineDebounce >= 1) {
                    g_phase = PHASE_TRACK;
                    g_lineDebounce = 0;
                    g_Trackline.integral = 0;
                    g_Trackline.last_error = 0;
                }
            } else {
                g_lineDebounce = 0;
            }
        }
        return;
    }

    /* ==================== 阶段3：yaw闭环原地旋转 ==================== */
    if (g_phase == PHASE_PIVOT) {
        /* 弯道2/4：低速传感器引导旋转；弯道1/3/5：高速yaw闭环旋转 */
        {
            uint8_t pos = (g_cornerCount - 1) % 5 + 1;
            if (pos == 2 || pos == 4) {
                Motor_YawControl(PIVOT_GUIDE_SPEED);

                float yawErr = g_straightTargetYaw - yaw_angle;
                while (yawErr > 180.0f) yawErr -= 360.0f;
                while (yawErr < -180.0f) yawErr += 360.0f;
                float absYawErr = yawErr > 0 ? yawErr : -yawErr;

                /* 转够45°后才用传感器找线 */
                if (absYawErr < 45.0f) {
                    No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
                    if (Get_Normalize_For_User(&sensor, Normal)) {
                        uint8_t line_mask = line_mask_from_normal(Normal);
                        if (line_mask & 0x18) {
                            Motor_SetSpeed(0, 0);
                            delay_ms(30);
                            g_Trackline.integral = 0;
                            g_Trackline.last_error = 0;
                            g_phase = PHASE_TRACK;
                            return;
                        }
                    }
                }
                /* 兜底：±10°内仍未找到线，切到YAW_STRAIGHT盲行 */
                if (absYawErr < 10.0f) {
                    Motor_Brake();
                    delay_ms(50);
                    g_Trackline.integral = 0;
                    g_Trackline.last_error = 0;
                    Motor_A_ResetEncoder();
                    Motor_B_ResetEncoder();
                    Motor_ResetSpeedControl();
                    g_phase = PHASE_YAW_STRAIGHT;
                }
                return;
            }
        }

        /* 弯道1/3/5：高速yaw闭环旋转 */
        {
            float yawErr = g_straightTargetYaw - yaw_angle;
            while (yawErr > 180.0f) yawErr -= 360.0f;
            while (yawErr < -180.0f) yawErr += 360.0f;
            float absErr = yawErr > 0 ? yawErr : -yawErr;
            int16_t maxSpeed = (absErr < 25.0f) ? 300 : PIVOT_SPEED;
            Motor_YawControl(maxSpeed);
        }
        if (Motor_YawIsAtTarget()) {
            Motor_SetSpeed(0, 0);
            delay_ms(80);
            if (g_cornerCount >= (g_laps * 4 + 1)) {
                g_phase = PHASE_DONE;
            } else {
                Motor_A_ResetEncoder();
                Motor_B_ResetEncoder();
                Motor_ResetSpeedControl();
                g_phase = PHASE_YAW_STRAIGHT;
            }
        }
        return;
    }

    /* ==================== 阶段2：yaw航向保持直行120mm ==================== */
    if (g_phase == PHASE_APPROACH) {
        int32_t enc = Motor_A_GetEncoderCnt();
        int32_t remain = g_approachPulses - enc;
        int16_t baseSpeed;

        if (remain <= APPROACH_RAMP_DOWN) {
            int32_t speedRange = APPROACH_SPEED - APPROACH_MIN_SPEED;
            baseSpeed = (remain < 0) ? 0 :
                (int16_t)(APPROACH_MIN_SPEED + speedRange * remain / APPROACH_RAMP_DOWN);
        } else {
            baseSpeed = APPROACH_SPEED;
        }

        if (enc >= g_approachPulses) {
            Motor_SetSpeed(0, 0);
            delay_ms(80);
            Motor_ResetSpeedControl();
            g_phase = PHASE_PIVOT;
            return;
        }

        float yawError = yaw_angle - g_approachTargetYaw;
        if (yawError > -YAW_HOLD_DEADBAND && yawError < YAW_HOLD_DEADBAND)
            yawError = 0.0f;
        int16_t yawCorr = (int16_t)(yawError * YAW_HOLD_KP);
        int16_t speedL = baseSpeed - yawCorr;
        int16_t speedR = baseSpeed + yawCorr;
        if (speedL > MOTOR_PWM_PERIOD)  speedL = MOTOR_PWM_PERIOD;
        if (speedL < -MOTOR_PWM_PERIOD) speedL = -MOTOR_PWM_PERIOD;
        if (speedR > MOTOR_PWM_PERIOD)  speedR = MOTOR_PWM_PERIOD;
        if (speedR < -MOTOR_PWM_PERIOD) speedR = -MOTOR_PWM_PERIOD;
        Motor_SpeedControl(speedL, speedR);
        return;
    }

    /* ==================== 阶段1：检测直角，线性减速停车 ==================== */
    if (g_phase == PHASE_STOP) {
        g_cornerCount++;
        Motor_SetSpeed(100, 100); delay_ms(15);
        Motor_SetSpeed( 60,  60); delay_ms(15);
        Motor_SetSpeed( 20,  20); delay_ms(15);
        Motor_SetSpeed(  0,   0); delay_ms(35);
        g_approachTargetYaw = yaw_angle;
        g_straightTargetYaw += ((g_turnDir == 1) ? 90.0f : -90.0f);
        while (g_straightTargetYaw > 180.0f) g_straightTargetYaw -= 360.0f;
        while (g_straightTargetYaw < -180.0f) g_straightTargetYaw += 360.0f;
        Motor_YawSetTarget(g_straightTargetYaw);
        Motor_A_ResetEncoder();
        Motor_B_ResetEncoder();
        Motor_ResetSpeedControl();
        g_approachPulses = APPROACH_PULSES;
        g_phase = PHASE_APPROACH;
        return;
    }

    /* ==================== 阶段0：正常8传感器PID巡线 ==================== */
    No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);

    if (!Get_Normalize_For_User(&sensor, Normal))
        return;

    uint8_t line_mask = line_mask_from_normal(Normal);

    int32_t numerator = 0;
    int32_t denominator = 0;
    for (int i = 0; i < 8; i++) {
        numerator += (int32_t)Normal[i] * WEIGHTS[i];
        denominator += Normal[i];
    }

    int16_t error;
    if (denominator != 0)
        error = (int16_t)(numerator / denominator);
    else
        error = g_Trackline.last_error;

    int16_t P = (int16_t)(g_Trackline.pid.Kp * error);
    g_Trackline.integral += error;
    if (g_Trackline.integral > 5000)  g_Trackline.integral = 5000;
    if (g_Trackline.integral < -5000) g_Trackline.integral = -5000;
    int16_t I = (int16_t)(g_Trackline.pid.Ki * g_Trackline.integral);
    int16_t D = (int16_t)(g_Trackline.pid.Kd * (error - g_Trackline.last_error));
    g_Trackline.last_error = error;

    int16_t correction = P + I + D;
    if (correction > g_Trackline.max_correction) correction = g_Trackline.max_correction;
    if (correction < -g_Trackline.max_correction) correction = -g_Trackline.max_correction;

    int16_t speedL = g_Trackline.base_speed - correction;
    int16_t speedR = g_Trackline.base_speed + correction;

    if (speedL > MOTOR_PWM_PERIOD)  speedL = MOTOR_PWM_PERIOD;
    if (speedL < -MOTOR_PWM_PERIOD) speedL = -MOTOR_PWM_PERIOD;
    if (speedR > MOTOR_PWM_PERIOD)  speedR = MOTOR_PWM_PERIOD;
    if (speedR < -MOTOR_PWM_PERIOD) speedR = -MOTOR_PWM_PERIOD;

    /* ==================== 直角特征识别（仅左转） ==================== */
    if ((line_mask & 0x01) && (line_mask & 0x18)) {
        if (g_cornerCount >= g_laps * 4) {
            Motor_SetSpeed(0, 0);
            Beep_Trigger(BEEP_MODE_LONG);
            g_phase = PHASE_DONE;
            return;
        }
        g_turnDir = 1;
        Beep_Trigger(BEEP_MODE_SINGLE);
        g_phase = PHASE_STOP;
        return;
    }

    Motor_SetSpeed(speedL, speedR);
}
