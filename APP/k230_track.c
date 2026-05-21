#include "k230_track.h"
#include "gimbal.h"
#include "Laser.h"
#include "ti_msp_dl_config.h"
#include <string.h>

volatile K230_TrackData_t g_k230_data = {0};
volatile uint8_t g_k230_new_frame = 0;

/* ================= RX 三态机 ================= */
#define FRAME_LEN  13
static uint8_t  g_rx_state = 0;       // 0=等0xAA, 1=等0x0D, 2=收payload
static uint8_t  g_rx_idx = 0;
static uint8_t  g_rx_buf[FRAME_LEN];

/* ================= 初始化 ================= */
void K230_Track_Init(void)
{
    DL_UART_disableDMAReceiveEvent(UART_0_INST, DL_UART_DMA_INTERRUPT_RX);
    DL_UART_enableInterrupt(UART_0_INST, DL_UART_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

/* ================= UART0 RX 中断 ================= */
void UART0_IRQHandler(void)
{
    DL_UART_IIDX iid = DL_UART_getPendingInterrupt(UART_0_INST);

    if (iid != DL_UART_IIDX_RX) return;

    uint8_t byte = DL_UART_receiveData(UART_0_INST);

    switch (g_rx_state) {
    case 0:   // 等 0xAA
        if (byte == 0xAA) {
            g_rx_buf[0] = byte;
            g_rx_idx = 1;
            g_rx_state = 1;
        }
        break;

    case 1:   // 等 0x0D
        if (byte == 0x0D) {
            g_rx_buf[1] = byte;
            g_rx_idx = 2;
            g_rx_state = 2;
        } else {
            g_rx_state = 0;
        }
        break;

    case 2:   // 收 11 字节 (payload + checksum)
        g_rx_buf[g_rx_idx++] = byte;
        if (g_rx_idx >= FRAME_LEN) {
            g_rx_state = 0;

            /* checksum */
            uint8_t sum = 0;
            for (int i = 0; i < 12; i++) sum += g_rx_buf[i];
            if (sum != g_rx_buf[12]) return;

            /* 填入结构体 (g_rx_buf[2]~[11] = 10 bytes) */
            g_k230_data.flag   = g_rx_buf[2];
            g_k230_data.s_dx   = g_rx_buf[3];
            g_k230_data.dx     = g_rx_buf[4];
            g_k230_data.s_dy   = g_rx_buf[5];
            g_k230_data.dy     = g_rx_buf[6];
            g_k230_data.idx    = g_rx_buf[7];
            g_k230_data.s_dx_b = g_rx_buf[8];
            g_k230_data.dx_b   = g_rx_buf[9];
            g_k230_data.s_dy_b = g_rx_buf[10];
            g_k230_data.dy_b   = g_rx_buf[11];

            g_k230_new_frame = 1;
        }
        break;
    }
}

/* ================= 追踪闭环任务 ================= */
static int32_t g_target_pan  = 0;
static int32_t g_target_pitch = 0;
static uint8_t  g_aim_started = 0;
static uint8_t  g_laser_latched = 0;
static uint8_t  g_update_cnt = 0;

/* PID 状态 */
static int32_t g_i_x = 0, g_i_y = 0;
static int32_t g_last_x = 0, g_last_y = 0;

void K230_Aim_Reset(void)
{
    g_aim_started = 0;
    g_laser_latched = 0;
    g_update_cnt = 0;
    g_i_x = 0;  g_i_y = 0;
    g_last_x = 0; g_last_y = 0;
}

void K230_Aim_Task(void)
{
    if (!g_aim_started) {
        Gimbal_Enable_All();
        g_target_pan  = 0;
        g_target_pitch = 0;
        g_aim_started = 1;
    }

    if (!g_k230_new_frame) return;
    g_k230_new_frame = 0;

    if (g_k230_data.flag != 0xBB) {
        g_i_x = 0;  g_i_y = 0;
        g_last_x = 0; g_last_y = 0;
        g_update_cnt = 0;
        return;
    }

    /* 像素 → 脉冲（符号 + 极性） */
    int32_t err_x = (int32_t)g_k230_data.dx * PULSE_PER_PX_X;
    int32_t err_y = (int32_t)g_k230_data.dy * PULSE_PER_PX_Y;
    if (g_k230_data.s_dx) err_x = -err_x;
    if (g_k230_data.s_dy) err_y = -err_y;
    err_x *= GIMBAL_X_POLARITY;
    err_y *= GIMBAL_Y_POLARITY;

    /* PID 计算 (定标 ×10) */
    int32_t p_x = err_x * TRACK_KP;
    int32_t p_y = err_y * TRACK_KP;
    g_i_x += err_x * TRACK_KI;
    g_i_y += err_y * TRACK_KI;
    if (g_i_x > TRACK_I_MAX)  g_i_x = TRACK_I_MAX;
    if (g_i_x < -TRACK_I_MAX) g_i_x = -TRACK_I_MAX;
    if (g_i_y > TRACK_I_MAX)  g_i_y = TRACK_I_MAX;
    if (g_i_y < -TRACK_I_MAX) g_i_y = -TRACK_I_MAX;
    int32_t d_x = (err_x - g_last_x) * TRACK_KD;
    int32_t d_y = (err_y - g_last_y) * TRACK_KD;
    g_last_x = err_x;
    g_last_y = err_y;
    int32_t adj_x = (p_x + g_i_x + d_x) / 10;
    int32_t adj_y = (p_y + g_i_y + d_y) / 10;

    /* 增益再缩放 */
    adj_x = adj_x * X_TRACK_GAIN / 100;
    adj_y = adj_y * Y_TRACK_GAIN / 100;

    /* 死区 → 对准锁存激光 */
    int32_t ax = err_x < 0 ? -err_x : err_x;
    int32_t ay = err_y < 0 ? -err_y : err_y;
    if (ax <= AIM_DEADBAND_X && ay <= AIM_DEADBAND_Y) {
        if (!g_laser_latched) {
            Gimbal_Stop(GIMBAL_ADDR_X);
            Gimbal_Stop(GIMBAL_ADDR_Y);
            Laser_On();
            g_laser_latched = 1;
        }
        g_update_cnt = 0;
        return;
    }

    /* 指令限速: 每 UPDATE_INTERVAL 帧发一次 */
    if (++g_update_cnt < UPDATE_INTERVAL) return;
    g_update_cnt = 0;

    /* 累计绝对位置 */
    g_target_pan  += adj_x;
    g_target_pitch += adj_y;

    /* Y轴限位 */
    if (g_target_pitch < Y_MIN_PULSE) g_target_pitch = Y_MIN_PULSE;
    if (g_target_pitch > Y_MAX_PULSE) g_target_pitch = Y_MAX_PULSE;

    Gimbal_MovePosition(GIMBAL_ADDR_X, g_target_pan,  AIM_SPEED, AIM_ACC, false);
    Gimbal_MovePosition(GIMBAL_ADDR_Y, g_target_pitch, AIM_SPEED, AIM_ACC, false);
}
