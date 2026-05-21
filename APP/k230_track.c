#include "k230_track.h"
#include "gimbal.h"
#include "Laser.h"
#include "ti_msp_dl_config.h"
#include <string.h>

volatile K230_TrackData_t g_k230_data = {0};
volatile uint8_t g_k230_new_frame = 0;

/* ================= RX 三态机 ================= */
#define FRAME_LEN  13
static uint8_t  g_rx_state = 0;
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
    case 0:
        if (byte == 0xAA) {
            g_rx_buf[0] = byte;
            g_rx_idx = 1;
            g_rx_state = 1;
        }
        break;

    case 1:
        if (byte == 0x0D) {
            g_rx_buf[1] = byte;
            g_rx_idx = 2;
            g_rx_state = 2;
        } else {
            g_rx_state = 0;
        }
        break;

    case 2:
        g_rx_buf[g_rx_idx++] = byte;
        if (g_rx_idx >= FRAME_LEN) {
            g_rx_state = 0;

            uint8_t sum = 0;
            for (int i = 0; i < 12; i++) sum += g_rx_buf[i];
            if (sum != g_rx_buf[12]) return;

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

/* ================= 两阶段追踪 ================= */
enum { AIM_SCAN, AIM_STATE_IDLE, AIM_COARSE_SEND, AIM_COARSE_WAIT,
       AIM_FINE_CHECK, AIM_FINE_WAIT, AIM_DONE };

static uint8_t  g_aim_state = AIM_STATE_IDLE;
static uint8_t  g_laser_latched = 0;
static uint8_t  g_wait_cnt = 0;
static uint8_t  g_fine_tries = 0;
static uint8_t  g_stable_cnt = 0;
static int32_t  g_pitch_pos = 0;

void K230_Aim_Reset(void)
{
    g_aim_state = AIM_STATE_IDLE;
    g_laser_latched = 0;
    g_wait_cnt = 0;
    g_fine_tries = 0;
    g_stable_cnt = 0;
    g_pitch_pos = 0;
}

static int32_t pixel_to_pulse(uint8_t s, uint8_t px,
                              uint8_t per_px, int8_t polarity)
{
    int32_t v = (int32_t)px * per_px;
    if (s) v = -v;
    v *= polarity;
    return v;
}

void K230_Aim_Task(void)
{
    if (!g_k230_new_frame) return;
    g_k230_new_frame = 0;

    /* === 扫描模式: 分段45°旋转找靶子 === */
    if (g_aim_state == AIM_SCAN) {
        if (g_k230_data.flag == 0xBB) {
            /* 找到靶子 → 急停 → 等6帧稳住再粗调 */
            if (g_stable_cnt == 0) {
                Gimbal_Stop(GIMBAL_ADDR_X);
                g_stable_cnt = 1;
                return;
            }
            if (g_stable_cnt < 6) {
                g_stable_cnt++;
                return;
            }
            g_stable_cnt = 0;
            Gimbal_Enable_All();
            g_wait_cnt = 0;
            g_aim_state = AIM_STATE_IDLE;
            /* 继续往下走 coarse */
        } else {
            if (g_wait_cnt == 0) {
                Gimbal_Enable_All();
                Gimbal_MovePosition(GIMBAL_ADDR_X, SCAN_STEP_PULSES,
                                    SCAN_STEP_SPEED, FINE_ACC, false, GIMBAL_MODE_REL);
                g_wait_cnt = 1;
                return;
            }
            if (g_wait_cnt < SCAN_STEP_WAIT) {
                g_wait_cnt++;
                return;
            }
            g_wait_cnt = 0;
            return;
        }
    }

    /* 靶子丢失 → 回到扫描 */
    if (g_k230_data.flag != 0xBB) {
        if (g_aim_state >= AIM_FINE_CHECK) {
            Laser_Off();
            g_laser_latched = 0;
        }
        g_aim_state = AIM_SCAN;
        g_wait_cnt = 0;
        g_fine_tries = 0;
        g_stable_cnt = 0;
        return;
    }

    int32_t err_x = pixel_to_pulse(g_k230_data.s_dx, g_k230_data.dx,
                                   PULSE_PER_PX_X, GIMBAL_X_POLARITY);
    int32_t err_y = pixel_to_pulse(g_k230_data.s_dy, g_k230_data.dy,
                                   PULSE_PER_PX_Y, GIMBAL_Y_POLARITY);
    int32_t ax = err_x < 0 ? -err_x : err_x;
    int32_t ay = err_y < 0 ? -err_y : err_y;

    switch (g_aim_state) {

    case AIM_STATE_IDLE:
        Gimbal_Enable_All();
        g_pitch_pos += err_y;
        Gimbal_MovePosition(GIMBAL_ADDR_X, err_x, COARSE_SPEED, COARSE_ACC, false, GIMBAL_MODE_REL);
        Gimbal_MovePosition(GIMBAL_ADDR_Y, err_y, COARSE_SPEED, COARSE_ACC, false, GIMBAL_MODE_REL);
        g_wait_cnt = 0;
        g_aim_state = AIM_COARSE_WAIT;
        return;

    case AIM_COARSE_WAIT:
        g_wait_cnt++;
        if (g_wait_cnt >= COARSE_WAIT_FRAMES) {
            g_fine_tries = 0;
            g_stable_cnt = 0;
            g_aim_state = AIM_FINE_CHECK;
            /* fall through to FINE_CHECK */
        } else {
            return;
        }

    case AIM_FINE_CHECK:
        /* 连续稳定N帧才锁定 */
        if (ax <= AIM_DEADBAND_X && ay <= AIM_DEADBAND_Y) {
            if (++g_stable_cnt >= 5) {
                if (!g_laser_latched) {
                    Gimbal_Stop(GIMBAL_ADDR_X);
                    Gimbal_Stop(GIMBAL_ADDR_Y);
                    Laser_On();
                    g_laser_latched = 1;
                }
                g_aim_state = AIM_DONE;
                return;
            }
            return;
        }
        g_stable_cnt = 0;

        /* 超出死区 → 精调 */
        if (g_fine_tries >= FINE_MAX_ATTEMPTS) {
            if (!g_laser_latched) {
                Laser_On();
                g_laser_latched = 1;
            }
            g_aim_state = AIM_DONE;
            return;
        }

        /* Y轴限位 */
        g_pitch_pos += err_y;
        if (g_pitch_pos < Y_MIN_PULSE) {
            err_y -= (g_pitch_pos - Y_MIN_PULSE);
            g_pitch_pos = Y_MIN_PULSE;
        }
        if (g_pitch_pos > Y_MAX_PULSE) {
            err_y -= (g_pitch_pos - Y_MAX_PULSE);
            g_pitch_pos = Y_MAX_PULSE;
        }

        Gimbal_MovePosition(GIMBAL_ADDR_X, err_x, FINE_SPEED, FINE_ACC, false, GIMBAL_MODE_REL);
        Gimbal_MovePosition(GIMBAL_ADDR_Y, err_y, FINE_SPEED, FINE_ACC, false, GIMBAL_MODE_REL);
        g_fine_tries++;
        g_wait_cnt = 0;
        g_aim_state = AIM_FINE_WAIT;
        return;

    case AIM_FINE_WAIT:
        g_wait_cnt++;
        if (g_wait_cnt >= FINE_WAIT_FRAMES) {
            g_aim_state = AIM_FINE_CHECK;
        }
        return;

    case AIM_DONE:
        if (ax > AIM_DEADBAND_X || ay > AIM_DEADBAND_Y) {
            Gimbal_MovePosition(GIMBAL_ADDR_X, err_x, COARSE_SPEED, COARSE_ACC, false, GIMBAL_MODE_REL);
            Gimbal_MovePosition(GIMBAL_ADDR_Y, err_y, COARSE_SPEED, COARSE_ACC, false, GIMBAL_MODE_REL);
            g_wait_cnt = 0;
            g_aim_state = AIM_COARSE_WAIT;
        }
        return;
    }
}
