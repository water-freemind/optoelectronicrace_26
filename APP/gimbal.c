#include "gimbal.h"
#include "Uart.h"
#include "delay.h"
#include <string.h>

/* ======================== 完成标志 ======================== */
volatile bool g_gimbal_x_done = false;
volatile bool g_gimbal_y_done = false;

/* ======================== DMA RX 状态 ====================== */
static uint8_t  g_rx_buf[32];
static volatile bool g_rx_capturing;

/* ======================== 私有函数声明 ===================== */
static void gimbal_send_frame(uint8_t addr, const uint8_t *data, uint8_t len);
static void gimbal_flush_rx(void);
static void gimbal_start_rx_capture(void);
static void gimbal_parse_response(void);

/* ======================== 初始化（含DMA RX）================ */
void Gimbal_Init(void)
{
    /* ---- DMA通道1：UART0 RX ---- */
    DL_DMA_setSrcAddr(DMA, GIMBAL_UART_RX_DMA_CHAN,
                      (uint32_t)&UART0->RXDATA);
    DL_DMA_setDestAddr(DMA, GIMBAL_UART_RX_DMA_CHAN,
                      (uint32_t)g_rx_buf);
    DL_DMA_setTransferSize(DMA, GIMBAL_UART_RX_DMA_CHAN,
                           sizeof(g_rx_buf));

    static const DL_DMA_Config gDMA_rxConfig = {
        .transferMode  = DL_DMA_SINGLE_TRANSFER_MODE,
        .extendedMode  = DL_DMA_NORMAL_MODE,
        .destIncrement = DL_DMA_ADDR_INCREMENT,
        .srcIncrement  = DL_DMA_ADDR_UNCHANGED,
        .destWidth     = DL_DMA_WIDTH_BYTE,
        .srcWidth      = DL_DMA_WIDTH_BYTE,
        .trigger       = DMA_UART0_RX_TRIG,
        .triggerType   = DL_DMA_TRIGGER_TYPE_EXTERNAL,
    };
    DL_DMA_initChannel(DMA, GIMBAL_UART_RX_DMA_CHAN,
                       (DL_DMA_Config *)&gDMA_rxConfig);

    /* ---- 使能UART0 RX DMA事件（每收到1字节触发1次DMA传输）---- */
    DL_UART_enableDMAReceiveEvent(UART_0_INST,
                                  DL_UART_DMA_INTERRUPT_RX);

    /* ---- 使能UART0 RX超时中断（检测帧结束）---- */
    DL_UART_setRXInterruptTimeout(UART_0_INST, 10);
    DL_UART_enableInterrupt(UART_0_INST,
                            DL_UART_INTERRUPT_RX_TIMEOUT_ERROR);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    g_rx_capturing = false;
}

/* ================= 清空UART RX FIFO / 残留数据 ============= */
static void gimbal_flush_rx(void)
{
    uint8_t dummy;
    while (DL_UART_receiveDataCheck(UART_0_INST, &dummy)) {}
}

/* ================== 启动新一轮RX捕获 ======================= */
static void gimbal_start_rx_capture(void)
{
    DL_DMA_disableChannel(DMA, GIMBAL_UART_RX_DMA_CHAN);
    gimbal_flush_rx();

    memset((void *)g_rx_buf, 0, sizeof(g_rx_buf));
    DL_DMA_setDestAddr(DMA, GIMBAL_UART_RX_DMA_CHAN,
                       (uint32_t)g_rx_buf);
    DL_DMA_setTransferSize(DMA, GIMBAL_UART_RX_DMA_CHAN,
                           sizeof(g_rx_buf));
    DL_DMA_enableChannel(DMA, GIMBAL_UART_RX_DMA_CHAN);
    g_rx_capturing = true;
}

/* ================= 解析电机应答帧 ========================== */
static void gimbal_parse_response(void)
{
    uint16_t remain = DL_DMA_getTransferSize(DMA,
                        GIMBAL_UART_RX_DMA_CHAN);
    uint16_t rx_len = sizeof(g_rx_buf) - remain;

    if (rx_len < 2) return;

    /* 首字节=电机地址，末字节应为0x6B */
    uint8_t addr = g_rx_buf[0];
    if (g_rx_buf[rx_len - 1] != GIMBAL_FRAME_END) return;

    if (addr == GIMBAL_ADDR_X) {
        g_gimbal_x_done = true;
    } else if (addr == GIMBAL_ADDR_Y) {
        g_gimbal_y_done = true;
    }
    g_rx_capturing = false;
}

/* ==================== UART0中断（RX超时）=================== */
void UART0_IRQHandler(void)
{
    DL_UART_IIDX iid = DL_UART_getPendingInterrupt(UART_0_INST);

    if (iid == DL_UART_IIDX_RX_TIMEOUT_ERROR) {
        if (g_rx_capturing) {
            DL_DMA_disableChannel(DMA, GIMBAL_UART_RX_DMA_CHAN);
            gimbal_parse_response();
            gimbal_flush_rx();
        }
    }
}

/* ======================== 发送帧 =========================== */
static void gimbal_send_frame(uint8_t addr,
                              const uint8_t *data, uint8_t len)
{
    uart0_send_char(addr);
    for (uint8_t i = 0; i < len; i++) {
        uart0_send_char(data[i]);
    }
}

/* ==================== 驱动层接口 ========================== */

void Gimbal_Enable(uint8_t addr, bool enable)
{
    uint8_t data[4] = {
        GIMBAL_CMD_ENABLE, 0xAB, enable ? 1 : 0, 0x00
    };
    gimbal_send_frame(addr, data, 4);
    uart0_send_char(GIMBAL_FRAME_END);
    delay_ms(2);
    gimbal_start_rx_capture();
}

void Gimbal_SetZero(uint8_t addr)
{
    uint8_t data[1] = { GIMBAL_CMD_ZERO };
    gimbal_send_frame(addr, data, 1);
    uart0_send_char(GIMBAL_FRAME_END);
    delay_ms(2);
    gimbal_start_rx_capture();
}

void Gimbal_Gozero(uint8_t addr, bool sync)
{
    uint8_t data[3] = {
        GIMBAL_CMD_GOZERO, 0x02, sync ? 0x01 : 0x00
    };
    gimbal_send_frame(addr, data, 3);
    uart0_send_char(GIMBAL_FRAME_END);
    delay_ms(2);
    gimbal_start_rx_capture();
}

void Gimbal_MovePosition(uint8_t addr, int32_t pos,
                         uint16_t speed, uint8_t acc, bool sync)
{
    uint32_t abs_pos = (pos >= 0) ? (uint32_t)pos : (uint32_t)(-pos);
    uint8_t dir = (pos >= 0) ? 0x00 : 0x01;

    uint8_t data[11] = {
        GIMBAL_CMD_POS_MOVE,
        dir,
        (uint8_t)((speed >> 8) & 0xFF),
        (uint8_t)(speed & 0xFF),
        acc,
        (uint8_t)((abs_pos >> 24) & 0xFF),
        (uint8_t)((abs_pos >> 16) & 0xFF),
        (uint8_t)((abs_pos >> 8) & 0xFF),
        (uint8_t)(abs_pos & 0xFF),
        0x01,
        sync ? 0x01 : 0x00
    };

    if (addr == GIMBAL_ADDR_X) g_gimbal_x_done = false;
    if (addr == GIMBAL_ADDR_Y) g_gimbal_y_done = false;

    gimbal_send_frame(addr, data, 11);
    uart0_send_char(GIMBAL_FRAME_END);
    delay_ms(2);
    gimbal_start_rx_capture();
}

void Gimbal_Stop(uint8_t addr)
{
    uint8_t data[3] = {
        GIMBAL_CMD_SYSTEM, GIMBAL_SUB_STOP, 0x00
    };
    gimbal_send_frame(addr, data, 3);
    uart0_send_char(GIMBAL_FRAME_END);
    delay_ms(2);
    gimbal_start_rx_capture();
}

/* ==================== 应用层接口 ========================== */

void Gimbal_Enable_All(void)
{
    Gimbal_Enable(GIMBAL_ADDR_X, true);
    Gimbal_Enable(GIMBAL_ADDR_Y, true);
}

void Gimbal_Gozero_All(void)
{
    g_gimbal_x_done = false;
    g_gimbal_y_done = false;

    Gimbal_Gozero(GIMBAL_ADDR_X, false);
    Gimbal_Gozero(GIMBAL_ADDR_Y, false);
}

void Gimbal_MoveXY_To_mm(float x_mm, float y_mm,
                          uint16_t speed, uint8_t acc, bool sync)
{
    if (x_mm > CABINET_WIDTH)  x_mm = CABINET_WIDTH;
    if (y_mm > CABINET_HEIGHT) y_mm = CABINET_HEIGHT;

    int32_t x_pulse = (int32_t)(x_mm * PULSE_PER_MM_XY);
    int32_t y_pulse = (int32_t)(y_mm * PULSE_PER_MM_XY);

    if (!g_gimbal_x_done) {
        Gimbal_MovePosition(GIMBAL_ADDR_X, x_pulse, speed, acc, sync);
        delay_ms(10);
    }
    if (!g_gimbal_y_done) {
        Gimbal_MovePosition(GIMBAL_ADDR_Y, y_pulse, speed, acc, sync);
        delay_ms(10);
    }
}
