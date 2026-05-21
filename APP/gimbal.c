#include "gimbal.h"
#include "Uart.h"
#include "delay.h"

/* ======================== 私有函数 ======================== */
static void gimbal_send_frame(uint8_t addr,
                              const uint8_t *data, uint8_t len)
{
    uart0_send_char(addr);
    for (uint8_t i = 0; i < len; i++) {
        uart0_send_char(data[i]);
    }
}

/* ======================== 初始化 ======================== */
void Gimbal_Init(void) {}

/* ==================== 驱动层接口 ========================== */

void Gimbal_Enable(uint8_t addr, bool enable)
{
    uint8_t data[4] = {
        GIMBAL_CMD_ENABLE, 0xAB, enable ? 1 : 0, 0x00
    };
    gimbal_send_frame(addr, data, 4);
    uart0_send_char(GIMBAL_FRAME_END);
    delay_ms(2);
}

void Gimbal_SetZero(uint8_t addr)
{
    uint8_t data[1] = { GIMBAL_CMD_ZERO };
    gimbal_send_frame(addr, data, 1);
    uart0_send_char(GIMBAL_FRAME_END);
    delay_ms(2);
}

void Gimbal_Gozero(uint8_t addr, bool sync)
{
    uint8_t data[3] = {
        GIMBAL_CMD_GOZERO, 0x02, sync ? 0x01 : 0x00
    };
    gimbal_send_frame(addr, data, 3);
    uart0_send_char(GIMBAL_FRAME_END);
    delay_ms(2);
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

    gimbal_send_frame(addr, data, 11);
    uart0_send_char(GIMBAL_FRAME_END);
    delay_ms(2);
}

void Gimbal_Stop(uint8_t addr)
{
    uint8_t data[3] = {
        GIMBAL_CMD_SYSTEM, GIMBAL_SUB_STOP, 0x00
    };
    gimbal_send_frame(addr, data, 3);
    uart0_send_char(GIMBAL_FRAME_END);
    delay_ms(2);
}

/* ==================== 应用层接口 ========================== */

void Gimbal_Enable_All(void)
{
    Gimbal_Enable(GIMBAL_ADDR_X, true);
    Gimbal_Enable(GIMBAL_ADDR_Y, true);
}

void Gimbal_Gozero_All(void)
{
    Gimbal_Gozero(GIMBAL_ADDR_X, false);
    Gimbal_Gozero(GIMBAL_ADDR_Y, false);
}
