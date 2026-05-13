#include "JY62.h"
#include "ti_msp_dl_config.h"
#include "delay.h"

volatile float roll_angle = 0.0f;
volatile float pitch_angle = 0.0f;
volatile float yaw_angle = 0.0f;
volatile float ax = 0.0f, ay = 0.0f, az = 0.0f;
volatile float wx = 0.0f, wy = 0.0f, wz = 0.0f;
volatile float temperature = 0.0f;
volatile uint8_t jy62_new_data = 0;

static volatile uint8_t rx_buffer[11] = {0};
static volatile uint8_t data_index = 0;
static volatile uint8_t frame_started = 0;

#define JY62_UART                 UART1
#define JY62_UART_IRQN            UART1_INT_IRQn

#define JY62_UART_TX_IOMUX        IOMUX_PINCM19
#define JY62_UART_RX_IOMUX        IOMUX_PINCM20
#define JY62_UART_TX_IOMUX_FUNC   IOMUX_PINCM19_PF_UART1_TX
#define JY62_UART_RX_IOMUX_FUNC   IOMUX_PINCM20_PF_UART1_RX

#define JY62_IBRD                 17
#define JY62_FBRD                 23

static const DL_UART_Main_ClockConfig gJY62UartClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gJY62UartConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

void JY62_Init(void)
{
    DL_UART_Main_enablePower(JY62_UART);
    delay_cycles(16);

    DL_GPIO_initPeripheralOutputFunction(JY62_UART_TX_IOMUX, JY62_UART_TX_IOMUX_FUNC);
    DL_GPIO_initPeripheralInputFunction(JY62_UART_RX_IOMUX, JY62_UART_RX_IOMUX_FUNC);

    DL_UART_Main_setClockConfig(JY62_UART, (DL_UART_Main_ClockConfig *)&gJY62UartClockConfig);
    DL_UART_Main_init(JY62_UART, (DL_UART_Main_Config *)&gJY62UartConfig);
    DL_UART_Main_setOversampling(JY62_UART, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(JY62_UART, JY62_IBRD, JY62_FBRD);

    DL_UART_Main_enableInterrupt(JY62_UART, DL_UART_MAIN_INTERRUPT_RX);
    DL_UART_Main_enable(JY62_UART);

    NVIC_ClearPendingIRQ(JY62_UART_IRQN);
    NVIC_EnableIRQ(JY62_UART_IRQN);

    JY62_ConfigMode();
}

void JY62_ConfigMode(void)
{
    delay_ms(500);

    uint8_t save[] = {0xFF, 0xAA, 0x00};
    uint8_t cmd[] = {0xFF, 0xAA, 0x52};

    uint8_t all_data[] = {0xFF, 0xAA, 0x62, 0x07};

    for (int i = 0; i < (int)sizeof(all_data); i++)
    {
        while (DL_UART_isBusy(JY62_UART));
        DL_UART_Main_transmitData(JY62_UART, all_data[i]);
    }
    delay_ms(100);

    for (int i = 0; i < (int)sizeof(save); i++)
    {
        while (DL_UART_isBusy(JY62_UART));
        DL_UART_Main_transmitData(JY62_UART, save[i]);
    }
    delay_ms(100);

    for (int i = 0; i < (int)sizeof(cmd); i++)
    {
        while (DL_UART_isBusy(JY62_UART));
        DL_UART_Main_transmitData(JY62_UART, cmd[i]);
    }
    delay_ms(100);
}

void JY62_ProcessData(uint8_t data)
{
    if (!frame_started)
    {
        if (data_index == 0 && data == 0x55)
        {
            rx_buffer[data_index++] = data;
        }
        else if (data_index == 1 && (data == 0x51 || data == 0x52 || data == 0x53))
        {
            rx_buffer[data_index++] = data;
            frame_started = 1;
        }
        else
        {
            data_index = 0;
        }
    }
    else
    {
        rx_buffer[data_index++] = data;

        if (data_index >= 11)
        {
            uint8_t checksum = 0;
            for (int i = 0; i < 10; i++)
            {
                checksum += rx_buffer[i];
            }

            if (checksum != rx_buffer[10])
            {
                data_index = 0;
                frame_started = 0;
                return;
            }

            short raw1 = (short)(rx_buffer[3] << 8) | rx_buffer[2];
            short raw2 = (short)(rx_buffer[5] << 8) | rx_buffer[4];
            short raw3 = (short)(rx_buffer[7] << 8) | rx_buffer[6];
            short temp_raw = (short)(rx_buffer[9] << 8) | rx_buffer[8];

            temperature = ((float)temp_raw) / 340.0f + 36.53f;

            switch (rx_buffer[1])
            {
                case 0x51:
                    ax = ((float)raw1 / 32768.0f) * 16.0f;
                    ay = ((float)raw2 / 32768.0f) * 16.0f;
                    az = ((float)raw3 / 32768.0f) * 16.0f;
                    break;

                case 0x52:
                    wx = ((float)raw1 / 32768.0f) * 2000.0f;
                    wy = ((float)raw2 / 32768.0f) * 2000.0f;
                    wz = ((float)raw3 / 32768.0f) * 2000.0f;
                    break;

                case 0x53:
                    roll_angle  = ((float)raw1 / 32768.0f) * 180.0f;
                    pitch_angle = ((float)raw2 / 32768.0f) * 180.0f;
                    yaw_angle   = ((float)raw3 / 32768.0f) * 180.0f;
                    break;
            }

            jy62_new_data = 1;

            data_index = 0;
            frame_started = 0;
        }
    }
}

void JY62_Task(void)
{
    uint8_t byte;
    while (DL_UART_Main_receiveDataCheck(JY62_UART, &byte))
    {
        JY62_ProcessData(byte);
    }
}

void UART1_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(JY62_UART))
    {
        case DL_UART_MAIN_IIDX_RX:
        {
            uint8_t data = DL_UART_Main_receiveData(JY62_UART);
            JY62_ProcessData(data);
            break;
        }
        default:
            break;
    }
}