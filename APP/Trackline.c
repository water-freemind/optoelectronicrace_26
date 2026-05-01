#include "Trackline.h"
#include "ti_msp_dl_config.h"
#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "Uart.h"
#include <stdio.h>
#include <string.h>
#include "delay.h"

static unsigned short Anolog[8] = {0};
static unsigned short white[8] = {1800, 1800, 1800, 1800, 1800, 1800, 1800, 1800};
static unsigned short black[8] = {300, 300, 300, 300, 300, 300, 300, 300};
static unsigned short Normal[8];
static unsigned char rx_buff[256] = {0};

static No_MCU_Sensor sensor;
static unsigned char Digtal;

static const int16_t WEIGHTS[8] = {-350, -250, -150, -50, 50, 150, 250, 350};

Trackline_Controller_t g_Trackline = {
    .base_speed = 500,
    .max_correction = 500,
    .pid = { .Kp = 0.4f, .Ki = 0.0f, .Kd = 0.0f },
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

void Trackline_Task(void)
{
    No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);

    if (!Get_Normalize_For_User(&sensor, Normal))
        return;

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
    if (correction > g_Trackline.max_correction)
        correction = g_Trackline.max_correction;
    if (correction < -g_Trackline.max_correction)
        correction = -g_Trackline.max_correction;

    int16_t speedL = g_Trackline.base_speed - correction;
    int16_t speedR = g_Trackline.base_speed + correction;

    if (speedL > MOTOR_PWM_PERIOD)  speedL = MOTOR_PWM_PERIOD;
    if (speedL < -MOTOR_PWM_PERIOD) speedL = -MOTOR_PWM_PERIOD;
    if (speedR > MOTOR_PWM_PERIOD)  speedR = MOTOR_PWM_PERIOD;
    if (speedR < -MOTOR_PWM_PERIOD) speedR = -MOTOR_PWM_PERIOD;

    Motor_SetSpeed(speedL, speedR);
}
