#include "Trackline.h"
#include "ti_msp_dl_config.h"
#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "Uart.h"
#include <stdio.h>
#include <string.h>
#include "delay.h"
#include "Beep.h"

static unsigned short Anolog[8] = {0};
static unsigned short white[8] = {1800, 1800, 1800, 1800, 1800, 1800, 1800, 1800};
static unsigned short black[8] = {350, 350, 350, 350, 350, 350, 350, 350};
static unsigned short Normal[8];
static unsigned char rx_buff[256] = {0};


static No_MCU_Sensor sensor;
static unsigned char Digtal;

static const int16_t WEIGHTS[8] = {-400, -300, -200, -50, 50, 200, 300, 400};

/* Right‑angle turn state machine */
typedef enum {
    TURN_NONE = 0,
    TURN_LEFT,
    TURN_RIGHT
} TurnState_t;

/* Turn execution state machine */
static TurnState_t g_turnState   = TURN_NONE;
static uint16_t    g_turnTimer   = 0;

/* Turn detection state machine */
typedef enum {
    DETECT_IDLE,
    DETECT_WATCH_LEFT,
    DETECT_WATCH_RIGHT
} DetectState_t;
static DetectState_t g_detectState = DETECT_IDLE;

Trackline_Controller_t g_Trackline = {
    .base_speed = 550,//负载750，空载550
    .max_correction = 500,
    .pid = { .Kp = 2.0f, .Ki = 0.01f, .Kd = 0.25f },
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

void Trackline_Task(void)
{
    No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);

    if (!Get_Normalize_For_User(&sensor, Normal))
        return;

    Digtal = Get_Digtal_For_User(&sensor);

    uint8_t center = ((Digtal >> 2) & 0x01) + ((Digtal >> 3) & 0x01) +
                     ((Digtal >> 4) & 0x01) + ((Digtal >> 5) & 0x01);

    /* ------------------------------------------------------------------
     *  State 1: Active turn — pivot until line reaches sensor 4
     *  Exit: turnTimer > 5 && sensor 4 sees black
     * ------------------------------------------------------------------ */
    if (g_turnState != TURN_NONE) {
        int16_t turnSpeed = g_Trackline.base_speed - g_Trackline.base_speed / 10;
        if (g_turnState == TURN_LEFT)
            Motor_SetSpeed(-turnSpeed, turnSpeed);
        else
            Motor_SetSpeed(turnSpeed, -turnSpeed);

        g_turnTimer++;
        if (g_turnTimer > 5 && (Digtal & 0x10) == 0x00) {
            g_detectState = DETECT_IDLE;
            g_turnState = TURN_NONE;
            g_turnTimer = 0;
            g_Trackline.integral = 0;
            g_Trackline.last_error = 0;
            Motor_SetSpeed(0, 0);
            delay_ms(50);
            Beep_Stop();
            return;
        }

        if (g_turnTimer > 3000) {
            g_detectState = DETECT_IDLE;
            g_turnState = TURN_NONE;
            g_turnTimer = 0;
            Motor_SetSpeed(0, 0);
            Beep_Stop();
        }
        return;
    }

    /* --- Normal PID tracking --- */
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

    /* ------------------------------------------------------------------
     *  Two‑phase turn detection state machine
     *  Phase 1: line pinned on edge → enter WATCH
     *  Phase 2: line completely lost → trigger turn
     *  If line returns to centre during WATCH → cancel (false alarm)
     * ------------------------------------------------------------------ */
    switch (g_detectState) {

    case DETECT_IDLE:
        /* Fallback: line lost while clearly tracking left/right */
        if (Digtal == 0xFF) {
            if (g_Trackline.last_error > 60) {
                g_turnState = TURN_LEFT;
                break;  /* fall through to turn entry */
            }
            if (g_Trackline.last_error < -60) {
                g_turnState = TURN_RIGHT;
                break;
            }
        }

        /* Line pinned on left → start watching for loss */
        if ((Digtal & 0xFC) == 0xFC && (Digtal & 0x03) != 0x03) {
            g_detectState = DETECT_WATCH_LEFT;
        }
        /* Line pinned on right → start watching for loss */
        else if ((Digtal & 0x1F) == 0x1F && (Digtal & 0xE0) != 0xE0) {
            g_detectState = DETECT_WATCH_RIGHT;
        }
        break;

    case DETECT_WATCH_LEFT:
        if (Digtal == 0xFF) {
            g_turnState = TURN_LEFT;
            break;
        }
        /* Line returned to centre → false alarm */
        if (center >= 2 && (Digtal & 0x18) == 0x00) {
            g_detectState = DETECT_IDLE;
        }
        break;

    case DETECT_WATCH_RIGHT:
        if (Digtal == 0xFF) {
            g_turnState = TURN_RIGHT;
            break;
        }
        if (center >= 2 && (Digtal & 0x18) == 0x00) {
            g_detectState = DETECT_IDLE;
        }
        break;
    }

    /* If a turn was requested by the state machine, enter turn now */
    if (g_turnState != TURN_NONE) {
        g_turnTimer = 0;
        g_detectState = DETECT_IDLE;
        Beep_Trigger(BEEP_MODE_CONTINUOUS);
        int16_t turnSpeed = g_Trackline.base_speed - g_Trackline.base_speed / 10;
        if (g_turnState == TURN_LEFT)
            Motor_SetSpeed(-turnSpeed, turnSpeed);
        else
            Motor_SetSpeed(turnSpeed, -turnSpeed);
        return;
    }

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
