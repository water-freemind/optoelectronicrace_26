#include "Beep.h"
#include "delay.h"

typedef enum {
    BEEP_STATE_IDLE,
    BEEP_STATE_ON,
    BEEP_STATE_OFF,
    BEEP_STATE_COMPLETE
} Beep_State_t;

static Beep_State_t s_state = BEEP_STATE_IDLE;
static Beep_Mode_t s_mode = BEEP_MODE_OFF;
static uint32_t s_tick_start = 0;
static uint16_t s_on_time = 0;
static uint16_t s_off_time = 0;
static uint8_t s_cycle_cnt = 0;
static uint8_t s_cycle_target = 0;

void Beep_Init(void)
{
    DL_GPIO_clearPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_PIN);
    s_state = BEEP_STATE_IDLE;
}

void Beep_Stop(void)
{
    DL_GPIO_clearPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_PIN);
    s_state = BEEP_STATE_IDLE;
    s_mode = BEEP_MODE_OFF;
}

static void start_on_phase(uint16_t ms)
{
    DL_GPIO_setPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_PIN);
    s_state = BEEP_STATE_ON;
    s_on_time = ms;
    s_tick_start = g_SystemTick;
}

static void start_off_phase(uint16_t ms)
{
    DL_GPIO_clearPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_PIN);
    s_state = BEEP_STATE_OFF;
    s_off_time = ms;
    s_tick_start = g_SystemTick;
}

// 触发蜂鸣器模式: SINGLE=短响一声 DOUBLE=短响两声 TRIPLE=短响三声 LONG=长响500ms CONTINUOUS=连续响 ERROR=急促响三声
void Beep_Trigger(Beep_Mode_t mode)
{
    if (mode == BEEP_MODE_OFF)
    {
        Beep_Stop();
        return;
    }

    s_mode = mode;
    s_cycle_cnt = 0;
    s_cycle_target = 1;

    switch (mode)
    {
        case BEEP_MODE_SINGLE:
            start_on_phase(100);
            break;
        case BEEP_MODE_DOUBLE:
            s_cycle_target = 2;
            start_on_phase(80);
            break;
        case BEEP_MODE_TRIPLE:
            s_cycle_target = 3;
            start_on_phase(60);
            break;
        case BEEP_MODE_LONG:
            start_on_phase(500);
            break;
        case BEEP_MODE_CONTINUOUS:
            s_cycle_target = 0;
            start_on_phase(200);
            break;
        case BEEP_MODE_ERROR:
            s_cycle_target = 3;
            start_on_phase(50);
            break;
        default:
            break;
    }
}

void Beep_Tick_1ms(void)
{
    if (s_state == BEEP_STATE_IDLE || s_state == BEEP_STATE_COMPLETE)
        return;

    uint32_t elapsed = g_SystemTick - s_tick_start;

    if (s_state == BEEP_STATE_ON)
    {
        if (elapsed >= s_on_time)
        {
            s_cycle_cnt++;
            if (s_cycle_target != 0 && s_cycle_cnt >= s_cycle_target)
            {
                DL_GPIO_clearPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_PIN);
                s_state = BEEP_STATE_COMPLETE;
                s_mode = BEEP_MODE_OFF;
                return;
            }
            start_off_phase(s_on_time);
        }
    }
    else if (s_state == BEEP_STATE_OFF)
    {
        if (elapsed >= s_off_time)
        {
            start_on_phase(s_on_time);
        }
    }
}
