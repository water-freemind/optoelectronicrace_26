#include "Motor.h"

/* ==================== PWM / 方向控制 ==================== */

static uint16_t clampSpeed(int16_t speed)
{
    if (speed > MOTOR_PWM_PERIOD)  return 0;
    if (speed < -MOTOR_PWM_PERIOD) return 0;
    uint16_t abs_val = (speed < 0) ? (uint16_t)(-speed) : (uint16_t)speed;
    return MOTOR_PWM_PERIOD - abs_val;
}

void Motor_Init(void)
{
    DL_TimerG_setCaptureCompareValue(PWM_0_INST, MOTOR_PWM_PERIOD, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_0_INST, MOTOR_PWM_PERIOD, DL_TIMER_CC_1_INDEX);
    Motor_Coast();
    Motor_B_EncoderInit();
}

void Motor_A_SetSpeed(int16_t speed)
{
    uint16_t cmp = clampSpeed(speed);
    DL_TimerG_setCaptureCompareValue(PWM_0_INST, cmp, DL_TIMER_CC_0_INDEX);

    if (speed > 0) {
        AIN1_OUT(0); AIN2_OUT(1);
    } else if (speed < 0) {
        AIN1_OUT(1); AIN2_OUT(1);
    } else {
        AIN1_OUT(0); AIN2_OUT(0);
    }
}

void Motor_B_SetSpeed(int16_t speed)
{
    uint16_t cmp = clampSpeed(speed);
    DL_TimerG_setCaptureCompareValue(PWM_0_INST, cmp, DL_TIMER_CC_1_INDEX);

    if (speed > 0) {
        BIN1_OUT(1); BIN2_OUT(0);
    } else if (speed < 0) {
        BIN1_OUT(0); BIN2_OUT(1);
    } else {
        BIN1_OUT(0); BIN2_OUT(0);
    }
}

void Motor_SetSpeed(int16_t speedA, int16_t speedB)
{
    Motor_A_SetSpeed(speedA);
    Motor_B_SetSpeed(speedB);
}

void Motor_Brake(void)
{
    AIN1_OUT(1); AIN2_OUT(1);
    BIN1_OUT(1); BIN2_OUT(1);
}

void Motor_Coast(void)
{
    AIN1_OUT(0); AIN2_OUT(0);
    BIN1_OUT(0); BIN2_OUT(0);
}

/* ==================== 编码器 A — QEI (TIMG8) ==================== */

void Motor_A_EncoderInit(void)
{
    DL_Timer_setTimerCount(QEI_0_INST,0);
}

int32_t Motor_A_GetEncoderCnt(void)
{
    int16_t raw = (int16_t)DL_TimerG_getTimerCount(QEI_0_INST);
    return -(int32_t)raw / 2;
}

void Motor_A_ResetEncoder(void)
{
    DL_Timer_setTimerCount(QEI_0_INST,0);
}

/* ==================== 编码器 B — GPIO 中断计数 ================== */

static volatile int32_t g_encoderB_cnt = 0;

void Motor_B_EncoderInit(void)
{
    g_encoderB_cnt = 0;
    NVIC_EnableIRQ(GPIO_ENCODER_B_INT_IRQN);
}

void Motor_B_EncoderIRQHandler(void)
{
    uint32_t status = DL_GPIO_getEnabledInterruptStatus(
        GPIO_ENCODER_B_PORT, GPIO_ENCODER_B_AE_PIN);

    if (status & GPIO_ENCODER_B_AE_PIN) {
        

        bool state_A = (DL_GPIO_readPins(GPIO_ENCODER_B_PORT, GPIO_ENCODER_B_AE_PIN) != 0);
        bool state_B = (DL_GPIO_readPins(GPIO_ENCODER_B_PORT, GPIO_ENCODER_B_BE_PIN) != 0);

        if (state_A ^ state_B) { 
            g_encoderB_cnt--;
        } else {
            g_encoderB_cnt++;
        }
        
        DL_GPIO_clearInterruptStatus(GPIO_ENCODER_B_PORT, GPIO_ENCODER_B_AE_PIN);
    }
}

int32_t Motor_B_GetEncoderCnt(void)
{
    return g_encoderB_cnt;
}

void Motor_B_ResetEncoder(void)
{
    g_encoderB_cnt = 0;
}
