#include "Motor.h"
#include "JY62.h"

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
        AIN1_OUT(1); AIN2_OUT(0);
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

/* ==================== 编码器速度环（PI闭环比对） ==================== */
#define VEL_KP     1.5f
#define VEL_KI     0.08f
#define VEL_INT_LIMIT  20000
#define VEL_COMP_LIMIT 400

static int32_t g_lastEncA = 0;
static int32_t g_lastEncB = 0;
static int32_t g_velIntA = 0;
static int32_t g_velIntB = 0;

void Motor_SpeedControl(int16_t targetA, int16_t targetB)
{
    int32_t encA = Motor_A_GetEncoderCnt();
    int32_t encB = Motor_B_GetEncoderCnt();

    int16_t actualA = (int16_t)(encA - g_lastEncA);
    int16_t actualB = (int16_t)(encB - g_lastEncB);

    g_lastEncA = encA;
    g_lastEncB = encB;

    int16_t errA = targetA - actualA;
    int16_t errB = targetB - actualB;

    g_velIntA += errA;
    g_velIntB += errB;
    if (g_velIntA > VEL_INT_LIMIT)  g_velIntA = VEL_INT_LIMIT;
    if (g_velIntA < -VEL_INT_LIMIT) g_velIntA = -VEL_INT_LIMIT;
    if (g_velIntB > VEL_INT_LIMIT)  g_velIntB = VEL_INT_LIMIT;
    if (g_velIntB < -VEL_INT_LIMIT) g_velIntB = -VEL_INT_LIMIT;

    int16_t compA = (int16_t)(errA * VEL_KP + g_velIntA * VEL_KI);
    int16_t compB = (int16_t)(errB * VEL_KP + g_velIntB * VEL_KI);

    if (compA > VEL_COMP_LIMIT)  compA = VEL_COMP_LIMIT;
    if (compA < -VEL_COMP_LIMIT) compA = -VEL_COMP_LIMIT;
    if (compB > VEL_COMP_LIMIT)  compB = VEL_COMP_LIMIT;
    if (compB < -VEL_COMP_LIMIT) compB = -VEL_COMP_LIMIT;

    Motor_A_SetSpeed(targetA + compA);
    Motor_B_SetSpeed(targetB + compB);
}

void Motor_ResetSpeedControl(void)
{
    g_lastEncA = Motor_A_GetEncoderCnt();
    g_lastEncB = Motor_B_GetEncoderCnt();
    g_velIntA = 0;
    g_velIntB = 0;
}

/* ==================== yaw角度环（PD伺服，直接PWM输出） ==================== */
#define YAW_KP        11.0f
#define YAW_KD        1.5f
#define YAW_DEADBAND  1.0f
#define YAW_SPEED_MIN 220

static float g_yawTarget    = 0.0f;
static float g_yawLastError = 0.0f;

uint8_t Motor_YawIsAtTarget(void)
{
    float err = g_yawTarget - yaw_angle;
    while (err > 180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    float absErr = err > 0 ? err : -err;
    return absErr < YAW_DEADBAND ? 1 : 0;
}

void Motor_YawReset(void)
{
    g_yawLastError = 0.0f;
}

void Motor_YawSetTarget(float target)
{
    g_yawTarget    = target;
    g_yawLastError = 0.0f;
}

void Motor_YawControl(int16_t maxSpeed)
{
    float yawError = g_yawTarget - yaw_angle;
    while (yawError > 180.0f) yawError -= 360.0f;
    while (yawError < -180.0f) yawError += 360.0f;

    float absErr = yawError > 0 ? yawError : -yawError;

    if (absErr < YAW_DEADBAND) {
        Motor_SetSpeed(0, 0);
        g_yawLastError = 0.0f;
        return;
    }

    float dErr = yawError - g_yawLastError;
    g_yawLastError = yawError;

    int16_t speed = (int16_t)(yawError * YAW_KP + dErr * YAW_KD);

    if (speed > maxSpeed)  speed = maxSpeed;
    if (speed < -maxSpeed) speed = -maxSpeed;
    if (speed > 0 && speed < YAW_SPEED_MIN)  speed = YAW_SPEED_MIN;
    if (speed < 0 && speed > -YAW_SPEED_MIN) speed = -YAW_SPEED_MIN;

    Motor_SetSpeed(-speed, speed);
}
