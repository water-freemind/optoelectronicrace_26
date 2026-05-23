#include "ti_msp_dl_config.h"
#include "delay.h"
#include "OLED.h"
#include "Easy_Menu.h"
#include "Drive/Knob/Knob_drv.h"
#include "Drive/Motor/Motor.h"
#include "APP/Trackline.h"
#include "APP/JY62.h"
#include "APP/Laser.h"
#include "APP/k230_track.h"
#include "APP/gimbal.h"
#include "Beep.h"
#include <stdint.h>
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"

extern No_MCU_Sensor sensor;

extern struct {
    unsigned char Cal_white_flag;
    unsigned char Cal_black_flag;
    unsigned char trackline__start_flag;
    unsigned char trackline__round;
    unsigned char still_aim;
    unsigned char still_aim_draw_Rectangle__data;
    unsigned char move_aim_center_round;
    unsigned char centeraim__start__data;
    float speed_pid__kp;
    float speed_pid__ki;
    float jy62_yaw_data;
    float jy62_yaw_acc_data;
    signed short int Lspeed;
    signed short int Rspeed;
} Easy_Menu_Ui_Data;

int main(void)
{
    static uint8_t last_start = 0;
    static uint8_t prev_aim = 0;
    static uint8_t aim_off_cnt = 0;
    static uint8_t prev_dr = 0;
    static uint8_t dr_off_cnt = 0;

    SYSCFG_DL_init();
    OLED_Init();
    Knob_Init();
    Easy_Menu_Init(Display_Char, Display_Char_Line, NULL, NULL);
    Motor_Init();
    Trackline_Init();
    Laser_Init();
    K230_Track_Init();
    JY62_Init();
    delay_ms(50);

    Easy_Menu_Ui_Data.speed_pid__kp = g_Trackline.pid.Kp;
    Easy_Menu_Ui_Data.speed_pid__ki = g_Trackline.pid.Kd;
    while (1) {
        Knob_get();
        JY62_Task();
        Easy_Menu_Display(g_SystemTick);
        No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
        if (Easy_Menu_Ui_Data.trackline__start_flag){
            if (!last_start) { Trackline_Reset(); }
            last_start = 1;
            Trackline_Task();
        } else {
            last_start = 0;
        }
        if (Easy_Menu_Ui_Data.still_aim_draw_Rectangle__data) {
            prev_dr = 1;
            dr_off_cnt = 0;
            K230_DrawRect_Task();
        } else if (prev_dr) {
            if (++dr_off_cnt >= 10) {
                prev_dr = 0;
                dr_off_cnt = 0;
                K230_DrawRect_Reset();
            }
        }
        if (Easy_Menu_Ui_Data.still_aim) {
            prev_aim = 1;
            aim_off_cnt = 0;
            K230_Aim_Task();
        } else if (prev_aim) {
            if (++aim_off_cnt >= 10) {
                prev_aim = 0;
                aim_off_cnt = 0;
                Gimbal_Stop(GIMBAL_ADDR_X);
                Gimbal_Stop(GIMBAL_ADDR_Y);
                Laser_Off();
                K230_Aim_Reset();
            }
        }
    }
}
