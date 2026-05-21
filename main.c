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
#include "Beep.h"
#include <stdint.h>

extern struct {
    unsigned char Cal_white_flag;
    unsigned char Cal_black_flag;
    unsigned char trackline__start_flag;
    unsigned char trackline__round;
    unsigned char still_aim;
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

    SYSCFG_DL_init();
    JY62_Init();
    OLED_Init();
    Knob_Init();
    Easy_Menu_Init(Display_Char, Display_Char_Line, NULL, NULL);
    Motor_Init();
    Trackline_Init();
    Laser_Init();
    K230_Track_Init();

    Easy_Menu_Ui_Data.speed_pid__kp = g_Trackline.pid.Kp;
    Easy_Menu_Ui_Data.speed_pid__ki = g_Trackline.pid.Kd;

    while (1) {
        Knob_get();
        JY62_Task();
        Easy_Menu_Display(g_SystemTick);
        if (Easy_Menu_Ui_Data.trackline__start_flag){
            if (!last_start) { Trackline_Reset(); }
            last_start = 1;
            Trackline_Task();
        } else {
            last_start = 0;
        }
        if (Easy_Menu_Ui_Data.still_aim) {
            K230_Aim_Task();
        }
    }
}
