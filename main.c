#include "ti_msp_dl_config.h"
#include "delay.h"
#include "OLED.h"
#include "Easy_Menu.h"
#include "Drive/Knob/Knob_drv.h"
#include "Drive/Motor/Motor.h"
#include "APP/Trackline.h"
#include "APP/JY62.h"
#include "Beep.h"
#include <stdint.h>

extern struct {
    unsigned char Cal_white_flag;
    unsigned char Cal_black_flag;
    float speed_pid__kp;
    float speed_pid__ki;
    unsigned char trackline__start_flag;
    unsigned char trackline__round;
    float jy62_yaw_data;
    unsigned char jy62_yaw_acc_data;
} Easy_Menu_Ui_Data;

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    Knob_Init();
    Easy_Menu_Init(Display_Char, Display_Char_Line, NULL, NULL);
    Motor_Init();
    JY62_Init();
    Trackline_Init();

    while (1) {
        Knob_get();
        JY62_Task();
        Easy_Menu_Display(g_SystemTick);
        if (Easy_Menu_Ui_Data.trackline__start_flag){
            Trackline_Task();
            //Trackline_Sensor_Test();
        }            
    }
}
