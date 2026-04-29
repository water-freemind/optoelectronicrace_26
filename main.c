#include "Drive/Knob/Knob_drv.h"
#include "Drive/Motor/Motor.h"
#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "ti_msp_dl_config.h"
#include "delay.h"
#include "Uart.h"
#include "OLED.h"
#include "Easy_Menu.h"
#include "Knob_drv.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stdio.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "Motor.h"


unsigned short Anolog[8]={0};
unsigned short white[8]={1800,1800,1800,1800,1800,1800,1800,1800};
unsigned short black[8]={300,300,300,300,300,300,300,300};
unsigned short Normal[8];
unsigned char rx_buff[256]={0};

No_MCU_Sensor sensor;
unsigned char Digtal;

int main(void)
{
    SYSCFG_DL_init();
    OLED_Init();
    Knob_Init();
    Easy_Menu_Init(Display_Char, Display_Char_Line, NULL, NULL);
    Motor_Init();

    //根据黑白校准值初始化传感器
	No_MCU_Ganv_Sensor_Init(&sensor,white,black);

    //设置DMA搬运的起始地址
    DL_DMA_setSrcAddr(DMA, DMA_detector_out_CHAN_ID, (uint32_t) &ADC0->ULLMEM.MEMRES[0]);
    //设置DMA搬运的目的地址
    DL_DMA_setDestAddr(DMA, DMA_detector_out_CHAN_ID, (uint32_t) &ADC_VALUE[0]);
    //开启DMA
    DL_DMA_enableChannel(DMA, DMA_detector_out_CHAN_ID);
    //开启ADC转换
    DL_ADC12_startConversion(ADC_line_detector_INST);
    int32_t a = 0;	
    int32_t b = 0;	
    while (1) {
        // Knob_get();
        // Easy_Menu_Display(g_SystemTick);
        // No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
        // //获取传感器数字量结果(只有当有黑白值传入进去了之后才会有这个值！！)
        // Digtal=Get_Digtal_For_User(&sensor);
        // sprintf((char *)rx_buff,"Digtal %d-%d-%d-%d-%d-%d-%d-%d\r\n",(Digtal>>0)&0x01,(Digtal>>1)&0x01,(Digtal>>2)&0x01,(Digtal>>3)&0x01,(Digtal>>4)&0x01,(Digtal>>5)&0x01,(Digtal>>6)&0x01,(Digtal>>7)&0x01);
        // uart0_send_string((char *)rx_buff);
        // memset(rx_buff,0,256);
        a = Motor_A_GetEncoderCnt();
        b = Motor_B_GetEncoderCnt();
        OLED_ShowNum(0, 0, a, 5, 8);
        OLED_ShowNum(0, 1, b, 5, 8);
        // //获取传感器模拟量结果(有黑白值初始化后返回1 没有返回 0)
        // if(Get_Anolog_Value(&sensor,Anolog)){
        // sprintf((char *)rx_buff,"Anolog %d-%d-%d-%d-%d-%d-%d-%d\r\n",Anolog[0],Anolog[1],Anolog[2],Anolog[3],Anolog[4],Anolog[5],Anolog[6],Anolog[7]);
        // uart0_send_string((char *)rx_buff);
        // memset(rx_buff,0,256);
        // }
        
        // //获取传感器归一化结果(只有当有黑白值传入进去了之后才会有这个值！！有黑白值初始化后返回1 没有返回 0)
        // if(Get_Normalize_For_User(&sensor,Normal)){ 
        // sprintf((char *)rx_buff,"Normalize %d-%d-%d-%d-%d-%d-%d-%d\r\n",Normal[0],Normal[1],Normal[2],Normal[3],Normal[4],Normal[5],Normal[6],Normal[7]);
        // uart0_send_string((char *)rx_buff);
        // memset(rx_buff,0,256);
        // }
        // delay_+ms(1);
    }
}
