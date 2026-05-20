#include "Laser.h"
#include "ti_msp_dl_config.h"

void Laser_Init(void)
{
#if LASER_ACTIVE_LOW
    DL_GPIO_setPins(GPIO_Laser_PORT, GPIO_Laser_ctrl_PIN);
#else
    DL_GPIO_clearPins(GPIO_Laser_PORT, GPIO_Laser_ctrl_PIN);
#endif
    DL_GPIO_enableOutput(GPIO_Laser_PORT, GPIO_Laser_ctrl_PIN);
}

void Laser_On(void)
{
#if LASER_ACTIVE_LOW
    DL_GPIO_clearPins(GPIO_Laser_PORT, GPIO_Laser_ctrl_PIN);
#else
    DL_GPIO_setPins(GPIO_Laser_PORT, GPIO_Laser_ctrl_PIN);
#endif
}

void Laser_Off(void)
{
#if LASER_ACTIVE_LOW
    DL_GPIO_setPins(GPIO_Laser_PORT, GPIO_Laser_ctrl_PIN);
#else
    DL_GPIO_clearPins(GPIO_Laser_PORT, GPIO_Laser_ctrl_PIN);
#endif
}
