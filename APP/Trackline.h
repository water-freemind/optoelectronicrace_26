#ifndef TRACKLINE_H
#define TRACKLINE_H

#include <stdint.h>
#include "Motor.h"

typedef struct {
    float Kp;
    float Ki;
    float Kd;
} PID_Params_t;

typedef struct {
    int16_t base_speed;
    int16_t max_correction;
    PID_Params_t pid;
    int16_t last_error;
    int32_t integral;
} Trackline_Controller_t;

extern Trackline_Controller_t g_Trackline;

void Trackline_Init(void);
void Trackline_Task(void);
void Trackline_Sensor_Test(void);

#endif
