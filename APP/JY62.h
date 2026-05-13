#ifndef APP_JY62_H_
#define APP_JY62_H_

#include <stdint.h>
#include <stdbool.h>

extern volatile float roll_angle;
extern volatile float pitch_angle;
extern volatile float yaw_angle;
extern volatile float ax, ay, az;
extern volatile float wx, wy, wz;
extern volatile float temperature;
extern volatile uint8_t jy62_new_data;

void JY62_Init(void);
void JY62_ConfigMode(void);
void JY62_ProcessData(uint8_t data);
void JY62_Task(void);

#endif