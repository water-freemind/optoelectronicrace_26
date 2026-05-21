#ifndef LASER_H
#define LASER_H

/* 触发电平: 1 = 低电平亮, 0 = 高电平亮 */
#define LASER_ACTIVE_LOW  0

void Laser_Init(void);
void Laser_On(void);
void Laser_Off(void);

#endif
