#ifndef __CONTROL_H
#define __CONTROL_H
 
#include "stm32f10x.h"
 

 // PID结构体定义
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    
    float Error;
    float Last_Error;
    float Integral;
    
    float Output;
} PID;

extern PID FindLinePID; // 声明外部全局变量

void PID_Init(float Kp, float Ki, float Kd);
float Position_PID_FindLine(float target_error);

// 函数声明
// 根据给定的角度和最大速度旋转车辆
void turn_degrees(float angle, int max_speed);
// 根据给定的距离和最大速度直线行驶
void move_straight(float distance_cm, int max_speed);

#endif
