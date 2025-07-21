#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h" // 假设使用STM32F1系列微控制器
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"

// 定义编码器相关的GPIO和定时器，这里只是示例，需要根据实际硬件连接修改
// 例如，使用TIM2和TIM3作为编码器接口

// 编码器初始化函数
void Encoder_Init(void);

// 读取左编码器脉冲数
int Read_Left_Encoder(void);

// 读取右编码器脉冲数
int Read_Right_Encoder(void);

// 清零编码器计数
void Clear_Encoder_Count(void);

#endif /* __ENCODER_H */
