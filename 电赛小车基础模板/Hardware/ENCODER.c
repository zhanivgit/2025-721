#include "ENCODER.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"

// 假设使用TIM2和TIM1作为编码器接口
// TIM2用于左轮编码器，TIM1用于右轮编码器

void Encoder_Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    // 1. 开启GPIO和定时器时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // 启用TIM2时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE); // 启用TIM1、GPIOA和AFIO时钟

    // 2. 配置GPIO引脚为上拉输入
    // TIM2_CH1 (PA.00), TIM2_CH2 (PA.01) - 左轮编码器
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 上拉输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // TIM1_CH1 (PA.08), TIM1_CH2 (PA.09) - 右轮编码器
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 上拉输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. 配置定时器为编码器模式
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 1 - 1; // 预分频器设置为0
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseStructure.TIM_Period = 65536 - 1; // 计数周期，最大值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    // 配置输入捕获
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 0xF; // 输入捕获滤波器，设置为0xF
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; // 不分频
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; // 直接输入

    // TIM2 编码器模式1 - 左轮
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; // 上升沿捕获
    TIM_ICInit(TIM2, &TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIM2, &TIM_ICInitStructure);
    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    // TIM1 编码器模式1 - 右轮
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInit(TIM1, &TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIM1, &TIM_ICInitStructure);
    TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    // 4. 使能定时器
    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

// 读取左编码器脉冲数 (TIM2)
int Read_Left_Encoder(void) {

    return (short)TIM_GetCounter(TIM2);
}

// 读取右编码器脉冲数 (TIM1)
int Read_Right_Encoder(void) {
    return (short)TIM_GetCounter(TIM1);
}

// 清零编码器计数
void Clear_Encoder_Count(void) {
    TIM_SetCounter(TIM2, 0);
    TIM_SetCounter(TIM1, 0);
}
