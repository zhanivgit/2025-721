#include "Buzzer.h"
#include "Delay.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

// 定义蜂鸣器连接的GPIO端口和引脚
#define BUZZER_GPIO_PORT GPIOA
#define BUZZER_GPIO_PIN  GPIO_Pin_11
#define BUZZER_GPIO_CLK  RCC_APB2Periph_GPIOA

void Buzzer_Init(void)
{
    RCC_APB2PeriphClockCmd(BUZZER_GPIO_CLK, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = BUZZER_GPIO_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BUZZER_GPIO_PORT, &GPIO_InitStructure);
    
    // 初始状态为关闭
    GPIO_ResetBits(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN);
}

void Buzzer_On(void)
{
    GPIO_SetBits(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN);
}

void Buzzer_Off(void)
{
    GPIO_ResetBits(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN);
}

// 蜂鸣器鸣叫指定的毫秒数
void Buzzer_Beep(uint32_t duration_ms)
{
    Buzzer_On();
    Delay_ms(duration_ms);
    Buzzer_Off();
}
