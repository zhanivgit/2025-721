#include "stm32f10x.h"                  // Device header
#include "Motor.h"
#include "stm32f10x_gpio.h" // 添加GPIO头文件
#include "stm32f10x_tim.h"  // 添加TIM头文件
#include "stm32f10x_rcc.h"  // 添加RCC头文件
/**
  * 函    数：直流电机初始化
  * 参    数：无
  * 返 回 值：无
 **/
void Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    /* 1. 开启相关时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);    // 开启GPIOB时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);     // 开启TIM3时钟
    
    /* 2. 配置方向控制GPIO - PB12~15 */
    GPIO_InitStructure.GPIO_Pin = MOTOR_AIN1_PIN | MOTOR_AIN2_PIN | 
                                 MOTOR_BIN1_PIN | MOTOR_BIN2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /* 3. 配置PWM输出GPIO - PB0和PB1 */
    GPIO_InitStructure.GPIO_Pin = MOTOR_PWMA_PIN | MOTOR_PWMB_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;          // 复用推挽输出
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /* 4. 配置TIM3 PWM */
    // 时基配置
    TIM_TimeBaseStructure.TIM_Period = 1000-1;               // 自动重装载值
    TIM_TimeBaseStructure.TIM_Prescaler = 72-1;              // 72MHz/72 = 1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    // PWM模式配置
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 0;                       // 初始占空比0%
    
    // 配置通道3(PB0)和通道4(PB1)
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    
    // 使能预装载寄存器
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
    
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
    
    /* 5. 初始状态设置 */
    GPIO_ResetBits(GPIOB, MOTOR_AIN1_PIN | MOTOR_AIN2_PIN |  // 方向引脚初始低电平
                        MOTOR_BIN1_PIN | MOTOR_BIN2_PIN);
}

/**
  * 函    数：直流电机设置速度
  * 参    数：Speed 要设置的速度，范围：-100~100
  * 返 回 值：无
  */
void MotorA_SetSpeed(int16_t speed)
{
    // 设置方向
    if(speed > 0) {
        // 正转：AIN1=1, AIN2=0
        GPIO_SetBits(GPIOB, MOTOR_AIN1_PIN);
        GPIO_ResetBits(GPIOB, MOTOR_AIN2_PIN);
    } 
    else if(speed < 0) {
        // 反转：AIN1=0, AIN2=1
        GPIO_ResetBits(GPIOB, MOTOR_AIN1_PIN);
        GPIO_SetBits(GPIOB, MOTOR_AIN2_PIN);
    }
    else {
        // 刹车：AIN1=1, AIN2=1
        GPIO_SetBits(GPIOB, MOTOR_AIN1_PIN);
        GPIO_SetBits(GPIOB, MOTOR_AIN2_PIN);
    }
    
    // 设置PWM占空比（取绝对值）
    uint16_t pwm = (speed < 0) ? -speed : speed;
    if(pwm > 1000) pwm = 1000;  // 限制最大值
    TIM_SetCompare4(TIM3, pwm);  // 通道4对应PB1（MOTOR_PWMA）
}

// 设置电机B速度（-1000到+1000）
void MotorB_SetSpeed(int16_t speed)
{
    // 设置方向
    if(speed > 0) {
        // 正转：BIN1=1, BIN2=0
        GPIO_SetBits(GPIOB, MOTOR_BIN1_PIN);
        GPIO_ResetBits(GPIOB, MOTOR_BIN2_PIN);
    } 
    else if(speed < 0) {
        // 反转：BIN1=0, BIN2=1
        GPIO_ResetBits(GPIOB, MOTOR_BIN1_PIN);
        GPIO_SetBits(GPIOB, MOTOR_BIN2_PIN);
    }
    else {
        // 刹车：BIN1=1, BIN2=1
        GPIO_SetBits(GPIOB, MOTOR_BIN1_PIN);
        GPIO_SetBits(GPIOB, MOTOR_BIN2_PIN);
    }
    
    // 设置PWM占空比
    uint16_t pwm = (speed < 0) ? -speed : speed;
    if(pwm > 1000) pwm = 1000;
    TIM_SetCompare3(TIM3, pwm);  // 通道3对应PB0（MOTOR_PWMB）
}
/**
  * 函    数：正向行驶
  * 参    数：Speed 要设置的速度，范围：-100~100
  * 返 回 值：无
  */
void Move(int16_t speed)
{
	MotorA_SetSpeed(speed);
	MotorB_SetSpeed(speed);
}


// 差速转向控制函数
// speed: 转向速度绝对值（0-1000）
// dir: 转向方向（0=左转，1=右转）
void Motor_TurnInPlace(uint16_t speed, uint8_t dir)
{
    // 限制速度范围
    if(speed > 1000) speed = 1000;
    
    if(dir == 0) { // 原地左转
        // 左轮后退
        GPIO_ResetBits(GPIOB, MOTOR_AIN1_PIN);
        GPIO_SetBits(GPIOB, MOTOR_AIN2_PIN);
        TIM_SetCompare4(TIM3, speed);  // 左轮PWM
        
        // 右轮前进
        GPIO_SetBits(GPIOB, MOTOR_BIN1_PIN);
        GPIO_ResetBits(GPIOB, MOTOR_BIN2_PIN);
        TIM_SetCompare3(TIM3, speed);  // 右轮PWM
    }
    else {        // 原地右转
        // 左轮前进
        GPIO_SetBits(GPIOB, MOTOR_AIN1_PIN);
        GPIO_ResetBits(GPIOB, MOTOR_AIN2_PIN);
        TIM_SetCompare4(TIM3, speed);  // 左轮PWM
        
        // 右轮后退
        GPIO_ResetBits(GPIOB, MOTOR_BIN1_PIN);
        GPIO_SetBits(GPIOB, MOTOR_BIN2_PIN);
        TIM_SetCompare3(TIM3, speed);  // 右轮PWM
    }
}

// 停止函数
void Motor_Stop(void)
{
    // 刹车停止
    GPIO_SetBits(GPIOB, MOTOR_AIN1_PIN | MOTOR_AIN2_PIN |
                       MOTOR_BIN1_PIN | MOTOR_BIN2_PIN);
    TIM_SetCompare3(TIM3, 0);
    TIM_SetCompare4(TIM3, 0);
}
