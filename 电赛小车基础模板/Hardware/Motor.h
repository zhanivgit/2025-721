#include "stm32f10x.h" 
#ifndef __MOTOR_H
#define __MOTOR_H

#define MOTOR_AIN1_PIN    GPIO_Pin_13
#define MOTOR_AIN2_PIN    GPIO_Pin_12
#define MOTOR_PWMA_PIN    GPIO_Pin_1
#define MOTOR_BIN1_PIN    GPIO_Pin_14
#define MOTOR_BIN2_PIN    GPIO_Pin_15
#define MOTOR_PWMB_PIN    GPIO_Pin_0
#define MOTOR_GPIO_PORT   GPIOB

void Motor_Init(void);
void MotorA_SetSpeed(int16_t speed);
void MotorB_SetSpeed(int16_t speed);
void Motor_TurnInPlace(uint16_t speed, uint8_t dir);
void Motor_Stop(void);
void Move(int16_t speed);
#endif
