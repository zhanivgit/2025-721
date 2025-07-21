#include "stm32f10x.h"                  // Device header
#include "Delay.h"



#include "OLED.h"  		//OLED
#include "LED.h"		//LED
#include "Key.h"       	//按键
#include "Buzzer.h"     //蜂鸣器
#include "Motor.h"     	//电机
#include "ENCODER.h"   	//编码器
#include "Serial.h"    	//串口
#include "Control.h"   //PID



// 参数宏定义
#define WHEEL_DIAMETER_MM   96.0f
#define WHEEL_BASE_MM       260.0f
#define ENCODER_PPR         1040 
#define PI                  3.1415926535f
// 全局变量，供Control.c使用
int left_current_pulses;
int right_current_pulses;
float rho_err;
int Base_Speed = 200; // 基础速度，可调


int main(void)
{
	OLED_Init();
	LED_Init();
	Key_Init();
	Buzzer_Init();
	Motor_Init();
	Encoder_Init();
	Serial_Init();
	Delay_ms(1000);
	// turn_degrees(90, 300); //转向测试
	// Delay_ms(1000);
	// turn_degrees(-90, 300); //转向测试
	// PID_Init(3, 0.0, 0.1);//视觉pid初始化
	// move_straight(100, 300); // 编码器前进测试
	while (1)
	{
		// int left_encoder = Read_Left_Encoder();    //编码器读取测试
        // int right_encoder = Read_Right_Encoder();
        // OLED_ShowString(2, 1, "L:");
        // OLED_ShowSignedNum(2, 3, left_encoder, 5); // 在第2行显示左编码器值
        
        // OLED_ShowString(3, 1, "R:");
        // OLED_ShowSignedNum(3, 3, right_encoder, 5); // 在第3行显示右编码器值
        
	}
}
