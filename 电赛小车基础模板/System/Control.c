#include "Control.h"
#include "math.h"
#include "stdlib.h"
#include "Motor.h"
#include "ENCODER.h"
#include "OLED.h"
#include "Delay.h"
#include "Buzzer.h"
#include "LED.h"

// 从main.c移入的全局变量
extern int left_current_pulses;
extern int right_current_pulses;
// 参数宏定义
#define WHEEL_DIAMETER_MM   96.0f
#define WHEEL_BASE_MM       260.0f
#define ENCODER_PPR         1040
#define PI                  3.1415926535f

// 使用编码器实现任意角度转弯（带完整PID控制）//已测试
void turn_degrees(float angle, int max_speed) {

    float center_arc_length = (fabs(angle) / 360.0f) * PI * WHEEL_BASE_MM;
    float turns = center_arc_length / (PI * WHEEL_DIAMETER_MM);
    int required_pulses = (int)(turns * ENCODER_PPR);

    float turn_kp = 1.8f;
    float turn_ki = 0.005f;
    float turn_kd = 2.0f;
    int min_speed = 150;
    float integral = 0;
    float last_error = 0;
    int direction = (angle > 0) ? 1 : -1;

    Clear_Encoder_Count();

    while (1) {
        left_current_pulses = abs(Read_Left_Encoder());
        right_current_pulses = abs(Read_Right_Encoder());
        int average_pulses = (left_current_pulses + right_current_pulses) / 2;
        int error = required_pulses - average_pulses;
        integral += error;
        float derivative = error - last_error;

        if (abs(error) <= 20 && abs(derivative) <= 5) {
            MotorA_SetSpeed(0);
            MotorB_SetSpeed(0);
            break;
        }

        int pid_output = (int)(turn_kp * error + turn_ki * integral + turn_kd * derivative);
        
        if (pid_output > max_speed) {
            pid_output = max_speed;
            integral -= error; 
        }
        if (pid_output < -max_speed) {
            pid_output = -max_speed;
            integral -= error;
        }
        
        if (pid_output > 0 && pid_output < min_speed) {
            pid_output = min_speed;
        } else if (pid_output < 0 && pid_output > -min_speed) {
            pid_output = -min_speed;
        }

        last_error = error;
        MotorA_SetSpeed(direction * pid_output);
        MotorB_SetSpeed(-direction * pid_output);

        OLED_ShowString(2, 1, "Err: ");
        OLED_ShowSignedNum(2, 5, error, 5);
        OLED_ShowString(3, 1, "PID: ");
        OLED_ShowSignedNum(3, 5, pid_output, 5);
        OLED_ShowString(4, 1, "Ang: ");
        OLED_ShowSignedNum(4, 5, (int)angle, 5);
        Delay_ms(10);
    }
    MotorA_SetSpeed(0);
    MotorB_SetSpeed(0);
}

// 使用编码器和PID控制直线行驶指定距离    //已测试(有微小偏差)
void move_straight(float distance_cm, int max_speed)
{
    float distance_mm = distance_cm * 10.0f;
    float turns = distance_mm / (PI * WHEEL_DIAMETER_MM);
    int required_pulses = (int)(turns * ENCODER_PPR);

    // 直线保持PID
    float straight_kp = 0.2f;
    float straight_ki = 0.005f;
    float straight_kd = 0.04f;
    int min_speed = 100; // 新增最小速度
    float integral = 0;
    float last_error = 0;

    Clear_Encoder_Count();

    while(1)
    {
        left_current_pulses = Read_Left_Encoder();
        right_current_pulses = Read_Right_Encoder();
        
        int average_pulses = (left_current_pulses + right_current_pulses) / 2;

        // 1. 判断是否到达目标距离
        if (average_pulses >= required_pulses) {
            MotorA_SetSpeed(0);
            MotorB_SetSpeed(0);
            break;
        }

        // 2. 计算直线保持PID
        int error = right_current_pulses - left_current_pulses;
        integral += error;
        // 积分限幅
        if (integral > 5000) integral = 5000;
        if (integral < -5000) integral = -5000;
        float derivative = error - last_error;
        last_error = error;

        int adjustment = (int)(straight_kp * error + straight_ki * integral + straight_kd * derivative);

        // 3. 计算左右轮速度
        int left_speed =   (max_speed+min_speed)/2 - adjustment;
        int right_speed = (max_speed+min_speed)/2+ adjustment;

        // 4. 速度限制
        // 速度限制
        if (left_speed > max_speed) left_speed = max_speed;
        if (left_speed < min_speed) left_speed = min_speed; // 确保不低于最小速度
        if (right_speed > max_speed) right_speed = max_speed;
        if (right_speed < min_speed) right_speed = min_speed; // 确保不低于最小速度

        MotorA_SetSpeed(left_speed);
        MotorB_SetSpeed(right_speed);

        // OLED显示调试信息
        OLED_ShowString(2, 1, "Goal:");
        OLED_ShowSignedNum(2, 6, required_pulses, 5);
        OLED_ShowString(3, 1, "Now :");
        OLED_ShowSignedNum(3, 6, average_pulses, 5);
        OLED_ShowString(4, 1, "L/R:");
        OLED_ShowSignedNum(4, 5, left_speed, 4);
        OLED_ShowSignedNum(4, 10, right_speed, 4);

        Delay_ms(10);
    }
    MotorA_SetSpeed(0);
    MotorB_SetSpeed(0);
}

PID FindLinePID; // 视觉PID变量     //已测试
/**
  * @brief  初始化PID结构体
  * @param  Kp, Ki, Kd: PID参数
  * @retval 无
  */
void PID_Init(float Kp, float Ki, float Kd)// 视觉PID          //已测试
{
    FindLinePID.Kp = Kp;
    FindLinePID.Ki = Ki;
    FindLinePID.Kd = Kd;
    FindLinePID.Error = 0;
    FindLinePID.Last_Error = 0;
    FindLinePID.Integral = 0;
    FindLinePID.Output = 0;
}

/**
  * @brief  位置式PID控制器，用于循线
  * @param  target_error: 目标误差，即OpenMV发来的rho_err
  * @retval 计算出的PID输出值（用于调整电机速度）
  */
float Position_PID_FindLine(float target_error)
{
    FindLinePID.Error = target_error;
    
    // 积分项
    FindLinePID.Integral += FindLinePID.Error;
    
    // 积分限幅，防止积分饱和
    if (FindLinePID.Integral > 3000) FindLinePID.Integral = 3000;
    if (FindLinePID.Integral < -3000) FindLinePID.Integral = -3000;
    
    // PID计算
    FindLinePID.Output = FindLinePID.Kp * FindLinePID.Error + 
                         FindLinePID.Ki * FindLinePID.Integral + 
                         FindLinePID.Kd * (FindLinePID.Error - FindLinePID.Last_Error);
                  
    FindLinePID.Last_Error = FindLinePID.Error;
    
    return FindLinePID.Output;
}
