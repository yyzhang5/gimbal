#include "foc.h"
#include "svpwm.h"
#include "stm32f4xx.h"
#include "control.h"
#include <math.h>

#define M_PI 3.14159265358979323846
#define TWO_PI  6.28318530718f

// 定义两个电机
FOC_Motor motor1, motor2;
void SVPWM_OpenLoop_Example(void)
{
    SVPWM_TypeDef svpwm1, svpwm2;

    float pwm_freq = 168000000.0f / ((167 + 1) * (99 + 1) * 2);
    float Ts = 1.0f / pwm_freq;

    SVPWM_Init(&svpwm1, 24.0f, Ts);
    SVPWM_Init(&svpwm2, 24.0f, Ts);

    float angle = 0.0f;
    float speed = 2 * M_PI * 0.1f;   // 0.1 Hz
    float amplitude = 6.0f;

    while (1)
    {
        svpwm1.Ualpha = amplitude * cosf(angle);
        svpwm1.Ubeta  = amplitude * sinf(angle);

        svpwm2.Ualpha = amplitude * cosf(angle + M_PI);
        svpwm2.Ubeta  = amplitude * sinf(angle + M_PI);

        SVPWM_Calc(&svpwm1);
        SVPWM_Calc(&svpwm2);

        SVPWM_SetDutyCycle(&svpwm1, TIM1);
        SVPWM_SetDutyCycle(&svpwm2, TIM8);

        angle += speed * Ts;
        if (angle > 2 * M_PI) angle -= 2 * M_PI;
    }    
}


// FOC闭环控制示例
void FOC_ClosedLoop_Example(void) {
    // 初始化电机1（TIM1，24V，21极对）
    FOC_Init(&motor1, TIM1, 24.0f, 21);
    
    // 初始化电机2（TIM8，24V，21极对）
    FOC_Init(&motor2, TIM8, 24.0f, 21);
    
    // 设置控制目标                                                                                         
    motor1.speed_ref = 100.0f; // 1000 RPM
    motor2.speed_ref = 100.0f; // 1500 RPM
    
    float Ts = 0.0001f; // 控制周期100us
    
    while (1) {
        // 这里需要添加电流采样代码
        // motor1.Ia = Read_ADC_Motor1_PhaseA();
        // motor1.Ib = Read_ADC_Motor1_PhaseB();
        // motor2.Ia = Read_ADC_Motor2_PhaseA();
        // motor2.Ib = Read_ADC_Motor2_PhaseB();
        
        // 这里需要添加角度获取代码（编码器或霍尔）
        // motor1.theta_e = Get_Encoder_Angle_Motor1();
        // motor2.theta_e = Get_Encoder_Angle_Motor2();
        
        // 更新电机1 FOC控制
        FOC_Update(&motor1, Ts);
        
        // 更新电机2 FOC控制
        FOC_Update(&motor2, Ts);
        
        // 延时或等待定时器中断
        HAL_Delay(1);
    }
}








