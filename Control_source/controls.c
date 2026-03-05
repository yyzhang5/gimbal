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
        if (angle > TWO_PI) angle -= TWO_PI; // 优化：使用定义常量
    }    
}

// FOC 闭环控制示例
void FOC_ClosedLoop_Example(void) {
    // 初始化电机 1（TIM1，24V，21 极对，绑定编码器 1）
    FOC_Init(&motor1, TIM1, 24.0f, 21, &encoder_motor1);
    
    // 初始化电机 2（TIM8，24V，21 极对，绑定编码器 2）
    FOC_Init(&motor2, TIM8, 24.0f, 21, &encoder_motor2);
    
    // 设置控制目标 (统一单位为 RPM)
    motor1.speed_ref = 100.0f; // 修正：100 RPM
    motor2.speed_ref = 100.0f; // 修正：与电机 1 保持一致或按需设定
    
    float Ts = 0.0001f; // 控制周期 100us
    
    while (1) {
        // --- 电机 1 控制流程 ---
        FOC_Update(&motor1, Ts);            // 1. 更新采样数据
        FOC_ClarkeTransform(&motor1);       // 2.  Clarke 变换 (Ia,Ib -> Ialpha,Ibeta)
        FOC_ParkTransform(&motor1);         // 3.  Park 变换 (Ialpha,Ibeta -> Id,Iq)
        FOC_SpeedControl(&motor1, Ts);      // 4. 速度环控制 (生成 Iq_ref)
        FOC_CurrentControl(&motor1, Ts);    // 5. 电流环控制 (生成 Vd,Vq)
        FOC_InverseParkTransform(&motor1);  // 6. 逆 Park 变换 (Vd,Vq -> Valpha,Vbeta)
        // FOC_ClarkeTransform 不需要再次调用
        FOC_SetDutyCycle(&motor1, TIM1);   // 7. 设置 PWM
        
        
        // --- 电机 2 控制流程 (补充缺失逻辑) ---
        FOC_Update(&motor2, Ts);
        FOC_ClarkeTransform(&motor2);
        FOC_ParkTransform(&motor2);
        FOC_SpeedControl(&motor2, Ts);
        FOC_CurrentControl(&motor2, Ts);
        FOC_InverseParkTransform(&motor2);
        FOC_SetDutyCycle(&motor2, TIM8);
    }
}