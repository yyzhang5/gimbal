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

 // ! FOC闭环控制
 // ! loop 电流、速度、位置环都是单独解耦的的函数
 // todo   FOC控制架构逻辑
// 1. ADC三相IaIbIc 获取电流
// 2. Clark变换 输出Ialpha Ibeta
// 3. Park变换 输出Iq Id（反馈回路）
// 4. 电流环ACR控制  输出
// 5. 速度环Vq
//! 位置速度电流控制等 放到中断中
// float CONTROL_PERIOD_S = 0.0001f 10KHz; .h定义

// 在中断外初始化电机（只调用一次）
void Motor_Init(void) {
    FOC_Init(&motor1, TIM1, 24.0f, 21, &encoder_motor1, PM1_ADC_U_IDX, PM1_ADC_V_IDX, PM1_ADC_W_IDX);

    FOC_Init(&motor2, TIM8, 24.0f, 21, &encoder_motor2, PM2_ADC_U_IDX, PM2_ADC_V_IDX, PM2_ADC_W_IDX);

    // 初始速度参考值由外部命令、按键或串口调整；避免默认0导致不动
    motor1.speed_ref = 50.0f;  // 应该是°？
    motor2.speed_ref = 0.0f;
}


void M1_Control(void) {
        motor1.theta_m = BISS_ReadAngleDeg(&encoder_motor1);           // 编码器数据（机械角度）
        CurrentSensorSuite(&motor1);             // ADC 电流采集
        motor1.theta_e = Pos_ElecTheta(&motor1);          // 电角度获取(弧度)
        // Clarke 变换
        FOC_ClarkeTransform(&motor1);
        // Park 变换
        FOC_ParkTransform(&motor1);
        Loop_Current(&motor1, CONTROL_PERIOD_S);         // 电流环控制
        //  速度环控制 (1kHz)
        static uint16_t speed_cnt_M1 = 0;
        if (++speed_cnt_M1 >= 10) {
            speed_cnt_M1 = 0;
            //! 此处补充角速度计算 及 滤波函数
            motor1.speed_m = M1_Speed_Get(motor1.theta_m,0.001);///57.295779513;   // 机械角速度
            Loop_Speed(&motor1, CONTROL_PERIOD_S * 10);
        }
        // 逆 Park 变换
        FOC_InverseParkTransform(&motor1);
        // SVPWM 输出
        motor1.svpwm.Ualpha = motor1.Valpha;
        motor1.svpwm.Ubeta = motor1.Vbeta;
        SVPWM_Calc(&motor1.svpwm);
        SVPWM_SetDutyCycle(&motor1.svpwm, motor1.TIM_PWM);
    }

    void M2_Control(void) {
        motor2.theta_m = BISS_ReadAngleDeg(&encoder_motor2);           // 编码器数据（机械角度）
        CurrentSensorSuite(&motor2);             // ADC 电流采集
        motor2.theta_e = Pos_ElecTheta(&motor2);          // 电角度获取
        
        // Clarke 变换
        FOC_ClarkeTransform(&motor2);
        // Park 变换
        FOC_ParkTransform(&motor2);
        Loop_Current(&motor2, CONTROL_PERIOD_S);        // 电流环控制
        //  速度环控制
        static uint16_t speed_cnt_M2 = 0;
        if (++speed_cnt_M2 >= 10) {
            speed_cnt_M2 = 0;
            motor2.speed_m = M2_Speed_Get(motor2.theta_m,0.001);///57.295779513;   // 机械角速度
            Loop_Speed(&motor2, CONTROL_PERIOD_S * 10);
        }
        // 逆 Park 变换
        FOC_InverseParkTransform(&motor2);
        // SVPWM 输出
        motor2.svpwm.Ualpha = motor2.Valpha;
        motor2.svpwm.Ubeta = motor2.Vbeta;
        SVPWM_Calc(&motor2.svpwm);
        SVPWM_SetDutyCycle(&motor2.svpwm, motor2.TIM_PWM);
    }