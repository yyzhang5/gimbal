#include "foc.h"
#include "svpwm.h"
#include "stm32f4xx_hal_tim.h"  // 添加HAL库头文件
#include "biss.h"
#include <math.h>



#define SQRT3_2 0.86602540378f
#define SQRT2_3 0.81649658093f
#define TWO_PI  6.28318530718f
#define ONE_BY_SQRT3 0.57735026919f
#define PI_VAL  3.14159265359f  // 替代 M_PI

/**
  * @brief  FOC初始化
  */
void FOC_Init(FOC_Motor *motor, TIM_TypeDef *TIMx, float Vdc, uint8_t pole_pairs, BISS_Encoder_t *encoder) {
    // 初始化电机参数
    motor->TIM_PWM = TIMx;
    motor->Vdc = Vdc;
    motor->pole_pairs = pole_pairs;

    // === 新增：绑定编码器 ===
    motor->encoder = encoder;
    
    // 默认电机参数（根据实际电机修改）
    motor->Rs = 0.1f;          // 0.1Ω
    motor->Ld = 0.0005f;       // 0.5mH
    motor->Lq = 0.0005f;       // 0.5mH
    motor->flux_linkage = 0.1f; // 0.1Wb
    
    // 初始化PI控制器
    // 电流环PI参数（需要根据实际电机调整）
    motor->pi_id.Kp = 0.5f;
    motor->pi_id.Ki = 10.0f;
    motor->pi_id.integral = 0;
    motor->pi_id.output_max = Vdc * 0.9f;
    motor->pi_id.output_min = -Vdc * 0.9f;
    
    motor->pi_iq.Kp = 0.5f;
    motor->pi_iq.Ki = 10.0f;
    motor->pi_iq.integral = 0;
    motor->pi_iq.output_max = Vdc * 0.9f;
    motor->pi_iq.output_min = -Vdc * 0.9f;
    
    // 速度环PI参数
    motor->pi_speed.Kp = 0.1f;
    motor->pi_speed.Ki = 0.5f;
    motor->pi_speed.integral = 0;
    motor->pi_speed.output_max = 10.0f; // 最大电流限制
    motor->pi_speed.output_min = -10.0f;
    
    // 初始化SVPWM
    float pwm_freq = 168000000.0f / ((167 + 1) * (99 + 1) * 2); // 中心对齐频率
    float pwm_period = 1.0f / pwm_freq;
    SVPWM_Init(&motor->svpwm, Vdc, pwm_period);
}

/**
  * @brief  Clarke变换 (3相->2相)
  */
void FOC_ClarkeTransform(FOC_Motor *motor) {
    // Clarke变换
    motor->Ialpha = motor->Ia;
    motor->Ibeta = (motor->Ia + 2.0f * motor->Ib) * ONE_BY_SQRT3;
}

/**
  * @brief  Park变换 (静止坐标系->旋转坐标系)
  */
void FOC_ParkTransform(FOC_Motor *motor) {
    float sin_theta = sinf(motor->theta_e);
    float cos_theta = cosf(motor->theta_e);
    
    // Park变换
    motor->Id = motor->Ialpha * cos_theta + motor->Ibeta * sin_theta;
    motor->Iq = -motor->Ialpha * sin_theta + motor->Ibeta * cos_theta;
}

/**
  * @brief  逆Park变换 (旋转坐标系->静止坐标系)
  */
void FOC_InverseParkTransform(FOC_Motor *motor) {
    float sin_theta = sinf(motor->theta_e);
    float cos_theta = cosf(motor->theta_e);
    
    // 逆Park变换
    motor->Valpha = motor->Vd * cos_theta - motor->Vq * sin_theta;
    motor->Vbeta = motor->Vd * sin_theta + motor->Vq * cos_theta;
}

/**
  * @brief  PI控制器更新
  */
void FOC_PI_Update(PI_Controller *pi, float error, float Ts) {
    // 积分项
    pi->integral += error * pi->Ki * Ts;
    
    // 抗积分饱和
    if (pi->integral > pi->output_max) {
        pi->integral = pi->output_max;
    } else if (pi->integral < pi->output_min) {
        pi->integral = pi->output_min;
    }
    
    // 计算输出
    pi->output = error * pi->Kp + pi->integral;
    
    // 输出限幅
    if (pi->output > pi->output_max) {
        pi->output = pi->output_max;
    } else if (pi->output < pi->output_min) {
        pi->output = pi->output_min;
    }
    
    pi->error_prev = error;
}

/**
  * @brief  电流环控制
  */
void FOC_CurrentControl(FOC_Motor *motor, float Ts) {
    // d轴电流PI控制
    float id_error = motor->Id_ref - motor->Id;
    FOC_PI_Update(&motor->pi_id, id_error, Ts);
    motor->Vd = motor->pi_id.output;
    
    // q轴电流PI控制
    float iq_error = motor->Iq_ref - motor->Iq;
    FOC_PI_Update(&motor->pi_iq, iq_error, Ts);
    motor->Vq = motor->pi_iq.output;
    
    // 前馈补偿（可选）
    // motor->Vd += -motor->speed_e * motor->Lq * motor->Iq;
    // motor->Vq += motor->speed_e * (motor->Ld * motor->Id + motor->flux_linkage);
}

/**
  * @brief  速度环控制
  */
void FOC_SpeedControl(FOC_Motor *motor, float Ts) {
    // 速度PI控制
    float speed_error = motor->speed_ref - motor->speed_m;
    FOC_PI_Update(&motor->pi_speed, speed_error, Ts);
    
    // 速度环输出作为q轴电流参考
    motor->Iq_ref = motor->pi_speed.output;
    
    // d轴电流参考通常设为0（最大转矩控制）
    motor->Id_ref = 0;
}

/**
  * @brief  获取电角度
  */
float FOC_GetElectricalAngle(FOC_Motor *motor) {
    float mechanical_angle_deg;
    float electrical_angle_rad;
    
    // === 调用 BISS 编码器读取机械角度 ===
    if (motor->encoder != NULL) {
        mechanical_angle_deg = BISS_ReadAngleDeg(motor->encoder);
    } else {
        // 无编码器时使用开环角度
        return motor->theta_e;
    }
    
    // === 机械角度 → 电角度转换 ===
    // 1. 度转弧度：angle_rad = angle_deg × π / 180
    // 2. 机械角度转电角度：electrical = mechanical × pole_pairs
    electrical_angle_rad = mechanical_angle_deg * PI_VAL / 180.0f * motor->pole_pairs;
    
    // === 限制在 0~2π 范围内 ===
    while (electrical_angle_rad > TWO_PI) {
        electrical_angle_rad -= TWO_PI;
    }
    while (electrical_angle_rad < 0) {
        electrical_angle_rad += TWO_PI;
    }
    
    return electrical_angle_rad;
}

/**
  * @brief  FOC主更新函数
  */
void FOC_Update(FOC_Motor *motor, float Ts) {
    // 1. 读取电流（需要 ADC 采样）
    // motor->Ia = ADC_GetCurrentA();
    // motor->Ib = ADC_GetCurrentB();
    
    // 2. === 获取电角度（调用编码器）===
    motor->theta_e = FOC_GetElectricalAngle(motor);
    
    // 3. 计算电角速度
    motor->speed_e = (motor->theta_e - motor->theta_e_prev) / Ts;
    motor->theta_e_prev = motor->theta_e;
    
    // 4. 计算机械速度（用于速度环）
    motor->speed_m = motor->speed_e / motor->pole_pairs * 60.0f / TWO_PI;
    
    // 5. Clarke 变换
    FOC_ClarkeTransform(motor);
    
    // 6. Park 变换
    FOC_ParkTransform(motor);
    
    // 7. 速度环控制
    FOC_SpeedControl(motor, Ts);
    
    // 8. 电流环控制
    FOC_CurrentControl(motor, Ts);
    
    // 9. 逆 Park 变换
    FOC_InverseParkTransform(motor);
    
    // 10. SVPWM 输出
    motor->svpwm.Ualpha = motor->Valpha;
    motor->svpwm.Ubeta = motor->Vbeta;
    SVPWM_Calc(&motor->svpwm);
    SVPWM_SetDutyCycle(&motor->svpwm, motor->TIM_PWM);
}

/**
  * @brief  设置占空比
  */
void FOC_SetDutyCycle(FOC_Motor* motor, TIM_TypeDef* timer) {
    // FOC 控制完成后，将电压矢量转换为 SVPWM 输入
    SVPWM_TypeDef svpwm;
    SVPWM_Init(&svpwm, motor->Vdc, 0.01);
    svpwm.Ualpha = motor->Valpha;
    svpwm.Ubeta = motor->Vbeta;
    
    
    SVPWM_Calc(&svpwm);
    
    // 调用 SVPWM 模块设置 PWM
    SVPWM_SetDutyCycle(&svpwm, timer);
}

