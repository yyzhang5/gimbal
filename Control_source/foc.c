#include "foc.h"
#include "svpwm.h"
#include "stm32f4xx_hal_tim.h"  // 添加HAL库头文件
#include "biss.h"
#include "interrupts.h"  
#include <math.h>



#define SQRT3_2 0.86602540378f
#define SQRT2_3 0.81649658093f
#define TWO_PI  6.28318530718f
#define ONE_BY_SQRT3 0.57735026919f
#define PI_VAL  3.14159265359f  // 替代 M_PI

// 在 foc.c 文件开头，include 之后添加
extern uint16_t ADC_Buff[];  // 引用 app.c 中定义的 ADC 缓冲区
CurrentOffset_t M1_Offset = {0};
CurrentOffset_t M2_Offset = {0};



/**
  * @brief  FOC初始化
  */
void FOC_Init(FOC_Motor *motor, TIM_TypeDef *TIMx, float Vdc, uint8_t pole_pairs, BISS_Encoder_t *encoder, uint8_t adc_idx_u, uint8_t adc_idx_v, uint8_t adc_idx_w) {       
    // 初始化电机参数
    motor->TIM_PWM = TIMx;
    motor->Vdc = Vdc;
    motor->pole_pairs = pole_pairs;

    // === 新增：绑定编码器 ===
    motor->encoder = encoder;

      // === 新增：ADC 通道配置 ===
    motor->adc_idx_u = adc_idx_u;
    motor->adc_idx_v = adc_idx_v;

    // 默认电机参数（根据实际电机修改）
    motor->Rs = 3.0f;          // 线电阻3Ω
    // motor->Ld = 0.0005f;       // 0.5mH  相间电感 31.5mH
    // motor->Lq = 0.0005f;       // 0.5mH
    
    
    // 初始化PI控制器
    // 电流环PI参数（需要根据实际电机调整）
    motor->pi_id.Kp = 0.1f;      // 比例增益 - 降低避免过冲
    motor->pi_id.Ki = 0.01f;     // 积分增益 - 提高响应速度
    motor->pi_id.integral = 0;
    motor->pi_id.output_max = Vdc * 0.6f;  // 限幅降低，避免饱和
    motor->pi_id.output_min = -Vdc * 0.6f;

    motor->pi_iq.Kp = 0.8f;      // 与 id 保持一致
    motor->pi_iq.Ki = 0.0001f;
    motor->pi_iq.integral = 0;
    motor->pi_iq.output_max = Vdc * 0.1f;
    motor->pi_iq.output_min = -Vdc * 0.1f;
    
    // 速度环PI参数
    motor->pi_speed.Kp = 0.6f;   // 降低比例增益
    motor->pi_speed.Ki = 0.00001f;    // 提高积分增益
    motor->pi_speed.integral = 0;
    motor->pi_speed.output_max = 2.0f;   // 最大电流参考值 (A)
    motor->pi_speed.output_min = -2.0f;
    
    // 根据定时器设置不同的 PWM 频率
    float pwm_freq;
    if (TIMx == TIM1) {
        pwm_freq = 168000000.0f / ((167 + 1) * (49 + 1) * 2);  // 10kHz   中心对齐频率
    } else if (TIMx == TIM8) {
        pwm_freq = 168000000.0f / ((167 + 1) * (49 + 1) * 2);  // 10kHz   中心对齐频率
    }
    
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
void Loop_Current(FOC_Motor *motor, float Ts) {
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
void Loop_Speed(FOC_Motor *motor, float Ts) {
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
float Pos_ElecTheta(FOC_Motor *motor)
{
    float mechanical_angle_deg;

    mechanical_angle_deg = motor->theta_m;

    // 3. 有效性验证（处理 720 等异常值）
    if (mechanical_angle_deg < 0.0f || mechanical_angle_deg >= 360.0f) {
        // 角度异常，返回上次值或进行修正
        mechanical_angle_deg = fmodf(mechanical_angle_deg, 360.0f);
        if (mechanical_angle_deg < 0.0f) {
            mechanical_angle_deg += 360.0f;
        }
    }

    // 4. 转换为电角度
    float mechanical_angle_rad = mechanical_angle_deg * PI_VAL / 180.0f;   //弧度制
    float electrical_angle = mechanical_angle_rad * motor->pole_pairs;

    // 5. 归一化到 [0, 2π)
    electrical_angle = fmodf(electrical_angle, TWO_PI);
    if (electrical_angle < 0.0f) {
        electrical_angle += TWO_PI;
    }

    return electrical_angle;
}

/********************************************速度M1 获取机械角速度********************************************/
/**
  * @brief  M1 获取机械角速度 并 滤波
  */
float M1_Temp_spd_dms = 0.0;
float M1_Temp_spd = 0.0;
float M1_Pos_MechTheta_pre = 0.0;
float M1_Temp_Spd_out_dms = 0.0;
float M1_Speed_Get(float M1_Pos_MechTheta, float M1_Time_interval)
{
    if ((M1_Pos_MechTheta-M1_Pos_MechTheta_pre)< -180)
    {
        M1_Pos_MechTheta_pre = M1_Pos_MechTheta_pre-360;
    }
    if ((M1_Pos_MechTheta-M1_Pos_MechTheta_pre)> 180)
    {
        M1_Pos_MechTheta_pre = M1_Pos_MechTheta_pre+360;
    }

    M1_Temp_spd = M1_Pos_MechTheta-M1_Pos_MechTheta_pre;
    M1_Temp_spd_dms = M1_Temp_spd / M1_Time_interval; // 速度只有在“当前-前一时刻”则速度和位置环才能好用
    M1_Temp_Spd_out_dms = M1_Speed_Filter(M1_Temp_spd_dms);     //速度滤波

    M1_Pos_MechTheta_pre = M1_Pos_MechTheta;

    return M1_Temp_Spd_out_dms;
}
/**
  * @brief  M1 速度获取后 滤波处理
  */
float M1_Spd_Flt_out[3]={0};
float M1_Spd_Flt_in[3]={0};
float Para_M1[3] = {0};      // 0是增益，1是输出前一个值，2是输出前2个值
float M1_Speed_Filter(float M1_Temp_spd_dms)
{
//    Para_M1[1] = 1.8226949, Para_M1[2] = -0.83718165, Para_M1+[0] = 0.00362168;     // 20Hz
//    Para_M1[1] = 1.7786318, Para_M1[2] = -0.8008026,  Para_M1[0] = 0.00554271;     // 25Hz
    Para_M1[1] = 1.5610181, Para_M1[2] = -0.64135154, Para_M1[0] = 0.02008337;     // 50Hz
//      Para_M1[1] = 1.4754804, Para_M1[2] = -0.58691950, Para_M1[0] = 0.05571953;     // 60Hz
//    Para_M1[1] = 1.1429805, Para_M1[2] = -0.41280160, Para_M1[0] = 0.06745527;     // 100Hz

    M1_Spd_Flt_in[0]  = M1_Temp_spd_dms;
    M1_Spd_Flt_out[0] = Para_M1[1] * M1_Spd_Flt_out[1] + Para_M1[2]*M1_Spd_Flt_out[2]+Para_M1[0]*M1_Spd_Flt_in[2]+2*Para_M1[0]*M1_Spd_Flt_in[1]+Para_M1[0]*M1_Spd_Flt_in[0];//20HZ
    M1_Spd_Flt_out[2] = M1_Spd_Flt_out[1];
    M1_Spd_Flt_out[1] = M1_Spd_Flt_out[0];
    M1_Spd_Flt_in[2]  = M1_Spd_Flt_in[1];
    M1_Spd_Flt_in[1]  = M1_Spd_Flt_in[0];

    return  M1_Spd_Flt_out[0];
}

/********************************************速度M2 获取机械角速度********************************************/
/**
  * @brief  M2 获取机械角速度 并 滤波
  */
float M2_Temp_spd_dms = 0.0;
float M2_Pos_MechTheta_pre = 0.0;
float M2_Temp_Spd_out_dms = 0.0;
float M2_Speed_Get(float M2_Pos_MechTheta, float M2_Time_interval)
{
    if ((M2_Pos_MechTheta-M2_Pos_MechTheta_pre)<-180)
    {
        M2_Pos_MechTheta_pre = M2_Pos_MechTheta_pre-360;
    }
    if ((M2_Pos_MechTheta-M2_Pos_MechTheta_pre)>180)
    {
        M2_Pos_MechTheta_pre = M2_Pos_MechTheta_pre+360;
    }

    M2_Temp_spd_dms = (M2_Pos_MechTheta-M2_Pos_MechTheta_pre) / M2_Time_interval;
    M2_Temp_Spd_out_dms = M2_Speed_Filter(M2_Temp_spd_dms);

    M2_Pos_MechTheta_pre = M2_Pos_MechTheta;

    return M2_Temp_Spd_out_dms;
}
/**
  * @brief  M2 速度获取后 滤波处理
  */
float M2_Spd_Flt_out[3]={0};
float M2_Spd_Flt_in[3]={0};
float Para_M2[3] = {0};      // 0是增益，1是输出前一个值，2是输出前2个值
float M2_Speed_Filter(float M2_Temp_spd_dms)
{
    // Para_M2[1] = 1.8226949, Para_M2[2] = -0.83718165, Para_M2[0] = 0.00362168;     // 20Hz
    // Para_M2[1] = 1.7786318, Para_M2[2] = -0.8008026,  Para_M2[0] = 0.00554271;     // 25Hz
    // Para_M2[1] = 1.5610181, Para_M2[2] = -0.64135154, Para_M2[0] = 0.02008337;     // 50Hz
    Para_M2[1] = 1.4754804, Para_M2[2] = -0.58691950, Para_M2[0] = 0.05571953;     // 60Hz
    // Para_M2[1] = 1.1429805, Para_M2[2] = -0.41280160, Para_M2[0] = 0.06745527;     // 100Hz

    M2_Spd_Flt_in[0]  = M2_Temp_spd_dms;
    M2_Spd_Flt_out[0] = Para_M2[1] * M2_Spd_Flt_out[1] + Para_M2[2]*M2_Spd_Flt_out[2]+Para_M2[0]*M2_Spd_Flt_in[2]+2*Para_M2[0]*M2_Spd_Flt_in[1]+Para_M2[0]*M2_Spd_Flt_in[0];//20HZ
    M2_Spd_Flt_out[2] = M2_Spd_Flt_out[1];
    M2_Spd_Flt_out[1] = M2_Spd_Flt_out[0];
    M2_Spd_Flt_in[2]  = M2_Spd_Flt_in[1];
    M2_Spd_Flt_in[1]  = M2_Spd_Flt_in[0];

    return  M2_Spd_Flt_out[0];
}



#define CALIB_SAMPLES 1000

void Current_Offset_Calibration(void)
{
    uint32_t sum_a1 = 0, sum_b1 = 0, sum_c1 = 0;
    uint32_t sum_a2 = 0, sum_b2 = 0, sum_c2 = 0;

    for (int i = 0; i < CALIB_SAMPLES; i++)
    {
        // 等待ADC更新（简单延时 or 标志位）
        HAL_Delay(1);

        sum_a1 += M1_ADC.amp_u;
        sum_b1 += M1_ADC.amp_v;
        sum_c1 += M1_ADC.amp_w;

        sum_a2 += M2_ADC.amp_u;
        sum_b2 += M2_ADC.amp_v;
        sum_c2 += M2_ADC.amp_w;
    }

    // 转换成电压偏置
    float scale = 3.3f / 4095.0f;

    M1_Offset.offset_a = (sum_a1 / (float)CALIB_SAMPLES) * scale;
    M1_Offset.offset_b = (sum_b1 / (float)CALIB_SAMPLES) * scale;
    M1_Offset.offset_c = (sum_c1 / (float)CALIB_SAMPLES) * scale;

    M2_Offset.offset_a = (sum_a2 / (float)CALIB_SAMPLES) * scale;
    M2_Offset.offset_b = (sum_b2 / (float)CALIB_SAMPLES) * scale;
    M2_Offset.offset_c = (sum_c2 / (float)CALIB_SAMPLES) * scale;
}

/**
  * @brief  ADC 值转电流值（使用结构体参数）
  */
float ADC_To_Current(uint16_t adc) {
    float dc_full_scale = 4095.0f;
    float vref = 3.3f;
    float current_offset = 1.25f;
    float current_gain = 0.12f;

    float voltage = (adc / dc_full_scale) * vref;
    return (voltage - current_offset) / current_gain;
}
 
/**
  * @brief  三相电流采集
  */
void CurrentSensorSuite(FOC_Motor *motor)
{
    if (motor == &motor1)
    {
        motor->Ia = ADC_To_Current(M1_ADC.amp_u);
        motor->Ib = ADC_To_Current(M1_ADC.amp_v);
        motor->Ic = ADC_To_Current(M1_ADC.amp_w);
    }
    else if (motor == &motor2)
    {
        motor->Ia = ADC_To_Current(M2_ADC.amp_u);
        motor->Ib = ADC_To_Current(M2_ADC.amp_v);
        motor->Ic = ADC_To_Current(M2_ADC.amp_w);
    }
}


// 相关定义说明
//  ADC_REF        3.3f
//  ADC_MAX        4095.0f
//  CURRENT_OFFSET 1.65f      // 偏置电压
//  SHUNT_RES      0.01f      // 采样电阻
//  AMP_GAIN       33.0f      // 运放增益




