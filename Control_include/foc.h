#ifndef __FOC_H
#define __FOC_H

#include "svpwm.h"
#include "stm32f4xx_hal.h"  // 添加HAL库头文件
#include <math.h>

// PI控制器结构体
typedef struct {
    float Kp;          // 比例系数
    float Ki;          // 积分系数
    float integral;    // 积分值
    float error_prev;  // 上次误差
    float output;      // 输出值
    float output_max;  // 输出最大值
    float output_min;  // 输出最小值
} PI_Controller;

// FOC电机结构体
typedef struct {
    // 电机参数
    float Rs;          // 定子电阻
    float Ld;          // d轴电感
    float Lq;          // q轴电感
    float flux_linkage; // 永磁体磁链
    uint8_t pole_pairs; // 极对数
    
    // 电流反馈
    float Ia;          // A相电流
    float Ib;          // B相电流
    float Ic;          // C相电流
    float Ialpha;      // α轴电流
    float Ibeta;       // β轴电流
    float Id;          // d轴电流
    float Iq;          // q轴电流
    
    // 电压指令
    float Valpha;      // α轴电压
    float Vbeta;       // β轴电压
    float Vd;          // d轴电压
    float Vq;          // q轴电压
    
    // 转子信息
    float theta_e;     // 电角度
    float theta_m;     // 机械角度
    float speed_e;     // 电角速度
    float speed_m;     // 机械转速
    
    // 控制目标
    float Id_ref;      // d轴电流参考
    float Iq_ref;      // q轴电流参考
    float speed_ref;   // 转速参考
    
    // PI控制器
    PI_Controller pi_id;  // d轴电流环
    PI_Controller pi_iq;  // q轴电流环
    PI_Controller pi_speed; // 速度环
    
    // SVPWM
    SVPWM_TypeDef svpwm;
    
    // 硬件相关
    TIM_TypeDef *TIM_PWM; // PWM定时器
    float Vdc;            // 直流母线电压
} FOC_Motor;

// 函数声明
void FOC_Init(FOC_Motor *motor, TIM_TypeDef *TIMx, float Vdc, uint8_t pole_pairs);
void FOC_ClarkeTransform(FOC_Motor *motor);
void FOC_ParkTransform(FOC_Motor *motor);
void FOC_InverseParkTransform(FOC_Motor *motor);
void FOC_PI_Update(PI_Controller *pi, float error, float Ts);
void FOC_CurrentControl(FOC_Motor *motor, float Ts);
void FOC_SpeedControl(FOC_Motor *motor, float Ts);
void FOC_Update(FOC_Motor *motor, float Ts);
float FOC_GetElectricalAngle(FOC_Motor *motor);

extern TIM_HandleTypeDef htim1;  // 添加外部声明
extern TIM_HandleTypeDef htim8;  // 添加外部声明

#endif