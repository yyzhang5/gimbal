#ifndef __CONTROL_H
#define __CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "foc.h"
#include "svpwm.h"
#include "stm32f4xx.h"
#include <math.h>

/* ==================== 宏定义 ==================== */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define TWO_PI  6.28318530718f

/* ==================== 外部变量声明 ==================== */
// 双电机控制对象
extern FOC_Motor motor1;
extern FOC_Motor motor2;

/* ==================== 函数声明 ==================== */
/**
 * @brief SVPWM 开环控制示例
 * @note 用于测试 SVPWM 波形输出，双电机同步运行
 */
void SVPWM_OpenLoop_Example(void);

/**
 * @brief FOC 闭环控制示例
 * @note 双电机速度闭环控制，需配合电流采样和角度反馈
 */
void FOC_ClosedLoop_Example(void);

/* ==================== 控制参数配置 ==================== */
// PWM 定时器配置
#define PWM_TIMER1      TIM1
#define PWM_TIMER2      TIM8
#define PWM_PRESCALER   167
#define PWM_PERIOD      99

// 电机参数配置
#define MOTOR1_VBUS     24.0f
#define MOTOR2_VBUS     24.0f
#define MOTOR1_POLE_PAIRS  21
#define MOTOR2_POLE_PAIRS  21

// 控制周期 (100us)
#define CONTROL_PERIOD_US  100
#define CONTROL_PERIOD_S   0.0001f

/* ==================== 接口函数声明 ==================== */
/**
 * @brief 电机 1 电流采样接口（需用户实现）
 */
// float Read_ADC_Motor1_PhaseA(void);
// float Read_ADC_Motor1_PhaseB(void);

/**
 * @brief 电机 2 电流采样接口（需用户实现）
 */
// float Read_ADC_Motor2_PhaseA(void);
// float Read_ADC_Motor2_PhaseB(void);

/**
 * @brief 电机 1 角度获取接口（需用户实现）
 */
// float Get_Encoder_Angle_Motor1(void);

/**
 * @brief 电机 2 角度获取接口（需用户实现）
 */
// float Get_Encoder_Angle_Motor2(void);

#ifdef __cplusplus
}
#endif

#endif /* __CONTROL_H */