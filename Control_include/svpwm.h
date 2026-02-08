#ifndef __SVPWM_H
#define __SVPWM_H

#include "stm32f4xx.h"
#include <math.h>

// 电机结构体
typedef struct {
    float Ualpha;      // α轴电压
    float Ubeta;       // β轴电压
    float Vdc;         // 直流母线电压
    float T;           // PWM周期
    uint16_t sector;   // 扇区号
    float Tx;          // 矢量作用时间1
    float Ty;          // 矢量作用时间2
    float T0;          // 零矢量作用时间
    float Ta;          // A相作用时间
    float Tb;          // B相作用时间
    float Tc;          // C相作用时间
    float Ua;          // A相占空比
    float Ub;          // B相占空比
    float Uc;          // C相占空比
} SVPWM_TypeDef;

// 函数声明
void SVPWM_Init(SVPWM_TypeDef *svpwm, float Vdc, float period);
void SVPWM_Calc(SVPWM_TypeDef *svpwm);
uint8_t SVPWM_GetSector(float Ualpha, float Ubeta);
void SVPWM_SetDutyCycle(SVPWM_TypeDef *svpwm, TIM_TypeDef *TIMx);

#endif