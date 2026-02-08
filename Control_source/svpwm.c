#include "svpwm.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "foc.h"


#define SQRT3 1.73205080757f
//#define ONE_BY_SQRT3 0.57735026919f
#define TWO_THIRD 0.66666666667f

/**
  * @brief  SVPWM初始化
  * @param  svpwm: SVPWM结构体指针
  * @param  Vdc: 直流母线电压
  * @param  period: PWM周期（秒）
  */
void SVPWM_Init(SVPWM_TypeDef *svpwm, float Vdc, float period) {
    svpwm->Vdc = Vdc;
    svpwm->T = period;
    svpwm->Ualpha = 0;
    svpwm->Ubeta = 0;
    svpwm->sector = 0;
}

/**
  * @brief  判断扇区
  * @param  Ualpha: α轴电压
  * @param  Ubeta: β轴电压
  * @retval 扇区号(1-6)
  */
uint8_t SVPWM_GetSector(float Ualpha, float Ubeta) {
    uint8_t sector = 0;
    
    if (Ubeta >= 0) {
        if (Ualpha >= 0) {
            sector = (Ubeta > SQRT3 * Ualpha) ? 2 : 1;
        } else {
            sector = (Ubeta > -SQRT3 * Ualpha) ? 2 : 3;
        }
    } else {
        if (Ualpha >= 0) {
            sector = (-Ubeta > SQRT3 * Ualpha) ? 5 : 6;
        } else {
            sector = (-Ubeta > -SQRT3 * Ualpha) ? 5 : 4;
        }
    }
    
    return sector;
}

/**
  * @brief  SVPWM计算
  * @param  svpwm: SVPWM结构体指针
//  */
void SVPWM_Calc(SVPWM_TypeDef *svpwm)
{
    float Ualpha = svpwm->Ualpha;
    float Ubeta  = svpwm->Ubeta;
    float Vdc    = svpwm->Vdc;

    /* αβ → abc（相电压，未归一） */
    float Ua = Ualpha;
    float Ub = -0.5f * Ualpha + 0.8660254f * Ubeta;
    float Uc = -0.5f * Ualpha - 0.8660254f * Ubeta;

    /* 零序电压注入（SVPWM 核心） */
    float Umax = Ua;
    if (Ub > Umax) Umax = Ub;
    if (Uc > Umax) Umax = Uc;

    float Umin = Ua;
    if (Ub < Umin) Umin = Ub;
    if (Uc < Umin) Umin = Uc;

    float U0 = -0.5f * (Umax + Umin);

    Ua += U0;
    Ub += U0;
    Uc += U0;

    /* 归一化到 duty（0~1） */
    svpwm->Ua = Ua / Vdc + 0.5f;
    svpwm->Ub = Ub / Vdc + 0.5f;
    svpwm->Uc = Uc / Vdc + 0.5f;

    /* 限幅 */
    if (svpwm->Ua > 1.0f) svpwm->Ua = 1.0f;
    if (svpwm->Ua < 0.0f) svpwm->Ua = 0.0f;

    if (svpwm->Ub > 1.0f) svpwm->Ub = 1.0f;
    if (svpwm->Ub < 0.0f) svpwm->Ub = 0.0f;

    if (svpwm->Uc > 1.0f) svpwm->Uc = 1.0f;
    if (svpwm->Uc < 0.0f) svpwm->Uc = 0.0f;
}




/**
  * @brief  设置PWM占空比到定时器
  * @param  svpwm: SVPWM结构体指针
  * @param  TIMx: 定时器指针
  */

extern TIM_HandleTypeDef htim1;  // 从main.c中引用
extern TIM_HandleTypeDef htim8;  // 从main.c中引用

uint16_t cmp_a;
uint16_t cmp_b;
uint16_t cmp_c;


void SVPWM_SetDutyCycle(SVPWM_TypeDef *svpwm, TIM_TypeDef *TIMx)
{
    uint16_t arr = TIMx->ARR;

     cmp_a = (uint16_t)(svpwm->Ua * arr);
     cmp_b = (uint16_t)(svpwm->Ub * arr);
     cmp_c = (uint16_t)(svpwm->Uc * arr);

    if (TIMx == TIM1) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, cmp_a);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, cmp_b);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, cmp_c);
    } else if (TIMx == TIM8) {
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, cmp_a);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, cmp_b);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, cmp_c);
    }
}

