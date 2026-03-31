#include "interrupts.h"
#include "main.h"
#include "app.h"

// 实际定义（分配内存）
volatile PhaseADC_t M1_ADC;
volatile PhaseADC_t M2_ADC;

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        M1_ADC.amp_u = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        M1_ADC.amp_v = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
        M1_ADC.amp_w = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);

        // // 1. 扣零偏
        // 2. 转成电流
        // 3. 做保护/滤波
        // 4. 给FOC或电流环使用
    }
    if (hadc->Instance == ADC2)
    {
        M2_ADC.amp_u = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        M2_ADC.amp_v = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
        M2_ADC.amp_w = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);

        // float m2_Iu = ADC_To_Current(m2_u_adc);
        // float m2_Iv = ADC_To_Current(m2_v_adc);
        // float m2_Iw = ADC_To_Current(m2_w_adc);
        // 1. 扣零偏
        // 2. 转成电流
        // 3. 做保护/滤波
        // 4. 给FOC或电流环使用
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {
        timer_callback();
        // 1. 执行FOC或电流环
        // 2. 执行FOC或电流环的保护
        // 3. 执行FOC或电流环的滤波
        // 4. 执行FOC或电流环的保护
    }
}