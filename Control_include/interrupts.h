#ifndef __INTERRUPTS_H__
#define __INTERRUPTS_H__

#include "main.h"
#include <stdint.h>  

extern void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc);
extern void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);


typedef struct
{
    uint16_t amp_u;
    uint16_t amp_v;
    uint16_t amp_w;
} PhaseADC_t;

extern volatile PhaseADC_t M1_ADC;
extern volatile PhaseADC_t M2_ADC;

#endif
