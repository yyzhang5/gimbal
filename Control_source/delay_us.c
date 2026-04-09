#include "delay.h"

extern TIM_HandleTypeDef htim6;
// ! Tim7 基本定时器

/* TIM7: PSC = 84-1 → 1 MHz 
         ARR = 100-1 → 100 µs / tick */
void delay_us_tim6(uint16_t us)
{
    uint16_t start = __HAL_TIM_GET_COUNTER(&htim6);

    while ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim6) - start) < us)
    {
        /* busy wait */
    }
}

extern TIM_HandleTypeDef htim7;
/* TIM7: PSC = 84-1 → 1 MHz 
         ARR = 100-1 → 100 µs / tick */
void delay_us_tim7(uint16_t us)
{ 
    uint16_t start = __HAL_TIM_GET_COUNTER(&htim7);

    while ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim7) - start) < us)
    {
        /* busy wait */
    }
}

// void delay_us(uint16_t us)
// {
//     uint32_t Tdata = us*SYSCLK_MHZ/5;
//     for(uint32_t i=0;i<Tdata;i++)
//     {
//         __NOP();
//     }
// }
