#include "delay.h"

extern TIM_HandleTypeDef htim7;

/* TIM7: PSC = 84-1 → 1 MHz → 1 µs / tick */
void delay_us_tim7(uint16_t us)
{
    uint16_t start = __HAL_TIM_GET_COUNTER(&htim7);

    while ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim7) - start) < us)
    {
        /* busy wait */
    }
}
