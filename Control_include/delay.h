#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f4xx_hal.h"
#include "main.h"

#define SYSCLK_MHZ (SystemCoreClock/1000000)      //系统时钟频率

void delay_us_tim6(uint16_t us);
void delay_us_tim7(uint16_t us);
void delay_us(uint16_t us);

#endif
