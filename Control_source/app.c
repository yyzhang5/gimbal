/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : function.c
	2026.01.20				: YYZ
	用于实现电机控制
*/

#include "svpwm.h"
#include "foc.h"
#include "app.h"
#include "params.h"
#include "gpio_dev.h"
#include "stm32f4xx_hal_adc.h"
#include "biss.h"



#define ADC1_CH_NUM 8
#define ADC2_CH_NUM 2
#define ADC3_CH_NUM 6

//uint16_t ADC1_Buff[ADC1_CH_NUM];
//uint16_t ADC2_Buff[ADC2_CH_NUM];
//uint16_t ADC3_Buff[ADC3_CH_NUM];

uint16_t ADC_Buff[ADC1_CH_NUM+ADC2_CH_NUM+ADC3_CH_NUM];


// ! 待完善功能
// /* 定义函数 */
// 编码器闭环
// {
//     SVPWM闭环
//     电流闭环
//     速度环闭环
// }




//功能函数

//初始化函数
uint8_t setup(void)
{
    
    set_led(0, 1);
    set_led(1, 1);
    
    HAL_TIM_Base_Start_IT(&htim7);
    
    /* 使能电机驱动（解除 Shutdown） */
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET);   // PM2_CTRL_SD
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);  // PM1_CTRL_SD

    
//    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Buff, ADC1_CH_NUM);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)(ADC_Buff+ADC1_CH_NUM), ADC2_CH_NUM);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t*)(ADC_Buff+ADC1_CH_NUM+ADC2_CH_NUM), ADC3_CH_NUM);
    
//M1 PWM
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // M1 PWM CH1
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);  // M1 PWM CH1N
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
//M1 PWM 
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); // M2 PWM CH1
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);  // M2 PWM CH1N
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
    
    /* 使能定时器的主输出（对于高级定时器TIM1和TIM8必须设置）*/
    __HAL_TIM_MOE_ENABLE(&htim1);
    __HAL_TIM_MOE_ENABLE(&htim8);
    
    /* 设置初始占空比为0%（安全启动）*/
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
    
    
    return 0;
}

void loop(void)
{

    static uint32_t cnt = 50;
          HAL_Delay(50); //延时 ms
//      cnt++;
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, cnt); // 设置占空比，占空比=cnt/ARR
    if(cnt ==100)
    {
        cnt = 0;
    }
}
    

void timer_callback(void)       //led灯高低电平翻转
{
    volatile static uint32_t cnt = 0;
    cnt++;
    if(cnt%10 == 0)
    {
        // any code
    }
    if(cnt%100 == 0)
    {
        // any code

    }
    if(cnt%1000 == 0)
    {
       toggle_led(1);
    //    toggle_led(0);
        // any code
    }
    if(cnt%10000 == 0)
    {
        //  toggle_led(1);
         toggle_led(0);
        // any code
    }
    if(cnt == 0xffffff)
    {
        cnt = 0;
    }
}








