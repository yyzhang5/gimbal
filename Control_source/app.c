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
#include "control.h"



#define ADC1_CH_NUM 2
#define ADC2_CH_NUM 2
#define ADC3_CH_NUM 6

volatile uint16_t ADC_Buff[ADC1_CH_NUM+ADC2_CH_NUM+ADC3_CH_NUM];
volatile uint8_t adc1_ready = 0;
volatile uint8_t adc2_ready = 0;
// volatile uint8_t adc3_ready = 0;

/* 用户可选：当所有ADC一次采集完成后才进入FOC控制 */
// inline uint8_t ADC_AllReady(void) {
//     return (adc1_ready && adc2_ready && adc3_ready);
// }






// ! 待完善功能
// /* 定义函数 */
// 编码器闭环
// {
//     SVPWM闭环
//     电流闭环
//     速度环闭环
// }






//初始化函数
uint8_t setup(void)
{

    set_led(0, 1);
    set_led(1, 1);

    /* 使能定时器的主输出（对于高级定时器TIM1和TIM8必须设置）*/
    __HAL_TIM_MOE_ENABLE(&htim1);
    __HAL_TIM_MOE_ENABLE(&htim8);
    
    // 启动定时器中断（TIM8为主，TIM1与TIM8同步，见下文）
    HAL_TIM_Base_Start_IT(&htim8);   // 必须先启动TIM8，因为它提供TRGO
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim7);

    /* 使能电机驱动（解除 Shutdown） */
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET);   // PM2_CTRL_SD
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);  // PM1_CTRL_SD



//M1 PWM
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // M1 PWM CH1
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);
    
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);  // M1 PWM CH1N
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_OC_Start(&htim8, TIM_CHANNEL_4);
//M1 PWM 
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); // M2 PWM CH1
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);  // M2 PWM CH1N
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
    
    /* 设置初始占空比为0%（安全启动）*/
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);

    HAL_ADCEx_InjectedStart_IT(&hadc1);
    HAL_ADCEx_InjectedStart_IT(&hadc2);

    HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
    HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);
    HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
    
    // // 启动ADC DMA（循环模式）
    // HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Buff, ADC1_CH_NUM);
    // HAL_ADC_Start_DMA(&hadc2, (uint32_t*)(ADC_Buff+ADC1_CH_NUM), ADC2_CH_NUM);
    // HAL_ADC_Start_DMA(&hadc3, (uint32_t*)(ADC_Buff+ADC1_CH_NUM+ADC2_CH_NUM), ADC3_CH_NUM);  //!这句话报错,debug时卡在这不运行
    // 明确指定每个 ADC 的通道顺序
    // ADC1: IN0, IN5, IN6, IN8, IN9, IN10, IN12, IN13
    // ADC2: IN3, IN4
    // ADC3: IN14, IN15, IN4, IN5, IN6, IN7

    
    // Current_Offset_Calibration();   //电流偏置校准
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

        static uint16_t last_val = 0;
    
    // 如果 ADC 值变化，翻转 GPIO
    if(ADC_Buff[0] != last_val)
    {
        toggle_led(0);  // 数据更新时翻转 LED
        last_val = ADC_Buff[0];
    }
    HAL_Delay(100);
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
    // toggle_led(1);
    // toggle_led(0);
        // any code
    }      
    // if(cnt%10000 == 0)
    {
        // toggle_led(1);
        // toggle_led(0);
        // any code
    }
    if(cnt == 0xffffff)
    {
        cnt = 0;
    }

    M1_Control();
    // M2_Control();
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc == &hadc1) {
        adc1_ready = 1;
    } else if (hadc == &hadc2) {
        adc2_ready = 1;
    }
}                                  