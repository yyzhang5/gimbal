/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_adc3;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY0_Pin GPIO_PIN_2
#define KEY0_GPIO_Port GPIOE
#define KEY1_Pin GPIO_PIN_3
#define KEY1_GPIO_Port GPIOE
#define KEY2_Pin GPIO_PIN_4
#define KEY2_GPIO_Port GPIOE
#define BEEP_Pin GPIO_PIN_0
#define BEEP_GPIO_Port GPIOF
#define PM2_CTRL_SD_Pin GPIO_PIN_2
#define PM2_CTRL_SD_GPIO_Port GPIOF
#define PM2_BEMFW_Pin GPIO_PIN_4
#define PM2_BEMFW_GPIO_Port GPIOF
#define PM2_BEMFV_Pin GPIO_PIN_5
#define PM2_BEMFV_GPIO_Port GPIOF
#define PM2_BEMFU_Pin GPIO_PIN_6
#define PM2_BEMFU_GPIO_Port GPIOF
#define PM1_BEMFW_Pin GPIO_PIN_7
#define PM1_BEMFW_GPIO_Port GPIOF
#define PM1_BEMFV_Pin GPIO_PIN_8
#define PM1_BEMFV_GPIO_Port GPIOF
#define PM1_BEMFU_Pin GPIO_PIN_9
#define PM1_BEMFU_GPIO_Port GPIOF
#define PM1_CTRL_SD_Pin GPIO_PIN_10
#define PM1_CTRL_SD_GPIO_Port GPIOF
#define PM2_VTEMP_Pin GPIO_PIN_0
#define PM2_VTEMP_GPIO_Port GPIOC
#define PM2_AMPU_Pin GPIO_PIN_2
#define PM2_AMPU_GPIO_Port GPIOC
#define PM2_AMPV_Pin GPIO_PIN_3
#define PM2_AMPV_GPIO_Port GPIOC
#define PM1_VTEMP_Pin GPIO_PIN_0
#define PM1_VTEMP_GPIO_Port GPIOA
#define PM1_AMPW_Pin GPIO_PIN_3
#define PM1_AMPW_GPIO_Port GPIOA
#define PM2_AMPW_Pin GPIO_PIN_4
#define PM2_AMPW_GPIO_Port GPIOA
#define PM2_VBUS_Pin GPIO_PIN_5
#define PM2_VBUS_GPIO_Port GPIOA
#define PM1_AMPV_Pin GPIO_PIN_6
#define PM1_AMPV_GPIO_Port GPIOA
#define PM1_AMPU_Pin GPIO_PIN_0
#define PM1_AMPU_GPIO_Port GPIOB
#define PM1_VBUS_Pin GPIO_PIN_1
#define PM1_VBUS_GPIO_Port GPIOB
#define RS485_TX_Pin GPIO_PIN_10
#define RS485_TX_GPIO_Port GPIOB
#define RS485_RX_Pin GPIO_PIN_11
#define RS485_RX_GPIO_Port GPIOB
#define PM1_BKIN_Pin GPIO_PIN_12
#define PM1_BKIN_GPIO_Port GPIOB
#define PM1_PWM_UL_Pin GPIO_PIN_13
#define PM1_PWM_UL_GPIO_Port GPIOB
#define PM1_PWM_VL_Pin GPIO_PIN_14
#define PM1_PWM_VL_GPIO_Port GPIOB
#define PM1_PWM_WL_Pin GPIO_PIN_15
#define PM1_PWM_WL_GPIO_Port GPIOB
#define PM1_ENC_SCK_Pin GPIO_PIN_6
#define PM1_ENC_SCK_GPIO_Port GPIOC
#define PM1_ENC_DAT_Pin GPIO_PIN_7
#define PM1_ENC_DAT_GPIO_Port GPIOC
#define PM1_PWM_UH_Pin GPIO_PIN_8
#define PM1_PWM_UH_GPIO_Port GPIOA
#define PM1_PWM_VH_Pin GPIO_PIN_9
#define PM1_PWM_VH_GPIO_Port GPIOA
#define PM1_PWM_WH_Pin GPIO_PIN_10
#define PM1_PWM_WH_GPIO_Port GPIOA
#define PM2_PWM_UL_Pin GPIO_PIN_13
#define PM2_PWM_UL_GPIO_Port GPIOH
#define PM2_PWM_VL_Pin GPIO_PIN_14
#define PM2_PWM_VL_GPIO_Port GPIOH
#define PM2_PWM_WL_Pin GPIO_PIN_15
#define PM2_PWM_WL_GPIO_Port GPIOH
#define PM2_ENC_SCK_Pin GPIO_PIN_15
#define PM2_ENC_SCK_GPIO_Port GPIOA
#define PM2_ENC_DAT_Pin GPIO_PIN_3
#define PM2_ENC_DAT_GPIO_Port GPIOB
#define UART1_TX_Pin GPIO_PIN_6
#define UART1_TX_GPIO_Port GPIOB
#define UART1_RX_Pin GPIO_PIN_7
#define UART1_RX_GPIO_Port GPIOB
#define LED0_Pin GPIO_PIN_0
#define LED0_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOE
#define PM2_BKIN_Pin GPIO_PIN_4
#define PM2_BKIN_GPIO_Port GPIOI
#define PM2_PWM_UH_Pin GPIO_PIN_5
#define PM2_PWM_UH_GPIO_Port GPIOI
#define PM2_PWM_VH_Pin GPIO_PIN_6
#define PM2_PWM_VH_GPIO_Port GPIOI
#define PM2_PWM_WH_Pin GPIO_PIN_7
#define PM2_PWM_WH_GPIO_Port GPIOI

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
