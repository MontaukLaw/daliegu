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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SW_4067_S0_Pin GPIO_PIN_13
#define SW_4067_S0_GPIO_Port GPIOC
#define BL_FLOW_CTL_Pin GPIO_PIN_14
#define BL_FLOW_CTL_GPIO_Port GPIOC
#define CHRG_Pin GPIO_PIN_15
#define CHRG_GPIO_Port GPIOC
#define SW_4067_EN_2_Pin GPIO_PIN_1
#define SW_4067_EN_2_GPIO_Port GPIOA
#define SW_4067_EN_1_Pin GPIO_PIN_2
#define SW_4067_EN_1_GPIO_Port GPIOA
#define OUT_13_Pin GPIO_PIN_3
#define OUT_13_GPIO_Port GPIOA
#define OUT_12_Pin GPIO_PIN_4
#define OUT_12_GPIO_Port GPIOA
#define OUT_11_Pin GPIO_PIN_5
#define OUT_11_GPIO_Port GPIOA
#define OUT_10_Pin GPIO_PIN_6
#define OUT_10_GPIO_Port GPIOA
#define OUT_9_Pin GPIO_PIN_7
#define OUT_9_GPIO_Port GPIOA
#define OUT_8_Pin GPIO_PIN_4
#define OUT_8_GPIO_Port GPIOC
#define OUT_7_Pin GPIO_PIN_0
#define OUT_7_GPIO_Port GPIOB
#define OUT_6_Pin GPIO_PIN_1
#define OUT_6_GPIO_Port GPIOB
#define OUT_5_Pin GPIO_PIN_2
#define OUT_5_GPIO_Port GPIOB
#define OUT_4_Pin GPIO_PIN_10
#define OUT_4_GPIO_Port GPIOB
#define MAIN_ADC_Pin GPIO_PIN_11
#define MAIN_ADC_GPIO_Port GPIOB
#define OUT_3_Pin GPIO_PIN_12
#define OUT_3_GPIO_Port GPIOB
#define OUT_2_Pin GPIO_PIN_13
#define OUT_2_GPIO_Port GPIOB
#define OUT_1_Pin GPIO_PIN_14
#define OUT_1_GPIO_Port GPIOB
#define OUT_0_Pin GPIO_PIN_15
#define OUT_0_GPIO_Port GPIOB
#define SW_4067_S1_Pin GPIO_PIN_6
#define SW_4067_S1_GPIO_Port GPIOC
#define KEY_Pin GPIO_PIN_11
#define KEY_GPIO_Port GPIOA
#define PWR_CTRL_Pin GPIO_PIN_12
#define PWR_CTRL_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_15
#define LED_G_GPIO_Port GPIOA
#define LED_R_Pin GPIO_PIN_10
#define LED_R_GPIO_Port GPIOC
#define LED_B_Pin GPIO_PIN_5
#define LED_B_GPIO_Port GPIOB
#define SW_4067_S3_Pin GPIO_PIN_6
#define SW_4067_S3_GPIO_Port GPIOB
#define BL_LINK_Pin GPIO_PIN_7
#define BL_LINK_GPIO_Port GPIOB
#define SW_4067_S2_Pin GPIO_PIN_9
#define SW_4067_S2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
