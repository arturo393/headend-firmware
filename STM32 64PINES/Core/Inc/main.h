/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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
#define DE_Pin GPIO_PIN_1
#define DE_GPIO_Port GPIOA
#define DC_A1_Pin GPIO_PIN_7
#define DC_A1_GPIO_Port GPIOA
#define RF_A4_Pin GPIO_PIN_0
#define RF_A4_GPIO_Port GPIOB
#define RF_A3_Pin GPIO_PIN_1
#define RF_A3_GPIO_Port GPIOB
#define RF_A2_Pin GPIO_PIN_2
#define RF_A2_GPIO_Port GPIOB
#define KA_LED_Pin GPIO_PIN_12
#define KA_LED_GPIO_Port GPIOB
#define NEXTION_LED_Pin GPIO_PIN_13
#define NEXTION_LED_GPIO_Port GPIOB
#define DATA_OK_Pin GPIO_PIN_14
#define DATA_OK_GPIO_Port GPIOB
#define ADC_CPLT_Pin GPIO_PIN_15
#define ADC_CPLT_GPIO_Port GPIOB
#define ALARM_A4_Pin GPIO_PIN_8
#define ALARM_A4_GPIO_Port GPIOC
#define ALARM_A3_Pin GPIO_PIN_9
#define ALARM_A3_GPIO_Port GPIOC
#define DC_A2_Pin GPIO_PIN_8
#define DC_A2_GPIO_Port GPIOA
#define DC_A3_Pin GPIO_PIN_11
#define DC_A3_GPIO_Port GPIOA
#define DC_A4_Pin GPIO_PIN_12
#define DC_A4_GPIO_Port GPIOA
#define ALARM_A2_Pin GPIO_PIN_10
#define ALARM_A2_GPIO_Port GPIOC
#define ALARM_A1_Pin GPIO_PIN_11
#define ALARM_A1_GPIO_Port GPIOC
#define ALARM_VIN_Pin GPIO_PIN_12
#define ALARM_VIN_GPIO_Port GPIOC
#define RF_A1_Pin GPIO_PIN_3
#define RF_A1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
