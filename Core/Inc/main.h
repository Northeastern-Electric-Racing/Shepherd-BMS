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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EXT_GPIO_1_Pin GPIO_PIN_13
#define EXT_GPIO_1_GPIO_Port GPIOC
#define EXT_GPIO_5_Pin GPIO_PIN_14
#define EXT_GPIO_5_GPIO_Port GPIOC
#define EXT_GPIO_4_Pin GPIO_PIN_15
#define EXT_GPIO_4_GPIO_Port GPIOC
#define SPI3_CS_Pin GPIO_PIN_0
#define SPI3_CS_GPIO_Port GPIOC
#define SPI2_CS_Pin GPIO_PIN_1
#define SPI2_CS_GPIO_Port GPIOC
#define INTERLOCK_READ_Pin GPIO_PIN_2
#define INTERLOCK_READ_GPIO_Port GPIOA
#define FAULT_OUTPUT_Pin GPIO_PIN_3
#define FAULT_OUTPUT_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define DEBUG_LED_2_Pin GPIO_PIN_4
#define DEBUG_LED_2_GPIO_Port GPIOC
#define I_SENSE_2_Pin GPIO_PIN_5
#define I_SENSE_2_GPIO_Port GPIOC
#define I_SENSE_3_Pin GPIO_PIN_0
#define I_SENSE_3_GPIO_Port GPIOB
#define I_SENSE_1_Pin GPIO_PIN_1
#define I_SENSE_1_GPIO_Port GPIOB
#define I_SENSE_0_Pin GPIO_PIN_2
#define I_SENSE_0_GPIO_Port GPIOB
#define DEBUG_LED_1_Pin GPIO_PIN_11
#define DEBUG_LED_1_GPIO_Port GPIOB
#define WATCHDOG_OUT_Pin GPIO_PIN_14
#define WATCHDOG_OUT_GPIO_Port GPIOB
#define FAN_PWM_3_Pin GPIO_PIN_6
#define FAN_PWM_3_GPIO_Port GPIOC
#define FAN_PWM_0_Pin GPIO_PIN_7
#define FAN_PWM_0_GPIO_Port GPIOC
#define FAN_PWM_1_Pin GPIO_PIN_8
#define FAN_PWM_1_GPIO_Port GPIOC
#define FAN_PWM_4_Pin GPIO_PIN_9
#define FAN_PWM_4_GPIO_Port GPIOC
#define FAN_PWM_2_Pin GPIO_PIN_8
#define FAN_PWM_2_GPIO_Port GPIOA
#define FAN_PWM_5_Pin GPIO_PIN_10
#define FAN_PWM_5_GPIO_Port GPIOA
#define EXT_GPIO_0_Pin GPIO_PIN_5
#define EXT_GPIO_0_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
