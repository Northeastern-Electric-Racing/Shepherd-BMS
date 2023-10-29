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
#include <stdlib.h>
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
#define FPGA_Reset_Pin GPIO_PIN_13
#define FPGA_Reset_GPIO_Port GPIOC
#define Communication_GPIO_Pin GPIO_PIN_14
#define Communication_GPIO_GPIO_Port GPIOC
#define Communication_GPIOC15_Pin GPIO_PIN_15
#define Communication_GPIOC15_GPIO_Port GPIOC
#define Communication_GPIOC0_Pin GPIO_PIN_0
#define Communication_GPIOC0_GPIO_Port GPIOC
#define SPI_2_CS_Pin GPIO_PIN_1
#define SPI_2_CS_GPIO_Port GPIOC
#define Interlock_Read_Pin GPIO_PIN_2
#define Interlock_Read_GPIO_Port GPIOA
#define Fault_Output_Pin GPIO_PIN_3
#define Fault_Output_GPIO_Port GPIOA
#define SPI_1_CS_Pin GPIO_PIN_4
#define SPI_1_CS_GPIO_Port GPIOA
#define Debug_LED_Pin GPIO_PIN_4
#define Debug_LED_GPIO_Port GPIOC
#define I_Sense_Pin GPIO_PIN_5
#define I_Sense_GPIO_Port GPIOC
#define I_SenseB0_Pin GPIO_PIN_0
#define I_SenseB0_GPIO_Port GPIOB
#define I_SenseB1_Pin GPIO_PIN_1
#define I_SenseB1_GPIO_Port GPIOB
#define I_SenseB2_Pin GPIO_PIN_2
#define I_SenseB2_GPIO_Port GPIOB
#define Debug_LEDB11_Pin GPIO_PIN_11
#define Debug_LEDB11_GPIO_Port GPIOB
#define Watchdog_Out_Pin GPIO_PIN_14
#define Watchdog_Out_GPIO_Port GPIOB
#define Fan_PWM_Pin GPIO_PIN_6
#define Fan_PWM_GPIO_Port GPIOC
#define Fan_PWMC7_Pin GPIO_PIN_7
#define Fan_PWMC7_GPIO_Port GPIOC
#define Fan_PWMC8_Pin GPIO_PIN_8
#define Fan_PWMC8_GPIO_Port GPIOC
#define Fan_PWMC9_Pin GPIO_PIN_9
#define Fan_PWMC9_GPIO_Port GPIOC
#define Fan_PWMA8_Pin GPIO_PIN_8
#define Fan_PWMA8_GPIO_Port GPIOA
#define Fan_PWMA10_Pin GPIO_PIN_10
#define Fan_PWMA10_GPIO_Port GPIOA
#define SPI_3_CS_Pin GPIO_PIN_15
#define SPI_3_CS_GPIO_Port GPIOA
#define External_GPIO_Pin GPIO_PIN_2
#define External_GPIO_GPIO_Port GPIOD
#define External_GPIOB3_Pin GPIO_PIN_3
#define External_GPIOB3_GPIO_Port GPIOB
#define External_GPIOB4_Pin GPIO_PIN_4
#define External_GPIOB4_GPIO_Port GPIOB
#define External_GPIOB5_Pin GPIO_PIN_5
#define External_GPIOB5_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
