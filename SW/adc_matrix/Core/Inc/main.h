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
#include "stm32g0xx_hal.h"

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
#define START_Pin GPIO_PIN_13
#define START_GPIO_Port GPIOC
#define S0_Pin GPIO_PIN_0
#define S0_GPIO_Port GPIOA
#define S1_Pin GPIO_PIN_1
#define S1_GPIO_Port GPIOA
#define S2_Pin GPIO_PIN_2
#define S2_GPIO_Port GPIOA
#define S3_Pin GPIO_PIN_3
#define S3_GPIO_Port GPIOA
#define S4_Pin GPIO_PIN_4
#define S4_GPIO_Port GPIOA
#define S5_Pin GPIO_PIN_5
#define S5_GPIO_Port GPIOA
#define S6_Pin GPIO_PIN_6
#define S6_GPIO_Port GPIOA
#define S7_Pin GPIO_PIN_7
#define S7_GPIO_Port GPIOA
#define ANREFIN_Pin GPIO_PIN_0
#define ANREFIN_GPIO_Port GPIOB
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_BT0_Pin GPIO_PIN_14
#define SWCLK_BT0_GPIO_Port GPIOA
#define ENSPWR_Pin GPIO_PIN_3
#define ENSPWR_GPIO_Port GPIOD
#define RED_Pin GPIO_PIN_3
#define RED_GPIO_Port GPIOB
#define GREEN_Pin GPIO_PIN_4
#define GREEN_GPIO_Port GPIOB
#define BLUE1_Pin GPIO_PIN_5
#define BLUE1_GPIO_Port GPIOB
#define BLUE2_Pin GPIO_PIN_6
#define BLUE2_GPIO_Port GPIOB
#define BLUE3_Pin GPIO_PIN_7
#define BLUE3_GPIO_Port GPIOB
#define U3TX_Pin GPIO_PIN_8
#define U3TX_GPIO_Port GPIOB
#define U3RX_Pin GPIO_PIN_9
#define U3RX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
