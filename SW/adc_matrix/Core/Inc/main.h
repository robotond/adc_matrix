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
#define BLUE_Pin GPIO_PIN_13
#define BLUE_GPIO_Port GPIOC
#define GREEN_Pin GPIO_PIN_14
#define GREEN_GPIO_Port GPIOC
#define RED_Pin GPIO_PIN_15
#define RED_GPIO_Port GPIOC
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
#define U3TX_Pin GPIO_PIN_5
#define U3TX_GPIO_Port GPIOA
#define START_Pin GPIO_PIN_6
#define START_GPIO_Port GPIOA
#define R0_Pin GPIO_PIN_7
#define R0_GPIO_Port GPIOA
#define R1_Pin GPIO_PIN_0
#define R1_GPIO_Port GPIOB
#define R2_Pin GPIO_PIN_1
#define R2_GPIO_Port GPIOB
#define R3_Pin GPIO_PIN_2
#define R3_GPIO_Port GPIOB
#define R4_Pin GPIO_PIN_10
#define R4_GPIO_Port GPIOB
#define U3RX_Pin GPIO_PIN_11
#define U3RX_GPIO_Port GPIOB
#define PB12_Pin GPIO_PIN_12
#define PB12_GPIO_Port GPIOB
#define PB13_Pin GPIO_PIN_13
#define PB13_GPIO_Port GPIOB
#define U3DE_Pin GPIO_PIN_14
#define U3DE_GPIO_Port GPIOB
#define PB15_Pin GPIO_PIN_15
#define PB15_GPIO_Port GPIOB
#define PA8_Pin GPIO_PIN_8
#define PA8_GPIO_Port GPIOA
#define PC7_Pin GPIO_PIN_6
#define PC7_GPIO_Port GPIOC
#define PC7C7_Pin GPIO_PIN_7
#define PC7C7_GPIO_Port GPIOC
#define PA11_Pin GPIO_PIN_11
#define PA11_GPIO_Port GPIOA
#define PA12_Pin GPIO_PIN_12
#define PA12_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_BT0_Pin GPIO_PIN_14
#define SWCLK_BT0_GPIO_Port GPIOA
#define PA15_Pin GPIO_PIN_15
#define PA15_GPIO_Port GPIOA
#define PD0_Pin GPIO_PIN_0
#define PD0_GPIO_Port GPIOD
#define PD1_Pin GPIO_PIN_1
#define PD1_GPIO_Port GPIOD
#define PD2_Pin GPIO_PIN_2
#define PD2_GPIO_Port GPIOD
#define PD3_Pin GPIO_PIN_3
#define PD3_GPIO_Port GPIOD
#define PB3_Pin GPIO_PIN_3
#define PB3_GPIO_Port GPIOB
#define PB4_Pin GPIO_PIN_4
#define PB4_GPIO_Port GPIOB
#define COL4_Pin GPIO_PIN_5
#define COL4_GPIO_Port GPIOB
#define COL3_Pin GPIO_PIN_6
#define COL3_GPIO_Port GPIOB
#define COL2_Pin GPIO_PIN_7
#define COL2_GPIO_Port GPIOB
#define COL1_Pin GPIO_PIN_8
#define COL1_GPIO_Port GPIOB
#define COL0_Pin GPIO_PIN_9
#define COL0_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
