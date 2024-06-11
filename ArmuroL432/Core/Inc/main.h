/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define lineSensor_middle_Pin GPIO_PIN_0
#define lineSensor_middle_GPIO_Port GPIOA
#define encoder_left_Pin GPIO_PIN_1
#define encoder_left_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define lineSensor_right_Pin GPIO_PIN_3
#define lineSensor_right_GPIO_Port GPIOA
#define battery_Pin GPIO_PIN_4
#define battery_GPIO_Port GPIOA
#define encoder_right_Pin GPIO_PIN_5
#define encoder_right_GPIO_Port GPIOA
#define lineSensor_left_Pin GPIO_PIN_7
#define lineSensor_left_GPIO_Port GPIOA
#define Phase1_L_CH2N_Pin GPIO_PIN_0
#define Phase1_L_CH2N_GPIO_Port GPIOB
#define Phase1_R_CH3N_Pin GPIO_PIN_1
#define Phase1_R_CH3N_GPIO_Port GPIOB
#define switch_right_Pin GPIO_PIN_8
#define switch_right_GPIO_Port GPIOA
#define switch_middle_Pin GPIO_PIN_9
#define switch_middle_GPIO_Port GPIOA
#define LED5_Pin GPIO_PIN_10
#define LED5_GPIO_Port GPIOA
#define switch_left_Pin GPIO_PIN_11
#define switch_left_GPIO_Port GPIOA
#define phase2_L_Pin GPIO_PIN_12
#define phase2_L_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_3
#define LD3_GPIO_Port GPIOB
#define LED_right_Pin GPIO_PIN_4
#define LED_right_GPIO_Port GPIOB
#define LED_left_Pin GPIO_PIN_5
#define LED_left_GPIO_Port GPIOB
#define phase2_R_Pin GPIO_PIN_6
#define phase2_R_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
