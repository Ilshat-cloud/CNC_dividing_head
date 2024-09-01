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
#include "stm32f1xx_hal.h"

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
#define Led_Pin GPIO_PIN_13
#define Led_GPIO_Port GPIOC
#define backlight_Pin GPIO_PIN_0
#define backlight_GPIO_Port GPIOA
#define RS_Pin GPIO_PIN_1
#define RS_GPIO_Port GPIOA
#define EN_Pin GPIO_PIN_2
#define EN_GPIO_Port GPIOA
#define DB4_Pin GPIO_PIN_3
#define DB4_GPIO_Port GPIOA
#define DB5_Pin GPIO_PIN_4
#define DB5_GPIO_Port GPIOA
#define DB6_Pin GPIO_PIN_5
#define DB6_GPIO_Port GPIOA
#define DB7_Pin GPIO_PIN_6
#define DB7_GPIO_Port GPIOA
#define STEP1_Pin GPIO_PIN_7
#define STEP1_GPIO_Port GPIOA
#define STEP2_Pin GPIO_PIN_0
#define STEP2_GPIO_Port GPIOB
#define DIR1_Pin GPIO_PIN_1
#define DIR1_GPIO_Port GPIOB
#define DIR2_Pin GPIO_PIN_2
#define DIR2_GPIO_Port GPIOB
#define EN1_Pin GPIO_PIN_10
#define EN1_GPIO_Port GPIOB
#define EN2_Pin GPIO_PIN_11
#define EN2_GPIO_Port GPIOB
#define EndSW1_Pin GPIO_PIN_12
#define EndSW1_GPIO_Port GPIOB
#define EndSW2_Pin GPIO_PIN_13
#define EndSW2_GPIO_Port GPIOB
#define ReachCtrlPnt1_Pin GPIO_PIN_14
#define ReachCtrlPnt1_GPIO_Port GPIOB
#define ReachCtrlPnt2_Pin GPIO_PIN_15
#define ReachCtrlPnt2_GPIO_Port GPIOB
#define Btn_Plus_Pin GPIO_PIN_8
#define Btn_Plus_GPIO_Port GPIOA
#define Btn_Enter_Pin GPIO_PIN_9
#define Btn_Enter_GPIO_Port GPIOA
#define Btn_Minus_Pin GPIO_PIN_10
#define Btn_Minus_GPIO_Port GPIOA
#define Btn_UP_Pin GPIO_PIN_11
#define Btn_UP_GPIO_Port GPIOA
#define Btn_Down_Pin GPIO_PIN_12
#define Btn_Down_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
