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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FO2_IN_Pin GPIO_PIN_0
#define FO2_IN_GPIO_Port GPIOA
#define FO1_IN_Pin GPIO_PIN_1
#define FO1_IN_GPIO_Port GPIOA
#define Led_defaut_code_Pin GPIO_PIN_2
#define Led_defaut_code_GPIO_Port GPIOA
#define Led_defaut_ressort_Pin GPIO_PIN_3
#define Led_defaut_ressort_GPIO_Port GPIOA
#define Led_pene_mouvement_Pin GPIO_PIN_4
#define Led_pene_mouvement_GPIO_Port GPIOA
#define Led_pene_IN_Pin GPIO_PIN_5
#define Led_pene_IN_GPIO_Port GPIOA
#define Led_pene_OUT_Pin GPIO_PIN_6
#define Led_pene_OUT_GPIO_Port GPIOA
#define F01_OUT_Pin GPIO_PIN_0
#define F01_OUT_GPIO_Port GPIOB
#define F02_OUT_Pin GPIO_PIN_1
#define F02_OUT_GPIO_Port GPIOB
#define IN2B_Pin GPIO_PIN_8
#define IN2B_GPIO_Port GPIOA
#define IN1B_Pin GPIO_PIN_9
#define IN1B_GPIO_Port GPIOA
#define IN2A_Pin GPIO_PIN_10
#define IN2A_GPIO_Port GPIOA
#define IN1A_Pin GPIO_PIN_11
#define IN1A_GPIO_Port GPIOA
#define ENA_Pin GPIO_PIN_3
#define ENA_GPIO_Port GPIOB
#define ENB_Pin GPIO_PIN_4
#define ENB_GPIO_Port GPIOB
#define Taquet_IN_Pin GPIO_PIN_5
#define Taquet_IN_GPIO_Port GPIOB
#define Taquet_OUT_Pin GPIO_PIN_6
#define Taquet_OUT_GPIO_Port GPIOB
#define Pilote_Pin GPIO_PIN_7
#define Pilote_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
