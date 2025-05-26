/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f2xx_hal.h"

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

ADC_HandleTypeDef* Get_HADC1_Ptr(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define User_LED_2_Pin GPIO_PIN_13
#define User_LED_2_GPIO_Port GPIOC
#define Right_Encoder_1_Pin GPIO_PIN_0
#define Right_Encoder_1_GPIO_Port GPIOA
#define Right_Encoder_2_Pin GPIO_PIN_1
#define Right_Encoder_2_GPIO_Port GPIOA
#define Right_Receiver_Pin GPIO_PIN_2
#define Right_Receiver_GPIO_Port GPIOA
#define Right_Emitter_Pin GPIO_PIN_3
#define Right_Emitter_GPIO_Port GPIOA
#define Right_Front_Emitter_Pin GPIO_PIN_5
#define Right_Front_Emitter_GPIO_Port GPIOA
#define Right_Front_Receiver_Pin GPIO_PIN_6
#define Right_Front_Receiver_GPIO_Port GPIOA
#define Left_Front_Emitter_Pin GPIO_PIN_7
#define Left_Front_Emitter_GPIO_Port GPIOA
#define Left_Front_Receiver_Pin GPIO_PIN_4
#define Left_Front_Receiver_GPIO_Port GPIOC
#define Left_Receiver_Pin GPIO_PIN_1
#define Left_Receiver_GPIO_Port GPIOB
#define Left_Emitter_Pin GPIO_PIN_12
#define Left_Emitter_GPIO_Port GPIOB
#define Driver_Input_1v2_Pin GPIO_PIN_6
#define Driver_Input_1v2_GPIO_Port GPIOC
#define Left_Encoder_1_Pin GPIO_PIN_8
#define Left_Encoder_1_GPIO_Port GPIOA
#define Left_Encoder_2_Pin GPIO_PIN_9
#define Left_Encoder_2_GPIO_Port GPIOA
#define User_LED_1_Pin GPIO_PIN_12
#define User_LED_1_GPIO_Port GPIOC
#define Driver_Input_2_Pin GPIO_PIN_7
#define Driver_Input_2_GPIO_Port GPIOB
#define Driver_Input_3_Pin GPIO_PIN_8
#define Driver_Input_3_GPIO_Port GPIOB
#define Driver_Input_4_Pin GPIO_PIN_9
#define Driver_Input_4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
