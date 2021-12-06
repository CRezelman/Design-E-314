/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern uint8_t ADC_Buffer[1024];
extern uint8_t Transmit[40960];
extern uint16_t DAC_Buffer[1024];
extern uint8_t i;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Headphones_Pin GPIO_PIN_4
#define Headphones_GPIO_Port GPIOA
#define Record_Pin GPIO_PIN_5
#define Record_GPIO_Port GPIOA
#define Record_EXTI_IRQn EXTI9_5_IRQn
#define Button_3_Pin GPIO_PIN_6
#define Button_3_GPIO_Port GPIOA
#define Button_3_EXTI_IRQn EXTI9_5_IRQn
#define Button_2_Pin GPIO_PIN_7
#define Button_2_GPIO_Port GPIOA
#define Button_2_EXTI_IRQn EXTI9_5_IRQn
#define CS_Pin GPIO_PIN_10
#define CS_GPIO_Port GPIOB
#define LED_2_Pin GPIO_PIN_7
#define LED_2_GPIO_Port GPIOC
#define LED_3_Pin GPIO_PIN_9
#define LED_3_GPIO_Port GPIOA
#define Record_LED_Pin GPIO_PIN_10
#define Record_LED_GPIO_Port GPIOA
#define LED_1_Pin GPIO_PIN_6
#define LED_1_GPIO_Port GPIOB
#define Button_1_Pin GPIO_PIN_8
#define Button_1_GPIO_Port GPIOB
#define Button_1_EXTI_IRQn EXTI9_5_IRQn
#define Stop_Pin GPIO_PIN_9
#define Stop_GPIO_Port GPIOB
#define Stop_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
