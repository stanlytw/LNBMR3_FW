/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f7xx_hal.h"

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
#define USE_HAL_TIM_REGISTER_CALLBACKS 1U
#define USE_HAL_SPI_REGISTER_CALLBACKS 0U 
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TMP_ALM_Pin GPIO_PIN_12
#define TMP_ALM_GPIO_Port GPIOG
#define D5_Pin GPIO_PIN_4
#define D5_GPIO_Port GPIOE
#define IAP_DL_Pin GPIO_PIN_3
#define IAP_DL_GPIO_Port GPIOF
#define INCB_Pin GPIO_PIN_2
#define INCB_GPIO_Port GPIOG
#define INCA_Pin GPIO_PIN_14
#define INCA_GPIO_Port GPIOD
#define INCZ_Pin GPIO_PIN_15
#define INCZ_GPIO_Port GPIOD
#define LNB_OK_Pin GPIO_PIN_11
#define LNB_OK_GPIO_Port GPIOD
#define SSI_NSL_Pin GPIO_PIN_8
#define SSI_NSL_GPIO_Port GPIOD
#define VM_VP_Pin GPIO_PIN_2
#define VM_VP_GPIO_Port GPIOA
#define MR3_RST_Pin GPIO_PIN_9
#define MR3_RST_GPIO_Port GPIOD
#define CS1_Pin GPIO_PIN_8
#define CS1_GPIO_Port GPIOE
#define CS2_Pin GPIO_PIN_4
#define CS2_GPIO_Port GPIOA
#define MR3_NL_Pin GPIO_PIN_3
#define MR3_NL_GPIO_Port GPIOA
#define SSI_CLK_Pin GPIO_PIN_13
#define SSI_CLK_GPIO_Port GPIOB
#define SSI_CLK_EXTI_IRQn EXTI15_10_IRQn
#define CS2_V1_Pin GPIO_PIN_9
#define CS2_V1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */




/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
