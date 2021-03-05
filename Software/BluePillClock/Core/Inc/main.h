/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#define BP_LED_Pin GPIO_PIN_13
#define BP_LED_GPIO_Port GPIOC
#define BP_06_Pin GPIO_PIN_0
#define BP_06_GPIO_Port GPIOA
#define BP_10_Pin GPIO_PIN_1
#define BP_10_GPIO_Port GPIOA
#define BP_05_Pin GPIO_PIN_2
#define BP_05_GPIO_Port GPIOA
#define BP_01_Pin GPIO_PIN_3
#define BP_01_GPIO_Port GPIOA
#define BP_00_Pin GPIO_PIN_4
#define BP_00_GPIO_Port GPIOA
#define BP_04_Pin GPIO_PIN_5
#define BP_04_GPIO_Port GPIOA
#define BP_09_Pin GPIO_PIN_6
#define BP_09_GPIO_Port GPIOA
#define BP_08_Pin GPIO_PIN_7
#define BP_08_GPIO_Port GPIOA
#define BP_B_Pin GPIO_PIN_0
#define BP_B_GPIO_Port GPIOB
#define BP_F_Pin GPIO_PIN_1
#define BP_F_GPIO_Port GPIOB
#define BP_BOOT1_Pin GPIO_PIN_2
#define BP_BOOT1_GPIO_Port GPIOB
#define BP_G_Pin GPIO_PIN_10
#define BP_G_GPIO_Port GPIOB
#define BP_A_Pin GPIO_PIN_11
#define BP_A_GPIO_Port GPIOB
#define BP_E_Pin GPIO_PIN_12
#define BP_E_GPIO_Port GPIOB
#define BP_D_Pin GPIO_PIN_13
#define BP_D_GPIO_Port GPIOB
#define BP_C_Pin GPIO_PIN_14
#define BP_C_GPIO_Port GPIOB
#define BP_P_Pin GPIO_PIN_15
#define BP_P_GPIO_Port GPIOB
#define BP_SW1_Pin GPIO_PIN_8
#define BP_SW1_GPIO_Port GPIOA
#define BP_SW2_Pin GPIO_PIN_9
#define BP_SW2_GPIO_Port GPIOA
#define BP_NOTUSED_2_Pin GPIO_PIN_10
#define BP_NOTUSED_2_GPIO_Port GPIOA
#define BP_NOTUSED_1_Pin GPIO_PIN_15
#define BP_NOTUSED_1_GPIO_Port GPIOA
#define BP_11_Pin GPIO_PIN_3
#define BP_11_GPIO_Port GPIOB
#define BP_02_Pin GPIO_PIN_4
#define BP_02_GPIO_Port GPIOB
#define BP_07_Pin GPIO_PIN_5
#define BP_07_GPIO_Port GPIOB
#define BP_12_Pin GPIO_PIN_6
#define BP_12_GPIO_Port GPIOB
#define BP_03_Pin GPIO_PIN_7
#define BP_03_GPIO_Port GPIOB
#define BP_13_Pin GPIO_PIN_8
#define BP_13_GPIO_Port GPIOB
#define BP_NOTUSED_0_Pin GPIO_PIN_9
#define BP_NOTUSED_0_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
