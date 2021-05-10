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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define User_Button_Pin GPIO_PIN_13
#define User_Button_GPIO_Port GPIOC
#define Green_LED_Pin GPIO_PIN_0
#define Green_LED_GPIO_Port GPIOB
#define Red_LED_Pin GPIO_PIN_14
#define Red_LED_GPIO_Port GPIOB
#define R3_Pin GPIO_PIN_11
#define R3_GPIO_Port GPIOD
#define R2_Pin GPIO_PIN_12
#define R2_GPIO_Port GPIOD
#define R1_Pin GPIO_PIN_13
#define R1_GPIO_Port GPIOD
#define R4_Pin GPIO_PIN_2
#define R4_GPIO_Port GPIOD
#define BL_Pin GPIO_PIN_3
#define BL_GPIO_Port GPIOD
#define C1_Pin GPIO_PIN_5
#define C1_GPIO_Port GPIOD
#define C2_Pin GPIO_PIN_6
#define C2_GPIO_Port GPIOD
#define C3_Pin GPIO_PIN_7
#define C3_GPIO_Port GPIOD
#define Blue_LED_Pin GPIO_PIN_7
#define Blue_LED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define COLUMNS 3
#define ROWS    5

    uint8_t scanKeypad(void);
    uint8_t findRow(GPIO_TypeDef* gpiox, uint16_t colPin);
    uint8_t getResult(uint8_t row, uint8_t col);
    void disableRow(GPIO_TypeDef* gpiox, uint16_t pin);
    void enableRow(GPIO_TypeDef* gpiox, uint16_t pin);
    void setAllRows(void);
    void resetAllRows(void);
    GPIO_PinState debounceUser(void);
    GPIO_PinState debounce(GPIO_TypeDef* gpiox, uint16_t pin);
    void indicate(uint8_t keyPress);
    void test(uint8_t mode);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
