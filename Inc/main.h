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

    #include "checksum.h"
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
#define COL1_Pin GPIO_PIN_5
#define COL1_GPIO_Port GPIOE
#define COL1_EXTI_IRQn EXTI9_5_IRQn
#define COL2_Pin GPIO_PIN_6
#define COL2_GPIO_Port GPIOE
#define COL2_EXTI_IRQn EXTI9_5_IRQn
#define USER_Pin GPIO_PIN_13
#define USER_GPIO_Port GPIOC
#define USER_EXTI_IRQn EXTI15_10_IRQn
#define GreenLED_Pin GPIO_PIN_0
#define GreenLED_GPIO_Port GPIOB
#define COL3_Pin GPIO_PIN_7
#define COL3_GPIO_Port GPIOE
#define COL3_EXTI_IRQn EXTI9_5_IRQn
#define RedLED_Pin GPIO_PIN_14
#define RedLED_GPIO_Port GPIOB
#define ROW3_Pin GPIO_PIN_11
#define ROW3_GPIO_Port GPIOD
#define ROW2_Pin GPIO_PIN_12
#define ROW2_GPIO_Port GPIOD
#define ROW1_Pin GPIO_PIN_13
#define ROW1_GPIO_Port GPIOD
#define ROW4_Pin GPIO_PIN_2
#define ROW4_GPIO_Port GPIOD
#define BL_Pin GPIO_PIN_3
#define BL_GPIO_Port GPIOD
#define TMR_Pin GPIO_PIN_4
#define TMR_GPIO_Port GPIOD
#define IRQ_Pin GPIO_PIN_5
#define IRQ_GPIO_Port GPIOD
#define BlueLED_Pin GPIO_PIN_7
#define BlueLED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define COLUMNS 3
#define ROWS    4//5
    typedef enum RowIndex { ROW1, ROW2, ROW3, ROW4, ROW5, ROW_INVAL } RowIndex;
    typedef enum ColIndex { COL1, COL2, COL3, COL_INVAL } ColIndex;
    void handleKeypress(void);
    uint8_t scanRows();
    GPIO_PinState debounce(void);
    uint8_t findRow();
    GPIO_PinState readRow(RowIndex rowIndex);
    uint8_t getResult(uint8_t rowPin);
    void disableAllBut(RowIndex rowIndex);
    void enableAllRows();
    void disableRow(RowIndex rowIndex);
    void enableRow(RowIndex rowIndex);
    void setAllRows(void);
    void resetAllRows(void);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
