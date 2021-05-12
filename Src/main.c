/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t activeColumn = 0;
uint8_t activeKey = 0;
uint8_t txBuffer[11];
uint16_t colPin[COLUMNS] = { C1_Pin, C2_Pin, C3_Pin };
GPIO_TypeDef* colPort[COLUMNS] = { C1_GPIO_Port, C2_GPIO_Port, C3_GPIO_Port };

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  setAllRows();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 13;
  RCC_OscInitStruct.PLL.PLLN = 195;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GreenLED_Pin|RedLED_Pin|BlueLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, R3_Pin|R2_Pin|R1_Pin|R4_Pin
                          |BL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : C1_Pin C2_Pin C3_Pin */
  GPIO_InitStruct.Pin = C1_Pin|C2_Pin|C3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : GreenLED_Pin RedLED_Pin BlueLED_Pin */
  GPIO_InitStruct.Pin = GreenLED_Pin|RedLED_Pin|BlueLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : R3_Pin R2_Pin R1_Pin R4_Pin
                           BL_Pin */
  GPIO_InitStruct.Pin = R3_Pin|R2_Pin|R1_Pin|R4_Pin
                          |BL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    HAL_TIM_Base_Stop_IT(&htim1);

    // CMD
    txBuffer[0] = 1;
    // Length
    txBuffer[1] = 0;
    txBuffer[2] = 0;
    txBuffer[3] = 0;
    txBuffer[4] = 6;
    // CRC32
    txBuffer[7] = 0;
    txBuffer[8] = 0;
    txBuffer[9] = 0;
    txBuffer[10] = 0;

    GPIO_PinState pinState = HAL_GPIO_ReadPin(colPort[activeColumn], colPin[activeColumn]);

    if (activeKey == 0)
    {
        // Key just pressed
        if (pinState == GPIO_PIN_SET)
        {
            activeKey = scanRows();

            if (activeKey != 0)
            {
                txBuffer[5] = activeKey;
                txBuffer[6] = 1;
                *(uint32_t *)&txBuffer[7] = crc_32(txBuffer, 7);
                CDC_Transmit_FS(txBuffer, 11);
                HAL_TIM_Base_Start_IT(&htim1);
            }
        }
    }
    else if (pinState == GPIO_PIN_SET)
    {
        // Key previously pressed and is still pressed.
        HAL_TIM_Base_Start_IT(&htim1);
    }
    else
    {
        // Key previously pressed and has been released.
        txBuffer[5] = activeKey;
        txBuffer[6] = 0;
        activeKey = 0;
        *(uint32_t *)&txBuffer[7] = crc_32(txBuffer, 7);
        CDC_Transmit_FS(txBuffer, 11);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
    case C1_Pin:
        HAL_GPIO_TogglePin(GreenLED_GPIO_Port, GreenLED_Pin);
        activeColumn = 1;
        break;
    case C2_Pin:
        HAL_GPIO_TogglePin(BlueLED_GPIO_Port, BlueLED_Pin);
        activeColumn = 2;
        break;
    case C3_Pin:
        HAL_GPIO_TogglePin(RedLED_GPIO_Port, RedLED_Pin);
        activeColumn = 3;
        break;
    default:
        __NOP();
    }

    if (activeColumn)
    {
        HAL_TIM_Base_Start_IT(&htim1);
    }
}

uint8_t scanRows(void)
{
    uint8_t result = 0;

    if (activeColumn != 0)
    {
    }
    //setAllRows();
    HAL_Delay(1);

    uint8_t i = 
    GPIO_PinState pinState = HAL_GPIO_ReadPin(colPort[activeColumn], colPin[activeColumn]);

    if ((pinState == GPIO_PIN_SET) && (activeKey == 0))
    {
        uint8_t row = findRow(colPort[activeColumn], colPin[activeColumn]);

        if (row != 0)
        {
            uint8_t col = i + 1;

            result = getResult(row,col);
            break;
        }
    }

    //resetAllRows();

    return result;
}

uint8_t findRow(GPIO_TypeDef* colPort, uint16_t colPin)
{
    uint8_t row = 0;
    GPIO_PinState pinState = GPIO_PIN_RESET;

    disableRow(R2_GPIO_Port, R2_Pin);
    disableRow(R3_GPIO_Port, R3_Pin);
    disableRow(R4_GPIO_Port, R4_Pin);
    HAL_Delay(1);
    pinState = HAL_GPIO_ReadPin(colPort, colPin);
    enableRow(R2_GPIO_Port, R2_Pin);
    enableRow(R3_GPIO_Port, R3_Pin);
    enableRow(R4_GPIO_Port, R4_Pin);

    if (pinState == GPIO_PIN_SET)
    {
        row = 1;
    }

    if (pinState == GPIO_PIN_RESET)
    {
        disableRow(R1_GPIO_Port, R1_Pin);
        disableRow(R3_GPIO_Port, R3_Pin);
        disableRow(R4_GPIO_Port, R4_Pin);
        HAL_Delay(1);
        pinState = HAL_GPIO_ReadPin(colPort, colPin);
        enableRow(R1_GPIO_Port, R1_Pin);
        enableRow(R3_GPIO_Port, R3_Pin);
        enableRow(R4_GPIO_Port, R4_Pin);

        if (pinState == GPIO_PIN_SET)
        {
            row = 2;
        }   
    }

    if (pinState == GPIO_PIN_RESET)
    {
        disableRow(R1_GPIO_Port, R1_Pin);
        disableRow(R2_GPIO_Port, R2_Pin);
        disableRow(R4_GPIO_Port, R4_Pin);
        HAL_Delay(1);
        pinState = HAL_GPIO_ReadPin(colPort, colPin);
        enableRow(R1_GPIO_Port, R1_Pin);
        enableRow(R2_GPIO_Port, R2_Pin);
        enableRow(R4_GPIO_Port, R4_Pin);

        if (pinState == GPIO_PIN_SET)
        {
            row = 3;
        }
    }

    if (pinState == GPIO_PIN_RESET)
    {
        disableRow(R1_GPIO_Port, R1_Pin);
        disableRow(R2_GPIO_Port, R2_Pin);
        disableRow(R3_GPIO_Port, R3_Pin);
        HAL_Delay(1);
        pinState = HAL_GPIO_ReadPin(colPort, colPin);
        enableRow(R1_GPIO_Port, R1_Pin);
        enableRow(R2_GPIO_Port, R2_Pin);
        enableRow(R3_GPIO_Port, R3_Pin);

        if (pinState == GPIO_PIN_SET)
        {
            row = 4;
        }
    }

    return row;
}

uint8_t getResult(uint8_t row, uint8_t col)
{
    uint8_t result = 0;

    switch (row)
    {
    case 1:
        result = col;
        break;
    case 2:
        result = col + 3;
        break;
    case 3:
        result = col + 6;
        break;
    case 4:
        result = col + 9;
        break;
    default:
        result = 0;
    }

    return result;
}

void disableRow(GPIO_TypeDef* gpiox, uint16_t pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_GPIO_WritePin(gpiox, pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(gpiox, &GPIO_InitStruct);
}

void enableRow(GPIO_TypeDef* gpiox, uint16_t pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(gpiox, &GPIO_InitStruct);
    HAL_GPIO_WritePin(gpiox, pin, GPIO_PIN_SET);
}

void setAllRows(void)
{
    HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, GPIO_PIN_SET);
}

void resetAllRows(void)
{
    HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, GPIO_PIN_RESET);
}

GPIO_PinState debounce(GPIO_TypeDef* gpiox, uint16_t pin)
{
      GPIO_PinState pinState = GPIO_PIN_RESET;

      if (HAL_GPIO_ReadPin(gpiox, pin) == GPIO_PIN_SET)
      {
          HAL_Delay(1);
          pinState = HAL_GPIO_ReadPin(gpiox, pin);
      }

      return pinState;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
