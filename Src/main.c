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

typedef enum KeyState { keyReleased, keyPressed } KeyState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */

ColIndex activeColumn = COL_INVAL;
uint8_t activeKey = 0;
uint8_t keyEvent = 0;
uint16_t colPins[COLUMNS] = { COL1_Pin, COL2_Pin, COL3_Pin };
GPIO_TypeDef* colPorts[COLUMNS] = { COL1_GPIO_Port, COL2_GPIO_Port, COL3_GPIO_Port };
uint16_t rowPins[ROWS] = { ROW1_Pin, ROW2_Pin, ROW3_Pin, ROW4_Pin };//, ROW5_Pin };
GPIO_TypeDef* rowPorts[ROWS] = { ROW1_GPIO_Port, ROW2_GPIO_Port, ROW3_GPIO_Port, ROW4_GPIO_Port };//, ROW5_GPIO_Port };
KeyState keyState = keyReleased;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GreenLED_GPIO_Port, GreenLED_Pin, GPIO_PIN_SET);
  setAllRows();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t txBuffer[6] = { 'R', 'e', 's', 'e', 't', '\n' };
  CDC_Transmit_FS(txBuffer, 6);

  txBuffer[2] = 0;
  while (1)
  {
      if (keyEvent != 0)
      {
          keyEvent = 0;

          if (activeKey == 0)
          {
              handleKeypress();

              if (activeKey != 0)
              {
                  txBuffer[0] = activeKey;
                  txBuffer[1] = keyState;
                  CDC_Transmit_FS(txBuffer, 3);
                  //COMM_SendKeyPress(activeKey, keyState);
              }
          }

          if (keyState == keyPressed)
          {
              HAL_TIM_Base_Start_IT(&htim14);
          }
          else
          {
              if (activeKey != 0)
              {
                  GPIO_PinState pinState = debounce();

                  if (pinState == GPIO_PIN_RESET)
                  {
                      txBuffer[0] = activeKey;
                      txBuffer[1] = keyState;
                      CDC_Transmit_FS(txBuffer, 3);
                      //COMM_SendKeyPress(activeKey, keyState);

                      activeKey = 0;
                      activeColumn = COL_INVAL;
                      HAL_GPIO_WritePin(GreenLED_GPIO_Port, GreenLED_Pin, GPIO_PIN_SET);
                      HAL_GPIO_WritePin(IRQ_GPIO_Port, IRQ_Pin, GPIO_PIN_RESET);
                      HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
                      HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
                  }
                  else
                  {
                      keyState = keyPressed;
                      HAL_TIM_Base_Start_IT(&htim14);
                  }
              }
              else
              {
                  activeKey = 0;
                  activeColumn = COL_INVAL;
                  HAL_GPIO_WritePin(GreenLED_GPIO_Port, GreenLED_Pin, GPIO_PIN_SET);
                  HAL_GPIO_WritePin(IRQ_GPIO_Port, IRQ_Pin, GPIO_PIN_RESET);
                  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
                  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
              }
          }
      }
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
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 16;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 42000;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GreenLED_Pin|RedLED_Pin|BlueLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, ROW3_Pin|ROW2_Pin|ROW1_Pin|ROW4_Pin
                          |BL_Pin|TMR_Pin|IRQ_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : COL1_Pin COL2_Pin COL3_Pin */
  GPIO_InitStruct.Pin = COL1_Pin|COL2_Pin|COL3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Pin */
  GPIO_InitStruct.Pin = USER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GreenLED_Pin RedLED_Pin BlueLED_Pin */
  GPIO_InitStruct.Pin = GreenLED_Pin|RedLED_Pin|BlueLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW3_Pin ROW2_Pin ROW1_Pin ROW4_Pin
                           BL_Pin TMR_Pin IRQ_Pin */
  GPIO_InitStruct.Pin = ROW3_Pin|ROW2_Pin|ROW1_Pin|ROW4_Pin
                          |BL_Pin|TMR_Pin|IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim14)
    {
        HAL_TIM_Base_Stop_IT(&htim14);
        HAL_GPIO_WritePin(TMR_GPIO_Port, TMR_Pin, GPIO_PIN_RESET);

        if (activeKey == 0)
        {
            // Ask main loop to handle keypress
            keyEvent = 1;
        }
        else
        {
            GPIO_PinState pinState = HAL_GPIO_ReadPin(colPorts[activeColumn], colPins[activeColumn]);

            if (pinState == GPIO_PIN_SET)
            {
                HAL_GPIO_WritePin(TMR_GPIO_Port, TMR_Pin, GPIO_PIN_SET);
                HAL_TIM_Base_Start_IT(&htim14);
            }
            else // keyReleased
            {
                keyState = keyReleased;
                keyEvent = 1;
            }
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
    HAL_GPIO_WritePin(GreenLED_GPIO_Port, GreenLED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IRQ_GPIO_Port, IRQ_Pin, GPIO_PIN_SET);

    switch (GPIO_Pin)
    {
    case COL1_Pin:
        activeColumn = COL1;
        break;
    case COL2_Pin:
        activeColumn = COL2;
        break;
    case COL3_Pin:
        activeColumn = COL3;
        break;
    case USER_Pin:
        HAL_GPIO_TogglePin(BlueLED_GPIO_Port, BlueLED_Pin);
        break;
    default:
        activeColumn = COL_INVAL;
    }

    if (activeColumn != COL_INVAL)
    {
        HAL_GPIO_WritePin(TMR_GPIO_Port, TMR_Pin, GPIO_PIN_SET);
        HAL_TIM_Base_Start_IT(&htim14);
    }
    else
    {
        HAL_GPIO_WritePin(GreenLED_GPIO_Port, GreenLED_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(IRQ_GPIO_Port, IRQ_Pin, GPIO_PIN_RESET);
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    }
}

void handleKeypress(void)
{
    if (activeColumn != COL_INVAL)
    {
        GPIO_PinState pinState = debounce();
        //GPIO_PinState pinState = HAL_GPIO_ReadPin(colPorts[col], colPins[col]);

        if (pinState == GPIO_PIN_SET)
        {
            // Yes a key has been pressed, find out which one
            activeKey = scanRows();

            if (activeKey != 0)
            {
                keyState = keyPressed;
            }
        }
    }

    if (keyState == keyReleased)
    {
        // Must have been noise
        activeKey = 0;
        activeColumn = 0;
    }    
}

uint8_t scanRows()
{
    uint8_t result = 0;

    //GPIO_PinState pinState = HAL_GPIO_ReadPin(colPorts[col], colPins[col]);
    GPIO_PinState pinState = debounce();

    // activeKey == 0 ignores new key presses while one is being depressed.
    if ((pinState == GPIO_PIN_SET) && (activeKey == 0))
    {
        // Key still pressed after 50 ms, find row to determine which one.
        RowIndex rowIndex = findRow();

        if (rowIndex != ROW_INVAL)
        {
            result = getResult(rowIndex);
        }
    }

    return result;
}

GPIO_PinState debounce()
{
      GPIO_PinState pinState = GPIO_PIN_RESET;

      if (activeColumn != COL_INVAL)
      {
          if (HAL_GPIO_ReadPin(colPorts[activeColumn], colPins[activeColumn]) == GPIO_PIN_SET)
          {
              HAL_Delay(1);
              pinState = HAL_GPIO_ReadPin(colPorts[activeColumn], colPins[activeColumn]);
          }
      }

      return pinState;
}

RowIndex findRow()
{
    RowIndex rowIndex = ROW_INVAL;

    if (activeColumn != COL_INVAL)
    {
        if (readRow(ROW1) == GPIO_PIN_SET)
        {
            rowIndex = ROW1;
        }
        else if (readRow(ROW2) == GPIO_PIN_SET)
        {
            rowIndex = ROW2;
        }
        else if (readRow(ROW3) == GPIO_PIN_SET)
        {
            rowIndex = ROW3;
        }
        else if (readRow(ROW4) == GPIO_PIN_SET)
        {
            rowIndex = ROW4;
        }
        /*else if (readRow(ROW5) == GPIO_PIN_SET)
        {
            rowIndex = ROW5;
            }*/
    }

    return rowIndex;
}

GPIO_PinState readRow(RowIndex rowIndex)
{
    disableAllBut(rowIndex);
    HAL_GPIO_WritePin(rowPorts[rowIndex], rowPins[rowIndex], GPIO_PIN_SET);
    HAL_Delay(1);
    GPIO_PinState pinState = debounce();
    //GPIO_PinState pinState = HAL_GPIO_ReadPin(colPorts[activeColumn], colPins[activeColumn]);
    enableAllRows();
    
    return pinState;
}

uint8_t getResult(RowIndex rowIndex)
{
    uint8_t result = 0;

    switch (rowIndex)
    {
    case ROW1:
        result = (uint8_t)activeColumn + 1;
        break;
    case ROW2:
        result = (uint8_t)activeColumn + 4;
        break;
    case ROW3:
        result = (uint8_t)activeColumn + 7;
        break;
    case ROW4:
        result = (uint8_t)activeColumn + 10;
        break;
    case ROW5:
        result = (uint8_t)activeColumn + 13;
        break;
    default:
        result = 0;
    }

    return result;
}

void disableAllBut(RowIndex rowIndex)
{
    switch (rowIndex)
    {
    case ROW1:
        disableRow(ROW2);
        disableRow(ROW3);
        disableRow(ROW4);
        //disableRow(ROW5);
        break;
    case ROW2:
        disableRow(ROW1);
        disableRow(ROW3);
        disableRow(ROW4);
        //disableRow(ROW5);
        break;
    case ROW3:
        disableRow(ROW1);
        disableRow(ROW2);
        disableRow(ROW4);
        //disableRow(ROW5);
        break;
    case ROW4:
        disableRow(ROW1);
        disableRow(ROW2);
        disableRow(ROW3);
        //disableRow(ROW5);
        break;
    case ROW5:
        disableRow(ROW1);
        disableRow(ROW2);
        disableRow(ROW3);
        disableRow(ROW4);
        break;
    case ROW_INVAL:
        break;
    }
}

void enableAllRows()
{
    enableRow(ROW1);
    enableRow(ROW2);
    enableRow(ROW3);
    enableRow(ROW4);
    //enableRow(ROW5);
}

void disableRow(RowIndex rowIndex)
{
    GPIO_TypeDef* gpiox = rowPorts[rowIndex];
    uint16_t pin = rowPins[rowIndex];
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_GPIO_WritePin(gpiox, pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(gpiox, &GPIO_InitStruct);
}

void enableRow(RowIndex rowIndex)
{
    GPIO_TypeDef* gpiox = rowPorts[rowIndex];
    uint16_t pin = rowPins[rowIndex];
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
    HAL_GPIO_WritePin(ROW1_GPIO_Port, ROW1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ROW2_GPIO_Port, ROW2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ROW3_GPIO_Port, ROW3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ROW4_GPIO_Port, ROW4_Pin, GPIO_PIN_SET);
    //HAL_GPIO_WritePin(ROW5_GPIO_Port, ROW5_Pin, GPIO_PIN_SET);
}

void resetAllRows(void)
{
    HAL_GPIO_WritePin(ROW1_GPIO_Port, ROW1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ROW2_GPIO_Port, ROW2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ROW3_GPIO_Port, ROW3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ROW4_GPIO_Port, ROW4_Pin, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(ROW5_GPIO_Port, ROW5_Pin, GPIO_PIN_RESET);
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
