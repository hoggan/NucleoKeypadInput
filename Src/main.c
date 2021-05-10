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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  GPIO_PinState pinState = GPIO_PIN_RESET;
  GPIO_PinState userButtonState = GPIO_PIN_RESET;
  uint8_t activeLED = 0;
  resetAllRows();

  while (1)
  {
      int userPress = 0;
      pinState = debounceUser();

      if (pinState != userButtonState)
      {
          userButtonState = pinState;

          if (userButtonState == GPIO_PIN_RESET)
	  {
              // button press
              userPress = 1;
              activeLED++;
              activeLED = activeLED % 9;
              //indicate(activeLED);

              /*switch (activeLED)
              {
              case 1:
                  resetAllRows();
                  HAL_GPIO_WritePin(GPIOD, R1_Pin, GPIO_PIN_SET);
                  break;
              case 2:
                  resetAllRows();                  
                  HAL_GPIO_WritePin(GPIOD, R2_Pin, GPIO_PIN_SET);
                  break;
              case 3:
                  resetAllRows();                  
                  HAL_GPIO_WritePin(GPIOD, R3_Pin, GPIO_PIN_SET);
                  break;
              case 4:
                  resetAllRows();                  
                  HAL_GPIO_WritePin(GPIOD, R4_Pin, GPIO_PIN_SET);
                  break;
              default:
                  resetAllRows();
                  }*/
              /*switch (activeLED)
                {
                case 1:
                HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_SET);
                break;
                case 2:
                HAL_GPIO_WritePin(Blue_LED_GPIO_Port, Blue_LED_Pin, GPIO_PIN_SET);
                break;
                case 3:
                HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_SET);
                break;
                default:
                HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(Blue_LED_GPIO_Port, Blue_LED_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_RESET);
                }*/
	  }
      }

      //test(activeLED);

      scanKeypad();
      /*if (userPress == 1)
      {
          //CDC_Transmit_FS("Hello ", 6);
          uint8_t pressedKey = scanKeypad();
          if (pressedKey != 255)
          {
              indicate(pressedKey);
          }
          else
          {
              indicate(0);
          }
          }*/

      HAL_Delay(50);

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Green_LED_Pin|Red_LED_Pin|Blue_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, R3_Pin|R2_Pin|R1_Pin|R4_Pin
                          |BL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : User_Button_Pin */
  GPIO_InitStruct.Pin = User_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(User_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Green_LED_Pin Red_LED_Pin Blue_LED_Pin */
  GPIO_InitStruct.Pin = Green_LED_Pin|Red_LED_Pin|Blue_LED_Pin;
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

  /*Configure GPIO pins : C1_Pin C2_Pin C3_Pin */
  GPIO_InitStruct.Pin = C1_Pin|C2_Pin|C3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

uint8_t scanKeypad(void)
{
    static GPIO_PinState colState[COLUMNS] = { GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET };

    uint8_t result = 255;
    uint16_t colPin[COLUMNS] = { C1_Pin, C2_Pin, C3_Pin };
    GPIO_TypeDef* colPort[COLUMNS] = { C1_GPIO_Port, C2_GPIO_Port, C3_GPIO_Port };

    for (int i = 0; i < COLUMNS; i++)
    {
        setAllRows();
        HAL_Delay(1);
        GPIO_PinState pinState = debounce(colPort[i], colPin[i]);

        if (pinState != colState[i])
        {
            colState[i] = pinState;
            uint8_t row = findRow(colPort[i], colPin[i]);

            if (row != 0)
            {
                uint8_t col = i + 1;

                result = getResult(row,col);
                CDC_Transmit_FS(&result, 1);
                break;
            }
        }
    }

    resetAllRows();

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
    pinState = debounce(colPort, colPin);
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
        pinState = debounce(colPort, colPin);
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
        pinState = debounce(colPort, colPin);
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
        pinState = debounce(colPort, colPin);
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
        result = col + 48;
        break;
    case 2:
        result = col + 51;
        break;
    case 3:
        result = col + 54;
    case 4:
        if (col == 1)
        {
            result = 127;
        }
        else if (col == 2)
        {
            result = 48;
        }
        else if (col == 3)
        {
            result = 13;
        }
    default:
        result = 0;
    }

    return result;
}

void disableRow(GPIO_TypeDef* gpiox, uint16_t pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    uint64_t moder = gpiox->MODER;
    HAL_GPIO_WritePin(gpiox, pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(gpiox, &GPIO_InitStruct);
    moder = gpiox->MODER;
}

void enableRow(GPIO_TypeDef* gpiox, uint16_t pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    uint64_t moder = gpiox->MODER;
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(gpiox, &GPIO_InitStruct);
    moder = gpiox->MODER;
    HAL_GPIO_WritePin(gpiox, pin, GPIO_PIN_SET);
}

void setAllRows(void)
{
    HAL_GPIO_WritePin(GPIOD, R1_Pin | R2_Pin | R3_Pin | R4_Pin, GPIO_PIN_SET);
}

void resetAllRows(void)
{
    HAL_GPIO_WritePin(GPIOD, R1_Pin | R2_Pin | R3_Pin | R4_Pin, GPIO_PIN_RESET);
}

GPIO_PinState debounceUser()
{
      GPIO_PinState pinState = GPIO_PIN_RESET;

      if (HAL_GPIO_ReadPin(User_Button_GPIO_Port, User_Button_Pin) == GPIO_PIN_SET)
      {
          HAL_Delay(1);

          pinState = HAL_GPIO_ReadPin(User_Button_GPIO_Port, User_Button_Pin);
      }

      return pinState;
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

void indicate(uint8_t keypress)
{
    if (keypress != 255)
    {
        if (keypress & 0b001)
        {
            HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_RESET);
        }

        if (keypress & 0b010)
        {
            HAL_GPIO_WritePin(Blue_LED_GPIO_Port, Blue_LED_Pin, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(Blue_LED_GPIO_Port, Blue_LED_Pin, GPIO_PIN_RESET);
        }

        if (keypress & 0b100)
        {
            HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_RESET);
        }
    }
    else
    {
/*        HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Blue_LED_GPIO_Port, Blue_LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_RESET);*/
    }
}

void test(uint8_t mode)
{
    if (mode == 1)
    {
        setAllRows();
        HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, debounce(C1_GPIO_Port, C1_Pin));
        HAL_GPIO_WritePin(Blue_LED_GPIO_Port, Blue_LED_Pin, debounce(C2_GPIO_Port, C2_Pin));
    }
    else if (mode == 2)
    {
        if ((R1_GPIO_Port->MODER & R1_Pin) == 0) HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_SET);
        R1_GPIO_Port->MODER |= R1_Pin;
        HAL_GPIO_ReadPin(R1_GPIO_Port, R1_Pin);
        if (R1_GPIO_Port->MODER & R1_Pin) HAL_GPIO_WritePin(Blue_LED_GPIO_Port, Blue_LED_Pin, GPIO_PIN_SET);
        R2_GPIO_Port->MODER &= ~R1_Pin;
        HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, GPIO_PIN_SET);
        if ((R1_GPIO_Port->MODER & R1_Pin) == 0) HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_SET);
    }
    else if (mode == 3)
    {
        HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Blue_LED_GPIO_Port, Blue_LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_RESET);
        setAllRows();
    }
    else if (mode == 4)
    {
        HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, debounce(C1_GPIO_Port, C1_Pin));
        HAL_GPIO_WritePin(Blue_LED_GPIO_Port, Blue_LED_Pin, debounce(C2_GPIO_Port, C2_Pin));
    }
    else if (mode == 5)
    {
        resetAllRows();
    }
    else if (mode == 6)
    {
        HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, debounce(C1_GPIO_Port, C1_Pin));
        HAL_GPIO_WritePin(Blue_LED_GPIO_Port, Blue_LED_Pin, debounce(C2_GPIO_Port, C2_Pin));
    }
    else if (mode == 7)
    {
        HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Blue_LED_GPIO_Port, Blue_LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_RESET);
    }
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
