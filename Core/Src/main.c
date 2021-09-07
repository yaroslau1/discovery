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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

PCD_HandleTypeDef hpcd_USB_FS;

osThreadId LD4_TaskHandle;
osThreadId LD3_TaskHandle;
osThreadId LD5_TaskHandle;
osThreadId LD6_TaskHandle;
osThreadId LD7_TaskHandle;
osThreadId LD8_TaskHandle;
osThreadId LD9_TaskHandle;
osThreadId LD10_TaskHandle;
osThreadId B1_TaskHandle;
osSemaphoreId bigMacHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_PCD_Init(void);
void LD4Task(void const * argument);
void LD3Task(void const * argument);
void LD5Task(void const * argument);
void LD6Task(void const * argument);
void LD7Task(void const * argument);
void LD8Task(void const * argument);
void LD9Task(void const * argument);
void LD10Task(void const * argument);
void B1Task(void const * argument);

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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of bigMac */
  osSemaphoreDef(bigMac);
  bigMacHandle = osSemaphoreCreate(osSemaphore(bigMac), 100);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of LD4_Task */
  osThreadDef(LD4_Task, LD4Task, osPriorityIdle, 0, 128);
  LD4_TaskHandle = osThreadCreate(osThread(LD4_Task), NULL);

  /* definition and creation of LD3_Task */
  osThreadDef(LD3_Task, LD3Task, osPriorityIdle, 0, 128);
  LD3_TaskHandle = osThreadCreate(osThread(LD3_Task), NULL);

  /* definition and creation of LD5_Task */
  osThreadDef(LD5_Task, LD5Task, osPriorityIdle, 0, 128);
  LD5_TaskHandle = osThreadCreate(osThread(LD5_Task), NULL);

  /* definition and creation of LD6_Task */
  osThreadDef(LD6_Task, LD6Task, osPriorityIdle, 0, 128);
  LD6_TaskHandle = osThreadCreate(osThread(LD6_Task), NULL);

  /* definition and creation of LD7_Task */
  osThreadDef(LD7_Task, LD7Task, osPriorityIdle, 0, 128);
  LD7_TaskHandle = osThreadCreate(osThread(LD7_Task), NULL);

  /* definition and creation of LD8_Task */
  osThreadDef(LD8_Task, LD8Task, osPriorityIdle, 0, 128);
  LD8_TaskHandle = osThreadCreate(osThread(LD8_Task), NULL);

  /* definition and creation of LD9_Task */
  osThreadDef(LD9_Task, LD9Task, osPriorityIdle, 0, 128);
  LD9_TaskHandle = osThreadCreate(osThread(LD9_Task), NULL);

  /* definition and creation of LD10_Task */
  osThreadDef(LD10_Task, LD10Task, osPriorityIdle, 0, 128);
  LD10_TaskHandle = osThreadCreate(osThread(LD10_Task), NULL);

  /* definition and creation of B1_Task */
  osThreadDef(B1_Task, B1Task, osPriorityIdle, 0, 128);
  B1_TaskHandle = osThreadCreate(osThread(B1_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

////		if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET) {
////			xSemaphoreGive(bigMacHandle);
////			HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
////		}
//		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
////		vTaskDelay(200);
//		HAL_Delay(200);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_LD4Task */
/**
 * @brief  Function implementing the LD4_Task thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_LD4Task */
void LD4Task(void const * argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t semaphore;
	/* Infinite loop */
	for (;;) {
		semaphore = uxSemaphoreGetCount(bigMacHandle);
		if (semaphore > 0) {
			HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
			xSemaphoreTake(bigMacHandle, portMAX_DELAY);
		}
		osDelay(100);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_LD3Task */
/**
 * @brief Function implementing the LD3_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_LD3Task */
void LD3Task(void const * argument)
{
  /* USER CODE BEGIN LD3Task */
	/* Infinite loop */
	for (;;) {
		if (uxSemaphoreGetCount(bigMacHandle) > 0) {
			HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
			xSemaphoreTake(bigMacHandle, portMAX_DELAY);
		}
		osDelay(200);
	}
  /* USER CODE END LD3Task */
}

/* USER CODE BEGIN Header_LD5Task */
/**
 * @brief Function implementing the LD5_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_LD5Task */
void LD5Task(void const * argument)
{
  /* USER CODE BEGIN LD5Task */
	/* Infinite loop */
	for (;;) {
		if (uxSemaphoreGetCount(bigMacHandle) > 0) {
			HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
			xSemaphoreTake(bigMacHandle, portMAX_DELAY);
		}
		osDelay(300);
	}
  /* USER CODE END LD5Task */
}

/* USER CODE BEGIN Header_LD6Task */
/**
 * @brief Function implementing the LD6_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_LD6Task */
void LD6Task(void const * argument)
{
  /* USER CODE BEGIN LD6Task */
	/* Infinite loop */
	for (;;) {
		if (uxSemaphoreGetCount(bigMacHandle) > 0) {
			HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
			xSemaphoreTake(bigMacHandle, portMAX_DELAY);
		}
		osDelay(400);
	}
  /* USER CODE END LD6Task */
}

/* USER CODE BEGIN Header_LD7Task */
/**
 * @brief Function implementing the LD7_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_LD7Task */
void LD7Task(void const * argument)
{
  /* USER CODE BEGIN LD7Task */
	/* Infinite loop */
	for (;;) {
		if (uxSemaphoreGetCount(bigMacHandle) > 0) {
			HAL_GPIO_TogglePin(LD7_GPIO_Port, LD7_Pin);
			xSemaphoreTake(bigMacHandle, portMAX_DELAY);
		}
		osDelay(500);
	}
  /* USER CODE END LD7Task */
}

/* USER CODE BEGIN Header_LD8Task */
/**
 * @brief Function implementing the LD8_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_LD8Task */
void LD8Task(void const * argument)
{
  /* USER CODE BEGIN LD8Task */
	/* Infinite loop */
	for (;;) {
		if (uxSemaphoreGetCount(bigMacHandle) > 0) {
			HAL_GPIO_TogglePin(LD8_GPIO_Port, LD8_Pin);
			xSemaphoreTake(bigMacHandle, portMAX_DELAY);
		}
		osDelay(600);
	}
  /* USER CODE END LD8Task */
}

/* USER CODE BEGIN Header_LD9Task */
/**
 * @brief Function implementing the LD9_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_LD9Task */
void LD9Task(void const * argument)
{
  /* USER CODE BEGIN LD9Task */
	/* Infinite loop */
	for (;;) {
		if (uxSemaphoreGetCount(bigMacHandle) > 0) {
			HAL_GPIO_TogglePin(LD9_GPIO_Port, LD9_Pin);
			xSemaphoreTake(bigMacHandle, portMAX_DELAY);
		}
		osDelay(700);
	}
  /* USER CODE END LD9Task */
}

/* USER CODE BEGIN Header_LD10Task */
/**
 * @brief Function implementing the LD10_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_LD10Task */
void LD10Task(void const * argument)
{
  /* USER CODE BEGIN LD10Task */
	/* Infinite loop */
	for (;;) {
		if (uxSemaphoreGetCount(bigMacHandle) > 0) {
			HAL_GPIO_TogglePin(LD10_GPIO_Port, LD10_Pin);
			xSemaphoreTake(bigMacHandle, portMAX_DELAY);
		}
		osDelay(800);
	}
  /* USER CODE END LD10Task */
}

/* USER CODE BEGIN Header_B1Task */
/**
 * @brief Function implementing the B1_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_B1Task */
void B1Task(void const * argument)
{
  /* USER CODE BEGIN B1Task */
	uint8_t semaphore;
	/* Infinite loop */
	for (;;) {
		if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET) {
			semaphore = uxSemaphoreGetCount(bigMacHandle);
			xSemaphoreGive(bigMacHandle);
//			HAL_GPIO_TogglePin(LD10_GPIO_Port, LD10_Pin);
		}
//		HAL_GPIO_TogglePin(LD9_GPIO_Port, LD9_Pin);
		osDelay(10);
	}
  /* USER CODE END B1Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
