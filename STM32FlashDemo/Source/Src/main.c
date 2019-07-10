/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "fw_update_api.h"
#include "dummy.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FW_START_ADDR 0x08020000 // Start address of Firmware in FLASH (128KB)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#if defined(DEBUG)
 #define DEBUG_PRINT(fmt, args...) printf(fmt, ##args)
#else
 #define DEBUG_PRINT(fmt, args...)
#endif
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

QSPI_HandleTypeDef hqspi;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t cmd_received;
int16_t current_qspi_buffer_index;
int16_t next_qspi_buffer_index;
fw_header_t current_firmware_info;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
//static void MX_QUADSPI_Init(void);
/* USER CODE BEGIN PFP */
void code_copy(void);
void update_firmware(void) __attribute__ ((section (".ramfunc")));
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
  // Copy object code from ROM to RAM for execution on RAM
  code_copy();
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
  MX_USART3_UART_Init();
//  MX_QUADSPI_Init();
  /* USER CODE BEGIN 2 */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  BSP_LED_Init(LED3);
  
  BSP_LED_Off(LED1);
  BSP_LED_Off(LED2);
  BSP_LED_Off(LED3);
  
  
  // Initilization
  FWUPDATE_InitFlash();
  FWUPDATE_InitQSPI();
  FWUPDATE_Init();
  
  // Read QSPI flash buffer management table
  // QSPI flash buffer index starts from 0, 1, 2, ...
  current_qspi_buffer_index = FWUPDATE_Get_Current_FW_Info(&current_firmware_info);
  
  next_qspi_buffer_index = (current_qspi_buffer_index + 1) % FW_AREA_NUM;

  if (current_qspi_buffer_index < 0) {
    // QSPI flash memory does not have any data
    DEBUG_PRINT("QSPI flash memory does not have any data\r\n");
    
    dummy_main();
  } else {
    update_firmware();
  }
  
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable 
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_QSPI;
  PeriphClkInitStruct.QspiClockSelection = RCC_QSPICLKSOURCE_D1HCLK;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

///**
//  * @brief QUADSPI Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_QUADSPI_Init(void)
//{
//
//  /* USER CODE BEGIN QUADSPI_Init 0 */
//
//  /* USER CODE END QUADSPI_Init 0 */
//
//  /* USER CODE BEGIN QUADSPI_Init 1 */
//
//  /* USER CODE END QUADSPI_Init 1 */
//  /* QUADSPI parameter configuration*/
//  hqspi.Instance = QUADSPI;
//  hqspi.Init.ClockPrescaler = 2;
//  hqspi.Init.FifoThreshold = 4;
//  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
//  hqspi.Init.FlashSize = 22;
//  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_3_CYCLE;
//  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
//  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
//  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
//  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN QUADSPI_Init 2 */
//
//  /* USER CODE END QUADSPI_Init 2 */
//
//}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/**
  * @brief Rx Transfer completed callback.
  * @param huart: UART handle.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  cmd_received = 1;
}

/**
  * @brief Copy object code from ROM to RAM for execution on RAM.
  * @retval None
  */
#pragma section = ".ramobj"
#pragma section = ".ramobj_init"
#pragma section = ".ramfunc"
#pragma section = ".ramfunc_init"
void code_copy(void)
{
  uint8_t * from;
  uint8_t * to;
  
  __disable_irq();
  
  /* Copy object code from ROM to RAM for execution on RAM */
  from = __section_begin(".ramobj_init");
  to   = __section_begin(".ramobj");
  memcpy(to, from, __section_size(".ramobj_init"));
  
  from = __section_begin(".ramfunc_init");
  to   = __section_begin(".ramfunc");
  memcpy(to, from, __section_size(".ramfunc_init"));
  
  __enable_irq();
}

void update_firmware(void)
{
  uint32_t qspi_address;
  qspi_address = current_qspi_buffer_index * FW_AREA_SIZE + FW_HEADER_SIZE;
  DEBUG_PRINT("Update Firmware from QSPI buffer %d\r\n", current_qspi_buffer_index);
  DEBUG_PRINT("QSPI address: 0x%08x\r\nFLASH Address: 0x%08x\r\nSize: %d bytes\r\n", qspi_address, FW_START_ADDR, current_firmware_info.size); 

  FWUPDATE_ERR_CODE ret;
  BSP_LED_On(LED2);
  ret = FWUPDATE_Update(qspi_address, FW_START_ADDR, current_firmware_info.size);
  BSP_LED_Off(LED2);
  if (ret == FWUPDATE_ERR_OK) {
    DEBUG_PRINT("Firmware Updated\r\n");
    current_firmware_info.state = FWUPDATE_AREA_STATE_VALID; // FWUPDATE_AREA_STATE_UPDATING? for rollback?
    FWUPDATE_Set_FW_Info(current_qspi_buffer_index, &current_firmware_info);
  } else {
    DEBUG_PRINT("Firmware Update Failed\r\n");
  }
  
  dummy_main();
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
  BSP_LED_On(LED3);
  while (1) {}
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
