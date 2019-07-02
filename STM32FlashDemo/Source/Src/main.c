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
#include "stm32h7xx_nucleo_144.h"
#include "fw_update_api.h"
//#include "flash_ext_drv.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  CMD_NONE = 0,
  CMD_UPDATE,
  CMD_FW_INFO,
  CMD_FW_DATA,
  CMD_NOTIFY,
  CMD_RESET,
  CMD_DUMMY
} cmd_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FW_DATA_PACKAGE_SIZE 512 // Package size sent via UART (bytes)
#define FW_START_ADDR 0x20000 // 128KB Start address of Firmware in FLASH
#define UART_BUFFER_SIZE 1024 // (bytes)

#define RESPONSE_OK "OK"
#define RESPONSE_NG "NG"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

QSPI_HandleTypeDef hqspi;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
volatile uint8_t cmd_received;
uint8_t cmd_buffer[UART_BUFFER_SIZE];
uint32_t firmware_size;
uint8_t firmware_checksum;
volatile uint8_t firmware_data_buffer[FW_DOWNLOAD_BUFFER_SIZE];
int16_t current_qspi_buffer_index;
int16_t next_qspi_buffer_index;
fw_header_t current_firmware_info;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_QUADSPI_Init(void);
/* USER CODE BEGIN PFP */
static void dummy_main(void);
static void command_analyze(void);
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
  MX_USART3_UART_Init();
//  MX_QUADSPI_Init();
  /* USER CODE BEGIN 2 */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  BSP_LED_Init(LED3);
  
  BSP_LED_Off(LED1);
  BSP_LED_Off(LED2);
  BSP_LED_Off(LED3);
  
  
  // AT: DEBUG
//  EXTROM_Init();
//  QSPI_Memmap();
//  while (1) {}
  
//  EXTROM_Init();
//  EXTROM_Erase(0x90000000, 0x10000);
//  
//  uint8_t buffer[4] = {8, 7, 2, 6};
//  EXTROM_Write((uint32_t)buffer, 0x90000000, 4);
//
//  uint8_t rbuffer[4];
//  EXTROM_Read(0x90000000, (uint32_t)rbuffer, 4);
//  
//  for (int i = 0; i < 4; i++) {
//    printf("%02x ", rbuffer[i]);
//  }
//  printf("\r\n");
//  QSPI_Memmap();
//  while (1) {}
  
  // TODO: Init
//  FWUPDATE_InitFlash();
  FWUPDATE_InitQSPI();
//  FWUPDATE_Init();
  
  // CHECK: Read QSPI flash buffer management table
  // QSPI flash buffer index starts from 0, 1, 2, ...
  current_qspi_buffer_index = FWUPDATE_Get_Current_FW_Info(&current_firmware_info);
  
  next_qspi_buffer_index = (current_qspi_buffer_index + 1) % FW_AREA_NUM;

  if (current_qspi_buffer_index < 0) {
    // QSPI flash memory does not have any data
    printf("QSPI flash memory does not have any data\r\n");
  } else {
    printf("Update Firmware from QSPI buffer %d\r\n", current_qspi_buffer_index);

    // TODO: execute on RAM
//    FWUPDATE_Update(current_qspi_buffer_index * FW_AREA_SIZE + sizeof(fw_header_t), FW_START_ADDR, current_firmware_info.size);
  }
  
  // TODO: Set dummy_main at fixed address and the function calling it must be in RAM.
  dummy_main();
  
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

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 2;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 22;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_3_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

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
  * @brief  This is the Dummy Main function.
  * @retval None
  */
void dummy_main(void)
{
  uint32_t start, end;
  uint32_t elapsed_time; // in ms
 
  cmd_received = 0;
  HAL_UART_Receive_IT(&huart3, cmd_buffer, 1);
  
  while (1) {
    // Loop for 300ms
    HAL_Delay(300);
    
    if (cmd_received) {
      cmd_received = 0;
      
      // DEBUG
      start = HAL_GetTick();
      
      // Command analyzer
      command_analyze();
      
      // DEBUG
      end = HAL_GetTick();
      elapsed_time = end - start;
      if (elapsed_time > 200) {
//        printf("Command analyze time: %d ms\r\n", elapsed_time);
        BSP_LED_On(LED3);
      }
    } else {
      // Loop for 200ms
      HAL_Delay(200);
    }
  }
}

/**
  * @brief  This function analyzes the command from UART.
  * @retval None
  */
void command_analyze(void)
{
  static cmd_t previous_cmd = CMD_NONE;
  static uint32_t remain_data_size = 0;
  cmd_t current_cmd = CMD_NONE;
  uint32_t size;
  static uint32_t firmware_data_buffer_len;
  static uint32_t qspi_address = 0;
  char buffer[32];
  FWUPDATE_ERR_CODE ret;
  
  if (previous_cmd == CMD_UPDATE) {
    current_cmd = CMD_FW_INFO;
    // Get Firmware info
    firmware_size = *(uint32_t*)&cmd_buffer[0];
    firmware_checksum = cmd_buffer[4];
    remain_data_size = firmware_size;
    firmware_data_buffer_len = 0;
    
    // DEBUG
    printf("Firmware_size: %d \r\nChecksum: %d\r\n", firmware_size, firmware_checksum);
    
    // Wait for Firmware data
    if (remain_data_size < FW_DATA_PACKAGE_SIZE) {
      size = remain_data_size;
    } else {
      size = FW_DATA_PACKAGE_SIZE;
    }
    HAL_UART_Receive_IT(&huart3, (uint8_t*)firmware_data_buffer, size);
    remain_data_size -= size;
    firmware_data_buffer_len += size;
    
    // CHECK: Calculate qspi_address based on next_qspi_buffer_index
    qspi_address = next_qspi_buffer_index * FW_AREA_SIZE + 4096; //sizeof(fw_header_t);
    
    HAL_UART_Transmit(&huart3, RESPONSE_OK, sizeof(RESPONSE_OK) - 1, 200);
  } else if (previous_cmd == CMD_FW_INFO || previous_cmd == CMD_FW_DATA) {
    if ((remain_data_size == 0) || (firmware_data_buffer_len >= FW_DOWNLOAD_BUFFER_SIZE)) {
      // Write buffer to QSPI Flash
      ret = FWUPDATE_Download(qspi_address, (uint32_t)firmware_data_buffer, firmware_data_buffer_len);
      
      if (ret != FWUPDATE_ERR_OK) {
        HAL_UART_Transmit_IT(&huart3, RESPONSE_NG, sizeof(RESPONSE_NG) - 1);
        
        current_cmd = CMD_NONE;
        previous_cmd = CMD_NONE;
        return;
      }
      
      // Calculate Checksum
      for (uint32_t i = 0; i < firmware_data_buffer_len; i++) {
        firmware_checksum ^= firmware_data_buffer[i];
      }
      printf("Checksum %08x\r\n", firmware_checksum);
      printf("Write %d bytes to QSPI at address 0x%08x\r\n", firmware_data_buffer_len, qspi_address);
      
      qspi_address += firmware_data_buffer_len;
      firmware_data_buffer_len = 0;
    }
    
    if (remain_data_size == 0) {
      // Set current_cmd to CMD_NONE to wait for next command
      current_cmd = CMD_NONE;
      
      // Calculate Checksum
      for (uint32_t i = 0; i < firmware_data_buffer_len; i++) {
        firmware_checksum ^= firmware_data_buffer[i];
      }
      printf("Checksum %08x\r\n", firmware_checksum);
      
      if (firmware_checksum == 0) {
        // CHECK: Update firmware state in QSPI
        fw_header_t fw_info;
        fw_info.size = firmware_size;
        fw_info.state = FWUPDATE_AREA_STATE_DOWNLOADED;
        fw_info.version = (current_firmware_info.version + 1) & 0xFF;
        FWUPDATE_Set_FW_Info(next_qspi_buffer_index, &fw_info);
      }
      
      HAL_UART_Receive_IT(&huart3, cmd_buffer, 1);
    } else {
      current_cmd = CMD_FW_DATA;
      if (remain_data_size < FW_DATA_PACKAGE_SIZE) {
        size = remain_data_size;
      } else {
        size = FW_DATA_PACKAGE_SIZE;
      }
      
      // Calculate Checksum
      for (uint32_t i = 0; i < firmware_data_buffer_len; i++) {
        firmware_checksum ^= firmware_data_buffer[i];
      }
      printf("Checksum %08x\r\n", firmware_checksum);
      
      HAL_UART_Receive_IT(&huart3, (uint8_t*)&firmware_data_buffer[firmware_data_buffer_len], size);
      remain_data_size -= size;
      firmware_data_buffer_len += size;
    }
    
    HAL_UART_Transmit(&huart3, RESPONSE_OK, sizeof(RESPONSE_OK) - 1, 200);
  } else {
    switch (cmd_buffer[0]) {
    case 'u':
      current_cmd = CMD_UPDATE;
      HAL_UART_Transmit_IT(&huart3, RESPONSE_OK, sizeof(RESPONSE_OK) - 1);
      
      // Use LED1 to show which QSPI flash buffer is used for downloading
      // OFF: even buffer, ON: odd buffer
      if (next_qspi_buffer_index % 2) {
        BSP_LED_On(LED1);
      } else {
        BSP_LED_Off(LED1);
      }
      
      // Get Firmware size and checksum
      HAL_UART_Receive_IT(&huart3, cmd_buffer, 5);
      break;
    case 'n':
      current_cmd = CMD_NOTIFY;
      
      if (firmware_checksum == 0) {
        HAL_UART_Transmit_IT(&huart3, RESPONSE_OK, sizeof(RESPONSE_OK) - 1);
      } else {
        HAL_UART_Transmit_IT(&huart3, RESPONSE_NG, sizeof(RESPONSE_NG) - 1);
      }
      
      HAL_UART_Receive_IT(&huart3, cmd_buffer, 1);
      break;
    case 'r':
      current_cmd = CMD_RESET;
      HAL_UART_Transmit(&huart3, RESPONSE_OK, sizeof(RESPONSE_OK) - 1, 1000);
      
      printf("Reset...\r\n");
      // Software Reset
      HAL_NVIC_SystemReset();
      
      while (1) {}
      break;
    case 'd':
      current_cmd = CMD_DUMMY;
      
      // DEBUG
      QSPI_Memmap();
      
      sprintf(buffer, "%02x", current_qspi_buffer_index & 0xFF);
      HAL_UART_Transmit_IT(&huart3, (uint8_t*)buffer, strlen(buffer));
      
      HAL_UART_Receive_IT(&huart3, cmd_buffer, 1);
      break;
      
    // DEBUG
    case 'e':
      printf("Erase Firmware header in QSPI...\r\n");
      for (int i = 0; i < FW_AREA_NUM; i++) {
        EXTROM_Erase(i * FW_AREA_SIZE, sizeof(fw_header_t));
      }
      printf("Done\r\n");
      break;
    }
  }
  previous_cmd = current_cmd;
}

/**
  * @brief Rx Transfer completed callback.
  * @param huart: UART handle.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  cmd_received = 1;
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
