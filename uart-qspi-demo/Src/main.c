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
#include "fw_update_api.h"
#include "flash_drv.h"
#include "flash_ext_drv.h"
#include "uart_interface.h"
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32h7xx_nucleo_144.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#pragma section =".qspi"
// #pragma section =".qspi_init"
// #pragma section =".boot_loader"
#pragma section =".main"
#pragma section =".main2"
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

QSPI_HandleTypeDef hqspi;

UART_HandleTypeDef huart3;
/* USER CODE BEGIN PV */



// static __IO uint8_t CmdCplt, RxCplt, TxCplt, StatusMatch;
// __IO uint8_t FlashReturn, FlashErrorReturn;
// static uint8_t ram_data[8];//[4*EXTROM_WRITE_BLOCK_SIZE_256BYTE];
// static uint8_t ram_data_rx[READ_BLOCK_SIZE_64KB];
// uint8_t program_write[64] = "456789ABCDEFGHIKLMNOPQRSTUVW+-*/456789ABCDEFGHIKLMNOPQRSTUVW";  // (256 bits)
uint32_t uwStart, uwEnd;

static __IO int uart_rx_cnt = 0;
static uint8_t uart_rx_buf[512];//[512];  // buffer read from UART
static uint8_t qspi_wr_buf[FW_DOWNLOAD_BUFFER_SIZE];  // buffer write to QSPI (4096)
const uint32_t qspi_buf_size = sizeof(qspi_wr_buf); // (4096)
const uint32_t uart_buf_size = sizeof(uart_rx_buf);  // (512)
#define UART_RX_CNT_MAX  (qspi_buf_size)/(uart_buf_size) // 8
static __IO int32_t program_count = 0;
static uint32_t program_size = 0;
static uint32_t program_addr = 0;

// /* FLASH_EraseProgram */
// uint32_t FirstSector = 0, NbOfSectors = 0;
// uint32_t Address = 0, SECTORError = 0, Index = 0;
// __IO uint32_t MemoryProgramStatus = 0;
// __IO uint64_t data64 = 0;



static __IO uint32_t tx_cnt = 0;
static __IO uint32_t rx_cnt = 0;
static __IO uint32_t uartRxCplt = 0;
static __IO uint32_t uartTxCplt = 0;
// static uint8_t tx_buf[600];//[1024];
// static uint8_t data512byte[512];
static uint8_t msg;
static uint32_t size = 0, checksum = 0;
static char s_msg[100];
// /* 1: process, 0: nothing */
static __IO uint8_t update_flag = 0, download_flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
// static void MX_USART3_UART_Init(void);
static void MX_QUADSPI_Init(void);
/* USER CODE BEGIN PFP */
static void CPU_CACHE_Enable(void);
static void Download_Task(void);
static void Update_Task(void);
static void boot_loader_update(void);
static inline void display_memory(uint32_t address);
static inline void display_qspi_memory(uint32_t address);
static inline void create_write_data(uint8_t* buf, uint32_t buf_size, uint8_t* str, uint32_t str_len);
static void main_old(void);
void main_new(void);
static void BlinkLed(void);

// static inline HAL_StatusTypeDef uart_read(uint8_t *pData, uint32_t size);
// static int uart_read_8bit(uint8_t *data8);
// static int uart_read_32bit(uint32_t *data32);
// static int uart_read_64bit(uint64_t *data64);
// static int uart_read_512byte(uint8_t *pData);

// void uart_print_msg(uint8_t* msg, uint32_t len);
// static void printUartData(uint8_t* pData, uint32_t Size);//test

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
  void (*main_fp)(void);
  main_fp = &main_old;

  // printf("main_fp 0x%08x\n", main_fp);
  boot_loader_update();

  // printf("Start main\n");
  main_fp();
  // printf("Stop main\n");
  
  /* Reset system */
  Reset_Handler();
  
  /* USER CODE END 1 */
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
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
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


void boot_loader_update(void)
{
   
  /* Enable the CPU Cache */
  CPU_CACHE_Enable();
	
  /* STM32H7xx HAL library initialization:

       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  uart_init();
  /* Configure the system clock to 400 MHz */
  SystemClock_Config();
 
  BSP_LED_Init(LED1);
  
  printf("[BootLoader] Initialize, check new firmware\n");
  
  FWUPDATE_InitFlash();
  
  /* Initialize QuadSPI ------------------------------------------------------ */
  if(FWUPDATE_InitQSPI() != FWUPDATE_ERR_OK)
  {
    printf("FWUPDATE_InitQSPI error\n");
    while(1)
    {
    }
  }
  
  FWUPDATE_Init();
  
  if(FWUPDATE_IsNewOS() == 1)
  {
    // uint32_t rom_addr = 0x081D0000; // 0x08160000; // Example Sector 3 Bank 1
    // uint32_t rom_addr = (uint32_t)(0x081E0000);  // empty sector
    uint32_t rom_addr = (uint32_t)(__section_begin(".main"));  
    // uint32_t rom_addr = (uint32_t)(__section_begin(".main2"));

    printf("[BootLoader] Update new main at ROM 0x%08x\n", rom_addr);
    sprintf(s_msg, "[BootLoader] Update new main at ROM 0x%08x\r\n", rom_addr);
    uart_print_msg(s_msg, strlen(s_msg));
    BSP_LED_On(LED1);
    FWUPDATE_Update(0, rom_addr, 0);

    LED2_GPIO_PORT->ODR ^= LED1_PIN;
    
    /* Reset system */
    printf("[BootLoader] Restart\n");
    // HAL_Delay(5000);
    Reset_Handler();
  } else
  {
    printf("[BootLoader] Normal startup\n");
  }

}


static void Download_Task(void)
{
  int ret = 0;
  
  // const uint32_t qspi_buf_size = FW_DOWNLOAD_BUFFER_SIZE;
  
  // /* Receive data from uart */
  // /* Write to QSPI write buffer */
  // create_write_data(qspi_wr_buf, sizeof(qspi_wr_buf), "!@#$%^&*()ABCDQWER9876ABCD<>:][+", 32);
  
  // uint32_t program_size = FW_DOWNLOAD_BUFFER_SIZE*4;
  
  uint32_t program_size = __section_size(".main"); // 0x081e0000, size 124 0x7c
  uint32_t program_addr = (uint32_t)(__section_begin(".main")); // 0x081e0000

  
  // uint32_t program_size = __section_size(".main2"); // 0x081c0000
  // uint32_t program_addr = (uint32_t)(__section_begin(".main2")); // 0x081c0000

  printf("[Download_Task] New firmware address 0x%08x, size %d 0x%x\n", program_addr, program_size, program_size);

  // uint32_t program_count = program_size;
  program_count = program_size;
  
  // if(program_size%qspi_buf_size)
  // {
    // program_size = (program_size/qspi_buf_size)*qspi_buf_size + (((program_size%qspi_buf_size + qspi_buf_size-1)/qspi_buf_size)*qspi_buf_size);
    // printf("[Download_Task] round up program_size %d 0x%x \n", program_size, program_size);
  // }
  
  // FWUPDATE_Download_Config(program_size, qspi_buf_size);

  do
  {
    if(program_count < qspi_buf_size)
    {
      memset(qspi_wr_buf, 0xff, qspi_buf_size); // clearn buffer
      memcpy(qspi_wr_buf, (void*)program_addr, program_count);
    } else 
    {
      ;
    }
    
    ret = FWUPDATE_Download((uint32_t)&qspi_wr_buf[0]); // Write 4096 bytes to QSPI
    
    program_addr  += qspi_buf_size;
    program_count -= qspi_buf_size;
    
    if(ret == FWUPDATE_ERR_OK)
    {
      printf("[Download_Task] OK\n");
      break;
    } else if(ret == FWUPDATE_ERR_BUSY)
    {
      printf("[Download_Task] busy\n");
    } else
    {
      printf("[Download_Task] Error %d\n", ret);
      if(ret == FWUPDATE_ERR_NOT_INITIALIZED) printf("[Download_Task] FWUPDATE_ERR_NOT_INITIALIZED\n", ret);
      if(ret == FWUPDATE_ERR_PARAM) printf("[Download_Task] FWUPDATE_ERR_PARAM\n", ret);
      if(ret == FWUPDATE_ERR_FATAL) printf("[Download_Task] FWUPDATE_ERR_FATAL\n", ret);
      BSP_LED_On(LED3);
      while(1);
    }
  } while(program_count > 0);
}

static void Update_Task(void)
{
  FWUPDATE_Update(EXTROM_AREA_2_ADDRESS, 0x08160000, 4096*4);
}

static void main_old(void) @ ".main"
{
  /* Enable the CPU Cache */
  CPU_CACHE_Enable();
  /* STM32H7xx HAL library initialization:

       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 400 MHz */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  // MX_GPIO_Init();
  // uart_init();
 
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED2);

  char rx_buf[8]; // [1024]
  // char c;
  
  char s[100];
  
  int nbytes = 0;   // bytes received each frame
  int total_bytes = 0;    // total bytes received  

  FWUPDATE_InitFlash();
  
  /* Initialize QuadSPI ------------------------------------------------------ */
  if(FWUPDATE_InitQSPI() != FWUPDATE_ERR_OK)
  {
    printf("FWUPDATE_InitQSPI error\n");
    Error_Handler();
  }

  // FWUPDATE_Init();
  // Download_Task();

  
  uart_print_msg_list();
  
  /* Infinite loop */
  while (1)
  {
    msg = FWUPDATE_MSG_NONE;
    
    
    /* Print message */
    // sprintf(s_msg, "Wait for PC request...\r\n");
    // uart_print_msg(s_msg, strlen(s_msg));
    
    /* Read request */
    uart_read_8bit(&msg);
    
    /* Response received message */
    switch (msg)
    {
      case FWUPDATE_MSG_UPDATE_REQUEST:
      {
        /* Print message */
        sprintf(s_msg, "Start updating...\r\n");
        uart_print_msg(s_msg, strlen(s_msg));
        
        /* Check device conditions */
        
        /* Set update flag */
        // update_flag = 1;

        break;
      }
      case FWUPDATE_MSG_DOWNLOAD_REQUEST:
      {
        /* Print message */
        sprintf(s_msg, "Start downloading...\r\n");
        uart_print_msg(s_msg, strlen(s_msg));
        
        /* Check device conditions */
        
        /* Set download flag */
        download_flag = 1;

        break;
      }
      case FWUPDATE_MSG_QSPI_ERASE:
      {
        /* Print message */
        sprintf(s_msg, "Start clearing all QSPI...\r\n");
        uart_print_msg(s_msg, strlen(s_msg));
        break;
      }
      case FWUPDATE_MSG_FW_INFO:
      case FWUPDATE_MSG_FW_INFO_SIZE:
      case FWUPDATE_MSG_FW_INFO_CHECKSUM:
      case FWUPDATE_MSG_FW_INFO_DATA:
      case FWUPDATE_MSG_RESET_REQUEST:
      case FWUPDATE_MSG_NONE:
      default:
      {
        break;
      }
    }
    
    /* Process message requirement */
    if(msg == FWUPDATE_MSG_DOWNLOAD_REQUEST)
    {
      checksum = 0;
      size = 0;
      nbytes = 0;
      total_bytes = 0;
      uart_rx_cnt = 0;
      program_addr = (uint32_t)qspi_wr_buf;
      program_count = program_size = 0;
      
      /* Response to PC */
      sprintf(s_msg, "OK");
      uart_print_msg(s_msg, strlen(s_msg));
      
      
    }
    
    if(msg == FWUPDATE_MSG_FW_INFO)
    {
      nbytes = 0;
      total_bytes = 0;
      uart_rx_cnt = 0;
      program_addr = (uint32_t)qspi_wr_buf;
      program_count = program_size = 0;
      
      /* Receive size, checksum */
      size = 0;
      checksum = 0;
      uart_read_32bit(&size);
      uart_read_32bit(&checksum);
      
      total_bytes = 0;
      
      /* Response to PC */
      sprintf(s_msg, "OK");
      uart_print_msg(s_msg, strlen(s_msg));
    }
    
    if(msg == FWUPDATE_MSG_FW_INFO_SIZE)
    {
      /* Receive size */
      size = 0;
      uart_read_32bit(&size);
      
      /* Response to PC */
      sprintf(s_msg, "OK");
      uart_print_msg(s_msg, strlen(s_msg));
    }
    
    if(msg == FWUPDATE_MSG_FW_INFO_CHECKSUM)
    {
      /* Receive checksum */
      checksum = 0;
      uart_read_32bit(&checksum);
      
      /* Response to PC */
      sprintf(s_msg, "OK");
      uart_print_msg(s_msg, strlen(s_msg));
    }
    
    if(msg == FWUPDATE_MSG_FW_INFO_DATA)
    {
      /* Receive 512-bytes data */
      nbytes = 0;
      uint8_t* p = qspi_wr_buf;
      
      if(program_count < uart_buf_size)
      {
        memset(uart_rx_buf, 0xff, uart_buf_size);  // Clear uart rx buffer
        nbytes = uart_read_buffer(uart_rx_buf, program_count, 3000);  // read < 512 bytes from UART
      } else 
      {
        nbytes = uart_read_buffer(uart_rx_buf, uart_buf_size, 3000);  // read 512 bytes from UART
      }

      if( (uart_rx_cnt == 0) && (program_count < qspi_buf_size) )
      {
        memset((void*)&qspi_wr_buf, 0xff, qspi_buf_size);  // Clear qspi data
      }
      
      /* Add uart buffer to qspi buffer */
      printf("uart_rx_buf %d  0x%08x\n", uart_rx_cnt, (void*)(p + uart_rx_cnt*uart_buf_size));
      
      memcpy((void*)(p + uart_rx_cnt*uart_buf_size), (void*)uart_rx_buf , uart_buf_size);
      

      uart_rx_cnt++;

      
      if(program_count == (uart_buf_size*uart_rx_cnt) )
      {
        uart_rx_cnt = UART_RX_CNT_MAX;
      }
      
      if(uart_rx_cnt == UART_RX_CNT_MAX)
      {
        uart_rx_cnt = 0;
        /* Copy to QSPI buffer */

        // do {
          if(program_count < qspi_buf_size)
          {
            memset(qspi_wr_buf, 0xff, qspi_buf_size); // clearn buffer
            memcpy(qspi_wr_buf, (void*)program_addr, program_count);
          } else 
          {
            ;
          }
          // int ret = FWUPDATE_Download((uint32_t)&qspi_wr_buf[0]); // Write 4096 bytes to QSPI
          
          program_count -= qspi_buf_size;
          
          // if(ret == FWUPDATE_ERR_OK)
          // {
            // printf("[Download_Task] OK\n");
            // break;
          // } else if(ret == FWUPDATE_ERR_BUSY)
          // {
            // printf("[Download_Task] busy\n");
          // } else
          // {
            // printf("[Download_Task] Error %d\n", ret);
            // if(ret == FWUPDATE_ERR_NOT_INITIALIZED) printf("[Download_Task] FWUPDATE_ERR_NOT_INITIALIZED\n", ret);
            // if(ret == FWUPDATE_ERR_PARAM) printf("[Download_Task] FWUPDATE_ERR_PARAM\n", ret);
            // if(ret == FWUPDATE_ERR_FATAL) printf("[Download_Task] FWUPDATE_ERR_FATAL\n", ret);
            // BSP_LED_On(LED3);
            // while(1);
          // }
        // } while(program_count > 0);
      
      }
      
      
      /* Increase total byte received */
      total_bytes += nbytes;
      
      /* Response to PC */
      sprintf(s_msg, "OK");
      uart_print_msg(s_msg, strlen(s_msg));

    }
    
    /* Process message requirement */
    if(msg == FWUPDATE_MSG_QSPI_ERASE)
    {
      /* Erase 4MB of QSPI Flash - Area 1+2*/  
      // uwStart = HAL_GetTick();
      if(EXTROM_Erase(EXTROM_AREA_1_ADDRESS, 2*EXTROM_AREA_SIZE_2MB) != FLASH_ERR_OK)
      {
        /* Response to PC */
        sprintf(s_msg, "NG");
        uart_print_msg(s_msg, strlen(s_msg))  ; 
        Error_Handler();
      }
      // uwEnd = HAL_GetTick();
      // (uwEnd - uwStart > 1) ? printf("Erase all QSPI: %ld : %ld (%ld ms) \n", uwStart, uwEnd, uwEnd - uwStart) : printf("Erase all QSPI: %ld : %ld ( < 1 ms) \n", uwStart, uwEnd);
    
      /* Response to PC */
      sprintf(s_msg, "OK");
      uart_print_msg(s_msg, strlen(s_msg));
    }
    
    if(msg == FWUPDATE_MSG_RESET_REQUEST)
    {
      /* Check MCU status */
      HAL_Delay(5000); // do nothing
      
      /* Response to PC */
      sprintf(s_msg, "OK");
      uart_print_msg(s_msg, strlen(s_msg));
      
      /* Exit main loop to reset */
      break;
    }
    
    if(size > 0)
    {
      sprintf(s_msg, "size %d 0x%08x\n", size, size);
      uart_print_msg(s_msg, strlen(s_msg));
      
      if(program_size == 0)
      {
        /* Set one time */
        program_count = size;
        program_size = size;
        program_addr = (uint32_t)qspi_wr_buf;

        if(program_size%qspi_buf_size)
        {
          program_size = (program_size/qspi_buf_size)*qspi_buf_size + (((program_size%qspi_buf_size + qspi_buf_size-1)/qspi_buf_size)*qspi_buf_size);
          printf("[Download_Task] round up program_size %d 0x%x \n", program_size, program_size);
        }
      
        /* Configure total program size will be downloaded */
        FWUPDATE_Download_Config(program_size, qspi_buf_size);
      } 
      
    }
    
    if(checksum > 0)
    {
      sprintf(s_msg, "checksum %d 0x%08x\n", checksum, checksum);
      uart_print_msg(s_msg, strlen(s_msg));
    }
    
    if(nbytes != 0)
    {
      sprintf(s_msg, "data received %d\n", nbytes);
      uart_print_msg(s_msg, strlen(s_msg));
    }
  
    if(total_bytes > 0 && total_bytes == (int)size)
    {
      /* Clear update flag */
      download_flag = 0;
      nbytes = 0;
      
      /* Response to PC */
      sprintf(s_msg, "Download completed %d bytes\r\n", total_bytes);
      uart_print_msg(s_msg, strlen(s_msg));
      sprintf(s_msg, "Request to reset\r\n");
      uart_print_msg(s_msg, strlen(s_msg));
    }
  }
  
  /* End of program, reset */
  /* USER CODE END 3 */
  
}

void main_new(void) @ ".main2"
{
  
  /* Enable the CPU Cache */
  CPU_CACHE_Enable();
  HAL_Init();

  BSP_LED_Init(LED1);
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED2);
  
  printf("[main] Initialze\n");
  FWUPDATE_InitFlash();
  
  /* Initialize QuadSPI ------------------------------------------------------ */
  if(FWUPDATE_InitQSPI() != FWUPDATE_ERR_OK)
  {
    printf("FWUPDATE_InitQSPI error\n");
    Error_Handler();
  }
  
  // FWUPDATE_Init();

  Download_Task(); // v2
  
/*     printf("[main_new] New version now\n");
  BSP_LED_On(LED1);
  BSP_LED_On(LED2);
  BSP_LED_On(LED3);
  HAL_Delay(10000);
  LED1_GPIO_PORT->ODR ^= LED1_PIN | LED2_PIN | LED3_PIN;
 */
  printf("\n\nRestart for updating...\n");
  HAL_Delay(5000);  
}

static void display_qspi_memory(uint32_t address)
{
  uint8_t qspi_buf[256];
  
  if((address >= EXTROM_AREA_1_ADDRESS) && (address < EXTROM_AREA_2_ADDRESS + EXTROM_AREA_SIZE_2MB))
  {
    if(EXTROM_Read(address, (uint32_t)&qspi_buf[0], 256) != FLASH_ERR_OK)
    {
      printf("display_qspi_memory error 0x%08x\n", address);
      Error_Handler();
    }
  }  

  uint8_t* p = (uint8_t*)qspi_buf;
  printf("\n");
  for (int i = 0; i < 10; i++)
  {
    printf("%d.[%08x] %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x    %c%c%c%c %c%c%c%c %c%c%c%c %c%c%c%c\n",
             i, address, 
             p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15],
             p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15] );
    p += 16;	
    address += 16;
  }
  printf("\n");
}

static inline void create_write_data(uint8_t* buf, uint32_t buf_size, uint8_t* str, uint32_t str_len)
{
  memset(buf, 'x', buf_size);
  for(int i = 0; i < buf_size; i += str_len)
  {
    memcpy(buf, str, str_len);
    buf += str_len;
  }
}

static inline void display_memory(uint32_t address)
{
  uint8_t* p = (uint8_t*)address;
  printf("\n");
  for (int i = 0; i < 10; i++)
  {
    printf("%d.[%08p] %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x    %c%c%c%c %c%c%c%c %c%c%c%c %c%c%c%c\n",
             i, p, 
             p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15],
             p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15] );
    p += 16;	
  }
  printf("\n");
}

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
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
  while(1);
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
