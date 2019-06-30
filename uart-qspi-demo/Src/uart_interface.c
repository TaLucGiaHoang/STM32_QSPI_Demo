/**
  ******************************************************************************
  * @file           : uart_interface.c
  * @brief          : 
  ******************************************************************************

  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "uart_interface.h"
#include "fw_update_api.h"

/* Private includes ----------------------------------------------------------*/
#include <string.h>
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
static __IO uint32_t tx_cnt = 0;
static __IO uint32_t rx_cnt = 0;
static __IO uint32_t uartRxCplt = 0;
static __IO uint32_t uartTxCplt = 0;
static uint8_t tx_buf[600];//[1024];
static uint8_t data512byte[512];
static uint8_t msg;
static uint32_t size = 0, checksum = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static inline HAL_StatusTypeDef uart_read(uint8_t *pData, uint16_t size, uint32_t Timeout);
static void Error_Handler(void);


/* Private user code ---------------------------------------------------------*/

/* 
 * Return the number of byte received, otherwise return -1
 */
static inline HAL_StatusTypeDef uart_read(uint8_t *pData, uint16_t size, uint32_t Timeout)
{
  // HAL_StatusTypeDef ret;
  // uartRxCplt = 0;
  // do
  // {
    // ret = HAL_UART_Receive_IT(&huart3, pData, size);
  // } while(uartRxCplt == 0);
  // uartRxCplt = 0;

  __IO int16_t RxXferCount;
  HAL_StatusTypeDef ret;
  int nbytes = 0;
  
  if ((pData == NULL) || (size == 0U))
  {
    return -1;
  }
  
  RxXferCount = size;

  while (RxXferCount > 0U)
  {
    ret = HAL_UART_Receive(&huart3, pData, size, Timeout);
    if (ret == HAL_TIMEOUT)
    {
      return 0;
    }
    
    if (ret == HAL_ERROR)
    {
      return -1;
    }
    
    if (ret == HAL_OK)
    {
      return size;
    }

    RxXferCount -= size;
  }
  return size;
  
  // do 
  // {
    // ret = HAL_UART_Receive(&huart3, pData, size, Timeout);

    // if(ret == HAL_TIMEOUT)
    // {
      // return 0;
    // }
    // if(ret == HAL_ERROR)
    // {
      // return -1;
    // }
    // if(ret == HAL_OK)
    // {
      // return size;
      // break;
    // }
  // } while(1);
  
  // return ret;
}

int uart_read_8bit(uint8_t *data8, uint32_t Timeout)
{
  return uart_read((uint8_t*)data8, 1, Timeout);
}

int uart_read_32bit(uint32_t *data32, uint32_t Timeout)
{
  return uart_read((uint8_t*)data32, 4, Timeout);
}

int uart_read_64bit(uint64_t *data64, uint32_t Timeout)
{
  return uart_read((uint8_t*)data64, 8, Timeout);
}



int uart_read_buffer(uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  uint8_t  *pdata64bytes;
  uint16_t RxXferSize;
  __IO int16_t RxXferCount;
  HAL_StatusTypeDef ret;
  int nbytes = 0;
  
  uint16_t block_size = 64;
    if ((pData == NULL) || (Size == 0U))
    {
      return -1;
      // return  HAL_ERROR;
    }
  
  RxXferSize = Size;
  RxXferCount = Size;
  pdata64bytes = pData;

  /* as long as data have to be received */
  while (RxXferCount > 0U)
  {
    ret = HAL_UART_Receive(&huart3, pdata64bytes, block_size, Timeout);
    if (ret == HAL_TIMEOUT)
    {
      // continue;
      // return HAL_TIMEOUT;
      break;
    }
    
    if (ret == HAL_ERROR)
    {
      return -1;
      // return HAL_ERROR;
    }
    
    pdata64bytes += 64;
    RxXferCount -= 64;
    nbytes += 64;
  }

  // return HAL_OK;
  return nbytes;
}

int uart_read_512byte(uint8_t *pData)
{
  return uart_read_buffer(pData, 512, 1000);
}

void uart_print_msg(uint8_t* msg, uint32_t len)
{
  HAL_UART_Transmit(&huart3, msg, len, 1000);
}

void uart_print_msg_list(void)
{
  char s_msg[100];
  sprintf(s_msg, "FWUPDATE_MSG_NONE 0x%02x\r\n", FWUPDATE_MSG_NONE);
  uart_print_msg(s_msg, strlen(s_msg));
  sprintf(s_msg, "FWUPDATE_MSG_OK 0x%02x\r\n", FWUPDATE_MSG_OK);
  uart_print_msg(s_msg, strlen(s_msg));
  sprintf(s_msg, "FWUPDATE_MSG_NG 0x%02x\r\n", FWUPDATE_MSG_NG);
  uart_print_msg(s_msg, strlen(s_msg));
  sprintf(s_msg, "FWUPDATE_MSG_UPDATE_REQUEST 0x%02x\r\n", FWUPDATE_MSG_UPDATE_REQUEST);
  uart_print_msg(s_msg, strlen(s_msg));
  sprintf(s_msg, "FWUPDATE_MSG_FW_INFO 0x%02x\r\n", FWUPDATE_MSG_FW_INFO);
  uart_print_msg(s_msg, strlen(s_msg));
  sprintf(s_msg, "FWUPDATE_MSG_FW_INFO_DATA 0x%02x\r\n", FWUPDATE_MSG_FW_INFO_DATA);
  uart_print_msg(s_msg, strlen(s_msg));
  sprintf(s_msg, "FWUPDATE_MSG_RESET_REQUEST 0x%02x\r\n", FWUPDATE_MSG_RESET_REQUEST);
  uart_print_msg(s_msg, strlen(s_msg));
  sprintf(s_msg, "FWUPDATE_MSG_QSPI_ERASE 0x%02x\r\n", FWUPDATE_MSG_QSPI_ERASE);
  uart_print_msg(s_msg, strlen(s_msg)); 
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
void uart_init(void)
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
void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
/**
  * @brief Tx Transfer completed callback.
  * @param huart: UART handle.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_TxCpltCallback can be implemented in the user file.
   */
  
  uartTxCplt++;
}

/**
  * @brief Rx Transfer completed callback.
  * @param huart: UART handle.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_RxCpltCallback can be implemented in the user file.
   */
 
  uartRxCplt++;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
static void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

