/**
  ******************************************************************************
  * @file    fw_update_api.c
  * @author  SH Consulting
  * @brief   Dummy main
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "dummy.h"
#include "main.h"
#include "fw_update_api.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum {
  CMD_NONE = 0,
  CMD_UPDATE,
  CMD_FW_INFO,
  CMD_FW_DATA,
  CMD_NOTIFY,
  CMD_RESET,
  CMD_DUMMY
} cmd_t;

/* Private define ------------------------------------------------------------*/
#define DUMMY_TIME 300 // ms
#define FWUPDATE_MAX_TIME 200 // ms

#define RESPONSE_OK "OK"
#define RESPONSE_NG "NG"

#define FW_DATA_PACKAGE_SIZE 512 // Package size sent via UART (bytes)
#define UART_BUFFER_SIZE 1024 // (bytes)
#define FW_CHECKSUM_CHUNK_SIZE 0x10000 // 64KB

/* Private macro -------------------------------------------------------------*/
// #define DEBUG
#if defined(DEBUG)
 #define DEBUG_PRINT(fmt, args...) printf(fmt, ##args)
#else
 #define DEBUG_PRINT(fmt, args...)
#endif

/* Private variables ---------------------------------------------------------*/
uint8_t cmd_buffer[UART_BUFFER_SIZE];
uint32_t firmware_size;
uint8_t firmware_checksum;
uint8_t firmware_data_buffer[FW_DOWNLOAD_BUFFER_SIZE];

/* Extern variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart3;
extern int16_t current_qspi_buffer_index;
extern int16_t next_qspi_buffer_index;
extern fw_header_t current_firmware_info;
extern uint8_t cmd_received;

/* Private function prototypes -----------------------------------------------*/
static uint8_t verify_checksum(uint32_t start, uint32_t length, uint8_t checksum);
static void command_analyze(void);

/**
  * @brief  This is the Dummy Main function.
  * @retval None
  */
void dummy_main(void)
{
  uint32_t start, end;
  uint32_t elapsed_time; // in ms
  
//  DEBUG_PRINT("Firmware V2.0\r\n");
//  HAL_UART_Transmit_IT(&huart3, "V2", 2);
 
  cmd_received = 0;
  HAL_UART_Receive_IT(&huart3, cmd_buffer, 1);
  
  while (1) {
    // Loop for 300ms
    HAL_Delay(DUMMY_TIME);
    
    if (cmd_received) {
      // DEBUG
      start = HAL_GetTick();
      
      // Command analyzer
      command_analyze();
      
      // DEBUG
      end = HAL_GetTick();
      elapsed_time = end - start;
      if (elapsed_time > FWUPDATE_MAX_TIME) {
        DEBUG_PRINT("Command analyze time: %d ms\r\n", elapsed_time);
        Error_Handler();
      }
    } else {
      // Loop for 200ms
      HAL_Delay(FWUPDATE_MAX_TIME);
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
  static uint32_t checksum_start_addr = 0;
  cmd_t current_cmd = CMD_NONE;
  uint32_t size;
  static uint32_t firmware_data_buffer_len;
  static uint32_t qspi_address = 0;
  char buffer[32]; // response buffer
  FWUPDATE_ERR_CODE ret;
  
  if (previous_cmd == CMD_UPDATE) {
    cmd_received = 0;
    current_cmd = CMD_FW_INFO;
    // Get Firmware info
    firmware_size = *(uint32_t*)&cmd_buffer[0];
    firmware_checksum = cmd_buffer[4];
    remain_data_size = firmware_size;
    firmware_data_buffer_len = 0;
    
    // DEBUG
    DEBUG_PRINT("Firmware_size: %d \r\nChecksum: 0x%02x\r\n", firmware_size, firmware_checksum);
    
    // Wait for Firmware data
    if (remain_data_size < FW_DATA_PACKAGE_SIZE) {
      size = remain_data_size;
    } else {
      size = FW_DATA_PACKAGE_SIZE;
    }
    HAL_UART_Receive_IT(&huart3, firmware_data_buffer, size);
    remain_data_size -= size;
    firmware_data_buffer_len += size;
    
    // Calculate qspi_address based on next_qspi_buffer_index
    qspi_address = next_qspi_buffer_index * FW_AREA_SIZE + FW_HEADER_SIZE;
    
    HAL_UART_Transmit(&huart3, RESPONSE_OK, sizeof(RESPONSE_OK) - 1, 200);
  } else if (previous_cmd == CMD_FW_INFO || previous_cmd == CMD_FW_DATA) {
    cmd_received = 0;
    if ((remain_data_size == 0) || (firmware_data_buffer_len >= FW_DOWNLOAD_BUFFER_SIZE)) {
      // Write buffer to QSPI Flash
      ret = FWUPDATE_Download(qspi_address, (uint32_t)firmware_data_buffer, firmware_data_buffer_len);
      
      if (ret != FWUPDATE_ERR_OK) {
        HAL_UART_Transmit_IT(&huart3, RESPONSE_NG, sizeof(RESPONSE_NG) - 1);
        
        current_cmd = CMD_NONE;
        previous_cmd = CMD_NONE;
        return;
      }
      
      DEBUG_PRINT("Write %d bytes to QSPI at address 0x%08x\r\n", firmware_data_buffer_len, qspi_address);
      
      qspi_address += firmware_data_buffer_len;
      firmware_data_buffer_len = 0;
    }
    
    if (remain_data_size == 0) {
      // Set current_cmd to CMD_NONE to wait for next command
      current_cmd = CMD_NONE;
      
      /* Wait for next command */
      HAL_UART_Receive_IT(&huart3, cmd_buffer, 1);
    } else {
      /* Wait for next data buffer */
      current_cmd = CMD_FW_DATA;
      if (remain_data_size < FW_DATA_PACKAGE_SIZE) {
        size = remain_data_size;
      } else {
        size = FW_DATA_PACKAGE_SIZE;
      }
      HAL_UART_Receive_IT(&huart3, &firmware_data_buffer[firmware_data_buffer_len], size);
      remain_data_size -= size;
      firmware_data_buffer_len += size;
    }
    
    HAL_UART_Transmit(&huart3, RESPONSE_OK, sizeof(RESPONSE_OK) - 1, 200);
  } else {
    switch (cmd_buffer[0]) {
    case 'u':
      cmd_received = 0;
      current_cmd = CMD_UPDATE;
      HAL_UART_Transmit_IT(&huart3, RESPONSE_OK, sizeof(RESPONSE_OK) - 1);
      
      checksum_start_addr = 0;
      
      // Use LED1 to show which QSPI flash buffer is used for downloading
      // ON: even buffer, OFF: odd buffer
      if (next_qspi_buffer_index % 2) {
        BSP_LED_Off(LED1);
      } else {
        BSP_LED_On(LED1);
      }
      
      /* Get Firmware size and checksum */
      HAL_UART_Receive_IT(&huart3, cmd_buffer, 5);
      break;
    case 'n':
      current_cmd = CMD_NOTIFY;
      
      if (checksum_start_addr + FW_CHECKSUM_CHUNK_SIZE < firmware_size) {
        size = FW_CHECKSUM_CHUNK_SIZE;
      } else {
        size = firmware_size - checksum_start_addr;
      }
      
      if (size > 0) {
        firmware_checksum = verify_checksum((uint32_t)(next_qspi_buffer_index * FW_AREA_SIZE + FW_HEADER_SIZE) + checksum_start_addr, size, firmware_checksum);
        checksum_start_addr += size;
      } else {
        if (firmware_checksum == 0) 
        {
          // Update firmware state in QSPI
          fw_header_t fw_info;
          fw_info.size = firmware_size;
          fw_info.state = FWUPDATE_AREA_STATE_DOWNLOADED;
          fw_info.version = (current_firmware_info.version + 1) & 0xFF;
          
          DEBUG_PRINT("New downloaded firmware info: size %d, version %d\r\n", fw_info.size, fw_info.version);
          FWUPDATE_Set_FW_Info(next_qspi_buffer_index, &fw_info);
        }
        
        /* Resonse to PC */
        if (firmware_checksum == 0) {
          HAL_UART_Transmit_IT(&huart3, RESPONSE_OK, sizeof(RESPONSE_OK) - 1);
        } else {
          HAL_UART_Transmit_IT(&huart3, RESPONSE_NG, sizeof(RESPONSE_NG) - 1);
        }
        
        cmd_received = 0;
        
        /* Wait for next command */
        HAL_UART_Receive_IT(&huart3, cmd_buffer, 1);
      }
      break;
    case 'r':
      cmd_received = 0;
      current_cmd = CMD_RESET;
      
      /* Resonse to PC */
      HAL_UART_Transmit(&huart3, RESPONSE_OK, sizeof(RESPONSE_OK) - 1, 1000);
      
      DEBUG_PRINT("Reset...\r\n");
      // Software Reset
      HAL_NVIC_SystemReset();
      
      while (1) {}
      break;
    case 'd':
      cmd_received = 0;
      current_cmd = CMD_DUMMY;
      
      /* Resonse to PC */
      sprintf(buffer, "%02x", current_qspi_buffer_index & 0xFF);
      HAL_UART_Transmit_IT(&huart3, (uint8_t*)buffer, strlen(buffer));
      
      /* Wait for next command */
      HAL_UART_Receive_IT(&huart3, cmd_buffer, 1);
      break;
      
    // DEBUG
    case 'e':
      cmd_received = 0;
      DEBUG_PRINT("Erase Firmware header in QSPI...\r\n");
      for (int i = 0; i < FW_AREA_NUM; i++) {
        EXTROM_Erase(i * FW_AREA_SIZE, sizeof(fw_header_t));
      }
      DEBUG_PRINT("Done\r\n");
      
      /* Resonse to PC */
      HAL_UART_Transmit(&huart3, RESPONSE_OK, sizeof(RESPONSE_OK) - 1, 1000);
      
      /* Wait for next command */
      HAL_UART_Receive_IT(&huart3, cmd_buffer, 1);
      break;
    }
    
    
  }
  previous_cmd = current_cmd;
}

/**
  * @brief  This function calculates checksum for downloaded firmware.
  * @retval None
  */
uint8_t verify_checksum(uint32_t start, uint32_t length, uint8_t checksum)
{
  uint8_t qspi_buf[QSPI_PAGE_SIZE];
  uint32_t size;

  while(length > 0)
  {
    if (length > QSPI_PAGE_SIZE) {
      size = QSPI_PAGE_SIZE;
    } else {
      size = length;
    }
    
    if(EXTROM_Read(start, (uint32_t)qspi_buf, size) != FLASH_ERR_OK)
    {
      Error_Handler(); 
    }
    for (uint32_t i = 0; i < size; i++) {
      checksum ^= qspi_buf[i];
    }
    start += size;
    length -= size;
  }
  
  return checksum;
}