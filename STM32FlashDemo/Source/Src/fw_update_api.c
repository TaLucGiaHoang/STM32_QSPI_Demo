/**
  ******************************************************************************
  * @file    fw_update_api.c
  * @author  SH Consulting
  * @brief   
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "fw_update_api.h"
#include <stdio.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

// #define DEBUG
#if defined(DEBUG)
 #define DEBUG_PRINT(fmt, args...) printf(fmt, ##args)
#else
 #define DEBUG_PRINT(fmt, args...)
#endif

/* Private macro -------------------------------------------------------------*/

/* External variables ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint8_t qspi_buf[FW_DOWNLOAD_BUFFER_SIZE];

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  Initialize HW so that the internal flash memory can be accessed
  *         FWUPDATE_InitFlash() just call FLASH_Init() of Flash Memory Access 
  *         Module.
  * @param  None
  * @retval FWUPDATE_ERR_OK     Successful initialization of HW of internal 
  *                             flash memory
  *         FWUPDATE_ERR_FATAL  Failed to initialize HW of internal flash memory
  */
FWUPDATE_ERR_CODE FWUPDATE_InitFlash(void)
{
  if(FLASH_Init() != FLASH_ERR_OK)
  {
    return FWUPDATE_ERR_FATAL;
  }
  return FWUPDATE_ERR_OK;
}

/**
  * @brief  Initialize HW to access QSPI flash memory.
  *         FWUPDATE_InitQSPI() just call EXTROM_Init() of Flash Memory Access 
  *         Module.
  * @param  None
  * @retval FWUPDATE_ERR_OK     Succeeded in initializing QSPI HW
  *         FWUPDATE_ERR_FATAL  Failed to initialize QSPI HW
  */
FWUPDATE_ERR_CODE FWUPDATE_InitQSPI(void)
{
  if(EXTROM_Init() != FLASH_ERR_OK)
  {
    return FWUPDATE_ERR_FATAL;
  }
  return FWUPDATE_ERR_OK;
}

/**
  * @brief  Start the firmware update task
  *         (In this development, there is no RTOS. So actually no Task. So, 
  *         this time, FWUPDATE_Init() just initialize the internal flags, 
  *         states, ... of "Firmware Update” module.
  * @param  None
  * @retval FWUPDATE_ERR_OK     Started the FW Update task successfully
  *         FWUPDATE_ERR_NOT_INITIALIZED  Flash access module has not been
  *         initialized (It mean, FWUPDATE_InitQSPI() or FWUPDATE_InitFlash () 
  *         has not been called.)
  *         FWUPDATE_ERR_ALREADY_INITIALIZED  FW Update task has already been 
  *         started
  */
FWUPDATE_ERR_CODE FWUPDATE_Init(void)
{
  return FWUPDATE_ERR_OK;
}  

/**
  * @brief  Execute FW download process. The FW data received from the UART is
  *         saved in the QSPI flash memory. All data of FW will be downloaded
  *         by calling this function many times. The downloaded data coming
  *         via UART should be saved on QSPI flash memory.
  *         FWUPDATE_Download() execute following operation.
  *         + Erase QSPI flash memory before download new FW.
  *         + Receive new FW to serial buffer and copy QSPI Write Buffer
  *         + Write data of QSPI Write Buffer to QSPI flash memory when QSP 
  *         Write Buffer is full.
  *
  *         MCU memory usage
  *     [Serial Receive Buffer]　-->　[QSPI Write Buffer]　-->　[QSPI Flash Memory]
  *        Command data max              4096[byte]                2[MB]
  *           1024[byte]
  *      Downloaded FW portion　　　　　　
  *      is max 512[byte
  *
  * Please note, this operation should be in 200ms.
  * Because, main routine allows FW_Update module to spend only 200ms per 
  * 1-operation. (1 function call.)
  * So #define value which is specified the flash writing size is important.
  * Need to consider the reasonable value that can be finished writing operation
  * within 200 ms.
  * When the downloading is finished (All of FW data is written to QSPI flash 
  * memory), need to notify it to the boot portion (or early portion of Main 
  * routine). 
  * So that the boot portion will call “FWUPDATE_Update()” to update firmware in 
  * Internal Flash memory.
  *
  * @param  address Address to write data on QSPI write buffer.
  * @param  data Address of the data
  * @param  num_bytes number of bytes to write
  * @retval FWUPDATE_ERR_OK     Download start succeeded
  *         FWUPDATE_ERR_NOT_INITIALIZED  FW Update task has not been started
  *         FWUPDATE_ERR_BUSY   In processing
  *         FWUPDATE_ERR_PARAM  Illegal parameter
  *         FWUPDATE_ERR_FATAL  Other error
  */
FWUPDATE_ERR_CODE FWUPDATE_Download(uint32_t address, uint32_t data, uint32_t num_bytes)
{
  int32_t ret;

  // Erase EXTROM area
  ret = EXTROM_Erase(address, num_bytes);
  if (FLASH_ERR_PARAM == ret) {
    return FWUPDATE_ERR_PARAM;
  } else if (FLASH_ERR_FATAL == ret) {
    return FWUPDATE_ERR_FATAL;
  }
  
  // Write data to EXTROM
  ret = EXTROM_Write(data, address, num_bytes);
  if (FLASH_ERR_PARAM == ret) {
    return FWUPDATE_ERR_PARAM;
  } else if (FLASH_ERR_FATAL == ret) {
    return FWUPDATE_ERR_FATAL;
  }
 
  return FWUPDATE_ERR_OK;
}


/**
  * @brief  Write back FW data from QSPI flash memory to internal flash memory.
  *         This function is called by at the starting point of the program.
  *         (Early portion of Main, or Bootloader,...)
  *         No need to finish this function in 200ms. Take time.
  * @param  src Start address of data to be copied(External QSPI flash memory)
  * @param  dst Start address of copy destination (Internal flash memory)
  * @param  num_bytes Specifies the size of data to be copied in bytes
  * @retval FWUPDATE_ERR_OK     Update succeeded
  *         FWUPDATE_ERR_NOT_INITIALIZED  Flash access module has not been 
  *                             initialized (It mean, FWUPDATE_InitQSPI() or 
  *                             FWUPDATE_InitFlash () has not been called.)
  *         FWUPDATE_ERR_BUSY   In Processing
  *         FWUPDATE_ERR_PARAM  Illegal parameter
  *         FWUPDATE_ERR_FATAL  Other error
  */
FWUPDATE_ERR_CODE FWUPDATE_Update(uint32_t src, uint32_t dst, uint32_t num_bytes)
{
  uint32_t qspi_addr = src;
  uint32_t rom_addr = dst;
  uint32_t program_size = num_bytes;
  const uint32_t block_size_max = FW_DOWNLOAD_BUFFER_SIZE; // 4096
  uint32_t block_size, count;

  /* Erase Internal Flash */
  if(FLASH_Erase(rom_addr, program_size) == FLASH_ERR_FATAL)
  {
    DEBUG_PRINT("FLASH_Erase error\n");
    return FWUPDATE_ERR_PARAM;
  }
  
  count = program_size;
  do
  { 
    if(count < block_size_max)
    {
      block_size = count;
    } else
    {
      block_size = block_size_max;
    }
    
    /* Read from External Flash */
    EXTROM_Read(qspi_addr, (uint32_t)&qspi_buf[0], sizeof(qspi_buf));
    
    /* Write to Internal Flash */
    if(FLASH_Write((uint32_t)&qspi_buf[0], rom_addr, sizeof(qspi_buf)) != FLASH_ERR_OK)
    {
      return FWUPDATE_ERR_PARAM;
    }
    
    count -= block_size;
    rom_addr += block_size;
    qspi_addr += block_size;
  } while(count > 0);
  
  HAL_Delay(3000);
  
  return FWUPDATE_ERR_OK;
}

/**
  * @brief  Get current Firmware info
  * @param  fw_info Firmware info (OUTPUT)
  * @retval index of current firmware in QSPI Flash
  *         -1: No firmware found; otherwise: firmware index
  */
int16_t FWUPDATE_Get_Current_FW_Info(fw_header_t* fw_info)
{
  int16_t current_index = -1;
  fw_header_t tmp;
  fw_header_t current = {0, 0, 0};
  
  for (int16_t i = 0; i < FW_AREA_NUM; i++) {
    EXTROM_Read(i * FW_AREA_SIZE, (uint32_t)&tmp, sizeof(fw_header_t));
    if ((tmp.state == FWUPDATE_AREA_STATE_VALID) || (tmp.state == FWUPDATE_AREA_STATE_DOWNLOADED)) {
      if (current_index < 0) {
        current = tmp;
        current_index = i;
      } else {
        if (tmp.version == (current.version + 1) & 0xFF) {
          current = tmp;
          current_index = i;
        } else {
          break;
        }
      }
    } else if (current_index >= 0) {
      break;
    }
  }
  
  if (current_index >= 0) {
    *fw_info = current;
  }
  
  return current_index;
}

/**
  * @brief  Set Firmware info
  * @param  area index of current firmware in QSPI Flash
  * @param  fw_info Firmware info (INPUT)
  * @retval None
  */
void FWUPDATE_Set_FW_Info(uint8_t area, fw_header_t* fw_info)
{
  EXTROM_Erase(area * FW_AREA_SIZE, FW_HEADER_SIZE);
  
  EXTROM_Write((uint32_t)fw_info, area * FW_AREA_SIZE, sizeof(fw_header_t));
}