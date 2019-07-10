/**
  ******************************************************************************
  * @file    fw_update_drv.c
  * @author  SH Consulting
  * @brief   
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "flash_drv.h"
#include <string.h>
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
// #define DEBUG
#if defined(DEBUG)
 #define DEBUG_PRINT(fmt, args...) printf(fmt, ##args)
#else
 #define DEBUG_PRINT(fmt, args...)
#endif
/* Private variables ---------------------------------------------------------*/
static FLASH_EraseInitTypeDef EraseInitStruct;

/* Private function prototypes -----------------------------------------------*/
static void Flash_Print_Error(void);
static uint32_t GetSector(uint32_t Address);

/** 
  * @brief    FLASH Error Code 
  * @param    FLASH_Error_Code FLASH Error Code
  * @retval None
  */ 
void Flash_Print_Error(void)
{
  uint32_t ercd = HAL_FLASH_GetError();
  if (ercd == HAL_FLASH_ERROR_NONE)         DEBUG_PRINT("[FLASH] No error (0x%08x)\n", ercd);
  if (ercd == HAL_FLASH_ERROR_WRP)          DEBUG_PRINT("[FLASH] Write Protection Error (0x%08x)\n", ercd);
  if (ercd == HAL_FLASH_ERROR_PGS)          DEBUG_PRINT("[FLASH] Program Sequence Error (0x%08x)\n", ercd);
  if (ercd == HAL_FLASH_ERROR_STRB)         DEBUG_PRINT("[FLASH] Strobe Error (0x%08x)\n", ercd);
  if (ercd == HAL_FLASH_ERROR_INC)          DEBUG_PRINT("[FLASH] Inconsistency Error (0x%08x)\n", ercd);
  if (ercd == HAL_FLASH_ERROR_OPE)          DEBUG_PRINT("[FLASH] Operation Error (0x%08x)\n", ercd);
  if (ercd == HAL_FLASH_ERROR_RDP)          DEBUG_PRINT("[FLASH] Read Protection Error (0x%08x)\n", ercd);
  if (ercd == HAL_FLASH_ERROR_RDS)          DEBUG_PRINT("[FLASH] Read Secured Error (0x%08x)\n", ercd);
  if (ercd == HAL_FLASH_ERROR_SNECC)        DEBUG_PRINT("[FLASH] Single Detection ECC (0x%08x)\n", ercd);
  if (ercd == HAL_FLASH_ERROR_DBECC)        DEBUG_PRINT("[FLASH] Double Detection ECC (0x%08x)\n", ercd);
  if (ercd == HAL_FLASH_ERROR_WRP_BANK1)    DEBUG_PRINT("[FLASH] Write Protection Error on Bank 1 (0x%08x)\n", ercd);
  if (ercd == HAL_FLASH_ERROR_PGS_BANK1)    DEBUG_PRINT("[FLASH] Program Sequence Error on Bank 1 (0x%08x)\n", ercd);
  if (ercd == HAL_FLASH_ERROR_STRB_BANK1)   DEBUG_PRINT("[FLASH] Strobe Error on Bank 1 (0x%08x)\n", ercd);
  if (ercd == HAL_FLASH_ERROR_INC_BANK1)    DEBUG_PRINT("[FLASH] Inconsistency Error on Bank 1 (0x%08x)\n", ercd);
  if (ercd == HAL_FLASH_ERROR_OPE_BANK1)    DEBUG_PRINT("[FLASH] Operation Error on Bank 1 (0x%08x)\n", ercd);
  if (ercd == HAL_FLASH_ERROR_RDP_BANK1)    DEBUG_PRINT("[FLASH] Read Protection Error on Bank 1 (0x%08x)\n", ercd);
  if (ercd == HAL_FLASH_ERROR_RDS_BANK1)    DEBUG_PRINT("[FLASH] Read Secured Error on Bank 1 (0x%08x)\n", ercd);
  if (ercd == HAL_FLASH_ERROR_SNECC_BANK1)  DEBUG_PRINT("[FLASH] Single Detection ECC on Bank 1 (0x%08x)\n", ercd);
  if (ercd == HAL_FLASH_ERROR_DBECC_BANK1)  DEBUG_PRINT("[FLASH] Double Detection ECC on Bank 1 (0x%08x)\n", ercd);
  if (ercd == HAL_FLASH_ERROR_WRP_BANK2)    DEBUG_PRINT("[FLASH] Write Protection Error on Bank 2 (0x%08x)\n", ercd);
  if (ercd == HAL_FLASH_ERROR_PGS_BANK2)    DEBUG_PRINT("[FLASH] Program Sequence Error on Bank 2 (0x%08x)\n", ercd);
  if (ercd == HAL_FLASH_ERROR_STRB_BANK2)   DEBUG_PRINT("[FLASH] Strobe Error on Bank 2 (0x%08x)\n", ercd);
  if (ercd == HAL_FLASH_ERROR_INC_BANK2)    DEBUG_PRINT("[FLASH] Inconsistency Error on Bank 2 (0x%08x)\n", ercd);
  if (ercd == HAL_FLASH_ERROR_OPE_BANK2)    DEBUG_PRINT("[FLASH] Operation Error on Bank 2 (0x%08x)\n", ercd);
  if (ercd == HAL_FLASH_ERROR_RDP_BANK2)    DEBUG_PRINT("[FLASH] Read Protection Error on Bank 2 (0x%08x)\n", ercd);
  if (ercd == HAL_FLASH_ERROR_RDS_BANK2)    DEBUG_PRINT("[FLASH] Read Secured Error on Bank 2 (0x%08x)\n", ercd);
  if (ercd == HAL_FLASH_ERROR_SNECC_BANK2)  DEBUG_PRINT("[FLASH] Single Detection ECC on Bank 2 (0x%08x)\n", ercd);
  if (ercd == HAL_FLASH_ERROR_DBECC_BANK2)  DEBUG_PRINT("[FLASH] Double Detection ECC on Bank 2 (0x%08x)\n", ercd);
  if (ercd == HAL_FLASH_ERROR_OB_CHANGE)    DEBUG_PRINT("[FLASH] Option Byte Change Error (0x%08x)\n", ercd);
}

/**
  * @brief  Gets the sector of a given address
  * @param  Address Address of the FLASH Memory
  * @retval The sector of a given address
  */
uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;
  
  if (((Address < ADDR_FLASH_SECTOR_1_BANK1) && (Address >= ADDR_FLASH_SECTOR_0_BANK1)) || \
     ((Address < ADDR_FLASH_SECTOR_1_BANK2) && (Address >= ADDR_FLASH_SECTOR_0_BANK2)))    
  {
    sector = FLASH_SECTOR_0;  
  }
  else if (((Address < ADDR_FLASH_SECTOR_2_BANK1) && (Address >= ADDR_FLASH_SECTOR_1_BANK1)) || \
          ((Address < ADDR_FLASH_SECTOR_2_BANK2) && (Address >= ADDR_FLASH_SECTOR_1_BANK2)))    
  {
    sector = FLASH_SECTOR_1;  
  }
  else if (((Address < ADDR_FLASH_SECTOR_3_BANK1) && (Address >= ADDR_FLASH_SECTOR_2_BANK1)) || \
          ((Address < ADDR_FLASH_SECTOR_3_BANK2) && (Address >= ADDR_FLASH_SECTOR_2_BANK2)))    
  {
    sector = FLASH_SECTOR_2;  
  }
  else if (((Address < ADDR_FLASH_SECTOR_4_BANK1) && (Address >= ADDR_FLASH_SECTOR_3_BANK1)) || \
          ((Address < ADDR_FLASH_SECTOR_4_BANK2) && (Address >= ADDR_FLASH_SECTOR_3_BANK2)))    
  {
    sector = FLASH_SECTOR_3;  
  }
  else if (((Address < ADDR_FLASH_SECTOR_5_BANK1) && (Address >= ADDR_FLASH_SECTOR_4_BANK1)) || \
          ((Address < ADDR_FLASH_SECTOR_5_BANK2) && (Address >= ADDR_FLASH_SECTOR_4_BANK2)))    
  {
    sector = FLASH_SECTOR_4;  
  }
  else if (((Address < ADDR_FLASH_SECTOR_6_BANK1) && (Address >= ADDR_FLASH_SECTOR_5_BANK1)) || \
          ((Address < ADDR_FLASH_SECTOR_6_BANK2) && (Address >= ADDR_FLASH_SECTOR_5_BANK2)))    
  {
    sector = FLASH_SECTOR_5;  
  }
  else if (((Address < ADDR_FLASH_SECTOR_7_BANK1) && (Address >= ADDR_FLASH_SECTOR_6_BANK1)) || \
          ((Address < ADDR_FLASH_SECTOR_7_BANK2) && (Address >= ADDR_FLASH_SECTOR_6_BANK2)))    
  {
    sector = FLASH_SECTOR_6;  
  }
  else if (((Address < ADDR_FLASH_SECTOR_0_BANK2) && (Address >= ADDR_FLASH_SECTOR_7_BANK1)) || \
          ((Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_7_BANK2)))
  {
     sector = FLASH_SECTOR_7;  
  }
  else
  {
    sector = FLASH_SECTOR_7;
  }

  return sector;
}

/* Exported functions --------------------------------------------------------*/
/**
  * @brief  Initialize the internal flash access module
  * @param  
  * @retval 
  */
int32_t FLASH_Init(void)
{
  return FLASH_ERR_OK;
}


/**
  * @brief  Erase the internal flash memory
  * @param  start　Start address to erase
  * @param  num_bytes Number of bytes to erase
  * @retval FLASH_ERR_OK    Succeeded in erasing the internal
  *         FLASH_ERR_PARAM   Illegal parameter
  *         FLASH_ERR_FATAL   Failed to erase internal flash memory
  */
int32_t FLASH_Erase(uint32_t start, uint32_t num_bytes)
{
  uint32_t FirstSector = 0, NbOfSectors = 0;
  uint32_t SECTORError = 0;
  uint32_t end = start + num_bytes - 1;
  int32_t  ret = FLASH_ERR_OK;

  /* Check the parameters */
  if (!(IS_FLASH_PROGRAM_ADDRESS(start)))
  {
    ret = FLASH_ERR_PARAM;
  }
  
  if (!(IS_FLASH_PROGRAM_ADDRESS(start + num_bytes -1)))
  {
    ret = FLASH_ERR_PARAM;
  }
  
  /* Select Bank to erase */
  if (IS_FLASH_PROGRAM_ADDRESS_BANK1(start))
  {
    /* Get the 1st sector to erase */
    FirstSector = GetSector(start);
    /* Get the number of sector to erase from 1st sector*/
    NbOfSectors = GetSector(end) - FirstSector + 1;

    /* Fill EraseInit structure*/
    EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Banks         = FLASH_BANK_1;
    EraseInitStruct.Sector        = FirstSector;
    EraseInitStruct.NbSectors     = NbOfSectors;
  }
  else if (IS_FLASH_PROGRAM_ADDRESS_BANK2(start))
  {
    /* Get the 1st sector to erase */
    FirstSector = GetSector(start);
    /* Get the number of sector to erase from 1st sector*/
    NbOfSectors = GetSector(end) - FirstSector + 1;

    /* Fill EraseInit structure*/
    EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Banks         = FLASH_BANK_2;
    EraseInitStruct.Sector        = FirstSector;
    EraseInitStruct.NbSectors     = NbOfSectors;
  }
  else
  {
    // Should never get here
    return FLASH_ERR_FATAL;
  }
 
  DEBUG_PRINT("[FLASH] Flash erase [0x%08x:0x%08x] (0x%x bytes) Sector %d, NbSectors %d\n", start, end, num_bytes, FirstSector, NbOfSectors);
  HAL_FLASH_Unlock();
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
  {
    /*
      Error occurred while sector erase.
      User can add here some code to deal with this error.
      SECTORError will contain the faulty sector and then to know the code error on this sector,
      user can call function 'HAL_FLASH_GetError()'
    */
    Flash_Print_Error();
    ret = FLASH_ERR_FATAL;
  }
  HAL_FLASH_Lock();

  return ret;
}

/**
  * @brief  Read data from internal flash memory to RAM
  * @param  src Start address of read data
  * @param  dst Save destination address of read data
  * @param  num_bytes  Number of bytes to read
  * @retval FLASH_ERR_OK     Succeeded in reading from internal flash memory
            FLASH_ERR_PARAM  Illegal parameter
            FLASH_ERR_FATAL   Failed to read from internal flash memory
  */
int32_t FLASH_Read(uint32_t src, uint32_t dst, uint32_t num_bytes)
{
  const uint32_t block_size_max = 8;
  uint32_t block_size;
  uint64_t *src_addr = (uint64_t*)src;
  uint64_t *dest_addr = (uint64_t*)dst;
  uint32_t count = num_bytes;
 
  /* Check the parameters */
  if (!(IS_FLASH_PROGRAM_ADDRESS(src)))
  {
    return FLASH_ERR_PARAM;
  }
  
  if (!(IS_FLASH_PROGRAM_ADDRESS(src + num_bytes -1)))
  {
    return FLASH_ERR_PARAM;
  }
  
  while (count > 0)
  {
    if (count < block_size_max)
    {
      block_size = count;
    } else
    {
      block_size = block_size_max;
    }

    *dest_addr++ = *src_addr++;
    __DSB();
    count -= block_size;
  }  
  
  return FLASH_ERR_OK;
}

/**
  * @brief  Write data to internal flash memory
  * @param  src Start address of write data (Data address)
  * @param  dst Write destination address (Flash address)
  * @param  num_bytes  Number of bytes to write
  * @retval FLASH_ERR_OK     Succeeded in writing to internal flash memory
            FLASH_ERR_PARAM  Illegal parameter
            FLASH_ERR_FATAL  Failed to write to internal flash memory
  */
int32_t FLASH_Write(uint32_t src, uint32_t dst, uint32_t num_bytes)
{
  uint32_t FlashAddress = dst;
  uint64_t DataAddress = (uint64_t)src;
  uint32_t block_size_max = 32; // 256 bit
  uint32_t block_size = 0;
  uint32_t count = 0;

  /* Check the parameters */
  if (!(IS_FLASH_PROGRAM_ADDRESS(dst)))
  {
    return FLASH_ERR_PARAM;
  }
  
  if (!(IS_FLASH_PROGRAM_ADDRESS(dst + num_bytes -1)))
  {
    return FLASH_ERR_PARAM;
  }

  count = num_bytes;
  
  HAL_FLASH_Unlock();

  while (count > 0)
  {
    if (count < block_size_max)
    {
      block_size = count;
    } else
    {
      block_size = block_size_max;
    }
    
    /* Program flash word of 256 bits at a specified address */
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, FlashAddress, DataAddress) != HAL_OK)
    {
      /* Error occurred while writing data in Flash memory.
         User can add here some code to deal with this error */
      Flash_Print_Error();
      return FLASH_ERR_FATAL;
    }
    
    /* Increment for the next Flash word*/
    FlashAddress += block_size;
    DataAddress  += block_size;
    count        -= block_size;
  }
  
  HAL_FLASH_Lock();
  
  return FLASH_ERR_OK;
}
