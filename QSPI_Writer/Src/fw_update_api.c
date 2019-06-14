/**
  ******************************************************************************
  * @file    fw_update_api.c
  * @author  SH Consulting
  * @brief   
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "fw_update_api.h"


// static FWUPDATE_CALLBACK s_callback = NULL;
/* Private variables */
static __IO uint32_t backup_addr = EXTROM_AREA_1_ADDRESS;
static __IO uint32_t download_addr = EXTROM_AREA_2_ADDRESS;
static __IO uint32_t update_addr = EXTROM_AREA_2_ADDRESS;

/* Status flags:  1 done, 0 on process */
static uint8_t backup_done = 1; 
static uint8_t update_done = 0;
static uint8_t download_done = 1;

/* Private function prototypes */
static int32_t flash_swap(void);
static int32_t flash_erase(uint32_t bank);
static int32_t ext_flash_erase(uint32_t area);
static int32_t backup_firmware(void);  // Defaul copy Flash Bank1 to QSPI area1
static int32_t save_new_firmware(uint32_t area);


/* Private functions */
static int32_t flash_swap(void)
{
  return 0;
}

static int32_t flash_erase(uint32_t bank)
{
  return 0;
}

static int32_t ext_flash_erase(uint32_t area)
{
  int32_t ret = FLASH_ERR_FATAL;
  switch (area)
  {
    case AREA_1:
    {
      ret = EXTROM_Erase(EXTROM_AREA_1_ADDRESS, EXTROM_AREA_SIZE_2MB);
    }
    break;
    case AREA_2:
    {
      ret = EXTROM_Erase(EXTROM_AREA_2_ADDRESS, EXTROM_AREA_SIZE_2MB);
    }
    break;
    default:
    {
      ret = FLASH_ERR_FATAL;
    }
    break;
  }

  return ret;
}

static int32_t backup_firmware(void)
{
  return 0;
}

static int32_t save_new_firmware(uint32_t area)
{
  
}


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
  backup_done = 1; 
  update_done = 0;
  download_done = 1;
  
  backup_addr = EXTROM_AREA_1_ADDRESS;
  update_addr = EXTROM_AREA_2_ADDRESS;
  download_addr = EXTROM_AREA_2_ADDRESS;
  
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
  * @param  src Address to write data on QSPI write buffer.
  * @retval FWUPDATE_ERR_OK     Download start succeeded
  *         FWUPDATE_ERR_NOT_INITIALIZED  FW Update task has not been started
  *         FWUPDATE_ERR_BUSY   In processing
  *         FWUPDATE_ERR_PARAM  Illegal parameter
  *         FWUPDATE_ERR_FATAL  Other error
  */
FWUPDATE_ERR_CODE FWUPDATE_Download(uint32_t src)
{
  int32_t ret;
  __IO uint32_t data_addr = src;
  const uint32_t size = FW_DOWNLOAD_BUFFER_SIZE;
  
  /* Check parameters */
  // printf("download_done %d\n", download_done);
  if(download_done == 1)
  {
    printf("Erase QSPI Flash area 2 0x%08x\n", EXTROM_AREA_2_ADDRESS);
    /* Erase External QSPI Flash area2 before saving new firmware */
    ret = ext_flash_erase(AREA_2);
    if(ret != FLASH_ERR_OK)
    {
      printf("Error Erase External QSPI Flash area 2 to save new firmware\n");
    }else
    {
      /* Re-initialize download flag */
      download_done = 0;
      download_addr = EXTROM_AREA_2_ADDRESS;
    }
  }
  
  if(download_done == 0)
  { 
    /* Save new firmware to External QSPI Flash */
    ret = EXTROM_Write(data_addr, download_addr, size);
    if(ret != FLASH_ERR_OK)
    {
      printf("EXTROM_Write error\n");
    }
    
    download_addr += size;
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
  __IO uint64_t *src_addr = (__IO uint64_t *)src; // External QSPI Flash 
  __IO uint64_t *dest_addr = (__IO uint64_t*)dst; // Internal Flash
  /* Check flag */
  
  /* Read from QSPI Flash */
  
  /* Erase Internal Flash */
  
  /* Write to Internal Flash */
  
  
  return FWUPDATE_ERR_OK;
}


/* Callback functions */
/**
  * @brief  
  * @param  None
  * @retval 
  */

