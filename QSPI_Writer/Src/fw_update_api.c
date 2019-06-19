/**
  ******************************************************************************
  * @file    fw_update_api.c
  * @author  SH Consulting
  * @brief   
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "fw_update_api.h"

/* Private macro */
#define EXTROM_ERASE_BLOCK_SIZE_64KB      0x10000
#define FW_INFO_SIZE                      0x10000

// static FWUPDATE_CALLBACK s_callback = NULL;
/* Private variables */
static __IO uint32_t backup_addr = EXTROM_AREA_1_ADDRESS + FW_INFO_SIZE;
static __IO uint32_t download_addr = EXTROM_AREA_2_ADDRESS + FW_INFO_SIZE;
static __IO uint32_t update_addr = EXTROM_AREA_2_ADDRESS;
static struct FW_Info_ST FW[2]; /* [NUM_AREA]; */
static struct FW_Info_ST fw_info;

/* Status flags:  1 done, 0 on process */
static uint8_t backup_done = 1; 
static uint8_t update_done = 0;
static uint8_t download_done = 1;
static uint32_t backup_area = AREA_1;
static uint32_t update_area = AREA_NUM - 1;
static uint32_t latest_version;
static uint32_t total_size = 0, qspi_size_max = 0;

/* Private function prototypes */
static void get_fw_info(uint32_t area, struct FW_Info_ST* fwp);
static void set_fw_info(uint32_t area, const struct FW_Info_ST* fwp);
static void print_fw_info(struct FW_Info_ST* fwp);
static int32_t flash_swap(void);
static int32_t flash_erase(uint32_t bank);
static int32_t ext_flash_erase(uint32_t area);
static int32_t backup_firmware(void);  // Defaul copy Flash Bank1 to QSPI area1
static int32_t save_new_firmware(uint32_t area);
static void display_qspi_memory(uint32_t address);







static void display_qspi_memory(uint32_t address)
{
  uint8_t qspi_buf[256];
  
  if((address >= EXTROM_AREA_1_ADDRESS) && (address < EXTROM_AREA_2_ADDRESS + EXTROM_AREA_SIZE_2MB))
  {
    if(EXTROM_Read(address, (uint32_t)&qspi_buf[0], 256) != FLASH_ERR_OK)
    {
      printf("display_qspi_memory error 0x%08x\n", address);
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

static void foo(void)
{

  
}

static void get_fw_info(uint32_t area, struct FW_Info_ST* fwp)
{
  // struct FW_Info_ST* fwp;
  uint32_t area_address = 0;
  uint8_t qspi_rx_buf[100];
  
  
  switch (area)
  {
    case AREA_1:
    {
      area_address = EXTROM_AREA_1_ADDRESS;
    }
    break;
    case AREA_2:
    {
      area_address = EXTROM_AREA_2_ADDRESS;
    }
    break;
    default:
      goto exit;
    break;
  }
  
  EXTROM_Read(area_address, (uint32_t)fwp, sizeof(struct FW_Info_ST));
  // print_fw_info(fwp);  // TEST
  
  exit:
}

static void set_fw_info(uint32_t area, const struct FW_Info_ST* fwp)
{
  uint32_t area_address;
  switch (area)
  {
    case AREA_1:
    {
      area_address = EXTROM_AREA_1_ADDRESS;
    }
    break;
    case AREA_2:
    {
      area_address = EXTROM_AREA_2_ADDRESS;
    }
    break;
    default:
      printf("error area\n");
      goto exit;
    break;
  }

  /* Need to erase 64KB before writing to QSPI memory */
  EXTROM_Erase(area_address, EXTROM_ERASE_BLOCK_SIZE_64KB);
  
  if(EXTROM_Write((uint32_t)fwp, area_address, sizeof(struct FW_Info_ST)) != FLASH_ERR_OK)
  {
    printf("set fw error\n");
  }
  
  exit:
}

static void print_fw_info(struct FW_Info_ST* fwp)
{
  uint32_t used_size = fwp->program_size + sizeof(struct FW_Info_ST);
  
  printf("\n");
  printf("state: %d\n", fwp->state);
  printf("area: 0x%08x , size 0x%08x (%d)\n", fwp->area_start_addr, fwp->area_size, fwp->area_size);
  printf("program: 0x%08x , size 0x%08x (%d) , ver %d\n", fwp->program_start_addr, fwp->program_size, fwp->program_size, fwp->version);
  // printf("used_size: 0x%x %d\n", used_size, used_size);
  printf("\n");
}

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
  __IO uint32_t area_address;
  struct FW_Info_ST* fwp = &FW[area];
   
  printf("Erased area %d\n", area);  
  switch (area)
  {
    case AREA_1:
    {
      area_address = EXTROM_AREA_1_ADDRESS;
    }
    break;
    case AREA_2:
    {
      area_address = EXTROM_AREA_2_ADDRESS; 
    }
    break;
    default:
    {
      ret = FLASH_ERR_FATAL;
      goto exit;
    }
    break;
  }
  
  ret = EXTROM_Erase(area_address, EXTROM_AREA_SIZE_2MB);
  
  fwp->state = FWUPDATE_AREA_STATUS_ERASED;  // empty area
  fwp->area_start_addr = area_address;
  fwp->area_size = EXTROM_AREA_SIZE_2MB;
  fwp->program_start_addr = area_address + FW_INFO_SIZE; //sizeof(struct FW_Info_ST);
  fwp->program_size = 0;
  fwp->version = 0; //////////////!!!!
  set_fw_info(area, fwp);
  
  exit:
  return ret;
}

static int32_t backup_firmware(void)
{
  return 0;
}

static int32_t save_new_firmware(uint32_t area)
{
  
}


FWUPDATE_ERR_CODE FWUPDATE_EraseExtFlash(uint32_t area)
{
  ext_flash_erase(area);
  return FWUPDATE_ERR_OK;
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
  uint32_t area = AREA_1;
  backup_done = 1; 
  update_done = 0;
  download_done = 1;
  
  backup_addr = EXTROM_AREA_1_ADDRESS;
  update_addr = EXTROM_AREA_2_ADDRESS;
  download_addr = EXTROM_AREA_2_ADDRESS;
  
  qspi_size_max = FW_DOWNLOAD_BUFFER_SIZE; // 4096 bytes
  
  
  /* Check all area state */
  get_fw_info(AREA_1, &FW[AREA_1]);
  get_fw_info(AREA_2, &FW[AREA_2]);
  
  for(area = AREA_1; area < AREA_NUM; area++) 
  {
    uint32_t state;
    printf("area %d\n", area);

    /* Format dump area */
    if( (FW[area].state != FWUPDATE_AREA_STATUS_DOWNLOADING) &&
        (FW[area].state != FWUPDATE_AREA_STATUS_DOWNLOAD_COMPLETE) &&
        (FW[area].state != FWUPDATE_AREA_STATUS_UPDATING) &&
        (FW[area].state != FWUPDATE_AREA_STATUS_UPDATE_COMPLETE) &&
        (FW[area].state != FWUPDATE_AREA_STATUS_ERASED)
    )
    {
      printf("Erase area %d\n", area);
      ext_flash_erase(area);
    }
    
    /* Get backup area */
    if(FW[area].version < FW[backup_area].version)
    {
      backup_area = area;
    }
    
    
    /* Get update area */
    if(FW[area].version >= FW[backup_area].version)
    {
      if(area != backup_area)
      {
        update_area = area;///////////////////need check for area > 2
      }
    }

  } 
  
  if(FW[update_area].version == FW[backup_area].version)
  {
    FW[update_area].version = FW[update_area].version + 1;
    latest_version = FW[update_area].version;
    set_fw_info(update_area, &FW[update_area]);     
  }
    
  printf("backup_area %d, update_area %d latest_version %d\n", backup_area, update_area, latest_version);
  
  memset(&FW[AREA_1], 0x88, sizeof(FW[AREA_1]));
  memset(&FW[AREA_2], 0x88, sizeof(FW[AREA_1]));
  // print_fw_info(&FW[AREA_1]);
  // print_fw_info(&FW[AREA_2]);
  // get_fw_info(AREA_1, &FW[AREA_1]);
  // get_fw_info(AREA_2, &FW[AREA_2]);

  return FWUPDATE_ERR_OK;
}


FWUPDATE_ERR_CODE FWUPDATE_Download_Config(uint32_t firmware_size, uint32_t qspi_wr_size)
{
  total_size = firmware_size;
  qspi_size_max = qspi_wr_size;
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

static uint32_t program_size = 0;
FWUPDATE_ERR_CODE FWUPDATE_Download(uint32_t src)
{
  int32_t ret;
  __IO uint32_t data_addr = (__IO uint32_t)src;
  const uint32_t size = qspi_size_max;
  struct FW_Info_ST* fwp = &fw_info;
  uint32_t used_size = 0;
  
  /* Check parameters */
  
  /* Read from QSPI area header */
	get_fw_info(update_area, fwp);
  
  if(fwp->state == FWUPDATE_AREA_STATUS_ERASED)
  {
    /* Check total size will be download */
    if(total_size == 0)
    {
      printf("total_size = %d\n", total_size);
      ret = FWUPDATE_ERR_NOT_INITIALIZED;
      return ret;
    }
    
    /* Set program size to 0 */
    fwp->program_size = 0;

    /* Change to next state */
    fwp->state = FWUPDATE_AREA_STATUS_DOWNLOADING;
    
    /* Write to QSPI area header */
    set_fw_info(update_area, fwp);
  }
  
  // /* Save new firmware to External QSPI Flash */
  if(fwp->state == FWUPDATE_AREA_STATUS_DOWNLOADING)
  {
    /* Check space for download */
    used_size = FW_INFO_SIZE + fwp->program_size + size;
    if(used_size > EXTROM_AREA_SIZE_2MB)
    {
      printf("Error out of memory");  //////// test
      ret = FWUPDATE_ERR_FATAL;
      return ret;
    }
    
    // /* Set download address */
    download_addr = fwp->program_start_addr + fwp->program_size;
    
	
    printf("- download_addr 0x%08x , data_addr 0x%08x(%d bytes)\n", download_addr, data_addr, size);
    
    /* Write 4096 bytes each calling */
    if(EXTROM_Write(data_addr, download_addr, size) != FLASH_ERR_OK)
    {
      ret = FWUPDATE_ERR_FATAL;
      printf("Downloading EXTROM_Write error\n");
    }

    // display_qspi_memory(download_addr); // test

    /* Increase program size of QSPI Flash */
    fwp->program_size += size;
    printf("- program_size %d\n", fwp->program_size);

   
    /* Check space for next download */
    if(fwp->program_size == total_size)
    {
      fwp->state = FWUPDATE_AREA_STATUS_DOWNLOAD_COMPLETE;
      printf("- Download completed %d\n", fwp->program_size);
      print_fw_info(fwp);  // TEST
    } else if(fwp->program_size > total_size)
    {
      printf("- Error Download oversize %d\n", fwp->program_size); // test
      ret = FWUPDATE_ERR_FATAL;
      return ret;
    } else
    {
      fwp->state = FWUPDATE_AREA_STATUS_DOWNLOADING; // keep downloading
    }
    
    /* Write to QSPI area header */
    set_fw_info(update_area, fwp);
    
    ret = FWUPDATE_ERR_BUSY;
  }
  
  /* Send download completed message */
  if(fwp->state == FWUPDATE_AREA_STATUS_DOWNLOAD_COMPLETE)
  {
    ret = FWUPDATE_ERR_OK;
  }
  return FWUPDATE_ERR_OK; // test
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

