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
static __IO uint32_t flash_main_address = ADDR_FLASH_SECTOR_0_BANK2; // test BANK 2
static struct FW_Info_ST FW[2]; /* [NUM_AREA]; */
static struct FW_Info_ST fw_info;
static uint8_t qspi_buf[FW_DOWNLOAD_BUFFER_SIZE];

/* Status flags:  1 done, 0 on process */
static uint32_t backup_area, update_area;
static uint32_t latest_version;
static uint32_t total_size = 0, qspi_size_max = 0;

/* Private function prototypes */
static inline uint32_t calc_backup_area(void);
static inline uint32_t calc_update_area(void);
static void get_fw_info(uint32_t area, struct FW_Info_ST* fwp);
static void set_fw_info(uint32_t area, const struct FW_Info_ST* fwp);
static void print_fw_info(struct FW_Info_ST* fwp);
static int32_t flash_swap(void);
static int32_t flash_erase(uint32_t bank);
static int32_t ext_flash_erase(uint32_t area);
static int32_t backup_firmware(void);  // Defaul copy Flash Bank1 to QSPI area1
static int32_t save_new_firmware(uint32_t area);
static void display_qspi_memory(uint32_t address);



/* Exported variables */
/* 0: Nothing , 1: Detect new version */
uint32_t new_version_flag = 0;



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

static inline uint32_t calc_backup_area(void)
{
  return ( (update_area == AREA_1) ? (AREA_NUM-1) : (update_area-1) );
}

static inline uint32_t calc_update_area(void)
{
  return ( (update_area == (AREA_NUM-1)) ? (AREA_1) : (update_area+1) );
}

static void get_fw_info(uint32_t area, struct FW_Info_ST* fwp)
{
  // struct FW_Info_ST* fwp;
  uint32_t area_address = 0;
  
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
  // uint32_t used_size = fwp->program_size + sizeof(struct FW_Info_ST);

  printf("[QSPI] area: 0x%08x , size 0x%08x (%d), state: %d\n", fwp->area_start_addr, fwp->area_size, fwp->area_size, fwp->state);
  printf("[QSPI] program: 0x%08x , size 0x%08x (%d) , version %d\n", fwp->program_start_addr, fwp->program_size, fwp->program_size, fwp->version);
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
  __IO uint32_t bank_address;
  switch (bank)
  {
    case FLASH_BANK_1:
    {
      bank_address = ADDR_FLASH_SECTOR_0_BANK1; // 0x08000000
    }
    break;
    case FLASH_BANK_2:
    {
      bank_address = ADDR_FLASH_SECTOR_0_BANK2; // 0x08100000
    }
    break;
    default:
    {
      return FLASH_ERR_FATAL;
    }
    break;
  }
  
  return FLASH_Erase(bank_address, 0x100000); // erase 1MB
}

static int32_t ext_flash_erase(uint32_t area)
{
  int32_t ret = FLASH_ERR_FATAL;
  __IO uint32_t area_address;
  struct FW_Info_ST* fwp = &FW[area];

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
  printf("Erase area %d 0x%08x\n", area, area_address);  
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
  struct FW_Info_ST* fwp = &FW[backup_area];
  
  

  /* Not implemented yet */
  // printf("backup_firmware() do nothing\n");

  
  printf("backup_firmware() backup_area %d set backup completed (not implemeted)\n", backup_area);
  fwp->state = FWUPDATE_AREA_STATUS_BACKUP_COMPLETE; //// Not implemet backup process
  set_fw_info(backup_area, fwp);
  return 0;
}

static int32_t save_new_firmware(uint32_t area)
{
  return 0;
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
  uint32_t erased_area_cnt = 0;
  // backup_done = 1; 
  // update_done = 0;
  // download_done = 1;

  new_version_flag = 0;  // Initialize 
  
  latest_version = 0;
  total_size = 0;
  qspi_size_max = FW_DOWNLOAD_BUFFER_SIZE; // 4096 bytes

  backup_area = AREA_1;
  update_area = AREA_2;
  /* Check all area state */
  for(int32_t area = AREA_1; area < AREA_NUM; area++) 
  {
    struct FW_Info_ST* fwp = &FW[area];
    get_fw_info(area, fwp);
    
    printf("area %d state %d\n", area, fwp->state);
    switch (fwp->state)
    {
      case FWUPDATE_AREA_STATUS_BACKUPING:
      case FWUPDATE_AREA_STATUS_DOWNLOADING:
      case FWUPDATE_AREA_STATUS_UPDATING:
      {
        /* Erase not complete program stored in area */
        ext_flash_erase(area);
        /* Count erased area */
        erased_area_cnt++;
        break;
      }
      case FWUPDATE_AREA_STATUS_DOWNLOAD_COMPLETE:
      {
        /* Set flag to copy new firmware to ROM */
        printf("Detected new version %d in QSPI area %d\n", fwp->version, area);
        
        new_version_flag = 1;  
        update_area = area;
        backup_area = calc_backup_area();
        break;
      }
      case FWUPDATE_AREA_STATUS_UPDATE_COMPLETE:
      {
        /* Mark as used area */
        /* Change updated state to backup state */        
        update_area = area;   // updated area is this area
        /* Shift update area to next area */
        update_area = calc_update_area();
        /* Set previous area to backup area */
        backup_area = calc_backup_area();
        fwp->state = FWUPDATE_AREA_STATUS_BACKUP_COMPLETE;
        printf("area %d (version %d) change to state %d \n", area, fwp->version, fwp->state); // test
        set_fw_info(area, fwp);
        break;
      }
      case FWUPDATE_AREA_STATUS_BACKUP_COMPLETE:
      {
        /* Do nothing */
        break;
      }
      case FWUPDATE_AREA_STATUS_ERASED:
      {
        /* Count erased area */
        erased_area_cnt++;
        break;
      }
      default:
      {
        printf("Unregconize, format area %d\n", area);
        /* Format dump area */
        ext_flash_erase(area);
        /* Count erased area */
        erased_area_cnt++;
        break;
      }
    }
    
    if(latest_version < fwp->version)
    {
      latest_version = fwp->version;
    }
  } 
  

  

  /* Check if all area are empty */
  if(erased_area_cnt == AREA_NUM)
  {
    /* Save backup firmware version 1 */
    latest_version = 1;   // Lowest version
    backup_area = AREA_1; // Start area
    update_area = AREA_2;
    
    printf("Backup firmware version %d to area %d\n", latest_version, backup_area);
    FW[backup_area].state = FWUPDATE_AREA_STATUS_BACKUPING; //// Not implemet backup process
    FW[backup_area].version = latest_version;
    
    set_fw_info(backup_area, &FW[backup_area]);
    
    /* Copy current firmware to QSPI Flash */
    backup_firmware();
  }
  
  /* Print backup area info */
  printf("Backup area %d info:\n", backup_area);
  get_fw_info(backup_area, &FW[backup_area]);
  print_fw_info(&FW[backup_area]);
  
  /* Print update area info */
  printf("Update area %d info:\n", update_area);
  get_fw_info(update_area, &FW[update_area]);
  print_fw_info(&FW[update_area]);
  
  // printf("backup_area %d, update_area %d latest_version %d\n", backup_area, update_area, latest_version);
  // get_fw_info(AREA_1, &FW[AREA_1]);
  // get_fw_info(AREA_2, &FW[AREA_2]);
  // print_fw_info(&FW[AREA_1]);
  // print_fw_info(&FW[AREA_2]);
  return FWUPDATE_ERR_OK;
}

uint32_t FWUPDATE_IsNewOS(void)
{
  return new_version_flag;
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
FWUPDATE_ERR_CODE FWUPDATE_Download(uint32_t src)
{
  int32_t ret;
  __IO uint32_t data_addr = (__IO uint32_t)src;
  __IO uint32_t download_addr;
  const uint32_t size = qspi_size_max;
  struct FW_Info_ST* fwp = &fw_info;
  uint32_t used_size = 0;

  /* Check parameters */
  /* Check total size will be download */
  if(total_size == 0)
  {
    printf("Not initialize variable: total_size = %d\n", total_size); // test
    return FWUPDATE_ERR_NOT_INITIALIZED;
  }
    
    
  /* Read from QSPI area header */
	get_fw_info(update_area, fwp);
  printf("- Download to QSPI (area %d, address 0x%08x)\n", update_area, fwp->program_start_addr);
  
  if(fwp->state == FWUPDATE_AREA_STATUS_BACKUP_COMPLETE)
  {
    /* Erase old backup area before downloading */
    printf("- Erase old backup area %d (version %d) before downloading\n", update_area, fwp->version);
    ext_flash_erase(update_area);
    
    /* Read area header after erasing */
    get_fw_info(update_area, fwp);
  }
  
  if(fwp->state == FWUPDATE_AREA_STATUS_ERASED)
  {
    /* Set program size to 0 */
    fwp->program_size = 0;

    /* Change to next state */
    fwp->state = FWUPDATE_AREA_STATUS_DOWNLOADING;
    
    /* Write to QSPI area header */
    set_fw_info(update_area, fwp);
  }
  
  /* Save new firmware to External QSPI Flash */
  if(fwp->state == FWUPDATE_AREA_STATUS_DOWNLOADING)
  {
    /* Check space for download */
    used_size = FW_INFO_SIZE + fwp->program_size + size;
    if(used_size > EXTROM_AREA_SIZE_2MB)
    {
      printf("- Error out of memory");  //////// test
      return FWUPDATE_ERR_FATAL;
    }
    
    /* Set download address */
    download_addr = fwp->program_start_addr + fwp->program_size;
        
    /* Write 4096 bytes each calling */
    if(EXTROM_Write(data_addr, download_addr, size) != FLASH_ERR_OK)
    {
      printf("- EXTROM_Write error\n");
      return FWUPDATE_ERR_FATAL;
    }

    /* Increase program size of QSPI Flash */
    fwp->program_size += size;
    // printf("- program_size %d\n", fwp->program_size);

   
    /* Check space for next download */
    if(fwp->program_size == total_size)
    {
      fwp->state = FWUPDATE_AREA_STATUS_DOWNLOAD_COMPLETE;
      latest_version += 1;
      fwp->version = latest_version;
      printf("- Download completed version %d to QSPI (area %d, address 0x%08x, size %d bytes)\n", fwp->version, update_area, fwp->program_start_addr, fwp->program_size);
      // print_fw_info(fwp);  // TEST
    } else if(fwp->program_size > total_size)
    {
      printf("- Error Download oversize %d > %d\n", fwp->program_size, total_size); // test
      return FWUPDATE_ERR_FATAL;
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
    printf("- Download completed, send reset request... (not implemented)\n");
    ret = FWUPDATE_ERR_OK;
  }
  
  return ret;
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
  struct FW_Info_ST* fwp = &fw_info;
  __IO uint32_t qspi_addr;
  __IO uint32_t rom_addr = dst;
  const uint32_t block_size_max = FW_DOWNLOAD_BUFFER_SIZE; // 4096
  uint32_t block_size, count;
  uint32_t total_size = 0; // test
  
  /* Check flag */
  /* Check parameters */
  // if((src < EXTROM_AREA_1_ADDRESS) || (src >= EXTROM_AREA_2_ADDRESS + EXTROM_AREA_SIZE_2MB))
  // {
    // return FWUPDATE_ERR_PARAM;
  // }
  
  // if( !(IS_FLASH_PROGRAM_ADDRESS(dst)) || !(IS_FLASH_PROGRAM_ADDRESS(dst + num_bytes -1)) )
  // {
    // return FWUPDATE_ERR_PARAM;
  // }


  /* Read from QSPI Flash */
  /* Read from QSPI area header */
	get_fw_info(update_area, fwp);
  
  
  /* Check ROM space */
  if( !(IS_FLASH_PROGRAM_ADDRESS(rom_addr)) || !(IS_FLASH_PROGRAM_ADDRESS(rom_addr + fwp->program_size -1)) )
  {
    return FWUPDATE_ERR_PARAM;
  }
  
  /* Check QSPI area state */
  if(fwp->state == FWUPDATE_AREA_STATUS_DOWNLOADING || fwp->state == FWUPDATE_AREA_STATUS_ERASED)
  {
    printf("- Area download has not been completed\n");
    return FWUPDATE_ERR_FATAL;
  }
  
  if(fwp->state == FWUPDATE_AREA_STATUS_DOWNLOAD_COMPLETE)
  {
    fwp->state = FWUPDATE_AREA_STATUS_UPDATING;
    set_fw_info(update_area, fwp);
  }
  
  if(fwp->state == FWUPDATE_AREA_STATUS_UPDATING)
  {

    qspi_addr = fwp->program_start_addr;
    printf("- Update new firmware version %d from QSPI 0x%08x (%d bytes) to FLASH 0x%08x\n", fwp->version, fwp->program_start_addr, fwp->program_size, rom_addr); 
    // printf("qspi_buf 0x%08x\n", (uint32_t)&qspi_buf[0]);
    /* Erase Internal Flash */
    if(FLASH_Erase(rom_addr, fwp->program_size) == FLASH_ERR_FATAL)
    {
      Flash_Print_Error();
      return FWUPDATE_ERR_PARAM;
    }
    
    count = fwp->program_size;
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
      if((qspi_addr >= EXTROM_AREA_1_ADDRESS) && (qspi_addr < EXTROM_AREA_2_ADDRESS + EXTROM_AREA_SIZE_2MB))
      {
        EXTROM_Read(qspi_addr, (uint32_t)&qspi_buf[0], sizeof(qspi_buf));
      } else 
      {
        return FWUPDATE_ERR_PARAM;
      }
      
      /* Write to Internal Flash */
      if(FLASH_Write((uint32_t)&qspi_buf[0], rom_addr, sizeof(qspi_buf)) != FLASH_ERR_OK)
      {
        Flash_Print_Error();
        return FWUPDATE_ERR_PARAM;
      }
      
      total_size += block_size;
      // printf("Update to rom_addr 0x%08x , updated size %d bytes\n", rom_addr, total_size);
      
      count -= block_size;
      rom_addr += block_size;
      qspi_addr += block_size;
    } while(count > 0);
    
    if(total_size == fwp->program_size)
    {
      /* Mark area has been used */
      fwp->state = FWUPDATE_AREA_STATUS_UPDATE_COMPLETE;
      set_fw_info(update_area, fwp);
      
      printf("- Update completed (%d bytes)\n", total_size);
    } else if(total_size < fwp->program_size)
    {
      return FWUPDATE_ERR_BUSY;
    } else 
    {
      printf("- total_size %d > program_size %d\n", total_size, fwp->program_size);
      return FWUPDATE_ERR_FATAL;
    }
    
  }
  
  if(fwp->state == FWUPDATE_AREA_STATUS_UPDATE_COMPLETE)
  {
    printf("- Update completed, send reset request... (not implemented)\n");
    new_version_flag = 0;
    return FWUPDATE_ERR_OK;
  }

  return FWUPDATE_ERR_NOT_INITIALIZED;
}


/* Callback functions */
/**
  * @brief  
  * @param  None
  * @retval 
  */

