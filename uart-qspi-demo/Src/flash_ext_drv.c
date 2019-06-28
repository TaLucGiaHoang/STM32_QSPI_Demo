/**
  ******************************************************************************
  * @file    flash_ext_drv.c
  * @author  SH Consulting
  * @brief   External QSPI Flash APIs
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "flash_ext_drv.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define RAM_START_ADDRESS          0x20000000



#define EXTROM_ERASE_BLOCK_SIZE_64KB      0x10000
#define EXTROM_WRITE_BLOCK_SIZE_256BYTE   0x100 //32
#define EXTROM_READ_BLOCK_SIZE_128BYTE      128
#define EXTROM_READ_BLOCK_SIZE_64KB      64*1024//0x10000


/* Private macro -------------------------------------------------------------*/
#pragma section =".qspi"
#pragma section =".qspi_init"

/* External variables ---------------------------------------------------------*/
QSPI_HandleTypeDef QSPIHandle;

/* Private variables ---------------------------------------------------------*/
static __IO uint8_t CmdCplt, RxCplt, TxCplt, StatusMatch;

/* Private function prototypes -----------------------------------------------*/
static void Error_Handler(void);
static void QSPI_WriteEnable(QSPI_HandleTypeDef *hqspi);
static void QSPI_AutoPollingMemReady(QSPI_HandleTypeDef *hqspi);
static void QSPI_DummyCyclesCfg(QSPI_HandleTypeDef *hqspi);
static HAL_StatusTypeDef QSPI_Init(QSPI_HandleTypeDef *hqspi);
static HAL_StatusTypeDef QSPI_Erase(QSPI_HandleTypeDef *hqspi, __IO uint32_t address);
static HAL_StatusTypeDef QSPI_Write(QSPI_HandleTypeDef *hqspi, __IO uint32_t address, uint8_t *pData, uint32_t size);
static HAL_StatusTypeDef QSPI_Read(QSPI_HandleTypeDef *hqspi, __IO uint32_t address, uint8_t *pData, uint32_t size);

/**
  * @brief  Command completed callbacks.
  * @param  hqspi: QSPI handle
  * @retval None
  */
void HAL_QSPI_CmdCpltCallback(QSPI_HandleTypeDef *hqspi)
{
  CmdCplt++;
   /* printf("callback set CmdCplt %d\n", CmdCplt); */
}

/**
  * @brief  Rx Transfer completed callbacks.
  * @param  hqspi: QSPI handle
  * @retval None
  */
void HAL_QSPI_RxCpltCallback(QSPI_HandleTypeDef *hqspi)
{
  RxCplt++;
   /* printf("callback set RxCplt %d\n", RxCplt); */
}

/**
  * @brief  Tx Transfer completed callbacks.
  * @param  hqspi: QSPI handle
  * @retval None
  */
void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hqspi)
{
  TxCplt++; 
  /* printf("callback set TxCplt %d\n", TxCplt); */
}

/**
  * @brief  Status Match callbacks
  * @param  hqspi: QSPI handle
  * @retval None
  */
void HAL_QSPI_StatusMatchCallback(QSPI_HandleTypeDef *hqspi)
{
  StatusMatch++;
   /* printf("callback set StatusMatch %d\n", StatusMatch); */
}


/**
  * @brief  This function send a Write Enable and wait it is effective.
  * @param  hqspi: QSPI handle
  * @retval None
  */
static void QSPI_WriteEnable(QSPI_HandleTypeDef *hqspi)
{
  QSPI_CommandTypeDef     sCommand;
  QSPI_AutoPollingTypeDef sConfig;

  /* Enable write operations ------------------------------------------ */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = WRITE_ENABLE_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_NONE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Configure automatic polling mode to wait for write enabling ---- */  
  sConfig.Match           = 0x02;
  sConfig.Mask            = 0x02;
  sConfig.MatchMode       = QSPI_MATCH_MODE_AND;
  sConfig.StatusBytesSize = 1;
  sConfig.Interval        = 0x10;
  sConfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  sCommand.Instruction    = READ_STATUS_REG_CMD;
  sCommand.DataMode       = QSPI_DATA_1_LINE;

  if (HAL_QSPI_AutoPolling(hqspi, &sCommand, &sConfig, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function read the SR of the memory and wait the EOP.
  * @param  hqspi: QSPI handle
  * @retval None
  */
static void QSPI_AutoPollingMemReady(QSPI_HandleTypeDef *hqspi)
{
  QSPI_CommandTypeDef     sCommand;
  QSPI_AutoPollingTypeDef sConfig;

  /* Configure automatic polling mode to wait for memory ready ------ */  
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = READ_STATUS_REG_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  sConfig.Match           = 0x00;
  sConfig.Mask            = 0x01;
  sConfig.MatchMode       = QSPI_MATCH_MODE_AND;
  sConfig.StatusBytesSize = 1;
  sConfig.Interval        = 0x10;
  sConfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_QSPI_AutoPolling_IT(hqspi, &sCommand, &sConfig) != HAL_OK)
  {
	  printf("Error QSPI_AutoPollingMemReady HAL_QSPI_AutoPolling_IT\n");
    Error_Handler();
  }
}

/**
  * @brief  This function configure the dummy cycles on memory side.
  * @param  hqspi: QSPI handle
  * @retval None
  */
static void QSPI_DummyCyclesCfg(QSPI_HandleTypeDef *hqspi)
{
  QSPI_CommandTypeDef sCommand;
  uint8_t reg;

  /* Read Volatile Configuration register --------------------------- */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = READ_VOL_CFG_REG_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  sCommand.NbData            = 1;

  if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_QSPI_Receive(hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Enable write operations ---------------------------------------- */
  QSPI_WriteEnable(hqspi);

  /* Write Volatile Configuration register (with new dummy cycles) -- */  
  sCommand.Instruction = WRITE_VOL_CFG_REG_CMD;
  MODIFY_REG(reg, 0xF0, (DUMMY_CLOCK_CYCLES_READ_QUAD << POSITION_VAL(0xF0)));
      
  if (HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_QSPI_Transmit(hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief  Initialize QSPI
  * @param  hqspi: QSPI handle
  * @retval HAL status
  */
static HAL_StatusTypeDef QSPI_Init(QSPI_HandleTypeDef *hqspi)
{
  hqspi->Instance = QUADSPI;
  HAL_QSPI_DeInit(hqspi);

  hqspi->Init.ClockPrescaler     = 2;
  hqspi->Init.FifoThreshold      = 4;
  hqspi->Init.SampleShifting     = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi->Init.FlashSize          = 22;
  hqspi->Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE; //QSPI_CS_HIGH_TIME_3_CYCLE;
  hqspi->Init.ClockMode          = QSPI_CLOCK_MODE_0;
  hqspi->Init.FlashID            = QSPI_FLASH_ID_1;
  hqspi->Init.DualFlash          = QSPI_DUALFLASH_DISABLE;
  
  return HAL_QSPI_Init(hqspi);
}

/**
  * @brief  Erase QSPI (64KB block erase)
  * @param  hqspi: QSPI handle
  * @param  address: QSPI address 
  * @retval HAL status
  */
static HAL_StatusTypeDef QSPI_Erase(QSPI_HandleTypeDef *hqspi, __IO uint32_t address)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  QSPI_CommandTypeDef      sCommand;

  /* Check range of QSPI address */
  if(address >= QSPI_START_ADDRESS && address <= QSPI_END_ADDRESS)  // [0x000000:0x3FFFFF]
  {
    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
    sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
    /* Enable write operations ------------------------------------------- */
    QSPI_WriteEnable(hqspi);

    /* Erasing Sequence -------------------------------------------------- */
    sCommand.Instruction = SECTOR_ERASE_CMD;
    sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
    sCommand.Address     = address;
    sCommand.DataMode    = QSPI_DATA_NONE;
    sCommand.DummyCycles = 0;
// printf("QSPI_Erase address 0x%x\n", address);
    CmdCplt = 0;
    status = HAL_QSPI_Command_IT(hqspi, &sCommand);
    if(status == HAL_OK)
    {
      while(CmdCplt == 0);    /* Wait for command send completed */
    }
  } else
  {
    return HAL_ERROR;
  }

  return status;
}

/**
  * @brief  Write to QSPI (1-256 byte write block)
  * @param  hqspi: QSPI handle
  * @param  address: 
  * @param  pData: pointer to data buffer
  * @retval HAL status
  */
static HAL_StatusTypeDef QSPI_Write(QSPI_HandleTypeDef *hqspi, __IO uint32_t address, uint8_t *pData, uint32_t size)
{
  HAL_StatusTypeDef status = HAL_OK;// HAL_ERROR;
 
  QSPI_CommandTypeDef      sCommand;
  const uint32_t block_size = EXTROM_WRITE_BLOCK_SIZE_256BYTE;

  if(size > block_size)
  {
    printf("size > %d\n", block_size);
    return HAL_ERROR;
  }
  
  address &= EXTROM_FLASH_ADDR_MASK;
  if((address + size - 1) > QSPI_END_ADDRESS)
  {
    printf("address 0x%x > 0x1fffff\n", (address + size));
    return HAL_ERROR;
  }
  
  if(address >= QSPI_START_ADDRESS && address <= QSPI_END_ADDRESS) 
  {
    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
    sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    TxCplt = 0;

    /* Enable write operations ----------------------------------------- */
    QSPI_WriteEnable(hqspi);

    /* Writing Sequence ------------------------------------------------ */
    sCommand.Instruction = QUAD_IN_FAST_PROG_CMD;
    sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
    sCommand.Address     = address;
    sCommand.DataMode    = QSPI_DATA_4_LINES;
    sCommand.NbData      = size;
    sCommand.DummyCycles = 0;

    status = HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
    if(status != HAL_OK)
    {
      return status;
    }

    // status = HAL_QSPI_Transmit_DMA(hqspi, pData);
    status = HAL_QSPI_Transmit_IT(hqspi, pData);
    if (status == HAL_OK)
    {
      while(TxCplt == 0);   /* Wait for transfer completed */
      
      if(TxCplt != 0)
      {
        /* TxCplt = 0; */
        StatusMatch = 0;

        /* Configure automatic polling mode to wait for end of program ----- */  
        QSPI_AutoPollingMemReady(hqspi);

        while(StatusMatch==0);
      }
    }
  }
  
  return status;
}

/**
  * @brief  Read from QSPI (1-256 byte write block)
  * @param  hqspi: QSPI handle
  * @param  address: 
  * @param  pData: pointer to data buffer
  * @retval HAL status
  */
static HAL_StatusTypeDef QSPI_Read(QSPI_HandleTypeDef *hqspi, __IO uint32_t address, uint8_t *pData, uint32_t size)
{
  HAL_StatusTypeDef status = HAL_OK;// HAL_ERROR;
 
  QSPI_CommandTypeDef      sCommand;
  QSPI_MemoryMappedTypeDef sMemMappedCfg;
  const uint32_t block_size = 256; //EXTROM_READ_BLOCK_SIZE_128BYTE;

  if(size > block_size)
  {
    return HAL_ERROR;
  }
  
  address &= EXTROM_FLASH_ADDR_MASK;
  if((address + size - 1) > EXTROM_FLASH_ADDR_MASK)
  {
    return HAL_ERROR;
  }
  
  if(address >= QSPI_START_ADDRESS && address <= QSPI_END_ADDRESS) 
  {
    sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
    sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
    sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    RxCplt = 0;

    /* Reading Sequence ------------------------------------------------ */
    sCommand.Instruction = QUAD_OUT_FAST_READ_CMD;
    sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
    sCommand.Address     = address;
    sCommand.DataMode    = QSPI_DATA_4_LINES;
    sCommand.NbData      = size;
    sCommand.DummyCycles = DUMMY_CLOCK_CYCLES_READ_QUAD; // (8)

    status = HAL_QSPI_Command(hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
    if(status != HAL_OK)
    {
      return status;
    }

    // status = HAL_QSPI_Receive_DMA(hqspi, pData);
    status = HAL_QSPI_Receive_IT(hqspi, pData);
    if (status == HAL_OK)
    {
      while(RxCplt == 0);   /* Wait for receive completed */
      
      if(RxCplt != 0)
      {
        /* RxCplt = 0; */
        StatusMatch = 0;

        /* Configure automatic polling mode to wait for end of reading ----- */  
        QSPI_AutoPollingMemReady(hqspi);

        while(StatusMatch==0);
      }
    }
  }
  
  return status;
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  BSP_LED_On(LED3);

  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}



/* Exported functions --------------------------------------------------------*/
/**
  * @brief  Initialize HW to access QSPI flash memory
  * @param  None
  * @retval FLASH_ERR_OK    Succeeded in initializing QSPI HW
            FLASH_ERR_FATAL Failed to initialize QSPI HW
  */
int32_t EXTROM_Init(void)
{
  if(QSPI_Init(&QSPIHandle) != HAL_OK)
  {
    return FLASH_ERR_FATAL;
  }
  
  return FLASH_ERR_OK;
}

/**
  * @brief  Erase the QSPI flash memory
  * @param  start Start address to erase
  * @param  num_bytes Number of bytes to erase
  * @retval FLASH_ERR_OK      Succeeded in erasing QSPI flash memory
  *         FLASH_ERR_PARAM   Illegal parameter
  *         FLASH_ERR_FATAL   Failed to erase QSPI flash memory
  */
int32_t EXTROM_Erase(__IO uint32_t start, uint32_t num_bytes)
{
  uint32_t size = num_bytes;
  uint32_t block_size = EXTROM_ERASE_BLOCK_SIZE_64KB;
  uint32_t count = num_bytes;
  QSPI_HandleTypeDef *hqspi = &QSPIHandle;
 
  /* [0x90000000:0x901FFFFF] - 1 area only*/
  /* [0x90000000:0x903FFFFF] - 2 area only*/
  if(start >= EXTROM_START_ADDRESS && (start + num_bytes - 1) <= EXTROM_END_ADDRESS)
  {
    start &= EXTROM_FLASH_ADDR_MASK;
    if((num_bytes % block_size) || (num_bytes == 0))
    {
      return FLASH_ERR_PARAM;
    }

    do 
    {
      /* Erase by 64KB block */
      if (QSPI_Erase(hqspi, start) != HAL_OK)
      {
        return FLASH_ERR_FATAL;
      }

      if(CmdCplt != 0)
      {
        CmdCplt = 0;
        StatusMatch = 0;

        /* Configure automatic polling mode to wait for end of erase ------- */  
        QSPI_AutoPollingMemReady(hqspi);
        while(StatusMatch == 0);   
      } 
    
      count -= block_size;
      start += block_size;
    } while(count > 0);  
  }
  else
  {
    return FLASH_ERR_PARAM;
  }

  return FLASH_ERR_OK;
}

/**
  * @brief  Write data to QSPI flash memory
  * @param  src Start address of write data
  * @param  dst Write destination address
  * @param  num_bytes Number of bytes to write
  * @retval FLASH_ERR_OK      Succeeded in writing to QSPI flash memory
  *         FLASH_ERR_PARAM   Illegal parameter
  *         FLASH_ERR_FATAL   Failed to write to QSPI flash memory
  */  
int32_t EXTROM_Write(__IO uint32_t src, __IO uint32_t dst, uint32_t num_bytes)
{
  __IO uint32_t qspi_addr, flash_addr;
  uint32_t block_size;
  const uint32_t block_size_max = EXTROM_WRITE_BLOCK_SIZE_256BYTE;
  uint32_t count;
  QSPI_HandleTypeDef *hqspi = &QSPIHandle;
  
  if(num_bytes > EXTROM_FLASH_SIZE)  // ROM max size
  {
    return FLASH_ERR_PARAM;
  }
  
  if(dst >= EXTROM_START_ADDRESS && (dst + num_bytes - 1) <= EXTROM_END_ADDRESS)
  {  
    dst &= EXTROM_FLASH_ADDR_MASK;

    count = num_bytes;
    flash_addr = src;
    qspi_addr = dst;

    do
    {
      if(count < block_size_max)
      {
        block_size = count;
      } else
      {
        block_size = block_size_max;
      }
      
      /* Write by 256 Bytes block */
      if (QSPI_Write(hqspi, qspi_addr, (uint8_t*)flash_addr, block_size) != HAL_OK)
      {
        return FLASH_ERR_FATAL;
      }
      count -= block_size;
      qspi_addr += block_size;
      flash_addr += block_size;
    } while(count > 0);

  } else
  {
    return FLASH_ERR_PARAM;
  }
  
  return FLASH_ERR_OK;
}

/**
  * @brief  Read data to QSPI flash memory
  * @param  src Read source address
  * @param  dst Start address of read data
  * @param  num_bytes Number of bytes to read
  * @retval FLASH_ERR_OK      Succeeded in writing to QSPI flash memory
  *         FLASH_ERR_PARAM   Illegal parameter
  *         FLASH_ERR_FATAL   Failed to read to QSPI flash memory
  */
int32_t EXTROM_Read(__IO uint32_t src, __IO uint32_t dst, uint32_t num_bytes)
{
  __IO uint32_t qspi_addr, flash_addr;
  const uint32_t block_size_max = 256;//128; // EXTROM_READ_BLOCK_SIZE_128BYTE
  uint32_t block_size;
  int32_t count;
  QSPI_HandleTypeDef *hqspi = &QSPIHandle;
  
  if(num_bytes > EXTROM_READ_BLOCK_SIZE_64KB) // Max data is stored on RAM
  {
    return FLASH_ERR_PARAM;
  }
  
  if(src >= EXTROM_START_ADDRESS && (src + num_bytes - 1) <= EXTROM_END_ADDRESS)
  {  
    src &= EXTROM_FLASH_ADDR_MASK;
    count = num_bytes;
    flash_addr = dst;
    qspi_addr = src;
    
    do
    {
      if(count < block_size_max)
      {
        block_size = count;
      } else
      {
        block_size = block_size_max;
      }
// printf("block_size %d num_bytes %d\n", block_size, num_bytes);
      /* Read by 128 Bytes block */
      if (QSPI_Read(hqspi, qspi_addr, (uint8_t*)flash_addr, block_size) != HAL_OK)
      {
        return FLASH_ERR_FATAL;
      }
     
      count -= block_size;
      qspi_addr += block_size;
      flash_addr += block_size;
    } while(count > 0);

  } else
  {
    return FLASH_ERR_PARAM;
  }
  
  return FLASH_ERR_OK;
}
