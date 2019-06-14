/**
  ******************************************************************************
  * @file    fw_update_api.h
  * @author  SH Consulting
  * @brief   Header file of Firmware Update APIs.
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FW_UPDATE_API_H
#define __FW_UPDATE_API_H

/* Includes ------------------------------------------------------------------*/
//#include "stm32h7xx_hal_def.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_nucleo_144.h"
#include "flash_drv.h"
#include "flash_ext_drv.h"

/* Error code definition */
typedef enum {
  FWUPDATE_ERR_OK,                    // Normal
  FWUPDATE_ERR_ALREADY_INITIALIZED,   // FW Update task already started
  FWUPDATE_ERR_NOT_INITIALIZED,       // FW Update task not started
  FWUPDATE_ERR_BUSY,                  // In processing
  FWUPDATE_ERR_PARAM,                 // Invalid parameter
  FWUPDATE_ERR_FATAL,                 // Other error
} FWUPDATE_ERR_CODE;

/* Export definitions */
/* QSPI Write buffer */
#define FW_DOWNLOAD_BUFFER_SIZE (4096)


/* Callback functions */

/* Functions */
FWUPDATE_ERR_CODE FWUPDATE_InitFlash();	// Initialize HW so that the internal flash memory can be accessed
FWUPDATE_ERR_CODE FWUPDATE_InitQSPI();	// Initialize HW to access QSPI flash memory.
FWUPDATE_ERR_CODE FWUPDATE_Init(void);    // Start the firmware update task
FWUPDATE_ERR_CODE FWUPDATE_Download(uint32_t src);  // Save FW receive from UART to QSPI flash memory.
FWUPDATE_ERR_CODE FWUPDATE_Update(uint32_t src, uint32_t dst, uint32_t num_bytes); // Write FW from QSPI flash memory to internal flash memory

#endif /*__FW_UPDATE_API_H */