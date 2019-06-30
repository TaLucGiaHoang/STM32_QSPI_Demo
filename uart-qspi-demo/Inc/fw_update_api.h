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

/* QSPI Flash area status definition */
typedef enum {
  FWUPDATE_AREA_STATUS_ERASED,             // empty area
  FWUPDATE_AREA_STATUS_DOWNLOADING, // Writting transferred data from UART to External memory
  FWUPDATE_AREA_STATUS_DOWNLOAD_COMPLETE,  // newest version
  FWUPDATE_AREA_STATUS_UPDATING,           // copying from External memory to Internal memory
  FWUPDATE_AREA_STATUS_UPDATE_COMPLETE,    // current running firmware
  FWUPDATE_AREA_STATUS_BACKUPING,             // 
  FWUPDATE_AREA_STATUS_BACKUP_COMPLETE,    // 
};

/* Task message */
typedef enum {
  FWUPDATE_MSG_NONE = 0x00,
  FWUPDATE_MSG_OK,           // Update ACK, Backup ACK, get FW info ACK, reset ACK
  FWUPDATE_MSG_NG,        // Response error
  // FWUPDATE_MSG_QSPI_READ,
  // FWUPDATE_MSG_QSPI_WRITE,           // QSPI write size 4096 bytes
  FWUPDATE_MSG_QSPI_ERASE,
  // FWUPDATE_MSG_FLASH_ERASE,
  // FWUPDATE_MSG_FLASH_READ,
  // FWUPDATE_MSG_FLASH_WRITE,
  // FWUPDATE_MSG_TX_DATA,          // FW data transmission (Packet size: 512 bytes)  
  FWUPDATE_MSG_UPDATE_REQUEST,
  // FWUPDATE_MSG_UPDATE_COMPLETE,
  // FWUPDATE_MSG_DOWNLOAD_REQUEST,
  // FWUPDATE_MSG_DOWNLOAD_REQUEST_ACK,
  // FWUPDATE_MSG_DOWNLOAD_COMPLETE,
  FWUPDATE_MSG_FW_INFO,          //  Send FW information (Packet size, Packet number, QSPI write size, checksum)
  // FWUPDATE_MSG_FW_INFO_SIZE,       // 1-byte size of new firmware
  // FWUPDATE_MSG_FW_INFO_CHECKSUM,   // 1-byte checksum of new firmware
  FWUPDATE_MSG_FW_INFO_DATA,       // 512-bytes data of new firmware
  FWUPDATE_MSG_RESET_REQUEST,
  FWUPDATE_MSG_VERIFY_REQUEST,
  // FWUPDATE_MSG_VERIFY_OK,
  // FWUPDATE_MSG_VERIFY_NG,
} FWUPDATE_MSG;

/* Export definitions */
/* QSPI Write buffer */
#define FW_DOWNLOAD_BUFFER_SIZE (4096)

/* Firmware structure */
struct FW_Info_ST {
  uint32_t state;                // QSPI flash area status
  __IO uint32_t area_start_addr;      // QSPI flash area address
  uint32_t area_size;            // 2MB
  __IO uint32_t program_start_addr;   // Start address of firmware
  uint32_t program_size;         // Program size
  uint32_t version;              // The highest value is the latest version
};



/* Callback functions */

/* Functions */
FWUPDATE_ERR_CODE FWUPDATE_EraseExtFlash(uint32_t area); 

FWUPDATE_ERR_CODE FWUPDATE_InitFlash();	// Initialize HW so that the internal flash memory can be accessed
FWUPDATE_ERR_CODE FWUPDATE_InitQSPI();	// Initialize HW to access QSPI flash memory.
FWUPDATE_ERR_CODE FWUPDATE_Init(void);    // Start the firmware update task
FWUPDATE_ERR_CODE FWUPDATE_Download_Config(uint32_t firmware_size, uint32_t qspi_wr_size);  // Set total size will be downloaded
FWUPDATE_ERR_CODE FWUPDATE_Download(uint32_t src);  // Save FW receive from UART to QSPI flash memory.
FWUPDATE_ERR_CODE FWUPDATE_Update(uint32_t src, uint32_t dst, uint32_t num_bytes); // Write FW from QSPI flash memory to internal flash memory

#endif /*__FW_UPDATE_API_H */