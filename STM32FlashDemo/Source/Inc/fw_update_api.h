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
enum {
  FWUPDATE_AREA_STATUS_ERASED,             // empty area
  FWUPDATE_AREA_STATUS_DOWNLOADING, // Writting transferred data from UART to External memory
  FWUPDATE_AREA_STATUS_DOWNLOAD_COMPLETE,  // newest version
  FWUPDATE_AREA_STATUS_UPDATING,           // copying from External memory to Internal memory
  FWUPDATE_AREA_STATUS_UPDATE_COMPLETE,    // current running firmware
  FWUPDATE_AREA_STATUS_BACKUPING,             // 
  FWUPDATE_AREA_STATUS_BACKUP_COMPLETE,    // 
};

typedef struct {
  uint32_t state;
  uint32_t size;
  uint32_t version;
} fw_header_t;

/* Export definitions */
#define FW_AREA_NUM 2 // Number of area in QPSI
#define FW_AREA_SIZE (2<<20) // Size of Firmware area in QSPI - 2MB

/* QSPI Write buffer */
#define FW_DOWNLOAD_BUFFER_SIZE (4096) // must be multiple of 4KB

/* Firmware Area State */
#define FWUPDATE_AREA_STATE_ERASED      0xFFFFFFFF
#define FWUPDATE_AREA_STATE_DOWNLOADING 0xFFFFFFFE
#define FWUPDATE_AREA_STATE_DOWNLOADED  0xFFFFFFFC
#define FWUPDATE_AREA_STATE_UPDATING    0xFFFFFFF8
#define FWUPDATE_AREA_STATE_VALID       0xFFFFFFF0
#define FWUPDATE_AREA_STATE_INVALID     0xFFFF0000

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
FWUPDATE_ERR_CODE FWUPDATE_InitFlash();	// Initialize HW so that the internal flash memory can be accessed
FWUPDATE_ERR_CODE FWUPDATE_InitQSPI();	// Initialize HW to access QSPI flash memory.
FWUPDATE_ERR_CODE FWUPDATE_Init(void);    // Start the firmware update task
// FWUPDATE_ERR_CODE FWUPDATE_Download_Config(uint32_t firmware_size, uint32_t qspi_wr_size);  // Set total size will be downloaded
FWUPDATE_ERR_CODE FWUPDATE_Download(uint32_t address, uint32_t data, uint32_t num_bytes);  // Save FW receive from UART to QSPI flash memory.
FWUPDATE_ERR_CODE FWUPDATE_Update(uint32_t src, uint32_t dst, uint32_t num_bytes); // Write FW from QSPI flash memory to internal flash memory

int16_t FWUPDATE_Get_Current_FW_Info(fw_header_t* fw_info);
void FWUPDATE_Set_FW_Info(uint8_t area, fw_header_t* fw_info);

#endif /*__FW_UPDATE_API_H */