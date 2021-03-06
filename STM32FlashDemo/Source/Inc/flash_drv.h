/**
  ******************************************************************************
  * @file    flash_drv.h
  * @author  SH Consulting
  * @brief   Header file of Firmware Update Driver.
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_DRV_H
#define __FLASH_DRV_H

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stm32h7xx_nucleo_144.h"

/* Exported macro ------------------------------------------------------------*/
#define FLASH_BASE_ADDR      FLASH_BASE
#define FLASH_END_ADDR       FLASH_END

/* Base address of the Flash sectors Bank 1 */
#define ADDR_FLASH_SECTOR_0_BANK1     ((uint32_t)0x08000000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK1     ((uint32_t)0x08020000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK1     ((uint32_t)0x08040000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK1     ((uint32_t)0x08060000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK1     ((uint32_t)0x08080000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK1     ((uint32_t)0x080A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK1     ((uint32_t)0x080C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK1     ((uint32_t)0x080E0000) /* Base @ of Sector 7, 128 Kbytes */

/* Base address of the Flash sectors Bank 2 */
#define ADDR_FLASH_SECTOR_0_BANK2     ((uint32_t)0x08100000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK2     ((uint32_t)0x08120000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK2     ((uint32_t)0x08140000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK2     ((uint32_t)0x08160000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK2     ((uint32_t)0x08180000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK2     ((uint32_t)0x081A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK2     ((uint32_t)0x081C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK2     ((uint32_t)0x081E0000) /* Base @ of Sector 7, 128 Kbytes */


/* Error code definition */
typedef enum {
  FLASH_ERR_OK,                    // normal
  FLASH_ERR_ALREADY_INITIALIZED,   // QSPI HW has already been initialized 
  FLASH_ERR_NOT_INITIALIZED,       // QSPI HW has not been initialized
  FLASH_ERR_PARAM,                 // Invalid parameter
  FLASH_ERR_FATAL,                 // Failed to flash operation such as read, write and erase
} FLASH_ERR_CODE;


/* Functions */
int32_t FLASH_Init(void);   // Initialize the internal flash access module
int32_t FLASH_Erase(uint32_t start, uint32_t num_bytes);       // Erase the built-in flash memory
int32_t FLASH_Read(uint32_t src, uint32_t dst, uint32_t num_bytes);    // Read data from internal flash memory
int32_t FLASH_Write(uint32_t src, uint32_t dst, uint32_t num_bytes);    // Write data to internal flash memory
#endif /*__FLASH_DRV_H */