/**
  ******************************************************************************
  * @file    flash_ext_drv.h
  * @author  SH Consulting
  * @brief   Header file of Firmware Update Driver.
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_EXT_DRV_H
#define __FLASH_EXT_DRV_H

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stm32h7xx_nucleo_144.h"

/* Exported macro ------------------------------------------------------------*/
// #define FLASH_BASE_ADDR      (uint32_t)(0x08000000)
// #define FLASH_END_ADDR       (uint32_t)(0x081FFFFF)
// #define FLASH_ALL_SIZE       (uint32_t)(FLASH_END_ADDR - FLASH_BASE_ADDR + 1)

/* Definition for client expand memory area */
#define NUM_AREA (2)

/* Exported types ------------------------------------------------------------*/
enum {
#if (NUM_AREA >= (1))
  AREA_1,
#endif
#if (NUM_AREA >= (2))
  AREA_2,
#endif
// #if (NUM_AREA >= (4))
  // AREA_3,
  // AREA_4,
// #endif
};

/* Exported constants --------------------------------------------------------*/
/* Definition for QSPI clock resources */
#define QSPI_CLK_ENABLE()          __HAL_RCC_QSPI_CLK_ENABLE()
#define QSPI_CLK_DISABLE()         __HAL_RCC_QSPI_CLK_DISABLE()
#define QSPI_CS_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()
#define QSPI_CLK_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define QSPI_D0_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOD_CLK_ENABLE()
#define QSPI_D1_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOD_CLK_ENABLE()
#define QSPI_D2_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOE_CLK_ENABLE()
#define QSPI_D3_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOD_CLK_ENABLE()
#define QSPI_MDMA_CLK_ENABLE()      __HAL_RCC_MDMA_CLK_ENABLE()

#define QSPI_FORCE_RESET()         __HAL_RCC_QSPI_FORCE_RESET()
#define QSPI_RELEASE_RESET()       __HAL_RCC_QSPI_RELEASE_RESET()

/* Definition for QSPI Pins */
#define QSPI_CS_PIN                GPIO_PIN_6
#define QSPI_CS_GPIO_PORT          GPIOB
#define QSPI_CLK_PIN               GPIO_PIN_2
#define QSPI_CLK_GPIO_PORT         GPIOB
#define QSPI_D0_PIN                GPIO_PIN_11
#define QSPI_D0_GPIO_PORT          GPIOD
#define QSPI_D1_PIN                GPIO_PIN_12
#define QSPI_D1_GPIO_PORT          GPIOD
#define QSPI_D2_PIN                GPIO_PIN_2
#define QSPI_D2_GPIO_PORT          GPIOE
#define QSPI_D3_PIN                GPIO_PIN_13
#define QSPI_D3_GPIO_PORT          GPIOD

/* Definition for QSPI DMA */
#define QSPI_DMA_INSTANCE          DMA2_Stream7
#define QSPI_DMA_REQUEST           $QSPI_DMA_REQUEST$
#define QSPI_DMA_IRQ               DMA2_Stream7_IRQn
#define QSPI_DMA_IRQ_HANDLER       DMA2_Stream7_IRQHandler

/* MT25TL01GHBA8ESF Micron memory */
/* Size of the flash */
#define QSPI_FLASH_SIZE                      22
#define QSPI_PAGE_SIZE                       256

/* Reset Operations */
#define RESET_ENABLE_CMD                     0x66
#define RESET_MEMORY_CMD                     0x99

/* Identification Operations */
#define READ_ID_CMD                          0x9E
#define READ_ID_CMD2                         0x9F
#define MULTIPLE_IO_READ_ID_CMD              0xAF
#define READ_SERIAL_FLASH_DISCO_PARAM_CMD    0x5A

/* Read Operations */
#define READ_CMD                             0x03
#define READ_4_BYTE_ADDR_CMD                 0x13

#define FAST_READ_CMD                        0x0B
#define FAST_READ_DTR_CMD                    0x0D
#define FAST_READ_4_BYTE_ADDR_CMD            0x0C

#define DUAL_OUT_FAST_READ_CMD               0x3B
#define DUAL_OUT_FAST_READ_DTR_CMD           0x3D
#define DUAL_OUT_FAST_READ_4_BYTE_ADDR_CMD   0x3C

#define DUAL_INOUT_FAST_READ_CMD             0xBB
#define DUAL_INOUT_FAST_READ_DTR_CMD         0xBD
#define DUAL_INOUT_FAST_READ_4_BYTE_ADDR_CMD 0xBC

#define QUAD_OUT_FAST_READ_CMD               0x6B
#define QUAD_OUT_FAST_READ_DTR_CMD           0x6D
#define QUAD_OUT_FAST_READ_4_BYTE_ADDR_CMD   0x6C

#define QUAD_INOUT_FAST_READ_CMD             0xEB
#define QUAD_INOUT_FAST_READ_DTR_CMD         0xED
#define QUAD_INOUT_FAST_READ_4_BYTE_ADDR_CMD 0xEC

/* Write Operations */
#define WRITE_ENABLE_CMD                     0x06
#define WRITE_DISABLE_CMD                    0x04

/* Register Operations */
#define READ_STATUS_REG_CMD                  0x05
#define WRITE_STATUS_REG_CMD                 0x01

#define READ_LOCK_REG_CMD                    0xE8
#define WRITE_LOCK_REG_CMD                   0xE5

#define READ_FLAG_STATUS_REG_CMD             0x70
#define CLEAR_FLAG_STATUS_REG_CMD            0x50

#define READ_NONVOL_CFG_REG_CMD              0xB5
#define WRITE_NONVOL_CFG_REG_CMD             0xB1

#define READ_VOL_CFG_REG_CMD                 0x85
#define WRITE_VOL_CFG_REG_CMD                0x81

#define READ_ENHANCED_VOL_CFG_REG_CMD        0x65
#define WRITE_ENHANCED_VOL_CFG_REG_CMD       0x61

#define READ_EXT_ADDR_REG_CMD                0xC8
#define WRITE_EXT_ADDR_REG_CMD               0xC5

/* Program Operations */
#define PAGE_PROG_CMD                        0x02
#define PAGE_PROG_4_BYTE_ADDR_CMD            0x12

#define DUAL_IN_FAST_PROG_CMD                0xA2
#define EXT_DUAL_IN_FAST_PROG_CMD            0xD2

#define QUAD_IN_FAST_PROG_CMD                0x32
#define EXT_QUAD_IN_FAST_PROG_CMD            0x12 /*0x38*/
#define QUAD_IN_FAST_PROG_4_BYTE_ADDR_CMD    0x34

/* Erase Operations */
#define SUBSECTOR_ERASE_CMD                  0x20
#define SUBSECTOR_ERASE_4_BYTE_ADDR_CMD      0x21

#define SECTOR_ERASE_CMD                     0xD8
#define SECTOR_ERASE_4_BYTE_ADDR_CMD         0xDC

#define BULK_ERASE_CMD                       0xC7

#define PROG_ERASE_RESUME_CMD                0x7A
#define PROG_ERASE_SUSPEND_CMD               0x75

/* One-Time Programmable Operations */
#define READ_OTP_ARRAY_CMD                   0x4B
#define PROG_OTP_ARRAY_CMD                   0x42

/* 4-byte Address Mode Operations */
#define ENTER_4_BYTE_ADDR_MODE_CMD           0xB7
#define EXIT_4_BYTE_ADDR_MODE_CMD            0xE9

/* Quad Operations */
#define ENTER_QUAD_CMD                       0x35
#define EXIT_QUAD_CMD                        0xF5

/* Default dummy clocks cycles */
#define DUMMY_CLOCK_CYCLES_READ              8
#define DUMMY_CLOCK_CYCLES_READ_QUAD         8

#define DUMMY_CLOCK_CYCLES_READ_DTR          6
#define DUMMY_CLOCK_CYCLES_READ_QUAD_DTR     8

/* End address of the QSPI memory */
#define QSPI_END_ADDR              (1 << QSPI_FLASH_SIZE)

/* Size of buffers */
#define BUFFERSIZE                 (COUNTOF(aTxBuffer) - 1)


/* QSPI FLASH AREA */
#if (NUM_AREA == (1))
#define EXTROM_START_ADDRESS         0x90000000
#define EXTROM_END_ADDRESS           0x901FFFFF
#else if (NUM_AREA == (2))
#define EXTROM_START_ADDRESS         0x90000000
#define EXTROM_END_ADDRESS           0x903FFFFF
// #else if (NUM_AREA == (4))
// #define EXTROM_START_ADDRESS         0x90000000
// #define EXTROM_END_ADDRESS           0x907FFFFF
#endif

#define EXTROM_FLASH_SIZE_2MB        0x200000
#define EXTROM_FLASH_SIZE_4MB        0x400000
#define EXTROM_FLASH_SIZE_8MB        0x800000

#if (NUM_AREA == (1))
#define EXTROM_FLASH_SIZE          (EXTROM_FLASH_SIZE_2MB)
#else if (NUM_AREA == (2))
#define EXTROM_FLASH_SIZE          (EXTROM_FLASH_SIZE_4MB)
// #else if (NUM_AREA == (4))
// #define EXTROM_FLASH_SIZE          (EXTROM_FLASH_SIZE_8MB)
#endif

#define EXTROM_FLASH_ADDR_MASK     (EXTROM_FLASH_SIZE-1)  // [0x000000-0x3fffff]
#define EXTROM_AREA_SIZE_2MB       0x200000

#if (NUM_AREA >= (1))
#define EXTROM_AREA_1_ADDRESS      0x90000000
#endif
#if (NUM_AREA >= (2))
#define EXTROM_AREA_2_ADDRESS      0x90200000
#endif
// #if (NUM_AREA >= (4))
// #define EXTROM_AREA_3_ADDRESS      0x90400000  // Reserved
// #define EXTROM_AREA_4_ADDRESS      0x90600000  // Reserved
// #endif

#define QSPI_START_ADDRESS        (EXTROM_FLASH_ADDR_MASK & EXTROM_START_ADDRESS)
#define QSPI_END_ADDRESS          (EXTROM_END_ADDRESS & EXTROM_START_ADDRESS)
/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)        (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

/* Exported functions ------------------------------------------------------- */
#include "flash_drv.h"

int32_t EXTROM_Init(void);   // Initialize the QSPI flash access module
int32_t EXTROM_Erase(__IO uint32_t start, uint32_t num_bytes);      // Erase the QSPI flash memory
int32_t EXTROM_Read(__IO uint32_t src, uint32_t dst, uint32_t num_bytes);    // Read data from QSPI flash memory
int32_t EXTROM_Write(__IO uint32_t src, __IO uint32_t dst, uint32_t num_bytes);   // Write data to QSPI flash memory

/* Callback functions */

#endif /*__FLASH_EXT_DRV_H */