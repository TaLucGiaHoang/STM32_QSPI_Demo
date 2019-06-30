/**
  ******************************************************************************
  * @file           : uart_interface.c
  * @brief          : 
  ******************************************************************************

  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_INTERFACE_H
#define __UART_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stm32h7xx_nucleo_144.h"
/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Public function prototypes -----------------------------------------------*/
void uart_init(void);

int uart_read_8bit(uint8_t *data8, uint32_t Timeout);
int uart_read_32bit(uint32_t *data32, uint32_t Timeout);
int uart_read_64bit(uint64_t *data64, uint32_t Timeout);
int uart_read_512byte(uint8_t *pData);
int uart_read_buffer(uint8_t *pData, uint16_t Size, uint32_t Timeout);
void uart_print_msg(uint8_t* msg, uint32_t len);
void uart_print_msg_list(void);

#ifdef __cplusplus
}
#endif

#endif /* __UART_INTERFACE_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/