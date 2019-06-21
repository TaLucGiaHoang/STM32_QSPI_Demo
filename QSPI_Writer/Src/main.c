/**
  ******************************************************************************
  * @file    QSPI/QSPI_ExecuteInPlace/Src/main.c
  * @author  MCD Application Team
  * @brief   This example describes how to configure and use QuadSPI through
  *          the STM32H7xx HAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fw_update_api.h"
#include "flash_drv.h"
#include "flash_ext_drv.h"
#include <string.h>
//#include "stm32h7xx_hal_flash_ex.h"
/** @addtogroup STM32H7xx_HAL_Examples
  * @{
  */

/** @addtogroup QSPI_ExecuteInPlace
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ROM_START_ADDRESS            0x08100000 // 0x08050000 //0x08100000// 0x08000000
#define ROM_END_ADDRESS              0x081FFFFF
#define ROM_FLASH_SIZE_2MB           0x200000
#define ROM_ERASE_BLOCK_SIZE_128KB   0x20000
#define ROM_WRITE_BLOCK_SIZE_32BYTE  32
#define ROM_READ_BLOCK_SIZE_4BYTE    4
#define RAM_START_ADDRESS          0x20000000

// #define QSPI_FLASH_SIZE_2MB        0x200000
// #define ERASE_BLOCK_SIZE_64KB      0x10000

#define READ_BLOCK_SIZE_64KB      64*1024//0x20000

/////////////////////
// #define EXTROM_START_ADDRESS         0x90000000
// #define EXTROM_ERASE_BLOCK_SIZE_64KB      0x10000
#define EXTROM_WRITE_BLOCK_SIZE_256BYTE   0x100 //32
// #define EXTROM_READ_BLOCK_SIZE_64KB      64*1024//0x10000
// #define EXTROM_READ_BLOCK_SIZE_128BYTE      128






/////////////////////

// FLASH_EraseProgram
/* Bank1 */
#define FLASH_USER_START_ADDR_1   ADDR_FLASH_SECTOR_2_BANK1      /* Start @ of user Flash area Bank1 */
#define FLASH_USER_END_ADDR_1     (ADDR_FLASH_SECTOR_0_BANK2 - 1) //(ADDR_FLASH_SECTOR_7_BANK1 - 1)  /* End @ of user Flash area Bank1*/
/* Bank2 */
#define FLASH_USER_START_ADDR_2   ADDR_FLASH_SECTOR_0_BANK2
#define FLASH_USER_END_ADDR_2     0x081FFFFF // ADDR_FLASH_SECTOR_7_BANK2 //(0x081FFFFF)  /* End @ of user Flash area Bank2*/
/* Private macro -------------------------------------------------------------*/
#pragma section =".qspi"
// #pragma section =".qspi_init"
#pragma section =".boot_loader"
#pragma section =".main"

/* Private variables ---------------------------------------------------------*/
QSPI_HandleTypeDef QSPIHandle;
static __IO uint8_t CmdCplt, RxCplt, TxCplt, StatusMatch;
// __IO uint8_t FlashReturn, FlashErrorReturn;
static uint8_t ram_data[8];//[4*EXTROM_WRITE_BLOCK_SIZE_256BYTE];
static uint8_t ram_data_rx[READ_BLOCK_SIZE_64KB];
uint8_t program_write[64] = "456789ABCDEFGHIKLMNOPQRSTUVW+-*/456789ABCDEFGHIKLMNOPQRSTUVW";  // (256 bits)
uint32_t uwStart, uwEnd;

static uint8_t uart_rx_buf[1024];
static uint8_t qspi_wr_buf[4096];

// FLASH_EraseProgram
uint32_t FirstSector = 0, NbOfSectors = 0;
uint32_t Address = 0, SECTORError = 0, Index = 0;
__IO uint32_t MemoryProgramStatus = 0;
__IO uint64_t data64 = 0;

uint64_t FlashWord[4] = { 0x0102030405060708,  // 8 bytes
                          0x1112131415161718,
                          0x2122232425262728,    
                          0x3132333435363738
                        };

/*Variable used for Erase procedure*/


/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static inline void Error_Handler(void);
static void CPU_CACHE_Enable(void);

static void Download_Task(void);
static void Update_Task(void);
static void boot_loader_update(void);
static inline void display_memory(uint32_t address);
static inline void display_qspi_memory(uint32_t address);
static inline void create_write_data(uint8_t* buf, uint32_t buf_size, uint8_t* str, uint32_t str_len);
static void main2(void);

static void BlinkLed(void);

// FLASH_EraseProgram
// static uint32_t GetSector(uint32_t Address);
static struct Program_ST {
  uint32_t size;
  __IO uint32_t *data;
} dump_program;

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  
  boot_loader_update();

  main2();

  /* Reset system */
  Reset_Handler();
}

// #pragma section =".boot_loader"
void boot_loader_update(void)
{
   
  /* Enable the CPU Cache */
  CPU_CACHE_Enable();
	
  /* STM32H7xx HAL library initialization:

       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 400 MHz */
  /* SystemClock_Config(); */
 
  BSP_LED_Init(LED1);
  // BSP_LED_Init(LED2);
  // BSP_LED_Init(LED3);
  
  printf("[BootLoader] Initialize, check new firmware\n");
  FWUPDATE_InitFlash();
  
  /* Initialize QuadSPI ------------------------------------------------------ */
  if(FWUPDATE_InitQSPI() != FWUPDATE_ERR_OK)
  {
    printf("FWUPDATE_InitQSPI error\n");
    Error_Handler();
  }
  
  FWUPDATE_Init();
  
  if(FWUPDATE_IsNewOS() == 1)
  {
    void (*fp)(void) = &main2;
    // uint32_t rom_addr = (uint32_t)fp;
    
    uint32_t rom_addr = 0x081D0000; // 0x08160000; // Example Sector 3 Bank 1
    
    // flash_addr = (uint8_t *)(__section_begin(".qspi_init"));
    
    printf("[BootLoader] Update\n");
    printf("[BootLoader] new main2 to ROM 0x%p\n", fp);
    BSP_LED_On(LED1);
    FWUPDATE_Update(0, rom_addr, 0);  // test update new firmware to 0x08160000
    HAL_Delay(1000);
    LED2_GPIO_PORT->ODR ^= LED1_PIN;
    
    /* Reset system */
    printf("[BootLoader] Restart\n");
    Reset_Handler();
  } else
  {
    printf("[BootLoader] Normal startup\n");
  }

}

uint8_t downldata[FW_DOWNLOAD_BUFFER_SIZE*4 + 500];
static void Download_Task(void)
{
  int ret = 0;
  // uint32_t program_size = FW_DOWNLOAD_BUFFER_SIZE*4 + 500;;
  const uint32_t qspi_buf_size = FW_DOWNLOAD_BUFFER_SIZE;
  
  
  // /* Receive data from uart */
  // /* Write to QSPI write buffer */

  // create_write_data(qspi_wr_buf, sizeof(qspi_wr_buf), "!@#$%^&*()ABCDQWER9876ABCD<>:][+", 32);
  
  
  uint32_t program_size = __section_size(".main");
  // uint32_t program_addr = (uint32_t)downldata;
  uint32_t program_addr = (uint32_t)(__section_begin(".main")); // examples 0x081e0001
  printf("New firmware size %d 0x%x\n", program_size, program_size);
  uint32_t count = program_size;

  
   // printf("program_size %d 0x%x \n", program_size, program_size);
  if(program_size%qspi_buf_size)
  {
    program_size = (program_size/qspi_buf_size)*qspi_buf_size + (((program_size%qspi_buf_size + qspi_buf_size-1)/qspi_buf_size)*qspi_buf_size);
printf("round up program_size %d 0x%x \n", program_size, program_size);
  }
  
 
  
  FWUPDATE_Download_Config(program_size, qspi_buf_size);

  do
  // for(int i = 0; i<6; i++)
  {
    if(count < qspi_buf_size)
    {
      memset(qspi_wr_buf, 0xff, qspi_buf_size); // clearn buffer
      memcpy(qspi_wr_buf, (void*)program_addr, count);
    } else 
    {
      ;
    }
    
    ret = FWUPDATE_Download((uint32_t)&qspi_wr_buf[0]); // Write 4096 bytes to QSPI
    
    program_addr  += qspi_buf_size;
    count -= qspi_buf_size;
    
    if(ret == FWUPDATE_ERR_OK)
    {
      printf("[Download_Task] OK\n");
      break;
    } else if(ret == FWUPDATE_ERR_BUSY)
    {
      printf("[Download_Task] busy\n");
    } else
    {
      printf("[Download_Task] Error %d\n", ret);
      if(ret == FWUPDATE_ERR_NOT_INITIALIZED) printf("[Download_Task] FWUPDATE_ERR_NOT_INITIALIZED\n", ret);
      if(ret == FWUPDATE_ERR_PARAM) printf("[Download_Task] FWUPDATE_ERR_PARAM\n", ret);
      if(ret == FWUPDATE_ERR_FATAL) printf("[Download_Task] FWUPDATE_ERR_FATAL\n", ret);
      BSP_LED_On(LED3);
      while(1);
    }
  } while(count > 0);
  
  // display_qspi_memory(0x90200000);
  // display_qspi_memory(EXTROM_AREA_2_ADDRESS+0x0fff0);

}

static void Update_Task(void)
{
  FWUPDATE_Update(EXTROM_AREA_2_ADDRESS, 0x08160000, 4096*4);
}


static void main2(void) @ ".main"
{
  
  /* Enable the CPU Cache */
  CPU_CACHE_Enable();
  /* STM32H7xx HAL library initialization:

       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 400 MHz */
  /* SystemClock_Config(); */
 
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED2);
  // BSP_LED_On(LED2);
  
  printf("[main] Initialze\n");
  FWUPDATE_InitFlash();
  
  /* Initialize QuadSPI ------------------------------------------------------ */
  if(FWUPDATE_InitQSPI() != FWUPDATE_ERR_OK)
  {
    printf("FWUPDATE_InitQSPI error\n");
    Error_Handler();
  }
  
  // FWUPDATE_Init();

  // /* Erase 2MB of QSPI Flash - Area 1+2*/  
  // uwStart = HAL_GetTick();
  // if(EXTROM_Erase(EXTROM_AREA_1_ADDRESS, 2*EXTROM_AREA_SIZE_2MB) != FLASH_ERR_OK)
  // {
    // printf("EXTROM_Erase 1 error\n");
    // Error_Handler();
  // }
  // uwEnd = HAL_GetTick();
  
  // (uwEnd - uwStart > 1) ? printf("Erase all QSPI: %ld : %ld (%ld ms) \n", uwStart, uwEnd, uwEnd - uwStart) : printf("Erase all QSPI: %ld : %ld ( < 1 ms) \n", uwStart, uwEnd);

  // printf("\nStart Hardware to backup v1 ... (not implemented)\n");
  // FWUPDATE_Init();
  // printf("\nReset Hardware to run v1 ... (not implemented)\n");
  // FWUPDATE_Init();
  // printf("\nReset Hardware to download v2 ... (not implemented)\n");
  // FWUPDATE_Init();
  Download_Task(); // v2
  // printf("\nReset Hardware to update v2 ... (not implemented)\n");
  // FWUPDATE_Init();
  // Update_Task();  // v2 
  // printf("\nReset Hardware to download v3 ... (not implemented)\n");
  // FWUPDATE_Init();
  // Download_Task(); // v3
  // printf("\nReset Hardware to update v3 ... (not implemented)\n");
  // FWUPDATE_Init();
  // Update_Task();   // v3
  // printf("\nReset Hardware to download v4 ... (not implemented)\n");
  // FWUPDATE_Init();
  // Download_Task(); // v4
  // printf("\nReset Hardware to update v4 ... (not implemented)\n");
  // FWUPDATE_Init();
  // Update_Task();   // v4
  // printf("\nReset Hardware to download v5 ... (not implemented)\n");
  // FWUPDATE_Init();
  // Download_Task(); // v5
  // printf("\nReset Hardware to update v5 ... (not implemented)\n");
  // FWUPDATE_Init();
  // Update_Task();   // v5
  // printf("\nReset Hardware to run v5... (not implemented)\n");
  // FWUPDATE_Init();
  // printf("\nReset Hardware to run v5... (not implemented)\n");
  // FWUPDATE_Init();
  // printf("\nReset Hardware to run v5... (not implemented)\n");
  // FWUPDATE_Init();




  // __IO uint32_t qspi_addr = 0x90000000;  
  // /* Erase 2MB of QSPI Flash - Area 1+2*/  
  // uwStart = HAL_GetTick();
  // if(EXTROM_Erase(EXTROM_AREA_1_ADDRESS, EXTROM_AREA_SIZE_2MB) != FLASH_ERR_OK)
  // {
    // printf("EXTROM_Erase 1 error\n");
    // Error_Handler();
  // }
  // uwEnd = HAL_GetTick();
  
  // (uwEnd - uwStart > 1) ? printf("Erasing Sequence1: %ld : %ld (%ld ms) \n", uwStart, uwEnd, uwEnd - uwStart) : printf("Erasing Sequence1: %ld : %ld ( < 1 ms) \n", uwStart, uwEnd);

  
  // uwStart = HAL_GetTick();
  // if(EXTROM_Erase(EXTROM_AREA_2_ADDRESS, EXTROM_AREA_SIZE_2MB) != FLASH_ERR_OK)
  // {
    // printf("EXTROM_Erase error 0x%08x\n", EXTROM_AREA_2_ADDRESS);
    // Error_Handler();
  // }
  // uwEnd = HAL_GetTick();
  
  // (uwEnd - uwStart > 1) ? printf("Erasing Sequence2: %ld : %ld (%ld ms) \n", uwStart, uwEnd, uwEnd - uwStart) : printf("Erasing Sequence2: %ld : %ld ( < 1 ms) \n", uwStart, uwEnd);


  
  // /* Write 2MB of QSPI Flash - Area 1 */
  // uwStart = HAL_GetTick();
  // for(int address = EXTROM_AREA_1_ADDRESS; 
      // address <  (EXTROM_AREA_1_ADDRESS + EXTROM_AREA_SIZE_2MB - 1); 
      // address += sizeof(ram_data))
  // {
    // if(EXTROM_Write((uint32_t)&ram_data[0], address , sizeof(ram_data) ) != FLASH_ERR_OK)
    // {
      // printf("EXTROM_Write 1 error\n");
      // Error_Handler();
    // }
  // }
  // uwEnd = HAL_GetTick();
  // (uwEnd - uwStart > 1) ? printf("Writing Sequence1: %ld : %ld (%ld ms) \n", uwStart, uwEnd, uwEnd - uwStart) : printf("Writing Sequence: %ld : %ld ( < 1 ms) \n", uwStart, uwEnd);
  
  
  /* Write 2MB of QSPI Flash - Area 2 */
  // uwStart = HAL_GetTick();
  // /* for(int address = EXTROM_AREA_2_ADDRESS; 
      // address < (EXTROM_AREA_2_ADDRESS + EXTROM_AREA_SIZE_2MB - 1) ; 
      // address += sizeof(ram_data )) */
  // for(int address = EXTROM_AREA_2_ADDRESS; 
      // address < (EXTROM_AREA_2_ADDRESS + 4*sizeof(ram_data)) ; 
      // address += sizeof(ram_data ))
  // {
    // if(EXTROM_Write((uint32_t)&ram_data[0], address , sizeof(ram_data) ) != FLASH_ERR_OK)
    // {
      // printf("EXTROM_Write error 0x%08x\n", address);
      // Error_Handler();
    // }
    
    // display_qspi_memory(address);
  // }
  // uwEnd = HAL_GetTick();
  // (uwEnd - uwStart > 1) ? printf("Writing Sequence2: %ld : %ld (%ld ms) \n", uwStart, uwEnd, uwEnd - uwStart) : printf("Writing Sequence2: %ld : %ld ( < 1 ms) \n", uwStart, uwEnd);
  
 


  // /* Read 2MB of QSPI Flash - Area 1*/
  // uwStart = HAL_GetTick();
  // for(int address = EXTROM_AREA_1_ADDRESS; 
     // address < (EXTROM_AREA_1_ADDRESS + EXTROM_AREA_SIZE_2MB - 1); 
     // address += sizeof(ram_data_rx) )
  // {
    // if(EXTROM_Read(address, (uint32_t)&ram_data_rx[0], 64*1024) != FLASH_ERR_OK)
    // {
      // printf("EXTROM_Read 1 error 0x%08x\n", address);
      // Error_Handler();
    // }
  // }
  // uwEnd = HAL_GetTick();
  // (uwEnd - uwStart > 1) ? printf("Reading Sequence 1: %ld : %ld (%ld ms) \n", uwStart, uwEnd, uwEnd - uwStart) : printf("Reading Sequence 1: %ld : %ld ( < 1 ms) \n", uwStart, uwEnd);

  // display_memory((uint32_t)&ram_data_rx[0]);
  
  // /* Read 2MB of QSPI Flash - Area 2*/
  // uwStart = HAL_GetTick();
  // for(int address = EXTROM_AREA_2_ADDRESS; 
     // address < (EXTROM_AREA_2_ADDRESS + 4096 /* EXTROM_AREA_SIZE_2MB */ - 1); 
     // address += sizeof(ram_data_rx) )
  // {
    // if(EXTROM_Read(address, (uint32_t)&ram_data_rx[0], 64*1024) != FLASH_ERR_OK)
    // {
      // printf("EXTROM_Read 2 error 0x%08x\n", address);
      // Error_Handler();
    // }
  // }
  // uwEnd = HAL_GetTick();
  // (uwEnd - uwStart > 1) ? printf("Reading Sequence 2: %ld : %ld (%ld ms) \n", uwStart, uwEnd, uwEnd - uwStart) : printf("Reading Sequence 2: %ld : %ld ( < 1 ms) \n", uwStart, uwEnd);
  
  
////////////////////////////
  // /* -3- Erase the user Flash area (area defined by FLASH_USER_START_ADDR_1 and FLASH_USER_END_ADDR_1) ***********/
  // uwStart = HAL_GetTick();
  // if(FLASH_Erase(0x08040000 , 0x080fffff+1-0x08040000 ) != FLASH_ERR_OK)
  // {
    // /* Infinite loop */
    // Flash_Print_Error();
    // while (1)
    // {
      // BSP_LED_On(LED3);
    // }
  // }
  // uwEnd = HAL_GetTick();
  // (uwEnd - uwStart > 1) ? printf("Erase Bank1: %ld : %ld (%ld ms) \n", uwStart, uwEnd, uwEnd - uwStart) : printf("Erase Bank1: %ld : %ld ( < 1 ms) \n", uwStart, uwEnd);
  
   
  /* -3.2 - Erase the user Flash area (bank2)
    (area defined by FLASH_USER_START_ADDR_1 and FLASH_USER_END_ADDR_1) ***********/
  // uwStart = HAL_GetTick();
  // if(FLASH_Erase(0x08100000, 0x081FFFFF-0x08100000+1) == FLASH_ERR_FATAL)
  // {
    // /* Infinite loop */
    // Flash_Print_Error();
    // while (1)
    // {
      // BSP_LED_On(LED3);
    // }
  // }
  // uwEnd = HAL_GetTick();
  // HAL_FLASH_Lock();
  // (uwEnd - uwStart > 1) ? printf("Erase Bank2: %ld : %ld (%ld ms) \n", uwStart, uwEnd, uwEnd - uwStart) : printf("Erase Bank2: %ld : %ld ( < 1 ms) \n", uwStart, uwEnd);
  
  
  
  
  // /* -4- Program the user Flash area word by word
    // (area defined by FLASH_USER_START_ADDR_1 and FLASH_USER_END_ADDR_1) ***********/
  // Address = FLASH_USER_START_ADDR_1;
  // uwStart = HAL_GetTick();

  // for(Address = 0x08040000; Address < 0x080fffff+1; Address += sizeof(FlashWord))
  // {
    // if (FLASH_Write((uint32_t)&FlashWord[0], Address, sizeof(FlashWord)) != FLASH_ERR_OK) // 0x08040000 to 0x080fffff (0xc0000 bytes)
    // {
      // BSP_LED_On(LED3);
    // }
  // }
  // uwEnd = HAL_GetTick();
  // (uwEnd - uwStart > 1) ? printf("Program Bank1: %ld : %ld (%ld ms) \n", uwStart, uwEnd, uwEnd - uwStart) : printf("Program Bank1: %ld : %ld ( < 1 ms) \n", uwStart, uwEnd);

  // /* -4.2- Program the user Flash area word by word (Bank2)
    // (area defined by FLASH_USER_START_ADDR_1 and FLASH_USER_END_ADDR_1) ***********/
  // Address = FLASH_USER_START_ADDR_2;
  // uwStart = HAL_GetTick();

  // for(Address = FLASH_USER_START_ADDR_2; Address < (FLASH_USER_END_ADDR_2+1); Address += sizeof(FlashWord))
  // {
    // if (FLASH_Write((uint32_t)&FlashWord[0], Address, sizeof(FlashWord)) != FLASH_ERR_OK)
    // {
      // BSP_LED_On(LED3);
    // }
  // }
  // uwEnd = HAL_GetTick();
  // (uwEnd - uwStart > 1) ? printf("Program Bank2: %ld : %ld (%ld ms) \n", uwStart, uwEnd, uwEnd - uwStart) : printf("Program Bank2: %ld : %ld ( < 1 ms) \n", uwStart, uwEnd);

  // /* Read 2MB Flash */
  // uwStart = HAL_GetTick();
  // FLASH_Read(FLASH_USER_START_ADDR_1, (uint32_t)ram_data_rx, sizeof(ram_data_rx));
  // uwEnd = HAL_GetTick();
  // (uwEnd - uwStart > 1) ? printf("Read Bank1 & Bank2: %ld : %ld (%ld ms) \n", uwStart, uwEnd, uwEnd - uwStart) : printf("Read Bank1 & Bank2: %ld : %ld ( < 1 ms) \n", uwStart, uwEnd);

  
  
  /* -6- (Bank1) Check if the programmed data is OK
      MemoryProgramStatus = 0: data programmed correctly
      MemoryProgramStatus != 0: number of words not programmed correctly ******/
  // printf("Compare data write to 0x%08x with read from 0x%08x\n", FLASH_USER_START_ADDR_1, (uint32_t)ram_data_rx);
  
  // Address = (uint32_t)ram_data_rx; //FLASH_USER_START_ADDR_1;
  // MemoryProgramStatus = 0x0;
  
  // while (Address < ((uint32_t)ram_data_rx + sizeof(ram_data_rx)) ) //FLASH_USER_END_ADDR_2)
  // {
    // for(Index = 0; Index<4; Index++)
    // {
      // data64 = *(uint64_t*)Address;
      // __DSB();
      // if(data64 != FlashWord[Index])
      // {
        // MemoryProgramStatus++;
      // }
      // Address +=8; // 64bit 8byte
    // }
  // }
  // (MemoryProgramStatus == 0) ? printf("OK\n") : printf("%d errors\n", MemoryProgramStatus);

  // Address = (uint32_t)ADDR_FLASH_SECTOR_0_BANK1; 
  // while (Address < ((uint32_t)0x08100000))
  // {
    // for(Index = 0; Index<4; Index++)
    // {
      // data64 = *(uint64_t*)Address;
      // __DSB();
      // *(uint64_t*)(Address+0x100000) = data64;
      // __DSB();
      // Address +=8; // 64bit 8byte
    // }
  // }


////////////////////////////////
  // printf("Blink LED\n");
  // /* Execute the code from QSPI memory ------------------------------- */
  // BlinkLed();

  // while(1);
  
  printf("\n\nRestart for updating...\n");
  HAL_Delay(10000);
  
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE BYPASS)
  *            SYSCLK(Hz)                     = 400000000 (CPU Clock)
  *            HCLK(Hz)                       = 200000000 (AXI and AHBs Clock)
  *            AHB Prescaler                  = 2
  *            D1 APB3 Prescaler              = 2 (APB3 Clock  100MHz)
  *            D2 APB1 Prescaler              = 2 (APB1 Clock  100MHz)
  *            D2 APB2 Prescaler              = 2 (APB2 Clock  100MHz)
  *            D3 APB4 Prescaler              = 2 (APB4 Clock  100MHz)
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 4
  *            PLL_N                          = 400
  *            PLL_P                          = 2
  *            PLL_Q                          = 4
  *            PLL_R                          = 2
  *            VDD(V)                         = 3.3
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;
  
  /*!< Supply configuration update enable */
  MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;

  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    Error_Handler();
  }
  
/* Select PLL as system clock source and configure  bus clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_PCLK1 | \
                                 RCC_CLOCKTYPE_PCLK2  | RCC_CLOCKTYPE_D3PCLK1);

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;  
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2; 
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2; 
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2; 
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
  if(ret != HAL_OK)
  {
    Error_Handler();
  }
}



static void display_qspi_memory(uint32_t address)
{
  uint8_t qspi_buf[256];
  
  if((address >= EXTROM_AREA_1_ADDRESS) && (address < EXTROM_AREA_2_ADDRESS + EXTROM_AREA_SIZE_2MB))
  {
    if(EXTROM_Read(address, (uint32_t)&qspi_buf[0], 256) != FLASH_ERR_OK)
    {
      printf("display_qspi_memory error 0x%08x\n", address);
      Error_Handler();
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


static inline void create_write_data(uint8_t* buf, uint32_t buf_size, uint8_t* str, uint32_t str_len)
{
  memset(buf, 'x', buf_size);
  for(int i = 0; i < buf_size; i += str_len)
  {
    memcpy(buf, str, str_len);
    buf += str_len;
  }
}
static inline void display_memory(uint32_t address)
{
  uint8_t* p = (uint8_t*)address;
  printf("\n");
  for (int i = 0; i < 10; i++)
  {
    printf("%d.[%08p] %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x    %c%c%c%c %c%c%c%c %c%c%c%c %c%c%c%c\n",
             i, p, 
             p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15],
             p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15] );
    p += 16;	
  }
  printf("\n");
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static inline void Error_Handler(void)
{
  BSP_LED_On(LED3);

  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @brief  Blink LED1
  * @param  None
  * @retval None
  */
static void BlinkLed(void) // @ ".qspi"
{
  volatile uint32_t i;
  while (1) {
    // for (i = 0; i < 10000000; i++);
    HAL_Delay(500); // comment SystemClock_Config
    // HAL_Delay(300);
    //BSP_LED_Toggle(LED1);
    LED1_GPIO_PORT->ODR ^= LED1_PIN;
  }
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
