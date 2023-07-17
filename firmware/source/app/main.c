/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Aleksandr Shabalin       <alexnv97@gmail.com>
 * @brief          : Main file
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin------------------- *
 ******************************************************************************
 ******************************************************************************
 */

/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_gpio.h"

#include "led.h"
#include "indication.h"
#include "log.h"
#include "crc.h"
#include "io_uart.h"
#include "fram.h"
#include "ext_flash.h"

#include "lwprintf/lwprintf.h"

/******************************************************************************/
/* Private defines ---------------------------------------------------------- */
/******************************************************************************/
#define START_ADDRESS           (0x08008000) //Flash Sector 2 start address

#define SRAM_SIZE               (128 * 1024) //128 Kbytes SRAM
#define SRAM_END                (SRAM_BASE + SRAM_SIZE)

#define FLASH_SIZE              (1024 * 1024)
#define FLASH_SECTORS_2_TO_4    (3)
#define FLASH_SECTOR_SIZE       (128 * 1024) // Sectors from 5 have 128 KBytes

#define ENABLE_PROTECTION       (0u)

#define NEW_FIRMWARE                  (true)
#define BACKUP_FIRMWARE               (false)

/******************************************************************************/
/* Private variables -------------------------------------------------------- */
/******************************************************************************/
typedef enum
{
  BOOTLOADER_OK = 0,                 /* No error                                 */
  BOOTLOADER_ERROR_NO_APP,           /* No application found in flash            */
  BOOTLOADER_ERROR_VERIFICATION,     /* Incorrect type of installing application */
  BOOTLOADER_ERROR_CHECKSUM,         /* Application checksum match error         */
  BOOTLOADER_ERROR_ERASE,            /* Flash erase error                        */
  BOOTLOADER_ERROR_WRITE,            /* Flash write error                        */
  BOOTLOADER_ERROR_OPTION_BYTES,     /* Flash option bytes programming error     */
  BOOTLOADER_MAX_ERRORS              /* Reached maximum installing errors before */
} BOOTLOADER_ERROR;

typedef struct
{
  uint32_t magic_number;           /* Image random magic number                             */
  uint8_t  reserved5;
  uint8_t  header_version;         /* Header version                                        */
  uint16_t firmware_version[7];    /* Firmware version                                      */
  uint32_t firmware_length;        /* Firmware length without header & with 0xFF in the EOF */
  uint32_t firmware_crc32;         /* Firmware CRC32 without header & with 0xFF in the EOF  */
  uint32_t image_length;           /* Full image length with the header                     */
} FIRMWARE_NOT_ENCRYPTED_HEADER;

typedef struct
{
  uint32_t image_load_address;     /* Flash Sector 2 Address             */
  uint32_t firmware_entry_point;   /* Flash Sector 2 Address + fw header */
  uint32_t reserved1[64 / 4];      /* image_signature   (in future)      */
  uint32_t reserved2;              /* option_flags      (in future)      */
  uint32_t reserved3;
  uint32_t reserved4;
  uint32_t unused[3];              /* Used for align struct size for aes-128 (multiple of 16) */
} FIRMWARE_ENCRYPTED_HEADER;

typedef struct
{
  FIRMWARE_NOT_ENCRYPTED_HEADER not_encrypted_header;
  FIRMWARE_ENCRYPTED_HEADER     encrypted_header;
} FIRMWARE_HEADER;

typedef struct
{
  uint32_t header_size_in_bytes;
  uint32_t fw_size_in_4words;
  uint32_t fw_size_in_words;
  uint8_t  fw_tail_in_words;
  uint8_t  fw_tail_in_bytes;
  uint8_t  last_encrypted_fw_bytes;
} FIRMWARE_SIZE;

volatile FIRMWARE_SIZE fw_size;
volatile FIRMWARE_HEADER fw_header;

uint32_t flash_ptr; // Internal MCU Flash programming pointer

uint8_t AES_KEY[] = { 0x4D, 0x61, 0x73, 0x74, 0x65, 0x72, 0x69, 0x6E,
                      0x67, 0x20, 0x20, 0x53, 0x54, 0x4D, 0x33, 0x32 };

/******************************************************************************/
/* Private function prototypes ---------------------------------------------- */
/******************************************************************************/
void prvInitializeSystem(void);
void prvSystemClockConfig(void);
void prvGPIOConfig(void);
bool prvCheckForInstall(bool new_fw);
uint8_t prvInstallFW(bool new_fw);
bool prvCheckInstallErrors(void);
bool prvModifyFlagInFram(uint16_t flag_address, bool state);


/******************************************************************************/


/**
 * @brief          Main endless cycle
 */
int main(void)
{
  prvInitializeSystem();

  for(;;);
}
/******************************************************************************/




/**
 * @brief          MCU initialization fns
 */
void prvInitializeMCU(void)
{
  HAL_Init();
  prvSystemClockConfig();
  prvGPIOConfig();
  IndicationInit();
  CRCInit();
  IoUartInit();
  FRAMInit();
  LogInit();

  if (!ExtFlashInit())
    if (!ExtFlashInit())
      if (!ExtFlashInit())
        PrintfLogsCRLF("Flash ERROR!!!");


#if ENABLE_MCU_PROTECTIONS
  prvCheckAndSetFlashProtection();
#endif

  PrintfLogsCRLF("ESS control board booting...");

  /* Bootloader execution: check for update flags in FRAM or jump to main application */
  if (prvCheckForInstall(NEW_FIRMWARE))
  {
    if (prvInstallFW(NEW_FIRMWARE) != BOOTLOADER_OK)
      NVIC_SystemReset();
  }
  else if (prvCheckForInstall(BACKUP_FIRMWARE))
  {
    if (prvInstallFW(BACKUP_FIRMWARE) != BOOTLOADER_OK)
      NVIC_SystemReset();
  }
  else
  {
    /*
     * Flags are not set in FRAM. We first check if the firmware_entry_point address field
     * of image header starting from FW_START_ADDRESS contains the MSP (end of SRAM).
     * If not, the Red LED blinks quickly.
     */
    if (prvCheckForApplication() == BOOTLOADER_ERROR_NO_APP)
    {
      PrintfLogsCRLF("ERROR: firmware is not found!");
      IndicationLedRed(5);

      while(1)
      {
      }
    }
  }
}
/******************************************************************************/




/**
 * @brief  This function installs new or backup firmware.
 * @return Bootloader error code ::BOOTLOADER_ERROR_CODE
 */
uint8_t prvInstallFW(bool new_fw)
{
  FIRMWARE_HEADER *pheader = (FIRMWARE_HEADER *)&fw_header;
  FIRMWARE_NOT_ENCRYPTED_HEADER *pneheader = (FIRMWARE_NOT_ENCRYPTED_HEADER *)&pheader->not_encrypted_header;

  if (prvCheckInstallErrors())
  {
    prvModifyFlagInFram(new_fw ? FRAM_OFFSET_OTA_FLAG : FRAM_OFFSET_BACKUP_FLAG, RESET);
    prvModifyFlagInFram(new_fw ? FRAM_OFFSET_SUCCESS_OTA_INSTALL_FLAG : FRAM_OFFSET_SUCCESS_BACKUP_INSTALL_FLAG, RESET);

    return BOOTLOADER_MAX_ERRORS;
  }

  if (new_fw)
    PrintfLogsCRLF("Installing new update...");
  else
    PrintfLogsCRLF("Restoring firmware...");

  //Get FW header from external flash
  ExtFlashReadArray(SELECT_FAST_READ,
                     new_fw ? NEW_FW_HEADER_ADDRESS: BACKUP_FW_HEADER_ADDRESS,
                     (void *)pheader,
                     sizeof(FIRMWARE_HEADER));

  return BOOTLOADER_OK;
}
/******************************************************************************/




/**
 * @brief  This function checks the OTA update flag or Backup restore flag in FRAM.
 * @retval bool: Install flag is set or reset
 */
bool prvCheckForInstall(bool new_fw)
{
  uint8_t update_flag;

  FRAMReadByte(new_fw ? FRAM_OFFSET_OTA_FLAG: FRAM_OFFSET_BACKUP_FLAG, &update_flag);

  if (update_flag == 0x01)
    return (true);
  else
    return (false);
}
/******************************************************************************/




/**
 * @brief  This function checks if the install error counter has reached 0 or not
 * @retval bool: true  if the max number of install errors has been reached
 *                     (error counter = 0);
 *               false if install errors counter != 0
 */
bool prvCheckInstallErrors(void)
{
  uint8_t errors_counter = 0x00;

  FRAMReadByte(FRAM_OFFSET_INSTALL_ERRORS_COUNT, &errors_counter);

  if (errors_counter == 0x00)
    return (true);
  else
    return (false);
}
/******************************************************************************/




/**
 * @brief  This function modifies one of the 3 flags in FRAM
 *         used by the bootloader
 * @retval bool: success or not
 */
bool prvModifyFlagInFram(uint16_t flag_address, bool state)
{
  uint8_t flag = 0x02;           /* true/false != 0x02 */

  /*
   * Modify Update Flag in FRAM
   * Temporary solution. It is necessary to call two functions, since the "last function"
   * for some unknown reason sends a NACK at the end. The functions BEFORE the "last
   * function" work correctly. See signal via logic analyzer
   */
  FRAMWriteByte(flag_address, state);
  FRAMWriteByte(flag_address, state);


  /* Make sure the flag has cleared */
  FRAMReadByte(flag_address, &flag);
  if (flag != state)
  {
    PrintfLogsCRLF("ERROR: Bootloader's flag in FRAM is not modified!");
    return false;
  }

  return true;
}
/******************************************************************************/




/**
 * @brief          System clock configuration
 */
void prvSystemClockConfig(void)
{
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  __DSB();

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_5);

  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

  LL_RCC_HSE_Enable();

  /* Wait till HSE is ready */
  while (!LL_RCC_HSE_IsReady());

  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_25, 336, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

  /* Wait till PLL is ready */
  while (!LL_RCC_PLL_IsReady());

  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

  /* Wait till System clock is ready */
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

  /* Init 1ms SysTick */
  LL_Init1msTick(168000000);
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SetSystemCoreClock(168000000);

  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  LL_SYSTICK_EnableIT();
}
/******************************************************************************/




/**
 * @brief          GPIO configuration
 */
void prvGPIOConfig(void)
{
  // Config Clocking of all GPIO ports
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  __DSB();
}
/******************************************************************************/





/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{

}
/******************************************************************************/



/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
  while (1)
  {

  }
}
/******************************************************************************/



/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{
  while (1)
  {

  }
}
/******************************************************************************/



/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler(void)
{
  while (1)
  {

  }
}
/******************************************************************************/



/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{
  while (1)
  {

  }
}
/******************************************************************************/



/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void)
{

}
/******************************************************************************/



/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
  HAL_IncTick();
  IndicationLedsUpdate();
}
/******************************************************************************/
