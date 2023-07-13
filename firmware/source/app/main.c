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
void prvInitializeMCU(void);
void prvSystemClockConfig(void);
void prvSystemHandlerControlConfig(void);
void prvGPIOConfig(void);


/******************************************************************************/


/**
 * @brief          Main endless cycle
 */
int main(void)
{
  prvInitializeMCU();

  for(;;);
}
/******************************************************************************/




/**
 * @brief          MCU initialization fns
 */
void prvInitializeMCU(void)
{
  HAL_Init();
  prvSystemHandlerControlConfig();
  prvSystemClockConfig();
  prvGPIOConfig();
  IndicationInit();
}
/******************************************************************************/




/**
 * @brief          System clock configuration
 */
void prvSystemClockConfig(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_5);

  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

  LL_RCC_HSE_Enable();

  /* Wait till HSE is ready */
  while (!LL_RCC_HSE_IsReady());

  LL_RCC_LSI_Enable();

   /* Wait till LSI is ready */
  while(LL_RCC_LSI_IsReady() != 1)

  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_25, 336, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_25, 336, LL_RCC_PLLQ_DIV_7);

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

  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
  LL_SYSTICK_EnableIT();
}
/******************************************************************************/




/**
 * @brief          System handler control and state register config
 */
void prvSystemHandlerControlConfig(void)
{
  //TODO ask about this code
  SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;
  NVIC_SetPriority(MemoryManagement_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));

  SCB->SHCSR |= SCB_SHCSR_BUSFAULTENA_Msk;
  NVIC_SetPriority(BusFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));

  SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk;
  NVIC_SetPriority(UsageFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));

  NVIC_SetPriority(SVCall_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));
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
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
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
  /* Clear overflow flag */
//  SysTick->CTRL;
  /* Check leds status */
  IndicationLedsUpdate();

#if (INCLUDE_xTaskGetSchedulerState == 1 )
if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
{
#endif /* INCLUDE_xTaskGetSchedulerState */
  xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
}
#endif /* INCLUDE_xTaskGetSchedulerState */
}
/******************************************************************************/






