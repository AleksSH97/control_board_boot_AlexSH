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
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_i2c.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_crc.h"
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

#include "aes.h"

#include "lwprintf/lwprintf.h"

/******************************************************************************/
/* Private defines ---------------------------------------------------------- */
/******************************************************************************/
#define FW_START_ADDRESS        (0x08008000) //Flash Sector 2 start address

#define SRAM_SIZE               (128 * 1024) //128 Kbytes SRAM
#define SRAM_END                (SRAM_BASE + SRAM_SIZE)

#define FLASH_SIZE              (1024 * 1024)
#define FLASH_SECTORS_2_TO_4    (3)
#define FLASH_SECTOR_SIZE       (128 * 1024) // Sectors from 5 have 128 KBytes

#define ENABLE_PROTECTION       (0u)

#define NEW_FIRMWARE            (true)
#define BACKUP_FIRMWARE         (false)

#define IMAGE_MAGIC_NUMBER      (0x414C4558) // Magic number: 'A' 'L' 'E' 'X' on ASCII

/* Crypto key (0xAB, 0xCD, ...) and it's initialization vector (0xBA, 0xDC, ...) */
uint8_t aes_fw_key[] = { 0x42, 0x26, 0x45, 0x29, 0x48, 0x2B, 0x4D, 0x62,
                    0x51, 0x65, 0x54, 0x68, 0x57, 0x6D, 0x5A, 0x71  };

uint8_t aes_init_vector[]     = { 0x24, 0x62, 0x54, 0x92, 0x84, 0xB2, 0xD4, 0x26,
                    0x15, 0x56, 0x45, 0x86, 0x75, 0xD6, 0xA5, 0x17  };

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
  BOOTLOADER_ERROR_DECREMENT,
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

bool prvExtFlashInit(void);

bool prvModifyFlagInFram(uint16_t flag_address, bool state);
uint8_t prvDecrementInstallErrorsCountInFram(void);

uint8_t prvInstallFW(bool new_fw);
uint8_t prvVerifyFW(FIRMWARE_NOT_ENCRYPTED_HEADER *pneheader);
bool prvCheckForInstall(bool new_fw);
bool prvCheckInstallErrors(void);
uint8_t prvCheckForApplication(void);

void prvDecryptPartOfHeader(FIRMWARE_HEADER * const pheader);
void prvPrintHeader(const FIRMWARE_HEADER *pheader);
void prvFillFWSize(FIRMWARE_NOT_ENCRYPTED_HEADER *pneheader);
uint8_t prvCheckFwCRC(bool new_fw, const FIRMWARE_HEADER *pheader);

uint8_t prvProgramFwToMcuFlash(bool new_fw, FIRMWARE_HEADER * const pheader);
uint8_t prvEraseMcuFlash(const FIRMWARE_HEADER *pheader);
uint8_t prvBeginProgramMcuFlash(const FIRMWARE_HEADER *pheader);
uint8_t prvNextProgramMcuFlash(const FIRMWARE_HEADER *pheader, void *data);
uint8_t prvEndProgramMcuFlash(void);

void prvDeInitSystem(void);
void prvDeInitGpios(void);


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

  if(!prvExtFlashInit())
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
  uint8_t res = BOOTLOADER_OK;
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

  // Get FW header from external flash
  ExtFlashReadArray(SELECT_FAST_READ,
                     new_fw ? NEW_FW_HEADER_ADDRESS : BACKUP_FW_HEADER_ADDRESS,
                     (void *)pheader,
                     sizeof(FIRMWARE_HEADER));

  // Verifying firmware
  res = prvVerifyFW(pneheader);

  if (res != BOOTLOADER_OK)
    return res;

  // Decrypt encrypted part of the header
  prvDecryptPartOfHeader(pheader);

  // Printf info about decrypted header
  prvPrintHeader(pheader);

  // Fill FIRMWARE SIZE struct for calculations
  prvFillFWSize(pneheader);

  // Check CRC32
  PrintfLogsCRLF("Checking CRC32 of the firmware...");
  if (prvCheckFwCRC(new_fw ? true : false, pheader) == BOOTLOADER_ERROR_CHECKSUM)
  {
    PrintfLogsCRLF(CLR_RD"ERROR: Calculated CRC32 is not matches the one in the image header!"CLR_DEF);
    prvDecrementInstallErrorsCountInFram();
    return BOOTLOADER_ERROR_CHECKSUM;
  }
  PrintfLogsCRLF(CLR_GR"OK"CLR_DEF);

  // Erase FLASH before programming
  PrintfLogsCRLF("Erasing MCU Flash...");
  if (prvEraseMcuFlash(pheader) == BOOTLOADER_ERROR_ERASE)
  {
    PrintfLogsCRLF(CLR_RD"ERROR: STM32 Flash erase fail!"CLR_DEF);
    prvDecrementInstallErrorsCountInFram();

    return BOOTLOADER_ERROR_ERASE;
  }
  PrintfLogsCRLF(CLR_GR"OK"CLR_DEF);

  // Install firmware to STM32 flash
  PrintfLogsCRLF("Installing firmware...");
  if (prvProgramFwToMcuFlash(new_fw ? true : false, pheader) == BOOTLOADER_ERROR_WRITE)
  {
    prvDecrementInstallErrorsCountInFram();

    return (BOOTLOADER_ERROR_WRITE);
  }
  PrintfLogsCRLF(CLR_GR"OK"CLR_DEF);

  // Ending of the firmware installing
  if (prvModifyFlagInFram(new_fw ? FRAM_OFFSET_OTA_FLAG : FRAM_OFFSET_BACKUP_FLAG, RESET))
  {
    // Set Success Install Flag for main firmware
    prvModifyFlagInFram(new_fw ? FRAM_OFFSET_SUCCESS_OTA_INSTALL_FLAG : FRAM_OFFSET_SUCCESS_BACKUP_INSTALL_FLAG, SET);

    // Reset board
    NVIC_SystemReset();
  }

  return BOOTLOADER_OK;
}
/******************************************************************************/




/**
 * @brief  This function checks whether a valid application exists in flash.
 *         The check is performed by checking the very first DWORD (4 bytes) of
 *         the application firmware. In case of a valid application, this DWORD
 *         must represent the initialization location of stack pointer - which
 *         must be within the boundaries of SRAM.
 *         If application exists in flash, the function performs the jump to the firmware.
 * @return Bootloader error code ::BOOTLOADER_ERROR_CODE
 * @retval BOOTLOADER_OK: never reached due to jump to main firmware
 * @retval BOOTLOADER_ERROR_NO_APP: if first DWORD value is not represent a valid stack pointer location
 */
uint8_t prvCheckForApplication(void)
{
  FIRMWARE_HEADER *pheader = (FIRMWARE_HEADER *)&fw_header;
  memcpy(pheader, (uint32_t *)FW_START_ADDRESS, sizeof(FIRMWARE_HEADER));

  if ((pheader->encrypted_header.firmware_entry_point > (FLASH_BASE + FLASH_SIZE)) ||
      (pheader->encrypted_header.firmware_entry_point < FLASH_BASE))
  {
    return (BOOTLOADER_ERROR_NO_APP);
  }

  if (*((volatile uint32_t *) pheader->encrypted_header.firmware_entry_point) != SRAM_END)
  {
    return (BOOTLOADER_ERROR_NO_APP);
  }
  else
  {
    /*
     * A valid program seems to exist in the third sector: we so prepare the MCU
     * to start the main firmware
     */
    prvDeInitGpios();           /* Set GPIOs to default state */
    SysTick->CTRL = 0x00;           /* Disable SysTick timer and its related interrupt */
    SysTick->LOAD = 0x00;
    SysTick->VAL  = 0x00;
    prvDeInitSystem();                /* Set MCU peripherals to default state */

    RCC->CIR = 0x00000000;           /* Disable all interrupts related to clock */
    __set_MSP(*((volatile uint32_t *) pheader->encrypted_header.firmware_entry_point));

    __DMB();                        /* ARM says to use a DMB instruction before relocating VTOR */
    SCB->VTOR = pheader->encrypted_header.firmware_entry_point;
    __DSB();                        /* ARM says to use a DSB instruction just after relocating VTOR */

    /* We are now ready to jump to the main firmware */
    uint32_t jump_address = *((volatile uint32_t *) (pheader->encrypted_header.firmware_entry_point + 4));
    void (*jump_to_firmware)(void) = (void *)jump_address;
    jump_to_firmware();
  }

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

  FRAMReadByte(new_fw ? FRAM_OFFSET_OTA_FLAG : FRAM_OFFSET_BACKUP_FLAG, &update_flag);

  if (update_flag == 0x01)
    return true;
  else
    return false;
}
/******************************************************************************/




/**
 * @brief  This function decrypts the encrypted part of the header and fills
 *         the FIRMWARE_HEADER struct with the decrypted data.
 * @retval None
 */
void prvDecryptPartOfHeader(FIRMWARE_HEADER * const pheader)
{
  uint8_t fw_buf[16];

  struct AES_ctx ctx;
  AES_init_ctx_iv(&ctx, aes_fw_key, aes_init_vector);

  uint8_t *ptr = (uint8_t *)&pheader->encrypted_header;
  uint16_t header_encrypt_4w_size = (sizeof(FIRMWARE_HEADER) - sizeof(FIRMWARE_NOT_ENCRYPTED_HEADER)) >> 4;
  for (uint32_t i = 0; i < header_encrypt_4w_size; i++)
  {
    memcpy(fw_buf, ptr, 16);
    AES_CBC_decrypt_buffer(&ctx, (uint8_t *)&fw_buf, sizeof(fw_buf));
    memcpy(ptr, fw_buf, 16);
    ptr += 16;
  }
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
    return true;
  else
    return false;
}
/******************************************************************************/




/**
 * @brief  This function verifies FW.
 * @return Bootloader error code ::BOOTLOADER_ERROR_CODE
 */
uint8_t prvVerifyFW(FIRMWARE_NOT_ENCRYPTED_HEADER *pneheader)
{
  if (pneheader->magic_number != IMAGE_MAGIC_NUMBER)
  {
    PrintfLogsCRLF("FW image is not correct");

    if(prvDecrementInstallErrorsCountInFram())
      return BOOTLOADER_ERROR_DECREMENT;

    return BOOTLOADER_ERROR_VERIFICATION;
  }

  return BOOTLOADER_OK;
}
/******************************************************************************/




/**
 * @brief  This function erases needed sectors to program of internal Flash.
 * @return Bootloader error code ::BOOTLOADER_ERROR_CODE
 * @retval BOOTLOADER_OK: upon success
 * @retval BOOTLOADER_ERROR_ERASE: upon failure
 */
uint8_t prvEraseMcuFlash(const FIRMWARE_HEADER *pheader)
{
  FIRMWARE_NOT_ENCRYPTED_HEADER *ptr = (FIRMWARE_NOT_ENCRYPTED_HEADER *)&pheader->not_encrypted_header;

  uint32_t                    nbr_of_sectors = 0;
  uint32_t                    sector_error = 0;
  FLASH_EraseInitTypeDef      pEraseInit;
  HAL_StatusTypeDef           status = HAL_OK;

  HAL_FLASH_Unlock();

  /* Get the number of sectors to erase */
  if (ptr->image_length < (96 * 1024))      /* All sectors up to the fifth have a size other than 128 Kbytes */
    nbr_of_sectors = FLASH_SECTORS_2_TO_4;  /* Erase them if image size < 96 Kbytes of sectors 2-4 */
  else
  {
    uint32_t tail;
    tail = ptr->image_length - (96 * 1024);

    if (tail > 0)
    {
      nbr_of_sectors = FLASH_SECTORS_2_TO_4 + (tail / FLASH_SECTOR_SIZE);

      if ((tail % FLASH_SECTOR_SIZE) != 0)
        nbr_of_sectors += 1;
    }
  }

  pEraseInit.TypeErase    = FLASH_TYPEERASE_SECTORS;
  pEraseInit.Banks        = FLASH_BANK_1;
  pEraseInit.Sector       = FLASH_SECTOR_2;
  pEraseInit.NbSectors    = nbr_of_sectors;
  pEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  status                  = HAL_FLASHEx_Erase(&pEraseInit, &sector_error);

  HAL_FLASH_Lock();

  return ((status == HAL_OK) ? BOOTLOADER_OK : BOOTLOADER_ERROR_ERASE);
}
/******************************************************************************/




/**
 * @brief  This function programs the header and firmware into the MCU Flash.
 *         Header confidential information is cleared before programming.
 * @retval BL_OK: upon success
 * @retval BL_ERROR_WRITE: upon failure
 */
uint8_t prvProgramFwToMcuFlash(bool new_fw, FIRMWARE_HEADER * const pheader)
{
  uint8_t fw_buf[16];
  uint8_t *ptr_u8 = (uint8_t *)pheader;
  FIRMWARE_HEADER     *ptr   = (FIRMWARE_HEADER *)pheader;
  FIRMWARE_ENCRYPTED_HEADER *ptr_e = (FIRMWARE_ENCRYPTED_HEADER *)&pheader->encrypted_header;

  struct AES_ctx ctx;
  AES_init_ctx_iv(&ctx, aes_fw_key, aes_init_vector);

  prvBeginProgramMcuFlash(ptr);

  /* Installing the header */
  memset(ptr_e->reserved1, 0, sizeof(ptr_e->reserved1));
  ptr_e->reserved2 = 0;
  ptr_e->reserved3 = 0;
  ptr_e->reserved4 = 0;

  for (uint32_t i = 0; i < (fw_size.header_size_in_bytes >> 4); i++)
  {
    if (i < (sizeof(FIRMWARE_HEADER) >> 4))
      memcpy(fw_buf, ptr_u8, sizeof(fw_buf));
    else
      memset(fw_buf, 0, sizeof(fw_buf));

    for (uint8_t j = 0; j < 4; j++)
    {
      /* Programming the decrypted firmware block */
      if (prvNextProgramMcuFlash(ptr, (uint32_t *)&fw_buf[j * 4]) == BOOTLOADER_ERROR_WRITE)
      {
        PrintfLogsCRLF("ERROR: Failure to program the %luth 4word of the header!", i);
        prvEndProgramMcuFlash();

        return BOOTLOADER_ERROR_WRITE;
      }
    }

    if (i < (sizeof(FIRMWARE_HEADER) >> 4))
      ptr_u8 += 16;
  }

  /* Installing the firmware */
  uint32_t i;
  for (i = 0; i < fw_size.fw_size_in_4words; i++)
  {
    /* Reading the encrypted firmware block from external flash */
    ExtFlashReadArray(SELECT_FAST_READ,
                       ((new_fw ? NEW_FW_ADDRESS: BACKUP_FW_ADDRESS) + (i * sizeof(fw_buf))),
                       (void *)fw_buf, sizeof(fw_buf));

    /* Decrypting the firmware block */
    AES_CBC_decrypt_buffer(&ctx, (uint8_t *)&fw_buf, sizeof(fw_buf));

    for (uint8_t j = 0; j < 4; j++)
    {
      /* Programming the decrypted firmware block */
      if (prvNextProgramMcuFlash(ptr, (uint32_t *)&fw_buf[j * 4]) == BOOTLOADER_ERROR_WRITE)
      {
        PrintfLogsCRLF("ERROR: Failure to program the %luth 4word of the firmware!", i);
        prvEndProgramMcuFlash();

        return BOOTLOADER_ERROR_WRITE;
      }
    }
  }

  prvEndProgramMcuFlash();

  return BOOTLOADER_OK;
}
/******************************************************************************/




/**
 * @brief  Begin flash programming: this function unlocks the flash and sets
 *         the data pointer to the start of application flash area.
 * @return Bootloader error code ::BOOTLOADER_ERROR_CODE
 * @retval BOOTLOADER_OK is returned in every case
 */
uint8_t prvBeginProgramMcuFlash(const FIRMWARE_HEADER *pheader)
{
  FIRMWARE_ENCRYPTED_HEADER *ptr = (FIRMWARE_ENCRYPTED_HEADER *)&pheader->encrypted_header;

  /* Reset flash destination address */
  flash_ptr = ptr->image_load_address;

  /* Unlock flash */
  HAL_FLASH_Unlock();

  return BOOTLOADER_OK;
}
/******************************************************************************/




/**
 * @brief  Program 32bit data into flash: this function writes an 4bytes (32bit)
 *         data chunk into the flash and increments the data pointer.
 * @param  data: 32bit data chunk to be written into flash
 * @return Bootloader error code ::BOOTLOADER_ERROR_CODE
 * @retval BOOTLOADER_OK: upon success
 * @retval BOOTLOADER_ERROR_WRITE: upon failure
 */
uint8_t prvNextProgramMcuFlash(const FIRMWARE_HEADER *pheader, void *data)
{
  FIRMWARE_ENCRYPTED_HEADER *ptr = (FIRMWARE_ENCRYPTED_HEADER *)&pheader->encrypted_header;

  if (!(flash_ptr <= (FLASH_BASE + FLASH_SIZE - 8)) || (flash_ptr < ptr->image_load_address))
  {
    HAL_FLASH_Lock();
    return BOOTLOADER_ERROR_WRITE;
  }

  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_ptr, *(uint32_t *)data) == HAL_OK)
  {
    /* Check the written value */
    if (*(uint32_t *)flash_ptr != *(uint32_t *)data)
    {
      /* Flash content doesn't match source content */
      HAL_FLASH_Lock();
      return BOOTLOADER_ERROR_WRITE;
    }
    /* Increment Flash destination address */
    flash_ptr += 4;
  }
  else
  {
    /* Error occurred while writing data into Flash */
    HAL_FLASH_Lock();
    return BOOTLOADER_ERROR_WRITE;
  }

  return BOOTLOADER_OK;
}
/******************************************************************************/




/**
 * @brief  Finish flash programming: this function finalizes the flash
 *         programming by locking the flash.
 * @return Bootloader error code ::BOOTLOADER_ERROR_CODE
 * @retval BOOTLOADER_OK is returned in every case
 */
uint8_t prvEndProgramMcuFlash(void)
{
  /* Lock flash */
  HAL_FLASH_Lock();

  return BOOTLOADER_OK;
}
/******************************************************************************/




/**
 * @brief  This function checks CRC32 of selected fw in external Flash.
 * @retval BOOTLOADER_OK: Calculated CRC32 matches the one in the image header
 * @retval BOOTLOADER_ERROR_CHECKSUM: upon failure
 */
uint8_t prvCheckFwCRC(bool new_fw, const FIRMWARE_HEADER *pheader)
{
  FIRMWARE_NOT_ENCRYPTED_HEADER *ptr = (FIRMWARE_NOT_ENCRYPTED_HEADER *)&pheader->not_encrypted_header;

  uint32_t calculated_crc;
  uint8_t  fw_buf[16];

  struct AES_ctx ctx_crc;
  AES_init_ctx_iv(&ctx_crc, aes_fw_key, aes_init_vector);

  calculated_crc = 0;


  /* Reset CRC peripheral before starting calculation */
  CRCReset();


  /* Calculate CRC of fw words and fw tail bytes if it exists */
  uint32_t i;
  for (i = 0; i < fw_size.fw_size_in_4words; i++)
  {
    ExtFlashReadArray(SELECT_FAST_READ,
                       ((new_fw ? NEW_FW_ADDRESS: BACKUP_FW_ADDRESS) + (i * sizeof(fw_buf))),
                       (void *)fw_buf, sizeof(fw_buf));

    /* Decrypting the firmware block */
    AES_CBC_decrypt_buffer(&ctx_crc, (uint8_t *)&fw_buf, sizeof(fw_buf));

    calculated_crc = CRCCalculate32(fw_buf, sizeof(fw_buf));
  }

  /* Calculating the CRC of encrypted firmware tail, which is mixed with encrypted 0xFF alignment bytes */
  if (fw_size.last_encrypted_fw_bytes != 0)
  {
    ExtFlashReadArray(SELECT_FAST_READ,
                       ((new_fw ? NEW_FW_ADDRESS: BACKUP_FW_ADDRESS) + (i * sizeof(fw_buf))),
                       (void *)fw_buf, sizeof(fw_buf));

    AES_CBC_decrypt_buffer(&ctx_crc, (uint8_t *)&fw_buf, sizeof(fw_buf));

    calculated_crc = CRCCalculate32(fw_buf, fw_size.last_encrypted_fw_bytes);
  }


  /* Compare calculated CRC with CRC from image header */
  if (calculated_crc == ptr->firmware_crc32)
    return BOOTLOADER_OK;
  else
    return BOOTLOADER_ERROR_CHECKSUM;
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
 * @brief  This function decrements error counter in FRAM
 *         if an error occurs while installing the firmware.
 * @retval Bootloader error code ::BOOTLOADER_ERROR_CODE
 */
uint8_t prvDecrementInstallErrorsCountInFram(void)
{
  uint8_t errors_counter = 0x00;
  uint8_t tmp = 0x00;

  FRAMReadByte(FRAM_OFFSET_INSTALL_ERRORS_COUNT, &errors_counter);

  if (errors_counter != 0x00)
  {
    errors_counter--;

    /* See prvModifyFlagInFram() for description of this problem */
    FRAMWriteByte(FRAM_OFFSET_INSTALL_ERRORS_COUNT, errors_counter);
    FRAMWriteByte(FRAM_OFFSET_INSTALL_ERRORS_COUNT, errors_counter);

    tmp = errors_counter;

    /* Make sure the flag has cleared */
    FRAMReadByte(FRAM_OFFSET_INSTALL_ERRORS_COUNT, &errors_counter);
    if (errors_counter != tmp)
      return BOOTLOADER_ERROR_DECREMENT;
  }

  return BOOTLOADER_OK;
}
/******************************************************************************/




/**
 * @brief  This function prints image information from header
 * @param  pheader: pointer to header struct ::FIRMWARE_HEADER
 * @retval None
 */
void prvPrintHeader(const FIRMWARE_HEADER *pheader)
{
  FIRMWARE_NOT_ENCRYPTED_HEADER *ptr_not_encrypted = (FIRMWARE_NOT_ENCRYPTED_HEADER *)&pheader->not_encrypted_header;
  FIRMWARE_ENCRYPTED_HEADER *ptr_encrypted = (FIRMWARE_ENCRYPTED_HEADER *)&pheader->encrypted_header;

  PrintfLogsCont("BOARD IMAGE HW %d.%d SW %d.%d.%d-",
      ptr_not_encrypted->firmware_version[0],
      ptr_not_encrypted->firmware_version[1],
      ptr_not_encrypted->firmware_version[2],
      ptr_not_encrypted->firmware_version[3],
      ptr_not_encrypted->firmware_version[4]);

  if (ptr_not_encrypted->firmware_version[5] != 0)
    PrintfLogsCont("RC%d.", ptr_not_encrypted->firmware_version[5]);

  PrintfLogsCRLF("%d", ptr_not_encrypted->firmware_version[6]);

  PrintfLogsCRLF("Full Image Size      : %lu bytes", ptr_not_encrypted->image_length);
  PrintfLogsCRLF("Firmware Size        : %lu bytes", ptr_not_encrypted->firmware_length);
  PrintfLogsCRLF("CRC32 Checksum       : 0x%08lX"  , ptr_not_encrypted->firmware_crc32);
  PrintfLogsCRLF("Image Load Address   : 0x%08lX"  , ptr_encrypted->image_load_address);
  PrintfLogsCRLF("Firmware Entry Point : 0x%08lX"  , ptr_encrypted->firmware_entry_point);
}
/******************************************************************************/




/**
 * @brief  This function fills FIRMWARE_SIZE struct for calculations
 * @param  pneheader: pointer to header struct ::FIRMWARE_NOT_ENCRYPTED_HEADER
 * @retval None
 */
void prvFillFWSize(FIRMWARE_NOT_ENCRYPTED_HEADER *pneheader)
{
  fw_size.header_size_in_bytes = pneheader->image_length - pneheader->firmware_length;
  fw_size.fw_size_in_4words = pneheader->firmware_length >> 4;
  fw_size.fw_size_in_words = pneheader->firmware_length >>2;
  fw_size.fw_tail_in_words = fw_size.fw_size_in_words % 4;
  fw_size.fw_tail_in_bytes = pneheader->firmware_length & 3;
  fw_size.last_encrypted_fw_bytes = (fw_size.fw_tail_in_words << 2) + fw_size.fw_tail_in_bytes;
}
/******************************************************************************/




/**
 * @brief  This function tries to init FLASH three times
 *         because sometimes prints error of FLASH and flash works correctly
 * @retval bool: success or not
 */
bool prvExtFlashInit(void)
{
  if (!ExtFlashInit())
    if (!ExtFlashInit())
      if (!ExtFlashInit())
        return false;

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




/**
 * @brief  De-initialize used GPIO registers function
 * @param  None
 * @retval None
 */
void prvDeInitGpios(void)
{
  LL_GPIO_DeInit(GPIOA);
  LL_GPIO_DeInit(GPIOB);
  LL_GPIO_DeInit(GPIOC);
  LL_GPIO_DeInit(GPIOH);
  __DSB();
}
/******************************************************************************/




/**
 * @brief  De-initialize system devices function
 * @param  None
 * @retval None
 */
void prvDeInitSystem(void)
{
  /* Reset of all peripherals */
  HAL_DeInit();
  LL_DMA_DeInit(DMA2, LL_DMA_STREAM_0);
  LL_DMA_DeInit(DMA2, LL_DMA_STREAM_3);
  LL_USART_DeInit(UART4);
  LL_I2C_DeInit(I2C1);
  LL_SPI_DeInit(SPI1);
  LL_CRC_DeInit(CRC);
  LL_PWR_DeInit();

  /* Disable all peripherals */
  RCC->APB1ENR = 0x00000000;
  RCC->APB2ENR = 0x00000000;
  RCC->AHB1ENR = 0x00100000;
  RCC->AHB2ENR = 0x00000000;
  RCC->AHB3ENR = 0x00000000;
  __DSB();

  /* Reset the clocks to the default reset state */
  LL_RCC_DeInit();
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
