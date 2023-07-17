/**
 ******************************************************************************
 * @file           : ext_flash.c
 * @brief          : External Flash SPI driver. This driver can only read
 *                   memory arrays of Flash.
 ******************************************************************************
 ******************************************************************************
 */

/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include <stddef.h>
#include <stdbool.h>

#include "stm32f4xx.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_spi.h"

#include "ext_flash.h"
#include "flash_spi.h"


/******************************************************************************/
/* Private defines ---------------------------------------------------------- */
/******************************************************************************/
#define MANUFAC_ID                 (0x1F460200)

#define BUSY_BIT_Pos               (0U)
#define BUSY_BIT_Msk               (0x1U << BUSY_BIT_Pos)
#define BUSY_BIT                   BUSY_BIT_Msk


/******************************************************************************/
/* Private variables -------------------------------------------------------- */
/******************************************************************************/


/******************************************************************************/
/* Private function prototypes ---------------------------------------------- */
/******************************************************************************/
bool prvCheckFlashID(void);
void prvIsBusy(void);
void prvSPIIsBusy(void);
void prvSPIConfigDmaAddressTransfer(bool rx_tx, void *buf, uint16_t len);


/******************************************************************************/


/**
 * @brief     External Flash init function
 * @retval    bool: Success or not when reading manufacturer id from flash
 */
bool ExtFlash_Init(void)
{
  FlashSPIInit();

  if (!prvCheckFlashID())
    return (false);

  return (true);
}
/******************************************************************************/




/**
 * @brief     This function implements reading the Flash Manufacturer ID
 * @retval    bool: Manufacturer ID readed or not
 */
bool prvCheckFlashID(void)
{
  uint8_t manuf_id[4];

  FlashSPIReadTransaction(MANUFAC_ID_CMD, -1, &manuf_id[0], (sizeof(manuf_id) / sizeof(manuf_id[0])));

  if(((manuf_id[0] << 24) | (manuf_id[1] << 16) | (manuf_id[2] << 8) | manuf_id[3]) == MANUFAC_ID)
    return (true);
  else
    return (false);
}
/******************************************************************************/


/**
 * @brief     This function reads the BUSY flag of the Flash
 * @retval    None
 */
void prvIsBusy(void)
{
  uint8_t status = 0;

  do {
    FlashSPIReadTransaction(RSTATUS, -1, &status, 1);
  } while (status & BUSY_BIT);
}
/******************************************************************************/



/**
 * @brief     This function implements sequential reading of Flash
 * @retval    None
 */
void ExtFlash_ReadArray(bool fast_read, uint32_t address, void *buf, uint32_t length)
{
  prvIsBusy();

  if (fast_read == SELECT_FAST_READ)
    FlashSPIReadTransaction(FAST_READ, address, buf, length);
  else if (SELECT_READ)
    FlashSPIReadTransaction(READ_CMD, address, buf, length);
}
/******************************************************************************/

