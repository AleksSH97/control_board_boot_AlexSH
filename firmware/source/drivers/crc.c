/**
 ******************************************************************************
 * @file           : crc.c
 * @brief          : STM32 CRC peripheral driver
 ******************************************************************************
 */

/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include <stddef.h>

#include "stm32f4xx.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_crc.h"

#include "crc.h"


/******************************************************************************/
/* Private defines ---------------------------------------------------------- */
/******************************************************************************/



/******************************************************************************/
/* Private variables -------------------------------------------------------- */
/******************************************************************************/



/******************************************************************************/
/* Private function prototypes ---------------------------------------------- */
/******************************************************************************/



/******************************************************************************/



/**
 * @brief          CRC peripheral init function
 */
void CRCInit(void)
{
  /* Peripheral clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);
}
/******************************************************************************/



/**
 * @brief          This function resets the CRC peripheral and must be called before
 *                     each new CRC calculation
 */
void CRCReset(void)
{
  LL_CRC_ResetCRCCalculationUnit(CRC);
}
/******************************************************************************/



/**
 * @brief          This function calculates checksum of input data in CRC peripheral
 * @return         Calculated CRC32 of input data
 */
uint32_t CRCCalculate32(const void *pbuffer, size_t num_of_byte)
{
  uint32_t  last_data;
  uint8_t   *pbuf8  = (uint8_t*)pbuffer;
  uint32_t  *pbuf32 = (uint32_t*)pbuffer;
  uint32_t  num_of_word = num_of_byte >> 2;
  uint32_t  num_of_tail_bytes =  num_of_byte & 3;

  while (num_of_word > 0)
  {
    LL_CRC_FeedData32(CRC, *pbuf32++);
    --num_of_word;
  }

  switch (num_of_tail_bytes)
  {
    case 0: {
      return (LL_CRC_ReadData32(CRC));
    }
    case 1: {
      last_data = pbuf8[num_of_byte - 1] << 24;
      break;
    }
    case 2: {
      last_data = *((uint16_t*)(&pbuf8[num_of_byte - 2]));
      last_data <<= 16;
      break;
    }
    case 3: {
      last_data = *((uint16_t*)(&pbuf8[num_of_byte - 3]));
      last_data <<= 8;
      last_data += pbuf8[num_of_byte - 1] << 24;
      break;
    }
    default:
      break;
  }

  LL_CRC_FeedData32(CRC, last_data);

  return (LL_CRC_ReadData32(CRC));
}
/******************************************************************************/
