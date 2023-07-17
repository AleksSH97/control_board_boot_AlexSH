/**
 ******************************************************************************
 * @file           : flash_spi.c
 * @author         : Shabalin Aleksandr    <alexnv97@gmail.com>
 * @brief          : SPI1 low level driver
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin ------------------ *
 ******************************************************************************
 ******************************************************************************
 */

/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "stm32f4xx.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_spi.h"

#include "flash_spi.h"


/******************************************************************************/
/* Private defines ---------------------------------------------------------- */
/******************************************************************************/
#define SPI1_NSS_Pin               LL_GPIO_PIN_4
#define SPI1_NSS_GPIO_Port         GPIOA
#define SPI1_SCK_Pin               LL_GPIO_PIN_5
#define SPI1_SCK_GPIO_Port         GPIOA
#define SPI1_MISO_Pin              LL_GPIO_PIN_6
#define SPI1_MISO_GPIO_Port        GPIOA
#define SPI1_MOSI_Pin              LL_GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port        GPIOA

#define TIMEOUT_TIME               (0x1000)

/******************************************************************************/
/* Private variables -------------------------------------------------------- */
/******************************************************************************/


/******************************************************************************/
/* Private function prototypes ---------------------------------------------- */
/******************************************************************************/
void prvFlashSPISetNSSPin(void);
void prvFlashSPIResetNSSPin(void);
uint8_t prvFlashSPISendByte(uint8_t tx_byte);
void prvFlashSPIReadBuffer(void *buffer, uint16_t length);
void prvFlashSPIIsBusy(void);
void prvFlashSPIConfigDmaAddressTransfer(bool rx_tx, void *buf, uint16_t len);


/******************************************************************************/


/**
 * @brief     SPI1 init function
 * @retval    None
 */
void FlashSPIInit(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  /*--------------------------------------------------------------------------*/
  /* SPI1 GPIO Configuration                                                  */
  /*  PA4 ------> SPI1_NSS                                                    */
  /*  PA5 ------> SPI1_SCK                                                    */
  /*  PA6 ------> SPI1_MISO                                                   */
  /*  PA7 ------> SPI1_MOSI                                                   */
  /*--------------------------------------------------------------------------*/
  GPIO_InitStruct.Pin = SPI1_NSS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(SPI1_NSS_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SPI1_SCK_Pin | SPI1_MISO_Pin | SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
  __DSB();

  /* SPI1 interrupt Init */
  NVIC_SetPriority(SPI1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(SPI1_IRQn);

  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI1, &SPI_InitStruct);

  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);

  LL_SPI_Enable(SPI1);

  /*----------------------------------------------------------------------------*/
  /* Configure DMA                                                              */
  /*----------------------------------------------------------------------------*/

  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
  __DSB();

  /* SPI1_TX Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_3, LL_DMA_CHANNEL_3);
  LL_DMA_ConfigTransfer(DMA2, LL_DMA_STREAM_3,
                        LL_DMA_DIRECTION_MEMORY_TO_PERIPH |
                        LL_DMA_PRIORITY_HIGH              |
                        LL_DMA_MODE_NORMAL                |
                        LL_DMA_PERIPH_NOINCREMENT         |
                        LL_DMA_MEMORY_INCREMENT           |
                        LL_DMA_PDATAALIGN_BYTE            |
                        LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_3);

  /* SPI1_RX Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_0, LL_DMA_CHANNEL_3);
  LL_DMA_ConfigTransfer(DMA2, LL_DMA_STREAM_0,
                        LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
                        LL_DMA_PRIORITY_HIGH              |
                        LL_DMA_MODE_NORMAL                |
                        LL_DMA_PERIPH_NOINCREMENT         |
                        LL_DMA_MEMORY_INCREMENT           |
                        LL_DMA_PDATAALIGN_BYTE            |
                        LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_0);

  /* DMA2_Streamx_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);

  NVIC_SetPriority(DMA2_Stream3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(DMA2_Stream3_IRQn);
}
/******************************************************************************/




/**
 * @brief     This function implements transfer pattern for data exchange with Flash
 * @retval    None
 */
void FlashSPIReadTransaction(uint8_t command, int32_t address, void *buf, uint32_t length)
{
  prvFlashSPIResetNSSPin();

  prvFlashSPISendByte(command);

  if (address >= 0)
  {
    uint8_t i = 3;
    while (i--)
      prvFlashSPISendByte(address >> (8 * i));
  }

  if (command == FAST_READ)
    prvFlashSPISendByte(DUMMY);

  if (buf != NULL)
    prvFlashSPIReadBuffer(buf, length);

  prvFlashSPIIsBusy();

  prvFlashSPISetNSSPin();
}
/******************************************************************************/




/**
 * @brief     Non-blocking function to receive array from SPI1 device
 * @retval    None
 */
void prvFlashSPIReadBuffer(void *buffer, uint16_t length)
{
  uint8_t rx_dummy = 0;

  LL_DMA_ClearFlag_TC3(DMA2);
  LL_DMA_ClearFlag_TE3(DMA2);
  LL_DMA_ClearFlag_TC0(DMA2);
  LL_DMA_ClearFlag_TE0(DMA2);

  prvFlashSPIConfigDmaAddressTransfer(DMA_RX, (void *) buffer, length);
  LL_SPI_EnableDMAReq_RX(SPI1);
  LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_0);
  LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_0);
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);

  /* To receive via SPI, we need to send something */
  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_3, LL_DMA_MEMORY_NOINCREMENT);
  prvFlashSPIConfigDmaAddressTransfer(DMA_TX, ((void *)&rx_dummy), length);
  LL_SPI_EnableDMAReq_TX(SPI1);
  LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_3);
  LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_3);
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_3);
}
/******************************************************************************/




/**
 * @brief     Blocking function to send 1 byte via SPI1 peripheral
 * @retval    u8: unused byte
 */
uint8_t prvFlashSPISendByte(uint8_t tx_byte)
{
  uint16_t timeout = TIMEOUT_TIME;

  while ((!LL_SPI_IsActiveFlag_TXE(SPI1)) & (timeout != 0))
    timeout--;

  LL_SPI_TransmitData8(SPI1, tx_byte);

  timeout = TIMEOUT_TIME;
  while ((!LL_SPI_IsActiveFlag_RXNE(SPI1)) & (timeout != 0))
    timeout--;

  return ((uint8_t)LL_SPI_ReceiveData8(SPI1));
}
/******************************************************************************/




/**
 * @brief     This function implements the configuration of DMA transfer
 *            direction and data buffer
 * @retval    None
 */
void prvFlashSPIConfigDmaAddressTransfer(bool rx_tx, void *buf, uint16_t len)
{
  if (rx_tx == DMA_RX)
  {
    LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_0,
                           LL_SPI_DMA_GetRegAddr(SPI1),
                           (uint32_t) buf,
                           LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0, len);
  }
  else if (rx_tx == DMA_TX)
  {
    LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_3,
                           (uint32_t) buf,
                           LL_SPI_DMA_GetRegAddr(SPI1),
                           LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_3, len);
  }
}
/******************************************************************************/




/**
 * @brief     Check if SPI1 peripheral is busy
 * @retval    None
 */
void prvFlashSPIIsBusy(void)
{
  while (LL_SPI_IsActiveFlag_BSY(SPI1));
}
/******************************************************************************/




/**
 * @brief     This function sets the NSS pin to stop the SPI1 transaction
 * @retval    None
 */
void prvFlashSPISetNSSPin(void)
{
  LL_GPIO_SetOutputPin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin);
}
/******************************************************************************/



/**
 * @brief     This function resets the NSS pin to start the SPI1 transaction
 * @retval    None
 */
void prvFlashSPIResetNSSPin(void)
{
  LL_GPIO_ResetOutputPin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin);
}
/******************************************************************************/




/**
 * @brief    This function handles DMA2 Stream0 global interrupt.
 */
void DMA2_Stream0_IRQHandler(void)
{
  if (LL_DMA_IsActiveFlag_TC0(DMA2))
  {
    LL_DMA_ClearFlag_TC0(DMA2);
    LL_SPI_DisableDMAReq_RX(SPI1);
    LL_DMA_DisableIT_TC(DMA2, LL_DMA_STREAM_0);
    LL_DMA_DisableIT_TE(DMA2, LL_DMA_STREAM_0);
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);
    LL_SPI_EnableIT_RXNE(SPI1);
  }
  else if (LL_DMA_IsActiveFlag_TE0(DMA2))
  {
    LL_DMA_ClearFlag_TE0(DMA2);

  }
}
/******************************************************************************/




/**
 * @brief    This function handles DMA2 Stream3 global interrupt.
 */
void DMA2_Stream3_IRQHandler(void)
{
  if (LL_DMA_IsActiveFlag_TC3(DMA2))
  {
    LL_DMA_ClearFlag_TC3(DMA2);
    LL_SPI_DisableDMAReq_TX(SPI1);
    LL_DMA_DisableIT_TC(DMA2, LL_DMA_STREAM_3);
    LL_DMA_DisableIT_TE(DMA2, LL_DMA_STREAM_3);
    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_3);
    LL_SPI_EnableIT_TXE(SPI1);
  }
  else if (LL_DMA_IsActiveFlag_TE3(DMA2))
  {
    LL_DMA_ClearFlag_TE3(DMA2);

  }
}
/******************************************************************************/




/**
 * @brief    This function handles SPI1 global interrupt.
 */
void SPI1_IRQHandler(void)
{
  if (LL_SPI_IsActiveFlag_TXE(SPI1))
  {
    LL_SPI_DisableIT_TXE(SPI1);
  }

  if (LL_SPI_IsActiveFlag_RXNE(SPI1))
  {
    LL_SPI_DisableIT_RXNE(SPI1);
  }
}
/******************************************************************************/
