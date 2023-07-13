/**
 ******************************************************************************
 * @file           : drivers/fram.c
 * @brief          : FRAM I2C driver
 ******************************************************************************
 ******************************************************************************
 */

/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include "stm32f4xx.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_i2c.h"

#include "fram.h"


/******************************************************************************/
/* Private defines ---------------------------------------------------------- */
/******************************************************************************/
#define I2C_SCL_Pin          LL_GPIO_PIN_6
#define I2C_SCL_GPIO_Port    GPIOB
#define I2C_SDA_Pin          LL_GPIO_PIN_7
#define I2C_SDA_GPIO_Port    GPIOB

#define FRAM_WP_Pin          LL_GPIO_PIN_5
#define FRAM_WP_GPIO_Port    GPIOB


#define FRAM_HW_ADDRESS      (0xA0)

#define I2C_REQUEST_WRITE    (0x0000)
#define I2C_REQUEST_READ     (0x0001)


/******************************************************************************/
/* Private variables -------------------------------------------------------- */
/******************************************************************************/



/******************************************************************************/
/* Private function prototypes ---------------------------------------------- */
/******************************************************************************/
void prvI2CInit(void);
void prvSetWriteProtect(bool wp);


/******************************************************************************/



/**
 * @brief     I2C1 init function
 * @retval    None
 */
void prvI2CInit(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_ResetOutputPin(FRAM_WP_GPIO_Port, FRAM_WP_Pin);

  /*--------------------------------------------------------------------------*/
  /* I2C1 GPIO Configuration                                                  */
  /*   PB6 ------> I2C_SCL_Pin                                                */
  /*   PB7 ------> I2C_SDA_Pin                                                */
  /*--------------------------------------------------------------------------*/
  GPIO_InitStruct.Pin = I2C_SCL_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(I2C_SCL_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = I2C_SDA_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(I2C_SDA_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = FRAM_WP_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(FRAM_WP_GPIO_Port, &GPIO_InitStruct);

  LL_GPIO_SetOutputPin(I2C_SCL_GPIO_Port, I2C_SCL_Pin);
  LL_GPIO_SetOutputPin(I2C_SDA_GPIO_Port, I2C_SDA_Pin);


  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
  __DSB();

  NVIC_SetPriority(I2C1_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(I2C1_EV_IRQn);

  NVIC_SetPriority(I2C1_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(I2C1_ER_IRQn);

  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 400000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);

  LL_I2C_SetOwnAddress2(I2C1, 0);
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);

  LL_I2C_Enable(I2C1);
}
/******************************************************************************/


/**
 * @brief     FRAM init function
 * @retval    bool:
 */
void FRAMInit(void)
{
  prvI2CInit();
  prvSetWriteProtect(true);
}
/******************************************************************************/



/**
 * @brief     This function writes 1 byte to FRAM
 * @retval    None
 */
void FRAMWriteByte(uint16_t address, uint8_t value)
{
  prvSetWriteProtect(false);


  LL_I2C_DisableBitPOS(I2C1);
  LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);

  LL_I2C_GenerateStartCondition(I2C1);
  while(!LL_I2C_IsActiveFlag_SB(I2C1)) {};

  //read state
  PROJ_UNUSED(I2C1->SR1);

  LL_I2C_TransmitData8(I2C1, FRAM_HW_ADDRESS | I2C_REQUEST_WRITE);
  while(!LL_I2C_IsActiveFlag_ADDR(I2C1)) {};
  LL_I2C_ClearFlag_ADDR(I2C1);

  LL_I2C_TransmitData8(I2C1, (uint8_t) (address >> 8));
  while(!LL_I2C_IsActiveFlag_TXE(I2C1)) {};

  LL_I2C_TransmitData8(I2C1, (uint8_t) address);
  while(!LL_I2C_IsActiveFlag_TXE(I2C1)) {};

  LL_I2C_TransmitData8(I2C1, value);
  while(!LL_I2C_IsActiveFlag_TXE(I2C1)) {};

  LL_I2C_GenerateStopCondition(I2C1);


  prvSetWriteProtect(true);
}
/******************************************************************************/



/**
 * @brief     This function reads 1 byte to FRAM
 * @retval    None
 */
void FRAMReadByte(uint16_t address, uint8_t *value)
{
  LL_I2C_DisableBitPOS(I2C1);
  LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);

  LL_I2C_GenerateStartCondition(I2C1);
  while(!LL_I2C_IsActiveFlag_SB(I2C1)) {};

  //read state
  PROJ_UNUSED(I2C1->SR1);

  LL_I2C_TransmitData8(I2C1, FRAM_HW_ADDRESS | I2C_REQUEST_WRITE);
  while(!LL_I2C_IsActiveFlag_ADDR(I2C1)) {};
  LL_I2C_ClearFlag_ADDR(I2C1);

  LL_I2C_TransmitData8(I2C1, (uint8_t) (address >> 8));
  while(!LL_I2C_IsActiveFlag_TXE(I2C1)) {};

  LL_I2C_TransmitData8(I2C1, (uint8_t) address);
  while(!LL_I2C_IsActiveFlag_TXE(I2C1)) {};

  LL_I2C_GenerateStartCondition(I2C1);
  while(!LL_I2C_IsActiveFlag_SB(I2C1)) {};

  PROJ_UNUSED(I2C1->SR1);

  LL_I2C_TransmitData8(I2C1, FRAM_HW_ADDRESS | I2C_REQUEST_READ);
  while (!LL_I2C_IsActiveFlag_ADDR(I2C1)) {};
  LL_I2C_ClearFlag_ADDR(I2C1);

  LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
  LL_I2C_GenerateStopCondition(I2C1);

  while(!LL_I2C_IsActiveFlag_RXNE(I2C1)) {};
  *value = LL_I2C_ReceiveData8(I2C1);
}
/******************************************************************************/



/**
 * @brief     This function sets or resets WriteProtection pin of FRAM
 * @retval    None
 */
void prvSetWriteProtect(bool wp)
{
  if (wp)
    LL_GPIO_SetOutputPin(FRAM_WP_GPIO_Port, FRAM_WP_Pin);
  else
    LL_GPIO_ResetOutputPin(FRAM_WP_GPIO_Port, FRAM_WP_Pin);
}
/******************************************************************************/
