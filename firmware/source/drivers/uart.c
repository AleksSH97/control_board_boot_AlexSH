/**
 ******************************************************************************
 * @file           : uart.c
 * @author         : Aleksandr Shabalin       <alexnv97@gmail.com>
 * @brief          : Uart usage file
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin------------------- *
 ******************************************************************************
 ******************************************************************************
 */

/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include "uart.h"

/******************************************************************************/
/* Private variables -------------------------------------------------------- */
/******************************************************************************/




/**
 * @brief          Initialization of all uarts
 */
bool UARTInit(struct uart *self, uart_ctrl_t *fns)
{
  if (self == NULL || fns == NULL || fns->receive_byte == NULL
      || fns->send_byte == NULL || fns->init == NULL)
    return false;

  memset((void*)self->buff_tx, 0x00, sizeof(self->buff_tx));
  memset((void*)self->buff_rx, 0x00, sizeof(self->buff_rx));

  self->receive = 0;
  self->transmit = 0;

  self->fns.init = fns->init;
  self->fns.receive_byte = fns->receive_byte;
  self->fns.send_byte = fns->send_byte;

  self->fns.init();

  return true;
}
/******************************************************************************/




/**
 * @brief          UART receive byte
 */
void UARTReceiveByte(USART_TypeDef *USARTx, struct uart *self)
{
  self->receive = LL_USART_ReceiveData8(USARTx);
  //IoSystemPutDataToRxBuffer(&self->receive, sizeof(uint8_t));
}
/******************************************************************************/




/**
 * @brief          UART send byte
 */
void UARTSendByte(USART_TypeDef *USARTx)
{
  uint8_t msg = 0x00;

  //lwrb_read(buff, &msg, sizeof(uint8_t));

  while (!LL_USART_IsActiveFlag_TXE(USARTx));
  LL_USART_TransmitData8(USARTx, msg);
}
/******************************************************************************/




/**
 * @brief          Uart RX callback
 */
void UARTCallback(USART_TypeDef *USARTx, struct uart *uart_ptr)
{
  if (USARTx == IOUART_Periph)
    uart_ptr->fns.receive_byte(IOUART_Periph, uart_ptr);
}
/******************************************************************************/
