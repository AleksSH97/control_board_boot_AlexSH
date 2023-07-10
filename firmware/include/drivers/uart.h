/**
 ******************************************************************************
 * @file           : uart.h
 * @author         : Aleksandr Shabalin    <alexnv97@gmail.com>
 * @brief          : Header file for uart
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin ------------------ *
 ******************************************************************************
 * This module is a confidential and proprietary property of Aleksandr Shabalin
 * and possession or use of this module requires written permission
 * of Aleksandr Shabalin.
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef UART_H_
#define UART_H_


/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "indication.h"
#include "io_uart.h"

#include "stm32f4xx.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_rcc.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/******************************************************************************/
/* Public defines ----------------------------------------------------------- */
/******************************************************************************/
#define UART_BUFF_SIZE    16u


/******************************************************************************/
/* Public variables --------------------------------------------------------- */
/******************************************************************************/
extern struct uart io_uart;

typedef void (*send_byte_fn)(USART_TypeDef *USARTx);
typedef void (*receive_byte_fn)(USART_TypeDef *USARTx, struct uart *uart);
typedef void (*init_fn)(void);

typedef struct {
  send_byte_fn       send_byte;
  receive_byte_fn    receive_byte;
  init_fn            init;
} uart_ctrl_t;

struct uart {
    uint8_t       buff_rx[UART_BUFF_SIZE];
    uint8_t       buff_tx[UART_BUFF_SIZE];

    uint8_t       receive;
    uint8_t       transmit;

    uart_ctrl_t   fns;
};


/******************************************************************************/
/* Public functions --------------------------------------------------------- */
/******************************************************************************/
bool UARTInit(struct uart *self, uart_ctrl_t *fns);
void UARTSendByte(USART_TypeDef *USARTx);
void UARTReceiveByte(USART_TypeDef *USARTx, struct uart *self);
void UARTCallback(USART_TypeDef *USARTx, struct uart *uart_ptr);

/******************************************************************************/


#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif /* UART_H_ */
