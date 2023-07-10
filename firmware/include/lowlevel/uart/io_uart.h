/**
 ******************************************************************************
 * @file           : io_uart.h
 * @author         : Aleksandr Shabalin    <alexnv97@gmail.com>
 * @brief          : Header file of IO UART
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin ------------------ *
 ******************************************************************************
 * This module is a confidential and proprietary property of Aleksandr Shabalin
 * and possession or use of this module requires written permission
 * of Aleksandr Shabalin.
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LOWLEVEL_UART_IO_UART_H_
#define LOWLEVEL_UART_IO_UART_H_


/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "log.h"
#include "uart.h"

#include "stm32f4xx.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_rcc.h"

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************************************/
/* Public defines ----------------------------------------------------------- */
/******************************************************************************/
#define IOUART_TX_Pin                 LL_GPIO_PIN_10
#define IOUART_RX_Pin                 LL_GPIO_PIN_11
#define IOUART_Port                   GPIOC
#define IOUART_GPIO_AF                LL_GPIO_AF_8

#define IOUART_Periph                 UART4
#define IOUART_ENABLE_CLOCK()         LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4)
#define IOUART_IRQn                   UART4_IRQn
#define IOUART_IRQHandler             UART4_IRQHandler

#define IOUART_RX_RB_SIZE             (64U)
#define IOUART_TX_RB_SIZE             (512U)

#define IOUART_RX_QUEUE_SIZE          (128U)

#define PROJ_UNUSED(x)                ((void)(x))


/******************************************************************************/
/* Public variables --------------------------------------------------------- */
/******************************************************************************/
extern struct uart io_uart;


/******************************************************************************/
/* Public functions --------------------------------------------------------- */
/******************************************************************************/
void IoUartInit(void);
void IoUartPutByte(uint8_t byte);


/******************************************************************************/


#ifdef __cplusplus
}
#endif


#endif /* LOWLEVEL_UART_IO_UART_H_ */
