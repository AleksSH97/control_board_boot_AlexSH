/**
 ******************************************************************************
 * @file           : led.h
 * @author         : Aleksandr Shabalin    <alexnv97@gmail.com>
 * @brief          : Header file for led driver
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin ------------------ *
 ******************************************************************************
 * This module is a confidential and proprietary property of Aleksandr Shabalin
 * and possession or use of this module requires written permission
 * of Aleksandr Shabalin.
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LED_H_
#define LED_H_


/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/******************************************************************************/
/* Public defines ----------------------------------------------------------- */
/******************************************************************************/


/******************************************************************************/
/* Public variables --------------------------------------------------------- */
/******************************************************************************/
struct mculed;

typedef enum mculedr {
	MCULED_OK = 0x00,
	MCULED_UNKNOWN = 0x01,
	MCULED_ERROR = 0xFF
} mculedr_t;

typedef enum mculed_mode {
	MCULED_OFF_STATE = 0,
	MCULED_ON_STATE,
	MCULED_BLINK_STATE,
	MCULED_TOGGLE_STATE,
	MCULED_LED_LOADING
} mculed_mode_t;

typedef void (*turn_on_fn)(struct mculed *self);
typedef void (*turn_off_fn)(struct mculed *self);
typedef void (*toggle_fn)(struct mculed *self);

typedef struct {
	turn_on_fn     turn_on;
	turn_off_fn    turn_off;
	toggle_fn      toggle;
} mculed_ctrl_t;

typedef struct mculed {

	struct {
		uint16_t on_ms;
		uint16_t off_ms;
		uint16_t delay_ms;
		uint16_t iterations_num;
	} setup;

	struct {
		uint32_t pin;
		uint32_t port;
		uint8_t mode;
	} hardware;

	struct {
		uint16_t on_timeout;
		uint16_t off_timeout;
		uint16_t iterations_counter;
	} status;

	mculed_ctrl_t fns;
} mculed_t;


/******************************************************************************/
/* Public functions --------------------------------------------------------- */
/******************************************************************************/
void LedFunction(mculed_t *led_ptr);
mculedr_t McuLedInit(mculed_t *self, mculed_ctrl_t *fns);
void LedUpdate(mculed_t *self);


/******************************************************************************/


#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


#endif /* LED_H_ */
