/**
 ******************************************************************************
 * @file           : indication.h
 * @author         : Aleksandr Shabalin    <alexnv97@gmail.com>
 * @brief          : Header file for indication configs
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin ------------------ *
 ******************************************************************************
 * This module is a confidential and proprietary property of Aleksandr Shabalin
 * and possession or use of this module requires written permission
 * of Aleksandr Shabalin.
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INDICATION_H_
#define INDICATION_H_


/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include <stdbool.h>

#include "log.h"
#include "led.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_gpio.h"


/******************************************************************************/
/* Public defines ----------------------------------------------------------- */
/******************************************************************************/
#define INDICATION_LED_LOADING_NUM                (10000u)
#define INDICATION_LED_READY_NUM                  (5u)
#define INDICATION_LED_ERROR_NUM                  (2u)
#define INDICATION_LED_BUTTON_NUM                 (1u)

#define N_LED                        (3u)

#define LED_RD_Pin                   LL_GPIO_PIN_13
#define LED_RD_GPIO_Port             GPIOC
#define LED_YL_Pin                   LL_GPIO_PIN_14
#define LED_YL_GPIO_Port             GPIOC
#define LED_GR_Pin                   LL_GPIO_PIN_15
#define LED_GR_GPIO_Port             GPIOC

#define ZERO_MS                      (0u)

#define LED_RD_TIME_ON               (100u)
#define LED_YL_TIME_ON               (130u)
#define LED_GR_TIME_ON               (160u)
#define LED_RIGHT_TIME_ON            (190u)

#define LED_RD_TIME_OFF              (130u)
#define LED_YL_TIME_OFF              (160u)
#define LED_GR_TIME_OFF              (190u)
#define LED_RIGHT_TIME_OFF           (220u)

#define LED_TIME_ON                  (80u)
#define LED_TIME_OFF                 (150u)


/******************************************************************************/
/* Public variables --------------------------------------------------------- */
/******************************************************************************/
enum indication_led_id {
	LED_RD = 0,
	LED_YL,
	LED_GR,
};

enum indication_led_speed {
	INDICATION_LED_SPEED_VERY_FAST = 1,
	INDICATION_LED_SPEED_FAST,
	INDICATION_LED_SPEED_MIDDLE,
	INDICATION_LED_SPEED_SLOW,
	INDICATION_LED_SPEED_VERY_SLOW = 6,
	INDICATION_LED_SPEED_HOLD = 20
};

extern enum indication_led_id id;
extern enum indication_led_mode mode;
extern enum indication_led_speed speed;

extern mculed_t mculed[N_LED];


/******************************************************************************/
/* Public functions --------------------------------------------------------- */
/******************************************************************************/
void IndicationInit(void);

void IndicationLedYellow(uint8_t blinks);

void IndicationLedGreen(uint8_t blinks);

void IndicationLedLoading(void);

void IndicationLedRed(uint8_t blinks);

void IndicationLedButton(void);
void IndicationLedReady(void);

void IndicationLedTurnOn(mculed_t *self);
void IndicationLedTurnOff(mculed_t *self);
void IndicationLedToggle(mculed_t *self);

void IndicationLedsUpdate(void);
void IndicationLedButtonHold(void);
void IndicationLedButtonDoubleClick(void);
void IndicationUpdateTask(void *argument);


/******************************************************************************/

#endif /* INDICATION_H_ */

