/**
 ******************************************************************************
 * @file           : led.c
 * @author         : Aleksandr Shabalin       <alexnv97@gmail.com>
 * @brief          : Driver for led usage.
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin------------------- *
 ******************************************************************************
 ******************************************************************************
 */

/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include "led.h"

/******************************************************************************/
/* Private variables -------------------------------------------------------- */
/******************************************************************************/


/**
 * @brief          Initialization of each led
 */
mculedr_t McuLedInit(mculed_t *self, mculed_ctrl_t *fns)
{
	if (self == NULL || fns == NULL || fns->turn_on == NULL || fns->turn_off == NULL) {
		return MCULED_ERROR;
	}

	self->setup.on_ms = 0;
	self->setup.off_ms = 0;
	self->setup.iterations_num = 0;
	self->setup.delay_ms = 0;

	self->status.iterations_counter = 0;
	self->status.off_timeout = 0;
	self->status.on_timeout = 0;

	self->fns.turn_on = fns->turn_on;
	self->fns.turn_off = fns->turn_off;

	return MCULED_OK;
}
/******************************************************************************/

/**
 * @brief          Led function for different modes of work
 */
void LedFunction(mculed_t *self)
{
	if (self->hardware.mode == MCULED_OFF_STATE) {
		self->fns.turn_off(self);
	}
	if (self->hardware.mode == MCULED_ON_STATE) {
		self->fns.turn_on(self);
	}
	if (self->hardware.mode == MCULED_TOGGLE_STATE) {
		self->fns.toggle(self);
	}
}
/******************************************************************************/



/**
 * @brief          Led update from SysTick_Handler for each led
 */
void LedUpdate(mculed_t *self)
{
	if (self->status.on_timeout) {
		if (!--self->status.on_timeout) {
			self->fns.turn_off(self); // TURN OFF
			if (self->setup.iterations_num) {
				self->status.iterations_counter++;
				if (self->status.iterations_counter >= self->setup.iterations_num) {
					self->setup.off_ms = 0;
					self->setup.delay_ms = 0;
				}
				self->status.off_timeout = (self->setup.off_ms + self->setup.delay_ms);
			}
		}
	}

	if (self->status.off_timeout) {
		if (!--self->status.off_timeout) {
			self->status.on_timeout = (self->setup.on_ms + self->setup.delay_ms);
			self->fns.turn_on(self); // TURN ON
		}
	}
}
/******************************************************************************/
