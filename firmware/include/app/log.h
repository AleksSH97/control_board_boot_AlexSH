/**
 ******************************************************************************
 * @file           : log.h
 * @author         : Aleksandr Shabalin    <alexnv97@gmail.com>
 * @brief          : Header file for logging
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin ------------------ *
 ******************************************************************************
 * This module is a confidential and proprietary property of Aleksandr Shabalin
 * and possession or use of this module requires written permission
 * of Aleksandr Shabalin.
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LOG_H_
#define LOG_H_


/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include <stdarg.h>
#include <stdio.h>

#include "lwprintf/lwprintf.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/******************************************************************************/
/* Public defines ----------------------------------------------------------- */
/******************************************************************************/
/* List of supported ESC symbol colors */
#define CLR_RD         "\033[31m"               /* Red color      */
#define CLR_GR         "\033[32m"               /* Green color    */
#define CLR_YL         "\033[33m"               /* Yellow color   */
#define CLR_BL         "\033[34m"               /* Blue color     */
#define CLR_PR         "\033[35m"               /* Purple color   */
#define CLR_MG         "\033[36m"               /* Magenta color  */
#define CLR_WT         "\033[37m"               /* White color    */
#define CLR_DEF        "\033[0m"                /* Default color  */

#define CLR_CLR        "\033[2J\033[H\033[0m"   /* Clear terminal */


/******************************************************************************/
/* Public variables --------------------------------------------------------- */
/******************************************************************************/


/******************************************************************************/
/* Public defines --------------------------------------------------------- */
/******************************************************************************/
#define    PrintfLogsCRLF(fmt, ...)               do { PrintfLogs((fmt), ## __VA_ARGS__); PrintfLogs("\r\n"); } while (0)

#define    PrintfLogsCont(fmt, ...)               PrintfLogs((fmt), ## __VA_ARGS__)

#define    PrintfConsoleCRLF(fmt, ...)           do { PrintfConsole((fmt), ## __VA_ARGS__); PrintfConsole("\r\n"); } while (0)

#define    PrintfConsoleCont(fmt, ...)           PrintfConsole((fmt), ## __VA_ARGS__)


/******************************************************************************/
/* Public functions --------------------------------------------------------- */
/******************************************************************************/
void LogClearScreen(void);

void LogInit(void);

void LogPrintErrorMsg(void);
void LogPrintWelcomeMsg(void);

int PrintfLogs(const char *fmt, ...);


/******************************************************************************/


#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif /* LOG_H_ */
