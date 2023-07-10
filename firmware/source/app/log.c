/**
 ******************************************************************************
 * @file           : log.c
 * @author         : Aleksandr Shabalin       <alexnv97@gmail.com>
 * @brief          : logging system
 ******************************************************************************
 * ----------------- Copyright (c) 2023 Aleksandr Shabalin------------------- *
 ******************************************************************************
 ******************************************************************************
 */

/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/
#include "log.h"

#include "lwprintf/lwprintf.h"


/******************************************************************************/
/* Private defines ---------------------------------------------------------- */
/******************************************************************************/
#define LOGS_QUEUE_SIZE            (512U)
#define CONSOLE_QUEUE_SIZE         (512U)


/******************************************************************************/
/* Private variables -------------------------------------------------------- */
/******************************************************************************/
static lwprintf_t logs;


/******************************************************************************/
/* Private function prototypes ---------------------------------------------- */
/******************************************************************************/
int prvLwprintfLogsOut(int ch, lwprintf_t* p);


/******************************************************************************/


/**
 * @brief          Init logging system
 */
void LogInit(void)
{
  lwprintf_init_ex(&logs, prvLwprintfLogsOut);
}
/******************************************************************************/




/**
 * @brief          Printf of logs
 */
int PrintfLogs(const char *fmt, ...)
{
  va_list args;
  int len;

  va_start(args, fmt);
  len = lwprintf_vprintf_ex(&logs, fmt, args);
  va_end(args);

  return (len);
}
/******************************************************************************/




/**
 * @brief          Printf logs out (add to logs queue)
 */
int prvLwprintfLogsOut(int ch, lwprintf_t* p)
{
  uint8_t c = (uint8_t)ch;

  if (c == '\0') {
    return ch;           //to prevent printing '0' in the end of any (char*)
  }

  return (ch);
}
/******************************************************************************/




/**
 * @brief          Print error message at the start
 */
void LogPrintErrorMsg(void)
{
  PrintfLogsCRLF(CLR_DEF"");
  PrintfLogsCRLF("");
  PrintfLogsCRLF(CLR_RD "ERROR INIT UART");
  PrintfLogsCRLF(CLR_DEF"");
  PrintfLogsCRLF("");
}
/******************************************************************************/




/**
 * @brief          Clear screen logs
 */
void LogClearScreen(void)
{
  PrintfLogsCRLF("\033[2J");
  PrintfLogsCRLF("\033[H");
}
/******************************************************************************/




/**
 * @brief          Print welcome message at the start
 */
void LogPrintWelcomeMsg(void)
{
  PrintfLogsCRLF(CLR_DEF"");
  PrintfLogsCRLF("");
  PrintfLogsCRLF(CLR_MG"Welcome to ESS control board " CLR_GR"bootloader" CLR_RD "by AlexSH");
  PrintfLogsCRLF(CLR_DEF"");
  PrintfLogsCRLF("");
}
/******************************************************************************/
