/**
 ******************************************************************************
 * @file           : drivers/ext_flash.h
 * @brief          : Header for external FLASH SPI1 driver module
 ******************************************************************************
 ******************************************************************************
 */

#ifndef DRIVERS_EXT_FLASH_H_
#define DRIVERS_EXT_FLASH_H_

/******************************************************************************/
/* Includes ----------------------------------------------------------------- */
/******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************************************/
/* Public defines ----------------------------------------------------------- */
/******************************************************************************/
#define NEW_FW_HEADER_ADDRESS         (1024)                    /* New Image Header address     */
#define BACKUP_FW_HEADER_ADDRESS      (524288)                  /* Backup Image Header address  */

#define NEW_FW_ADDRESS                (NEW_FW_HEADER_ADDRESS + 512)         /* New FW address        */
#define BACKUP_FW_ADDRESS             (BACKUP_FW_HEADER_ADDRESS + 512)      /* Backup FW address     */

#define SELECT_FAST_READ              (true)
#define SELECT_READ                   (false)


/******************************************************************************/
/* Public variables --------------------------------------------------------- */
/******************************************************************************/


/******************************************************************************/
/* Public functions --------------------------------------------------------- */
/******************************************************************************/
bool ExtFlash_Init(void);
void ExtFlash_ReadArray(bool fast_read, uint32_t address, void *buf, uint32_t length);


/******************************************************************************/


#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_EXT_FLASH_H_ */
