/*
 * eeprom.h
 *
 *  Created on: Jan 11, 2023
 *      Author: Karel Hevessy
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include <stdint.h>


#define EE_SPI          SPI2


typedef enum
{
    EeTimeout = -1,     /* Last operation ended with timeout */
    EeIdle = 0,         /* EEPROM is idle */
    EeRead = 1,         /* Reading a data */
    EeWrite = 2,        /* Writing a data */
    EeWriteWait = 3,    /* Waiting until page write is done */
    EeReadSt = 4,       /* Reading status register */
    EeWriteSt = 5,      /* Writing status register  */
    EeErase = 6,        /* Erasing EEPROM */
    EeEraseWait = 7,    /* Waiting until erasing is done */
} EeOperation;

/**
 * Initialize EEPROM by dummy read.
 * @param  None
 * @retval None
 */
void EepromInit(void);

/*
 * Read the EEPROM status register.
 */
int8_t EEPROMReadStatus(uint8_t * st, void (*callback)(int16_t));

/*
 * Write the EEPROM status register.
 */
int8_t EEPROMWriteStatus(uint8_t* st, void (*callback)(int16_t));

/*
 * Wait until operation is done.
 */
int8_t EEPROMWait(void);

/*
 * Return status of the current operation.
 */
EeOperation EEPROMStat(void);

/*
 * Read block of data.
 */
int8_t EEPROMReadData(uint16_t addr, uint8_t *pBuf, uint16_t len, void (*callback)(int16_t));

/*
 * Write block of data.
 */
int8_t EEPROMWriteData(uint16_t addr, uint8_t* pBuf, uint16_t len, void (*callback)(int16_t));

/*
 * Erase (write 0xFF) a block.
 */
int8_t EEPROMErase(uint16_t addr, uint16_t len, void (*callback)(int16_t));

/*
 * Called from SPI interrupt.
 */
uint8_t EepromSpiCallback(void);

/*
 * Debug function, not to be used normally.
 */
void TestEepromAccess(void);



#endif /* INC_EEPROM_H_ */
