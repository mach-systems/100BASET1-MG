/*
 * eepromConfig.c
 *
 *  Created on: Jan 12, 2023
 *      Author: Karel Hevessy
 */

#include <stddef.h> /* NULL */
#include "eepromConfig.h"
#include "eeprom.h"

uint8_t eepromByte = 0;

int8_t EECfgReadItem(uint16_t addr, uint8_t* pData, uint16_t len)
{
    int ret = 0;
    if (0 != len && NULL != pData)
    {
        EEPROMWait();
        ret = EEPROMReadData(addr, pData, len, NULL);
        if (0 == ret)
            ret = EEPROMWait();
    }
    return ret;
}

int8_t EECfgWriteItem(uint16_t addr, uint8_t* pData, uint16_t len)
{
    EEPROMWait();
    int8_t ret = 0;
    if ((ret = EEPROMWriteData(addr, pData, len, NULL)) != -1)
        ret = EEPROMWait();
    return ret;
}

int8_t EECfgWriteByte(uint16_t addr, uint8_t data)
{
    EEPROMWait();
    eepromByte = data;
    int8_t ret;
    if ((ret = EEPROMWriteData(addr, &eepromByte, 1, NULL)) != -1)
        ret = EEPROMWait();
    return ret;
}
