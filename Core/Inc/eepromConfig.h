/*
 * eepromConfig.h
 *
 *  Created on: Jan 12, 2023
 *      Author: Karel Hevessy
 */

#ifndef INC_EEPROMCONFIG_H_
#define INC_EEPROMCONFIG_H_

#include <stdint.h>

/*
 * Read one item of len bytes from EEPROM.
 */
int8_t EECfgReadItem(uint16_t addr, uint8_t* pData, uint16_t len);

/*
 * Write one item of len bytes to EEPROM.
 */
int8_t EECfgWriteItem(uint16_t  addr, uint8_t* pData, uint16_t len);

/*
 * Write a single byte to EEPROM.
 */
int8_t EECfgWriteByte(uint16_t addr, uint8_t data);



#endif /* INC_EEPROMCONFIG_H_ */
