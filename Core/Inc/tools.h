/*
 * tools.h
 *
 *  Created on: 23. 1. 2023
 *      Author: Petr Kolář
 */

#ifndef INC_TOOLS_H_
#define INC_TOOLS_H_

#include <limits.h>
#include "main.h"
/*
 * Defines
 */

/* Bytes operations */
#define FIRST_BYTE(x)             ((uint8_t) ((x) & 0xff)) //LSB
#define SECOND_BYTE(x)            ((uint8_t) (((x) >> 8) & 0xff))
#define THIRD_BYTE(x)             ((uint8_t) (((x) >> 16) & 0xff))
#define FOURTH_BYTE(x)            ((uint8_t) (((x) >> 24) & 0xff))
#define BYTES_TO_INT(hi0, lo0)  (uint32_t) (((uint32_t)(hi0) << 8) | ((uint32_t)(lo0)))
#define BYTES_TO_LONG(hi1, lo1, hi0, lo0)  (uint32_t) (((uint32_t)(hi1) << 24) | ((uint32_t)(lo1) << 16) | ((uint32_t)(hi0) << 8) | ((uint32_t)(lo0)))
#define BYTES_TO_LONG_LONG(hi3, lo3, hi2, lo2, hi1, lo1, hi0, lo0)  (uint64_t) (((uint64_t)(hi3) << 56) | ((uint64_t)(lo3) << 48) | ((uint64_t)(hi2) << 40) | ((uint64_t)(lo2)<< 32) | ((uint64_t)(hi1) << 24) | ((uint64_t)(lo1) << 16) | ((uint64_t)(hi0) << 8) | ((uint64_t)(lo0)))
#define ADDR_TO_HEAD(registerAddress)       (registerAddress & 0x1FFFFF) << 4;
#define BIT(n) (UINT16_C(1) << (n))
#define BIT_MASK(x) (((x) >= sizeof(unsigned) * CHAR_BIT) ? (unsigned) -1 : (1U << (x)) - 1)
#define IS_N_BIT_SET(n,x) ( ((x) >> (n)) & 1 )
#define BIT_INDEX(x) ((x) / (8*sizeof(uint16_t)))
/* MMD control */
#define MMD_CONTROL_REGISTR_ADDRESS 0xD
#define MMD_ADDRESS_REGISTR_ADDRESS 0xE

#define MMD_DATA_NO_INCREMENT       0x4000




/*
 * Public functions
 */
/*
 * @brief Definition of MMD MDIO write
 * @param heth ethernet handler
 * @param address address of the device connected to MDIO
 * @param device number of device
 * @param reg address of the register
 * @param data data to be written
 * @retval none
 * */
void MmdWrite (ETH_HandleTypeDef * heth, uint16_t address, uint8_t device, uint16_t reg, uint16_t data);

/*
 * @brief Definition of MMD MDIO write
 * @param heth ethernet handler
 * @param address address of the device connected to MDIO
 * @param device number of device
 * @param reg address of the register
 * @retval register value
 * */
uint32_t MmdRead (ETH_HandleTypeDef * heth, uint16_t address, uint8_t device, uint16_t reg);

uint32_t add_o(uint32_t x, uint32_t y);

#endif /* INC_TOOLS_H_ */
