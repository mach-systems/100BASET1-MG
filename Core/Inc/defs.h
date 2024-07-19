/*
 * defs.h
 *
 *  Created on: 27. 5. 2022
 *      Author: Karel Hevessy
 */

#ifndef INC_DEFS_H_
#define INC_DEFS_H_

#define DEFAULT_DATANIBBLE_SWAPPING 0

#if DEFAULT_DATANIBBLE_SWAPPING == 1
    #warning "Data nibble swapping is enabled by default."
#endif

#define IO_MAX_BIT_OFFSET       31      /* Maximum bit offset (8 nibbles) */
#define IO_MAX_LENGTH           32      /* Maximum bit length (8 nibbles) */
#define IO_MAX_VOLTAGE_OFFSET   4094    /* Maximum voltage offset */
#define IO_MAX_MULTIPLIER       4095    /* Maximum multiplier */
#define ADC_MIN_OUTPUT_MAX      4094    /* Maximum of minimum: 4094 mV */
#define ADC_MAX_OUTPUT          4095    /* Maximum: 4095 mV */
#define IO_MAX_CHANNEL          4       /* 0: Off,... 4: SENT 4 */
#define IO_NIBBLE_ORDER_CNT     2       /* Big Endian and Little Endian */

#define EEPROM_ERROR            1       /* Global flag that is set when EEPROM communication fails */

#define EMULATED_EEPROM         0
#define REAL_EEPROM             1

#define EEPROM_DRIVER           REAL_EEPROM /* From HW version 0.3, emulated EEPROM deprecated */



#endif /* INC_DEFS_H_ */
