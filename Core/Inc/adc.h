/*
 * adc.h
 *
 *  Created on: Jul 22, 2022
 *      Author: Karel Hevessy
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include <stdint.h>


#define ADC_COUNT   2   /* 2 ADC inputs */
#define VSENSE_CNT          3   /* VUSB, V3V, V5V, V1V2, VPWR  */
#define ADC1_INPUTS         7   /* AI1, AI2, VUSB, V5V, V3V3, V1V2, VPWR */

/*  0 - VUSB
 *  1 - A1
 *  2 - V5V
 *  3 - V1V2
 *  4 - VPWR
 *  5 - V3V3
 *  6 - A2
 */

#define ADC_SCALE           3.3 / 4096  /* 12-bit ADC with 3.3V reference */
#define ADIN_DIVIDER        10 / 1    /* AIx divider */
#define VPWR_DIVIDER        10 / 1
#define VSENSE_DIVIDER      2 / 1       /* SENSE inputs */
#define V1V2_DIVIDER        1 / 1

//extern uint16_t  Adc1RawValues[ADC1_INPUTS];
/*
 * Union for directly accessing bits of double variable (4 bytes)
 */
typedef union
{
    float f;
    uint32_t u;
} FloatBits;


/**
 * @brief Initialize ADCs.
 * @param none
 * @retval none
 */
void AdcInit(void);

/**
 * @brief ADC sequence conversion done callback.
 * @param adcNum Number of the adc. hadc1: 0, hadc2: 1
 * @retval       none
 */
void AdcConversionDone(uint8_t adcNum);

/**
 * @brief Convert 4 byte unsigned int to float (bit by bit).
 * @param num Value to convert
 * @retval    Converted value
 */
float uint32ToFloat(uint32_t num);

/**
 * @brief Get ADC input millivolt value.
 * @param channel Channel number (0 to 6)
 * @retval        Converted value
 */
double GetInputValue(uint8_t channel);




#endif /* INC_ADC_H_ */
