/*
 * adc.c
 *
 *  Created on: Jul 22, 2022
 *      Author: Karel Hevessy
 */

#include "adc.h"
#include "main.h"



uint16_t __ALIGNED(__SCB_DCACHE_LINE_SIZE) Adc1RawValues[ADC1_INPUTS];



void AdcInit(void)
{
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
    // Start the continuous conversion
   // SCB_InvalidateDCache_by_Addr((uint32_t*) Adc1RawValues, ADC1_INPUTS*2);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*) Adc1RawValues, ADC1_INPUTS);
}

float uint32ToFloat(uint32_t num)
{
    FloatBits tmp;
    tmp.u = num;
    return tmp.f;
}

double GetInputValue(uint8_t channel)
{
    // If this is too slow, buffers can be set as not bufferable and not cacheable in the MPU
    SCB_InvalidateDCache_by_Addr((uint32_t*) Adc1RawValues, ADC1_INPUTS*2);
    uint32_t value = Adc1RawValues[channel];
    return (double) value * ADC_SCALE * ADIN_DIVIDER * 1000;
}
