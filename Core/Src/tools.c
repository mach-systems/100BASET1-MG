/*
 * tools.c
 *
 *  Created on: 23. 1. 2023
 *      Author: Petr Kolář
 */

#include "tools.h"
#include "main.h"


void MmdWrite (ETH_HandleTypeDef * heth, uint16_t address, uint8_t device, uint16_t registr, uint16_t data){
  HAL_ETH_WritePHYRegister(heth,address,MMD_CONTROL_REGISTR_ADDRESS,device);
  HAL_ETH_WritePHYRegister(heth,address,MMD_ADDRESS_REGISTR_ADDRESS,registr);
  HAL_ETH_WritePHYRegister(heth,address,MMD_CONTROL_REGISTR_ADDRESS,device|MMD_DATA_NO_INCREMENT);
  HAL_ETH_WritePHYRegister(heth,address,MMD_ADDRESS_REGISTR_ADDRESS,data);
}

uint32_t MmdRead (ETH_HandleTypeDef * heth, uint16_t address, uint8_t device, uint16_t registr){
  uint32_t val = 0;
  HAL_ETH_WritePHYRegister(heth,address,MMD_CONTROL_REGISTR_ADDRESS,device);
  HAL_ETH_WritePHYRegister(heth,address,MMD_ADDRESS_REGISTR_ADDRESS,registr);
  HAL_ETH_WritePHYRegister(heth,address,MMD_CONTROL_REGISTR_ADDRESS,device|MMD_DATA_NO_INCREMENT);
  HAL_ETH_ReadPHYRegister(heth,address,MMD_ADDRESS_REGISTR_ADDRESS, &val);
  return val;
}


uint32_t inline add_o(uint32_t x, uint32_t y) {
    uint32_t s = x + y;
    s |= (uint32_t)(-(s < x));
    return s;
}
