/*
 * configuration.h
 *
 *  Created on: Feb 14, 2023
 *      Author: Petr Kolář
 */

#ifndef INC_CONFIGURATION_H_
#define INC_CONFIGURATION_H_
#include "main.h"
#include "switch.h"



/*
 * Status of dip switches
 */
typedef struct
{
        uint8_t dip1                :1;
        uint8_t dip2                :1;
        uint8_t dip3                :1;
        uint8_t dip4                :1;
}dip_stat;

/*
 * @brief Set the last dip switch state to opposite state than it is to make event in next call of SetConfigurationByDipswitch
 * @param dipNum number of dipswitch to be init. Indexed by 0.
 * @retval none
 */
void InitDip(uint8_t dipNum);

/*
 * @brief Change the master slave configuration of T1 port if the configuration is't fixed and the status of switch is changed
 * @param pHeth ehernet handler
 * @param dipState status of dipswitches
 * @param pDev configuration of the device
 * @retval none
 */
void SetConfigurationByDipswitch(ETH_HandleTypeDef *pHeth, dip_stat dipState, SwitchConfStruct * pDev);
#endif /* INC_CONFIGURATION_H_ */
