/*
 * sjaSetting.h
 *
 *  Created on: 23. 1. 2023
 *      Author: Petr Kolář
 */

#ifndef INC_SJASETTING_H_
#define INC_SJASETTING_H_

#include "main.h"
#include "NXP_SJA1105P_spi.h"
#include "NXP_SJA1105P_config.h"
#include "NXP_SJA1105P_switchCore.h"
#include "NXP_SJA1105P_diagnostics.h"
#include "NXP_SJA1105P_ethIf.h"


#define VALUE_CHECKING

/* General parameters block reconfiguration register (address 34h)*/
#define GP_DR_ADDR  0x34

typedef enum
{
    SPEED_SJA1105_1G = 1,
    SPEED_SJA1105_100M,
    SPEED_SJA1105_10M,
} sja_speed;

typedef enum
{
    SJA_OK = 0,
    SJA_erWRONG_ID,
    SJA_INIT_ERR,
    SJA_SPI_ERR,
} sja_err;


/*
 * @brief  This function call the SJA init function that load the static configuration of the Switch
 *  It also change the speed of the Rgmii pads to High and disable the RGMII delay because the delay is
 *  provide by the KSZ, because of better stability.
 * @param SPI handler
 * @retval {0: successful, else: failed}
 */
uint8_t SjaInit(SPI_HandleTypeDef * hspi);

/*
 * @brief Definition of SPI read to SJA registers
 * @param deviceSelect select of Physical switch (ono board is only one switch so this param should be always zero).
 * @param wordCount the count of word that should be sent
 * @param registeraAddress adress of SPI register
 * @param p_registerValue value of the register
 * @retval {0: successful, else: failed}
 */
uint8_t SjaSpiRead(uint8_t deviceSelect, uint8_t wordCount, uint32_t registerAddress, uint32_t *p_registerValue);

/*
 * @brief Definition of SPI write to SJA registers
 * @param deviceSelect select of Physical switch (ono board is only one switch so this param should be always zero).
 * @param wordCount the count of word that should be sent
 * @param registeraAddress adress of SPI register
 * @param p_registerValue value of the register
 * @retval {0: successful, else: failed}
 */
uint8_t SjaSpiWrite(uint8_t deviceSelect, uint8_t wordCount, uint32_t registerAddress, uint32_t *p_registerValue);

/*
 * @brief Change the configuration of the device without reset
 * @param speed of link
 * @param port number logical
 * @retval {0: successful, else: failed}
 */
uint8_t SjaDynamicReconfigurationSpeed(sja_speed speed, uint8_t port);

/*
 * @brief fill the MAC configuration structure with default values
 * @param pPk_macCfgTableEntry structure to fill
 */
void SjaSetDefaultConfigurationMac(uint8_t port);

/*
 * @brief Set the mirror port. All packets will be send on this port from ports where is the ingress or egress mirroring enabled.
 * @param portToMirror mirror port number 0..4, (number 5 disable mirroring)
 * @param egressTag VLAN tag of egress frames
 * @retval {0: successful, else: failed}
 */
uint8_t SjaSetMirrorPort(uint8_t portToMirror, uint32_t egressTag);

/*
 * @brief Write the MAC table to given port
 * @param pPk_macCfgTableEntry table entry to write
 * @param port on which the MAC table is configured logical
 * @retval {0: successful, else: failed}
 */
uint8_t SjaWriteMacConfigurationTable(SJA1105P_macCfgTableEntryArgument_t * pPk_macCfgTableEntry, uint8_t port);

/*
 * @brief Read the MAC table of given port
 * @param pPk_macCfgTableEntry table entry to write
 * @param port on which the MAC table is configured
 * @retval {0: successful, else: failed}
 */
uint8_t SjaReadMacConfigurationTable(SJA1105P_macCfgTableEntryArgument_t * pPk_macCfgTableEntry, uint8_t port);

/*
 * @brief Invalidate the VLAN tag table entry for given VLAN ID
 * @param port
 * @retval {0: successful, else: failed}
 */
uint8_t SjaDynamicReconfigurationMirroring(uint8_t port, uint8_t ingressEnable, uint8_t egressEnable, uint32_t ingressTag);

/*
 * @brief Set up the VLAN table, the ports numbers are represented binary 3rd port = 0x8
 * @param ingressPorts  set ports ingress members of VLAN  0x0 .. 0x1F
 * @param egressPorts  set ports egress members of VLAN 0x0 .. 0x1F
 * @param tag set the VLAN tag allowed values 0x1 ... 0x4094 0x1 not recommended
 * @param sendTag if set the VLAN tag is keep with the frame otherwise is striped
 * @retval {0: successful, else: failed}
 */
uint8_t SjaDynamicReconfigurationVlanTag(uint8_t ingressPorts, uint8_t egressPorts, uint16_t tag, uint8_t sendTag);

/*
 * @brief Set the VLAN ID which is allocate to ingress utagged frame
 * @param port port number 0..4
 * @param vlanId VLAN ID for untagged ports allowed values 0x1 ... 0x4094 0x1 not recommended
 * @retval {0: successful, else: failed}
 */
uint8_t SjaChangeUntagedVlanId(uint8_t port, uint16_t vlanId);

/*
 * @brief Invalidate the VLAN tag table entry for given VLAN ID
 * @param VLAN ID of VLAN table entry
 * @retval {0: successful, else: failed}
 */
uint8_t SjaDynamicReconfigurationInvalidateVlanTag(uint16_t vlanId);

/*
 * @brief Write a new entry to address lookup table
 * @param macAddress destination MAC address of ingress packet
 * @param destPorts the destination ports where the ingress packet should be routed
 * @param macAddressMask mask of MAC adress 0 are don't care 1 are valid bits
 * @param index address in physical memory of the switch, rules with lower indexes have higher priority
 * @retval {0: successful, else: failed}
 */
uint8_t SjaWriteL2AddressLookupTable(uint64_t macAddress, uint8_t destPorts, uint64_t macAddressMask, uint16_t index);

/*
 * @brief Write a new entry to address lookup table
 * @param port on which the dynamic learning is disabled
 * @retval {0: successful, else: failed}
 */
uint8_t SjaDisableDynamicLearningOnPort(uint8_t port);

/*
 * @brief Write a new entry to address lookup table
 * @param port on which the dynamic learning is enabled
 * @retval {0: successful, else: failed}
 */
uint8_t SjaEnableDynamicLearningOnPort(uint8_t port);

#endif /* INC_SJASETTING_H_ */
