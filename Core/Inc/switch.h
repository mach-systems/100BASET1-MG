/*
 * switch.h
 *
 *  Created on: 9. 5. 2023
 *      Author: Petr Kolář
 */

#ifndef INC_SWITCH_H_
#define INC_SWITCH_H_

#include "main.h"
#include "stdint.h"
#include "NXP_SJA1105P_addressResolutionTable.h"
#define SWITCH_PORTS_COUNT          5   /* Number of Switch ports */



typedef struct __attribute__ ((packed))
{
        uint8_t ingressEnabled : 1;
        uint8_t egressEnabled  : 1;
        uint8_t addressLearning :1;
        uint8_t portSpeed :3;
        uint8_t masterSelected:1;
        uint8_t masterForced:1;
        uint16_t vlanId:12;
        uint8_t egressMirroringEnabled:1;
        uint8_t ingressMirroringEnabled:1;
        uint16_t vlanTagMirroredIngress:12;

} PortConfStruct;

/* To save memory only standard SJA1105P_addressResolutionTableEntry_t + MAC address Mask is saved without SJA1105P_extendedAddressResolutionTableEntry_t */
typedef struct
{
        SJA1105P_addressResolutionTableEntry_t entry;
        uint64_t  dstMacAddressMask;
} AddressResolutionTableEntry_t;

typedef struct __attribute__ ((packed))
{
        uint8_t enabled :1;
        uint16_t ingressPorts :5;
        uint16_t egressPorts :5;
        uint16_t vlanId :12;
        uint8_t sendTag :1;
} VlanTaggingTableEntry_t;

typedef struct __attribute__ ((packed))
{
        uint16_t vlanTagMirroredEgress:12;
        uint8_t mirrorPort;
        PortConfStruct ports[SWITCH_PORTS_COUNT];
} SwitchConfStruct;


/*
 * Add entry to resolution table.
 */
void AddResolutionTableEntry(SJA1105P_addressResolutionTableEntry_t * entry, uint64_t  dstMacAddressMask );
/*
 * Get pointer on switch configuration
 */
SwitchConfStruct* GetSwitchConfigurationAddr(void);
/*
 * Get length of address resolution table
 */
uint16_t GetResolutionTableLen(void);
/*
 * Get entry from address resolution table
 */
AddressResolutionTableEntry_t * GetResolutionTableEntry(uint16_t index);
/*
 * Take the mutex for address resolution table. Restrict the access.
 */
uint8_t TakeTableAccess(void);
/*
 * Release the mutex for address resolution table
 */
void GiveAccessToTable(void);
/*
 * Set length to 0
 */
void ClearAddResolutionTable(void);



/*
 * Check is entry with given index is learned dynamically
 */
uint8_t IsDynamicLearned(uint16_t index);

/*
 * The information if the address resolution table entry is dynamic or static learned is stored outside of the table (it can't be get from the switch itself).
 * It is stored binary 0 means dynamic change 1 static
 */
void SetStaticLearned(uint16_t index);

/*
 * The information if the address resolution table entry is dynamic or static learned is stored outside of the table (it can't be get from the switch itself).
 * It is stored binary 0 means dynamic change 1 static
 */
void SetDynamicLearned(uint16_t index);

/*
 * Delete the entry from table. The command to delete the entry is send to the switch the table stay intact and it is changed in next table read.
 */
uint8_t DeleteEntryFromTableByIndex(uint16_t entryIndex);

/*
 * Remove the Entry form VLAN table. If it is last entry the size of table is shrink. Else the entry is invalidate.
 */
uint8_t RemoveVlanTaggingTableEntry(uint16_t vlanId);
/*
 *  Get VLAN Entry from table. It must be check if entry is active.
 */
VlanTaggingTableEntry_t * GetVlanTaggingEntryByIndex(uint16_t index);

/*
 * Add VLAN tagging entry to free space or table is expanded and is insert on end of the table. If entry exists update it.
 */
uint8_t AddVlanTaggingTableEntry(uint8_t ingressPorts, uint8_t egressPorts, uint16_t vlanId, uint8_t sendTag);

/*
 * Get size of VLAN table not all records are active
 */
uint16_t GetVlanTableLen(void);

/*
 * Apply the switch configuration to PHYs and switch
 */
uint8_t ApplySwitchConfiguration(SwitchConfStruct * switchConf);

#endif /* INC_SWITCH_H_ */
