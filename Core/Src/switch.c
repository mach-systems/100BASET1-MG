/*
 * switch.c
 *
 *  Created on: 9. 5. 2023
 *      Author: Petr Kolář
 */
#include "switch.h"
#include "main.h"
#include "tools.h"
#include "sjaSetting.h"
#include "lwip.h"
#include "tjaSetting.h"
#include "kszSetting.h"

#define MAX_TABLE_LENGTH    100
uint8_t tableLocked;
AddressResolutionTableEntry_t addressResolutionTable[MAX_TABLE_LENGTH];
uint16_t addressResolutionTableLength;
VlanTaggingTableEntry_t vlanTaggingTable[MAX_TABLE_LENGTH];
uint16_t vlanTaggingTableLength;
uint16_t dynamicLearnedEntries[64]; //0 if dynamic learned 1 otherwise
static uint16_t getEntryTableIndexByIndex(uint16_t index);





/*------------------------ Address Lookup Table functions ----------------------------*/

void ClearAddResolutionTable(void)
{
    addressResolutionTableLength = 0;
}
void AddResolutionTableEntry(SJA1105P_addressResolutionTableEntry_t * entry, uint64_t  dstMacAddressMask )
{
    if (addressResolutionTableLength < MAX_TABLE_LENGTH)
        addressResolutionTable[addressResolutionTableLength++] = (AddressResolutionTableEntry_t ){ .entry = *entry, .dstMacAddressMask = dstMacAddressMask};
}

 uint16_t GetResolutionTableLen(void)
{
    return addressResolutionTableLength;
}

 AddressResolutionTableEntry_t * GetResolutionTableEntry(uint16_t index)
{
    return &addressResolutionTable[index];
}
uint8_t TakeTableAccess(void)
{
    return osMutexAcquire(tableMutexHandle, osWaitForever) == osOK;
}
void GiveAccessToTable(void)
{
    osMutexRelease(tableMutexHandle);
}

void EnableAccessToTable(void)
{
    tableLocked = 0;
}
uint8_t IsDynamicLearned(uint16_t index)
{
   return !(IS_N_BIT_SET(index % (8*sizeof(uint16_t)), dynamicLearnedEntries[BIT_INDEX(index)]));
}
void SetDynamicLearned(uint16_t index){
    dynamicLearnedEntries[BIT_INDEX(index)] &= ~BIT(index %(8*sizeof(uint16_t)));
}
void SetStaticLearned(uint16_t index){
    dynamicLearnedEntries[BIT_INDEX(index)] |= BIT(index %(8*sizeof(uint16_t)));
}

static uint16_t getEntryTableIndexByIndex(uint16_t index){
    uint16_t i;
    for (i=0; i<addressResolutionTableLength; i++){
        if (index == addressResolutionTable[i].entry.index)
            break;
    }
    if (i==addressResolutionTableLength){
        i=1024; //Entry wasn't find
    }
    return i;
}

uint8_t DeleteEntryFromTableByIndex(uint16_t entryIndex){
    uint16_t tableIndex = 0;
    uint8_t retVal = 0;
    tableIndex = getEntryTableIndexByIndex(entryIndex);
    if (tableIndex != 1024){
        addressResolutionTable[tableIndex].entry.enabled=0;
    }
    else
        retVal=1;
    return retVal;
}

/*------------------------ VLAN Tagging Table functions ----------------------------*/
uint8_t AddVlanTaggingTableEntry( uint8_t ingressPorts, uint8_t egressPorts, uint16_t vlanId, uint8_t sendTag)
{
    uint8_t retVal = 0;
    if (!(SjaDynamicReconfigurationVlanTag(ingressPorts, egressPorts, vlanId, sendTag))){
        uint16_t i;
        uint8_t firstSpace = 0,firstSpaceFound = 0;
        uint8_t matched = 0;
        for (i = 0; i<vlanTaggingTableLength;i++){
            if (vlanTaggingTable[i].vlanId == vlanId){
                matched = 1;
                break;
            }
            if (vlanTaggingTable[i].enabled == 0 && !firstSpaceFound){ //Space found
                firstSpaceFound = 1;
                firstSpace=i;
            }
        }
        if (i==vlanTaggingTableLength && vlanTaggingTableLength < MAX_TABLE_LENGTH) //No matched Id no space
        {
            firstSpace = i;
            vlanTaggingTableLength++;
        }
        if (matched){ //Found matched Id replace its contend
            firstSpace = i;
        }
        vlanTaggingTable[firstSpace] = (VlanTaggingTableEntry_t )
            { .ingressPorts = ingressPorts, .egressPorts = egressPorts, .vlanId = vlanId, .sendTag=sendTag, .enabled = 1};

    }
    else{
        retVal = 1;
    }
    return retVal;
}
VlanTaggingTableEntry_t * GetVlanTaggingEntryByIndex(uint16_t index)
{
    return &vlanTaggingTable[index];
}
uint8_t RemoveVlanTaggingTableEntry(uint16_t vlanId)
{
    uint8_t retVal = 0;
    if (!(SjaDynamicReconfigurationInvalidateVlanTag(vlanId))){
        uint16_t i;
        for (i = 0; i<vlanTaggingTableLength;i++){
            if (vlanTaggingTable[i].vlanId == vlanId && vlanTaggingTable[i].enabled == 1){
                vlanTaggingTable[i].enabled = 0;
                break;
            }
        }
        //If it is the last member
        if (i==vlanTaggingTableLength-1)
            vlanTaggingTableLength--;
    }else{
        retVal = 1;
    }
    return retVal;
}

uint16_t GetVlanTableLen(void)
{
   return vlanTaggingTableLength;
}


/*---------------------------- Switch Configuration ------------------------------------*/
uint8_t ApplySwitchConfiguration(SwitchConfStruct * switchConf){
    uint8_t retVal = 0;
    SJA1105P_port_t physicalPort;
    SJA1105P_macCfgTableEntryArgument_t macCfgTableEntry;
    for (uint8_t i=0; i<SWITCH_PORTS_COUNT; i++){
        SJA1105P_getPhysicalPort(i, &physicalPort);
        //Read actual configuration
        retVal |= SjaReadMacConfigurationTable(&macCfgTableEntry, physicalPort.physicalPort);
        //Change reconfigurable values
        macCfgTableEntry.egress = switchConf->ports[i].egressEnabled;
        macCfgTableEntry.ingress = switchConf->ports[i].ingressEnabled;
        macCfgTableEntry.dynLearn = switchConf->ports[i].addressLearning;
        macCfgTableEntry.egrmirr = switchConf->ports[i].egressMirroringEnabled;
        macCfgTableEntry.ingmirr = switchConf->ports[i].ingressMirroringEnabled;
        macCfgTableEntry.vlanid = switchConf->ports[i].vlanId;




        retVal|= SjaWriteMacConfigurationTable(&macCfgTableEntry, physicalPort.physicalPort);
        if ((i==(TJA3_PORT-1) || i==(TJA2_PORT-1) || i==(TJA1_PORT-1)) && switchConf->ports[i].masterForced == 1){
            if (TakeHethAccess()){
                if (switchConf->ports[i].masterSelected == 1)
                    TjaSetMaster(&heth,ChanneloTjaAddr[i]);
                else
                    TjaSetSlave(&heth,ChanneloTjaAddr[i]);
                GiveAccessToHeth();
            }
        }
        if (i==(KSZ_PORT-1) && PhyDev.speedLimit != switchConf->ports[i].portSpeed){
            if (TakeHethAccess()){
                SetSpeedKsz(&heth,switchConf->ports[i].portSpeed);
                GiveAccessToHeth();
            }
            PhyDev.speedLimit = switchConf->ports[i].portSpeed;
        }
    }
    SJA1105P_getPhysicalPort(switchConf->mirrorPort, &physicalPort);
    retVal|= SjaSetMirrorPort(physicalPort.physicalPort,switchConf->vlanTagMirroredEgress);
    return retVal;
}
