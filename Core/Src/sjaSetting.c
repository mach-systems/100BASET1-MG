/*
 * sjaSetting.c
 *
 *  Created on: 23. 1. 2023
 *      Author: Petr Kolář
 */

#include "sjaSetting.h"
#include "tools.h"
#include "NXP_SJA1105P_clockGenerationUnit.h"
#include "NXP_SJA1105P_portConfig.h"
SPI_HandleTypeDef * hspiSja;

#define L2ART_SEARCH        0x1
#define L2ART_READ          0x2
#define L2ART_WRITE         0x3
#define L2ART_INVALIDATE    0x4

typedef struct
{
        uint64_t maccaddress :48;
        uint8_t  egressPorts :5;
        uint8_t  staticEntry :1;
        uint16_t index       :12;
        //TODO age?
} artToEntry;

sja_err SjaInit(SPI_HandleTypeDef * hspi)
{
    uint32_t id;
    sja_err retVal = SJA_OK;
    SJA1105P_cfgPadMiixArgument_t pk_cfgPadMiix;
    SJA1105P_cfgPadMiixIdArgument_t pk_cfgPadMiixId;
    SJA1105P_port_t physicalPortKsz, physicalPortMcu, physicalPort;

    //Assign the SPI handler to global variable for SPI TJA Api functions
    hspiSja = hspi;

    //Register the API functions callback
    SJA1105P_registerSpiRead32CB(SjaSpiRead);
    SJA1105P_registerSpiWrite32CB(SjaSpiWrite);

    if (SJA1105P_getDeviceId(&id, 0x0))
    {
        retVal = SJA_erWRONG_ID;
    }
    if (id != 0xae00030e)
    {
        retVal = SJA_erWRONG_ID;
    }

    if (SJA1105P_initSwitch())
        retVal = SJA_INIT_ERR;
    else
    {
        SJA1105P_getPhysicalPort(KSZ_PORT-1, &physicalPortKsz);
        SJA1105P_getPhysicalPort(MCU_PORT-1, &physicalPortMcu);
        pk_cfgPadMiix.clkIh = SJA1105P_e_padInputHysteresis_NON_SCHMITT;
        pk_cfgPadMiix.ctrlIh = SJA1105P_e_padInputHysteresis_NON_SCHMITT;
        pk_cfgPadMiix.clkIpud = SJA1105P_e_padInputStageSelection_PLAIN_INPUT;
        pk_cfgPadMiix.ctrlIpud = SJA1105P_e_padInputStageSelection_PLAIN_INPUT;
        pk_cfgPadMiix.clkOs = SJA1105P_e_padOutputStageSpeed_HIGH_SPEED;
        pk_cfgPadMiix.ctrlOs = SJA1105P_e_padOutputStageSpeed_HIGH_SPEED;
        pk_cfgPadMiix.d10Ih = SJA1105P_e_padInputHysteresis_NON_SCHMITT;
        pk_cfgPadMiix.d10Ipud = SJA1105P_e_padInputStageSelection_PLAIN_INPUT;
        pk_cfgPadMiix.d10Os = SJA1105P_e_padOutputStageSpeed_HIGH_SPEED;
        pk_cfgPadMiix.d32Ih = SJA1105P_e_padInputHysteresis_NON_SCHMITT;
        pk_cfgPadMiix.d32Ipud =  SJA1105P_e_padInputStageSelection_PLAIN_INPUT;
        pk_cfgPadMiix.d32Os = SJA1105P_e_padOutputStageSpeed_HIGH_SPEED;
        SJA1105P_setCfgPadMiix(&pk_cfgPadMiix, physicalPortKsz.physicalPort, SJA1105P_e_direction_RX, 0);
        SJA1105P_setCfgPadMiix(&pk_cfgPadMiix, physicalPortKsz.physicalPort, SJA1105P_e_direction_TX, 0);



        pk_cfgPadMiix.clkIh = SJA1105P_e_padInputHysteresis_SCHMITT;
        pk_cfgPadMiix.ctrlIh = SJA1105P_e_padInputHysteresis_NON_SCHMITT;
        pk_cfgPadMiix.clkIpud = SJA1105P_e_padInputStageSelection_PLAIN_INPUT;
        pk_cfgPadMiix.ctrlIpud = SJA1105P_e_padInputStageSelection_PLAIN_INPUT;
        pk_cfgPadMiix.clkOs = SJA1105P_e_padOutputStageSpeed_FAST_SPEED;
        pk_cfgPadMiix.ctrlOs = SJA1105P_e_padOutputStageSpeed_FAST_SPEED;
        pk_cfgPadMiix.d10Ih = SJA1105P_e_padInputHysteresis_NON_SCHMITT;
        pk_cfgPadMiix.d10Ipud = SJA1105P_e_padInputStageSelection_PLAIN_INPUT;
        pk_cfgPadMiix.d10Os = SJA1105P_e_padOutputStageSpeed_FAST_SPEED;
        pk_cfgPadMiix.d32Ih = SJA1105P_e_padInputHysteresis_NON_SCHMITT;
        pk_cfgPadMiix.d32Ipud =  SJA1105P_e_padInputStageSelection_PLAIN_INPUT;
        pk_cfgPadMiix.d32Os = SJA1105P_e_padOutputStageSpeed_FAST_SPEED;
        SJA1105P_setCfgPadMiix(&pk_cfgPadMiix, physicalPortMcu.physicalPort, SJA1105P_e_direction_RX, 0);
        SJA1105P_setCfgPadMiix(&pk_cfgPadMiix, physicalPortMcu.physicalPort, SJA1105P_e_direction_TX, 0);
        //RGMII delay on switch not used, delays are set in KSZ PHY
        pk_cfgPadMiixId.rxcBypass = 1;
        pk_cfgPadMiixId.rxcPd  = 1;
        pk_cfgPadMiixId.rxcDelay = 0x08;
        pk_cfgPadMiixId.txcBypass  =  1;
        pk_cfgPadMiixId.txcDelay = 0x08;
        pk_cfgPadMiixId.txcPd = 1;
        SJA1105P_setCfgPadMiixId(&pk_cfgPadMiixId, physicalPortKsz.physicalPort, 0);
    }

    if (retVal == SJA_OK){
        SjaSetDefaultConfigurationMac(physicalPortKsz.physicalPort);
        SJA1105P_getPhysicalPort(TJA1_PORT-1, &physicalPort);
        SjaSetDefaultConfigurationMac(physicalPort.physicalPort);
        SJA1105P_getPhysicalPort(TJA2_PORT-1, &physicalPort);
        SjaSetDefaultConfigurationMac(physicalPort.physicalPort);
        SJA1105P_getPhysicalPort(TJA3_PORT-1, &physicalPort);
        SjaSetDefaultConfigurationMac(physicalPort.physicalPort);
        SjaSetDefaultConfigurationMac(physicalPortMcu.physicalPort);
    }
    return retVal;
}

uint8_t SjaReadMacConfigurationTable(SJA1105P_macCfgTableEntryArgument_t * pPk_macCfgTableEntry, uint8_t port)
{
    SJA1105P_macCfgTableControlArgument_t confControl;
    uint8_t retVal = SJA_OK;

    confControl.port = port;
    confControl.valid = 0;
    confControl.rdwrset = 0;
    retVal |= SJA1105P_setMacCfgTableControl(&confControl, 0);
    retVal |= SJA1105P_getMacCfgTableEntry(pPk_macCfgTableEntry, 0);
    return retVal;
}

uint8_t SjaWriteMacConfigurationTable(SJA1105P_macCfgTableEntryArgument_t * pPk_macCfgTableEntry, uint8_t port)
{
    SJA1105P_macCfgTableControlArgument_t confControl;
    uint8_t retVal = SJA_OK;

    confControl.port = port;
    confControl.valid = 0;
    confControl.rdwrset = 1;

    //Reconfigure the table
    retVal |= SJA1105P_setMacCfgTableControl(&confControl, 0);
    retVal |= SJA1105P_setMacCfgTableEntry(pPk_macCfgTableEntry, 0);
    confControl.valid = 1;
    retVal |= SJA1105P_setMacCfgTableControl(&confControl, 0);

    return retVal;
}

/* ------------------------------------- Mirroring ------------------------------------*/
uint8_t SjaSetMirrorPort(uint8_t portToMirror, uint32_t egressTag)
{
    SJA1105P_generalParametersControlSetArgument_t confControl;
    SJA1105P_generalParametersEntryArgument_t pk_generalParametersEntry;
    uint8_t retVal = SJA_OK;
    confControl.valid = 0;
    confControl.rdwrset = 1;

    //PortToMirror
    pk_generalParametersEntry.mirrorp = portToMirror;
    pk_generalParametersEntry.arinc = 0;
    pk_generalParametersEntry.cascadingPort = 6;
    pk_generalParametersEntry.egrMirrOuterDei = 0;
    pk_generalParametersEntry.egrMirrOuterPcp = 0;
    pk_generalParametersEntry.egrMirrOuterTag = egressTag;
    pk_generalParametersEntry.hostPort = 0;
    pk_generalParametersEntry.id = 0;
    pk_generalParametersEntry.ignore2StepFlag = 0;
    pk_generalParametersEntry.includeSrcPort0 = 1;
    pk_generalParametersEntry.includeSrcPort1 = 1;
    pk_generalParametersEntry.macFilter0 = 0x000000000000;
    pk_generalParametersEntry.macFilter1 = 0xFFFFFF0000FF;
    pk_generalParametersEntry.macFilterResult0 = 0xFFFFFFFFFFFF;
    pk_generalParametersEntry.macFilterResult1 = 0x0180C2000003;
    pk_generalParametersEntry.mgmtPrio = 5;
    pk_generalParametersEntry.mirrorPortReconfigEnable = 1;
    pk_generalParametersEntry.sendFollowUpTs0 = 1;
    pk_generalParametersEntry.sendFollowUpTs1 = 0;
    pk_generalParametersEntry.takeEgrTsOnHostPort = 0;
    pk_generalParametersEntry.tpid = 0x88A8;
    pk_generalParametersEntry.tpid2 = 0x8100;
    pk_generalParametersEntry.vlMarker = 0xFFFFFFFF;
    pk_generalParametersEntry.vlMask = 0xFFFFFFFF;

    //Reconfigure the table
    retVal |= SJA1105P_setGeneralParametersControl(&confControl, 0);
    retVal |= SJA1105P_setGeneralParametersEntry(&pk_generalParametersEntry, 0);
    confControl.valid = 1;
    retVal |= SJA1105P_setGeneralParametersControl(&confControl, 0);
    return retVal;
}

uint8_t SjaDynamicReconfigurationMirroring(uint8_t port, uint8_t ingressEnable, uint8_t egressEnable, uint32_t ingressTag)
{
    SJA1105P_macCfgTableEntryArgument_t pk_macCfgTableEntry;
    uint8_t retVal = SJA_OK;

    /* Read the table */
    retVal |= SjaReadMacConfigurationTable(&pk_macCfgTableEntry, port);

    if (ingressEnable)
        pk_macCfgTableEntry.ingmirr = 1;
    else
        pk_macCfgTableEntry.ingmirr = 0;
    if (egressEnable)
        pk_macCfgTableEntry.egrmirr = 1;
    else
        pk_macCfgTableEntry.egrmirr = 0;

    pk_macCfgTableEntry.ingressMirroringOuterTag = ingressTag & 0xFFFU;
    pk_macCfgTableEntry.ingressMirroringOuterDei = 0;
    pk_macCfgTableEntry.ingressMirroringOuterPcp = 0;
    pk_macCfgTableEntry.mirrorConflictOnEgressTag = 1;
    pk_macCfgTableEntry.mirrorConflictOnIngressEgress = 1;
    pk_macCfgTableEntry.retag = 0;

    /* Reconfigure the table */
    retVal |= SjaWriteMacConfigurationTable(&pk_macCfgTableEntry, port);
    return retVal;
}
void SjaSetDefaultConfigurationMac(uint8_t port)
{
    SJA1105P_macCfgTableEntryArgument_t pk_macCfgTableEntry;
    pk_macCfgTableEntry.base0 = 0;
    pk_macCfgTableEntry.top0 = 63;
    pk_macCfgTableEntry.enabled0 = 1;
    pk_macCfgTableEntry.base1 = 64;
    pk_macCfgTableEntry.top1 = 127;
    pk_macCfgTableEntry.enabled1 = 1;
    pk_macCfgTableEntry.base2 = 128;
    pk_macCfgTableEntry.top2 = 191;
    pk_macCfgTableEntry.enabled2 = 1;
    pk_macCfgTableEntry.base3 = 192;
    pk_macCfgTableEntry.top3 = 255;
    pk_macCfgTableEntry.enabled3 = 1;
    pk_macCfgTableEntry.base4 = 256;
    pk_macCfgTableEntry.top4 = 319;
    pk_macCfgTableEntry.enabled4 = 1;
    pk_macCfgTableEntry.base5 = 320;
    pk_macCfgTableEntry.top5 = 383;
    pk_macCfgTableEntry.enabled5 = 1;
    pk_macCfgTableEntry.base6 = 384;
    pk_macCfgTableEntry.top6 = 447;
    pk_macCfgTableEntry.enabled6 = 1;
    pk_macCfgTableEntry.base7 = 448;
    pk_macCfgTableEntry.top7 = 511;
    pk_macCfgTableEntry.enabled7 = 1;
    pk_macCfgTableEntry.dropNonA664 = 0;
    pk_macCfgTableEntry.dropDoubleTagged = 0;
    pk_macCfgTableEntry.dropSingleOuterTagged = 0;
    pk_macCfgTableEntry.dropSingleInnerTagged = 0;
    pk_macCfgTableEntry.drpuntag = 0;
    pk_macCfgTableEntry.dynLearn = 1;
    pk_macCfgTableEntry.egress = 1;
    pk_macCfgTableEntry.ingress = 1;
    pk_macCfgTableEntry.egrmirr = 0;
    pk_macCfgTableEntry.ifg = 0;
    pk_macCfgTableEntry.ingmirr = 0;
    pk_macCfgTableEntry.ingressMirroringOuterDei = 0;
    pk_macCfgTableEntry.ingressMirroringOuterPcp = 0;
    pk_macCfgTableEntry.ingressMirroringOuterTag = 0;
    pk_macCfgTableEntry.maxAge = 255;
    pk_macCfgTableEntry.mirrorConflictOnEgressTag = 0;
    pk_macCfgTableEntry.mirrorConflictOnIngressEgress = 0;
    pk_macCfgTableEntry.retag = 0;
    pk_macCfgTableEntry.speed = 0;
    pk_macCfgTableEntry.tpdelin = 0;
    pk_macCfgTableEntry.tpdelout = 0;
    pk_macCfgTableEntry.vlanPrio = 0;
    pk_macCfgTableEntry.vlanid = 555;
    pk_macCfgTableEntry.speed = SPEED_SJA1105_1G;  /* Set speed to 1G it affects only the KSZ port because on other ports the speed is set static */

    //Reconfigure the table
    SjaWriteMacConfigurationTable(&pk_macCfgTableEntry, port);
}

uint8_t SjaDynamicReconfigurationSpeed(sja_speed speed, uint8_t port)
{
    SJA1105P_macCfgTableEntryArgument_t pk_macCfgTableEntry;
    uint8_t retVal = SJA_OK;
    SJA1105P_port_t physicalPort;
    SJA1105P_getPhysicalPort(port-1, &physicalPort);
    retVal |= SjaReadMacConfigurationTable(&pk_macCfgTableEntry, physicalPort.physicalPort);
    pk_macCfgTableEntry.speed = speed;

    //Reconfigure the table
    retVal |= SjaWriteMacConfigurationTable(&pk_macCfgTableEntry, physicalPort.physicalPort);

    SJA1105P_miixClockControlRegisterArgument_t miixClockControlRegister;
    SJA1105P_idivCControlRegisterArgument_t idivCControlRegister;
    miixClockControlRegister.autoblock = 1;
    miixClockControlRegister.pd        = 0;
    idivCControlRegister.autoblock     = 1;
    idivCControlRegister.pd            = 0;
    idivCControlRegister.idiv          = SJA1105P_e_idiv_BY_1;

    //Set correct speed of clock
    if (speed ==  SPEED_SJA1105_10M)
        idivCControlRegister.idiv          = SJA1105P_e_idiv_BY_10;
    else
        idivCControlRegister.idiv          = SJA1105P_e_idiv_BY_1;

    miixClockControlRegister.clksrc = SJA1105P_e_clksrc_IDIV4;
    if (speed == SPEED_SJA1105_1G)
    {
        miixClockControlRegister.clksrc = SJA1105P_e_clksrc_PLL0;
    }

    retVal |= SJA1105P_setMiixClockControlRegister(&miixClockControlRegister, physicalPort.physicalPort, SJA1105P_e_miixInternalClk_RGMII_TX_CLK, 0);
    retVal |= SJA1105P_setIdivCControlRegister(&idivCControlRegister, physicalPort.physicalPort, 0);
    return retVal;
}



/*----------------------------------------------- VLAN --------------------------------------*/
uint8_t SjaChangeUntagedVlanId(uint8_t port, uint16_t vlanId)
{
    SJA1105P_macCfgTableEntryArgument_t pk_macCfgTableEntry;
    uint8_t retVal = SJA_OK;
    /* Read the table */
    retVal |= SjaReadMacConfigurationTable(&pk_macCfgTableEntry, port);
    pk_macCfgTableEntry.vlanid = vlanId;
    /* Reconfigure the table */
    retVal |= SjaWriteMacConfigurationTable(&pk_macCfgTableEntry, port);

    return retVal;
}

uint8_t SjaDynamicReconfigurationVlanTag(uint8_t ingressPorts, uint8_t egressPorts, uint16_t vlanId, uint8_t sendTag)
{
    uint8_t retVal = SJA_OK;
    SJA1105P_vlanLookupTableControlArgument_t confControl;
    SJA1105P_vlanLookupTableEntryArgument_t pk_vlanLookupTableEntry;
    uint8_t physicalPortsIngress;
    uint8_t physicalPortsEgress;
    SJA1105P_getPhysicalPortVector(ingressPorts, 0, &physicalPortsIngress);
    SJA1105P_getPhysicalPortVector(egressPorts, 0, &physicalPortsEgress);
    confControl.valid = 0;
    confControl.rdwrset = 1;
    confControl.valident = 1;
    if (sendTag)
        pk_vlanLookupTableEntry.tagPort = 0x1F; /* The tag of previously untagged frames is send with frame */
    else
        pk_vlanLookupTableEntry.tagPort = 0x0; /* The tag of previously untagged frames is strip */
    pk_vlanLookupTableEntry.vegrMirr = 0;
    pk_vlanLookupTableEntry.vingMirr = 0;
    pk_vlanLookupTableEntry.vlanBc = physicalPortsEgress;
    pk_vlanLookupTableEntry.vmembPort = physicalPortsIngress;
    pk_vlanLookupTableEntry.vlanid = vlanId;
    retVal |= SJA1105P_setVlanLookupTableControl(&confControl, 0);
    /* register category vlan_lookup_table_entry */
    retVal |= SJA1105P_setVlanLookupTableEntry(&pk_vlanLookupTableEntry, 0);

    confControl.valid = 1;
    retVal |= SJA1105P_setVlanLookupTableControl(&confControl, 0);

    return retVal;
}

uint8_t SjaDynamicReconfigurationInvalidateVlanTag(uint16_t vlanId)
{
    uint8_t retVal = SJA_OK;
    SJA1105P_vlanLookupTableControlArgument_t confControl;
    SJA1105P_vlanLookupTableEntryArgument_t pk_vlanLookupTableEntry;
    confControl.valid = 0;
    confControl.rdwrset = 1;
    confControl.valident = 0;

    retVal |= SJA1105P_setVlanLookupTableControl(&confControl, 0);
    /* register category vlan_lookup_table_entry */
    retVal |= SJA1105P_setVlanLookupTableEntry(&pk_vlanLookupTableEntry, 0);
    confControl.valid = 1;
    retVal |= SJA1105P_setVlanLookupTableControl(&confControl, 0);

    return retVal;
}

uint8_t SjaWriteL2AddressLookupTable(uint64_t macAddress, uint8_t destPorts, uint64_t macAddressMask, uint16_t index)
{
    uint8_t retVal = SJA_OK;
    SJA1105P_l2AddressLookupTableControlSetArgument_t confControl;
    SJA1105P_l2ArtLockedEntryArgument_t l2ArtLockedEntry;

    confControl.hostCmd = L2ART_WRITE;
    confControl.mgmtroute = 0;
    confControl.lockeds = 1;
    confControl.valid = 0;
    confControl.rdwrset = 1;
    confControl.valident = 1;

    l2ArtLockedEntry.takeTs = 0;
    l2ArtLockedEntry.mirror = 0;
    l2ArtLockedEntry.retag = 0;
    l2ArtLockedEntry.mask =  macAddressMask; /* The vlanid iotag nad macaddr mask together */
    l2ArtLockedEntry.vlanid = 0;
    l2ArtLockedEntry.tsReg = 0; /* Choose one of two register where timestamp can be save */
    l2ArtLockedEntry.mirroredVlan = 0;
    l2ArtLockedEntry.enfport = 0;
    l2ArtLockedEntry.destports = destPorts;
    l2ArtLockedEntry.index = index;
    l2ArtLockedEntry.innerOuterVlan = 0;
    l2ArtLockedEntry.macaddr = macAddress;

    retVal |= SJA1105P_setL2AddressLookupTableControl(&confControl,0);
    /* register category l2_art_locked_entry */
    retVal |= SJA1105P_setL2ArtLockedEntry(&l2ArtLockedEntry, 0);
    confControl.valid = 1;
    retVal |= SJA1105P_setL2AddressLookupTableControl(&confControl,0);
    return retVal;
}

uint8_t SjaDisableDynamicLearningOnPort(uint8_t port)
{
    SJA1105P_macCfgTableEntryArgument_t pk_macCfgTableEntry;
    uint8_t retVal = SJA_OK;

    retVal |= SjaReadMacConfigurationTable(&pk_macCfgTableEntry, port);
    pk_macCfgTableEntry.dynLearn = 0;

    //Reconfigure the table
    retVal |= SjaWriteMacConfigurationTable(&pk_macCfgTableEntry, port);
    return retVal;
}

uint8_t SjaEnableDynamicLearningOnPort(uint8_t port)
{
    SJA1105P_macCfgTableEntryArgument_t pk_macCfgTableEntry;
    uint8_t retVal = SJA_OK;

    retVal |= SjaReadMacConfigurationTable(&pk_macCfgTableEntry, port);
    pk_macCfgTableEntry.dynLearn = 1;

    //Reconfigure the table
    retVal |= SjaWriteMacConfigurationTable(&pk_macCfgTableEntry, port);
    return retVal;
}
/*----------------------------------------------- SPI I/O  --------------------------------------*/
/* Call back for SPI SJA read */
uint8_t SjaSpiRead(uint8_t deviceSelect, uint8_t wordCount, uint32_t registerAddress, uint32_t *p_registerValue)
{

    //First send head of message operation type + register address
    uint32_t head = 0x00000000; //Add write signature
    uint8_t dataToSend[4];
    uint8_t dataToReceive[4] = {0};
    uint8_t retVal = 0;
    if (osMutexAcquire(spiSjaMutexHandle, osWaitForever) == osOK)
    {
        SWITCH_CS_RESET();

        head |= ADDR_TO_HEAD(registerAddress);
        head |= (wordCount & 0x3f) << 25;

        dataToSend[0] = FIRST_BYTE(head);
        dataToSend[1] = SECOND_BYTE(head);
        dataToSend[2] = THIRD_BYTE(head);
        dataToSend[3] = FOURTH_BYTE(head);

        retVal |= HAL_SPI_TransmitReceive(hspiSja, dataToSend, dataToReceive, 1, HAL_MAX_DELAY);
        p_registerValue[0] = BYTES_TO_LONG(dataToReceive[3],dataToReceive[2],dataToReceive[1],dataToReceive[0]);

        for (uint8_t i = 0; i < wordCount; i++)
        {
            dataToReceive[0] = 0x55;
            dataToReceive[1] = 0x55;
            dataToReceive[2] = 0xCC;
            dataToReceive[3] = 0xCC;
            retVal |= HAL_SPI_Receive(hspiSja, dataToReceive, 1, HAL_MAX_DELAY);
            p_registerValue[i] = BYTES_TO_LONG(dataToReceive[3],dataToReceive[2],dataToReceive[1],dataToReceive[0]);
        }
        SWITCH_CS_SET();
    }
    osMutexRelease(spiSjaMutexHandle);
    return retVal;
}


/* Call back for SPI SJA write */
uint8_t SjaSpiWrite(uint8_t deviceSelect, uint8_t wordCount, uint32_t registerAddress, uint32_t *p_registerValue)
{
    //First send head of message operation type + register address
    uint32_t head = 0x80000000; //Add write signature
    uint8_t dataToSend[4];
    uint8_t retVal = 0;
    if (osMutexAcquire(spiSjaMutexHandle, osWaitForever) == osOK)
    {
        SWITCH_CS_RESET();

        head |= ADDR_TO_HEAD(registerAddress);

        dataToSend[0] = FIRST_BYTE(head);
        dataToSend[1] = SECOND_BYTE(head);
        dataToSend[2] = THIRD_BYTE(head);
        dataToSend[3] = FOURTH_BYTE(head);

        retVal |= HAL_SPI_Transmit(hspiSja, dataToSend, 1,HAL_MAX_DELAY);

        for (uint8_t i = 0; i < wordCount; i++)
        {
            dataToSend[0] = FIRST_BYTE(p_registerValue[i]);
            dataToSend[1] = SECOND_BYTE(p_registerValue[i]);
            dataToSend[2] = THIRD_BYTE(p_registerValue[i]);
            dataToSend[3] = FOURTH_BYTE(p_registerValue[i]);
            retVal |= HAL_SPI_Transmit(hspiSja, dataToSend, 1, HAL_MAX_DELAY);
        }
        SWITCH_CS_SET();
        osMutexRelease(spiSjaMutexHandle);
    }
    return retVal;
}





