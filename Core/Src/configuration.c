/*
 * configuration.c
 *
 *  Created on: Feb 14, 2023
 *      Author: Petr Kolář
 */
#include "configuration.h"
#include "tjaSetting.h"

static dip_stat lastDipState;

void InitDip(uint8_t dipNum)
{
    switch (dipNum)
    {
        case 0:
            lastDipState.dip1 = !READ_DIP0();
            break;
        case 1:
            lastDipState.dip2 = !READ_DIP1();
            break;
        case 2:
            lastDipState.dip3 = !READ_DIP2();
            break;
        case 3:
            lastDipState.dip4 = !READ_DIP3();
            break;
    }
}
void  SetConfigurationByDipswitch(ETH_HandleTypeDef *heth, dip_stat dipState, SwitchConfStruct * dev)
{

    if (lastDipState.dip1 != dipState.dip1 && dev->ports[TJA1_PORT-1].masterForced == 0)
    {
        dev->ports[TJA1_PORT-1].masterSelected = dipState.dip1;
        if (dipState.dip1)
            TjaSetMaster(heth,PortToTjaAddr[TJA1_PORT]);
        else
            TjaSetSlave(heth,PortToTjaAddr[TJA1_PORT]);
    }
    if (lastDipState.dip2 != dipState.dip2 && dev->ports[TJA2_PORT-1].masterForced == 0)
    {
        dev->ports[TJA2_PORT-1].masterSelected = dipState.dip2;
        if (dipState.dip2)
            TjaSetMaster(heth,PortToTjaAddr[TJA2_PORT]);
        else
            TjaSetSlave(heth,PortToTjaAddr[TJA2_PORT]);
    }
    if (lastDipState.dip3 != dipState.dip3 && dev->ports[TJA3_PORT-1].masterForced == 0)
    {
        dev->ports[TJA3_PORT-1].masterSelected = dipState.dip3;
        if (dipState.dip3)
            TjaSetMaster(heth,PortToTjaAddr[TJA3_PORT]);
        else
            TjaSetSlave(heth,PortToTjaAddr[TJA3_PORT]);
    }
    lastDipState = dipState;
}
